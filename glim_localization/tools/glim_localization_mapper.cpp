// Headless GLIM mapper for evaluation: replays a dataset (see docs/DESIGN.md) through GLIM odometry, sub mapping and
// global mapping, saves a GLIM dump, and writes GLIM's optimized trajectory as a consistency reference.
// Written against GLIM's public headers. Not a supported mapper; use GLIM itself to build maps for production.
#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <glim/mapping/async_global_mapping.hpp>
#include <glim/mapping/async_sub_mapping.hpp>
#include <glim/mapping/global_mapping_base.hpp>
#include <glim/mapping/sub_mapping_base.hpp>
#include <glim/odometry/async_odometry_estimation.hpp>
#include <glim/odometry/odometry_estimation_base.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/util/config.hpp>

#include <glim_localization/replay/replay_dataset.hpp>

namespace fs = std::filesystem;
using namespace glim_localization;

namespace {

enum ExitCode { kOk = 0, kRuntimeFailure = 1, kBadUsage = 2, kBadData = 3 };

constexpr double kMaxSkippedFraction = 0.05;  // Above this share of frames without IMU coverage the map is not what was asked for
constexpr int kMaxQueuedFrames = 5;  // Backpressure: keeps the memory bounded when the odometry is slower than the feeder

struct UsageError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

struct Arguments {
  std::string data, config, out;
  std::optional<double> start, end;
};

void print_usage() {
  std::cerr << "usage: glim_localization_mapper --data <dir> --config <dir> --out <dir> [--start <s>] [--end <s>]\n"
               "\n"
               "  --data        replay dataset directory\n"
               "  --config      GLIM config directory (config.json plus sensors, preprocess, odometry, sub mapping and global mapping)\n"
               "  --out         output directory: dump/ (GLIM map), traj_glim.txt (TUM, T_world_imu), mapper_summary.json\n"
               "  --start/--end only use frames with start <= stamp <= end (absolute stamps [s])\n"
               "\n"
               "An evaluation utility. traj_glim.txt is GLIM's own trajectory: a consistency reference, not ground truth.\n"
               "exit codes: 0 ok, 1 runtime failure, 2 bad arguments/config, 3 invalid dataset\n";
}

double parse_double(const std::string& option, const std::string& text) {
  size_t used = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &used);
  } catch (const std::exception&) {
    used = 0;
  }
  if (used != text.size() || !std::isfinite(value)) {
    throw UsageError(option + " needs a finite number, got '" + text + "'");
  }
  return value;
}

Arguments parse_arguments(int argc, char** argv) {
  Arguments args;
  for (int i = 1; i < argc; i++) {
    const std::string option = argv[i];
    const auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw UsageError(option + " needs a value");
      }
      return argv[++i];
    };
    if (option == "--data") {
      args.data = next();
    } else if (option == "--config") {
      args.config = next();
    } else if (option == "--out") {
      args.out = next();
    } else if (option == "--start") {
      args.start = parse_double(option, next());
    } else if (option == "--end") {
      args.end = parse_double(option, next());
    } else {
      throw UsageError("unknown option " + option);
    }
  }
  if (args.data.empty() || args.config.empty() || args.out.empty()) {
    throw UsageError("--data, --config and --out are required");
  }
  return args;
}

double peak_rss_gb() {
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_maxrss / (1024.0 * 1024.0);  // KiB on Linux
}

template <typename Module>
std::shared_ptr<Module> load(const glim::Config& config, const char* section, const char* what) {
  const auto so_name = config.param<std::string>(section, "so_name");
  if (!so_name) {
    throw UsageError(std::string("the ") + what + " config has no " + section + "/so_name");
  }
  auto module = Module::load_module(*so_name);
  if (!module) {
    throw std::runtime_error("cannot load " + *so_name + " for the " + what + " (is it on LD_LIBRARY_PATH?)");
  }
  return module;
}

}  // namespace

int main(int argc, char** argv) {
  // The summary marks a finished run, so never keep an old one, even when the arguments, config or data turn out to be bad.
  // Looked up before the parsing; nothing is touched without a usable --out, and only that one file is removed.
  for (int i = 1; i + 1 < argc; i++) {
    if (std::string(argv[i]) == "--out" && argv[i + 1][0] != '\0' && std::string(argv[i + 1]).rfind("--", 0) != 0) {
      std::error_code ignored;
      fs::remove(fs::path(argv[i + 1]) / "mapper_summary.json", ignored);
      break;
    }
  }
  Arguments args;
  try {
    args = parse_arguments(argc, argv);
  } catch (const UsageError& e) {
    std::cerr << "error: " << e.what() << "\n\n";
    print_usage();
    return kBadUsage;
  }

  try {
    if (!fs::is_regular_file(fs::path(args.config) / "config.json")) {
      throw UsageError("cannot open " + (fs::path(args.config) / "config.json").string());
    }
    glim::GlobalConfig::instance(args.config, true);
    const auto dataset = ReplayDataset::open(args.data);
    fs::create_directories(args.out);

    glim::CloudPreprocessor preprocessor;
    const glim::Config odometry_config(glim::GlobalConfig::get_config_path("config_odometry"));
    const glim::Config sub_mapping_config(glim::GlobalConfig::get_config_path("config_sub_mapping"));
    const glim::Config global_mapping_config(glim::GlobalConfig::get_config_path("config_global_mapping"));
    auto odometry_module = load<glim::OdometryEstimationBase>(odometry_config, "odometry_estimation", "odometry");
    auto sub_mapping_module = load<glim::SubMappingBase>(sub_mapping_config, "sub_mapping", "sub mapping");
    auto global_mapping_module = load<glim::GlobalMappingBase>(global_mapping_config, "global_mapping", "global mapping");

    glim::AsyncOdometryEstimation odometry(odometry_module, odometry_module->requires_imu());
    glim::AsyncSubMapping sub_mapping(sub_mapping_module);
    glim::AsyncGlobalMapping global_mapping(global_mapping_module, 1);

    std::vector<glim::SubMap::Ptr> submaps;
    int frames_selected = 0;  // inside --start/--end, whether fed, skipped or empty
    int frames_fed = 0;
    int frames_skipped_no_imu = 0;
    int frames_to_sub_mapping = 0;
    int64_t odometry_results = 0;

    // Moves finished work down the pipeline: odometry -> sub mapping -> global mapping
    const auto drain = [&]() {
      std::vector<glim::EstimationFrame::ConstPtr> results, marginalized;
      odometry.get_results(results, marginalized);
      odometry_results += std::count_if(results.begin(), results.end(), [](const auto& frame) { return frame != nullptr; });
      for (const auto& frame : marginalized) {
        sub_mapping.insert_frame(frame);
        frames_to_sub_mapping++;
      }
      for (const auto& submap : sub_mapping.get_results()) {
        global_mapping.insert_submap(submap);
        submaps.push_back(submap);
      }
    };

    const auto begin = std::chrono::steady_clock::now();
    const auto& imu = dataset.imu();
    size_t imu_fed = 0;
    for (size_t index = 0; index < dataset.frames().size(); index++) {
      const double stamp = dataset.frames()[index].stamp_ns * 1e-9;
      if ((args.start && stamp < *args.start) || (args.end && stamp > *args.end)) {
        continue;
      }
      frames_selected++;
      const auto preprocessed = preprocessor.preprocess(dataset.load_cloud(index));

      // Every IMU sample up to the end of the scan, then the one after it (the same order as the replay)
      const auto after_end = std::upper_bound(
        imu.begin(), imu.end(), preprocessed->scan_end_time, [](double end_time, const ReplayImu& sample) { return end_time < sample.stamp_ns * 1e-9; });
      if (after_end == imu.end()) {
        // The odometry waits for an IMU sample after the end of the scan: a frame the IMU does not cover would block it for
        // good (and the feeder behind it), so skip it as the replay does
        frames_skipped_no_imu++;
        continue;
      }
      const size_t target = std::min(static_cast<size_t>(after_end - imu.begin()) + 1, imu.size());
      for (; imu_fed < target; imu_fed++) {
        const double t = imu[imu_fed].stamp_ns * 1e-9;
        odometry.insert_imu(t, imu[imu_fed].acc, imu[imu_fed].gyro);
        sub_mapping.insert_imu(t, imu[imu_fed].acc, imu[imu_fed].gyro);
        global_mapping.insert_imu(t, imu[imu_fed].acc, imu[imu_fed].gyro);
      }
      if (preprocessed->size() > 0) {
        odometry.insert_frame(preprocessed);
        frames_fed++;
      }

      drain();
      while (odometry.workload() > kMaxQueuedFrames || sub_mapping.workload() > kMaxQueuedFrames * 4 || global_mapping.workload() > kMaxQueuedFrames) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        drain();
      }
      if (frames_fed % 100 == 0 && frames_fed > 0) {
        spdlog::info("fed {} frames, {} submaps, peak RSS {:.2f} GB", frames_fed, submaps.size(), peak_rss_gb());
      }
    }

    // Shutdown in pipeline order, forwarding the remaining work at every stage
    odometry.join();
    drain();
    sub_mapping.join();
    drain();
    global_mapping.join();
    if (submaps.empty()) {
      throw std::runtime_error("no submap was created (is the sequence long enough, and does the IMU cover it?)");
    }

    const fs::path dump = fs::path(args.out) / "dump";
    fs::create_directories(dump);
    global_mapping.save(dump.string());
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();

    // Trajectory: the frames of a submap keep the poses of its own optimization (odometry world frame), while the global
    // mapping moves only T_world_origin. The origin frame still holds the pose T_world_origin had when the submap was made.
    std::map<long, std::pair<double, Eigen::Isometry3d>> poses;
    for (const auto& submap : submaps) {
      const Eigen::Isometry3d T_origin_odom = submap->origin_frame()->T_world_imu.inverse();
      for (const auto& frame : submap->frames) {
        poses[frame->id] = {frame->stamp, submap->T_world_origin * T_origin_odom * frame->T_world_imu};
      }
    }
    {
      std::ofstream traj(fs::path(args.out) / "traj_glim.txt", std::ios::trunc);
      for (const auto& entry : poses) {
        const auto& [stamp, T] = entry.second;
        const Eigen::Quaterniond q(T.linear());
        char line[256];
        std::snprintf(
          line, sizeof(line), "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n", stamp, T.translation().x(), T.translation().y(), T.translation().z(), q.x(), q.y(),
          q.z(), q.w());
        traj << line;
      }
      if (!traj) {
        throw std::runtime_error("cannot write traj_glim.txt");
      }
    }

    nlohmann::json summary;
    summary["frames_fed"] = frames_fed;
    summary["frames_skipped_no_imu"] = frames_skipped_no_imu;
    summary["frames_to_sub_mapping"] = frames_to_sub_mapping;
    summary["odometry_results"] = odometry_results;
    summary["num_submaps"] = submaps.size();
    summary["num_trajectory_poses"] = poses.size();
    // Over the selected frames, so skipped frames count as missing
    summary["frames_selected"] = frames_selected;
    summary["trajectory_coverage"] = frames_selected > 0 ? static_cast<double>(poses.size()) / frames_selected : 0.0;
    const bool too_many_skipped = frames_skipped_no_imu > kMaxSkippedFraction * frames_selected;
    summary["failed"] = too_many_skipped;
    if (too_many_skipped) {
      summary["failure"] = "more than 5% of the selected frames have no IMU coverage";
    }
    // Frames the odometry consumed while initializing (IMU state estimation) have no pose; coverage is also given without them
    summary["trajectory_coverage_of_estimated"] = odometry_results > 0 ? static_cast<double>(poses.size()) / odometry_results : 0.0;
    summary["wall_seconds"] = wall;
    summary["peak_rss_gb"] = peak_rss_gb();
    {
      std::ofstream ofs(fs::path(args.out) / "mapper_summary.json", std::ios::trunc);
      ofs << summary.dump(2) << "\n";
      ofs.flush();
      if (!ofs) {
        throw std::runtime_error("cannot write mapper_summary.json");
      }
    }
    spdlog::info(
      "mapped {} frames into {} submaps in {:.1f} s (peak RSS {:.2f} GB); {} trajectory poses", frames_fed, submaps.size(), wall, peak_rss_gb(), poses.size());
    if (too_many_skipped) {
      std::cerr << "error: invalid dataset: " << frames_skipped_no_imu << " of " << frames_selected
                << " frames have no IMU coverage (limit 5%); see mapper_summary.json" << std::endl;
      return kBadData;
    }
    return kOk;
  } catch (const InvalidDatasetError& e) {
    std::cerr << "error: invalid dataset: " << e.what() << std::endl;
    return kBadData;
  } catch (const UsageError& e) {
    std::cerr << "error: " << e.what() << std::endl;
    return kBadUsage;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << std::endl;
    return kRuntimeFailure;
  }
}
