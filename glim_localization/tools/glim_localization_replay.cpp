// Replays a recorded dataset (see docs/DESIGN.md) through the localization odometry and writes the trajectory.
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/util/config.hpp>

#include <glim_localization/map/map_manager.hpp>
#include <glim_localization/odometry/localization_backend.hpp>
#include <glim_localization/replay/replay_runner.hpp>

namespace fs = std::filesystem;
using namespace glim_localization;

namespace {

enum ExitCode { kOk = 0, kRuntimeFailure = 1, kBadUsage = 2, kBadData = 3, kNoPoses = 4 };

struct Arguments {
  std::string data, config, map, out;
  std::optional<std::vector<double>> init_pose;  // x y z qx qy qz qw
  std::optional<std::vector<double>> init_vel;
  std::optional<double> init_stamp, start, end, max_imu_gap;
  std::optional<long long> max_frames;
  ReplayOptions::OnImuGap on_imu_gap = ReplayOptions::OnImuGap::ABORT;
  double min_tracking_ratio = 0.9;
  std::optional<bool> wheel_odom;  // --wheel-odom on|off; unset keeps the config
};

struct UsageError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

void print_usage() {
  std::cerr << "usage: glim_localization_replay --data <dir> --config <dir> --map <dir> --out <dir>\n"
               "         [--init-pose x y z qx qy qz qw] [--init-vel vx vy vz] [--init-stamp <s>]\n"
               "         [--start <s>] [--end <s>] [--max-frames N] [--max-imu-gap <s>]\n"
               "         [--on-imu-gap abort|reset] [--min-tracking-ratio <r>] [--wheel-odom on|off]\n"
               "\n"
               "  --data        replay dataset directory (meta.json, imu.csv, points/*.bin)\n"
               "  --config      GLIM-style config directory containing config.json\n"
               "  --map         GLIM dump or compiled map directory\n"
               "  --out         output directory (traj_localization.txt, frames.csv, summary.json, replay_args.json)\n"
               "  --init-pose   T_world_imu at the start; required unless the config sets init_T_world_imu\n"
               "  --init-vel    initial world-frame velocity [m/s]\n"
               "  --init-stamp  frames before this stamp [s] are dropped and the pose is applied from here\n"
               "  --start/--end only process frames with start <= stamp <= end [s]\n"
               "  --max-frames  stop after N processed frames\n"
               "  --max-imu-gap largest allowed gap between IMU samples [s] (default 0.05); a median IMU period above it is rejected\n"
               "  --on-imu-gap  abort (default): an IMU gap after the first pose stops the run with exit 3.\n"
               "                reset: skip the frames that span a gap of at most imu_gap_reset_max (config, default 0.5 s) and let the\n"
               "                odometry reset itself from its last pose and velocity; a longer gap still exits 3\n"
               "  --min-tracking-ratio  fail with exit 1 if less than this share of the estimated frames, counted from the first\n"
               "                TRACKING frame, is TRACKING (default 0.9, 0 disables). A LOST frame always stops the run with exit 1\n"
               "  --wheel-odom  on: feed wheel_odom.csv of the dataset (required) to the odometry, overriding wheel_odom.enabled\n"
               "                of the config. off: do not use wheel odometry. Default: the config, fed if the dataset has the file\n"
               "\n"
               "exit codes: 0 ok, 1 runtime failure (also LOST or tracking ratio below the minimum), 2 bad arguments/config/map/initial pose,\n"
               "            3 invalid dataset, 4 finished without any pose\n";
}

double parse_double(const std::string& option, const std::string& text) {
  char* end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (text.empty() || end != text.c_str() + text.size() || !std::isfinite(value)) {
    throw UsageError("invalid number for " + option + ": '" + text + "'");
  }
  return value;
}

Arguments parse_arguments(int argc, char** argv) {
  Arguments args;
  int i = 1;
  const auto next = [&](const std::string& option) -> std::string {
    if (i + 1 >= argc) {
      throw UsageError("missing value for " + option);
    }
    return argv[++i];
  };
  const auto numbers = [&](const std::string& option, int count) {
    std::vector<double> values;
    for (int k = 0; k < count; k++) {
      values.push_back(parse_double(option, next(option)));
    }
    return values;
  };

  for (; i < argc; i++) {
    const std::string option = argv[i];
    if (option == "--data") {
      args.data = next(option);
    } else if (option == "--config") {
      args.config = next(option);
    } else if (option == "--map") {
      args.map = next(option);
    } else if (option == "--out") {
      args.out = next(option);
    } else if (option == "--init-pose") {
      args.init_pose = numbers(option, 7);
    } else if (option == "--init-vel") {
      args.init_vel = numbers(option, 3);
    } else if (option == "--init-stamp") {
      args.init_stamp = parse_double(option, next(option));
    } else if (option == "--start") {
      args.start = parse_double(option, next(option));
    } else if (option == "--end") {
      args.end = parse_double(option, next(option));
    } else if (option == "--max-imu-gap") {
      const double value = parse_double(option, next(option));
      if (!(value > 0.0)) {
        throw UsageError("--max-imu-gap must be positive");
      }
      args.max_imu_gap = value;
    } else if (option == "--on-imu-gap") {
      const std::string value = next(option);
      if (value == "abort") {
        args.on_imu_gap = ReplayOptions::OnImuGap::ABORT;
      } else if (value == "reset") {
        args.on_imu_gap = ReplayOptions::OnImuGap::RESET;
      } else {
        throw UsageError("--on-imu-gap must be abort or reset, got '" + value + "'");
      }
    } else if (option == "--wheel-odom") {
      const std::string value = next(option);
      if (value == "on") {
        args.wheel_odom = true;
      } else if (value == "off") {
        args.wheel_odom = false;
      } else {
        throw UsageError("--wheel-odom must be on or off, got '" + value + "'");
      }
    } else if (option == "--min-tracking-ratio") {
      args.min_tracking_ratio = parse_double(option, next(option));
      if (args.min_tracking_ratio < 0.0 || args.min_tracking_ratio > 1.0) {
        throw UsageError("--min-tracking-ratio must be in [0, 1]");
      }
    } else if (option == "--max-frames") {
      const double value = parse_double(option, next(option));
      if (value < 0.0 || value != std::floor(value)) {
        throw UsageError("--max-frames must be a non-negative integer");
      }
      args.max_frames = static_cast<long long>(value);
    } else {
      throw UsageError("unknown option " + option);
    }
  }
  for (const auto& [name, value] : {std::pair{"--data", args.data}, {"--config", args.config}, {"--map", args.map}, {"--out", args.out}}) {
    if (value.empty()) {
      throw UsageError(std::string("missing required option ") + name);
    }
  }
  return args;
}

/// Check the config files here: upstream only logs an error for a missing file and goes on with defaults.
void check_config_dir(const std::string& dir) {
  const fs::path root(dir);
  std::ifstream ifs(root / "config.json");
  if (!ifs) {
    throw UsageError("cannot open " + (root / "config.json").string());
  }
  nlohmann::json json;
  try {
    json = nlohmann::json::parse(ifs, nullptr, true, true);
  } catch (const std::exception& e) {
    throw UsageError("invalid " + (root / "config.json").string() + ": " + e.what());
  }
  for (const char* key : {"config_sensors", "config_preprocess", "config_odometry"}) {
    const bool named = json.contains("global") && json["global"].contains(key) && json["global"][key].is_string();
    if (!named || !fs::is_regular_file(root / json["global"][key].get<std::string>())) {
      throw UsageError(std::string("config.json: ") + key + " does not name an existing file in " + dir);
    }
  }
}

/// An IMU that is too slow for the estimator is a property of the data, so reject it before the map is read.
void check_imu_rate(const ReplayDataset& dataset, double max_imu_gap) {
  const auto& imu = dataset.imu();
  if (imu.size() < 2) {
    throw InvalidDatasetError("the dataset needs at least 2 IMU samples");
  }
  std::vector<int64_t> periods;
  periods.reserve(imu.size() - 1);
  for (size_t k = 0; k + 1 < imu.size(); k++) {
    periods.push_back(imu[k + 1].stamp_ns - imu[k].stamp_ns);
  }
  std::nth_element(periods.begin(), periods.begin() + periods.size() / 2, periods.end());
  const double median = periods[periods.size() / 2] * 1e-9;
  if (median > max_imu_gap) {
    char text[200];
    std::snprintf(
      text, sizeof(text), "the median IMU period is %.4f s (%.1f Hz), above the limit of %.4f s; the estimator needs a faster IMU (see --max-imu-gap)", median,
      1.0 / median, max_imu_gap);
    throw InvalidDatasetError(text);
  }
}

std::vector<double> pose_vector(const Eigen::Isometry3d& T) {
  const Eigen::Quaterniond q(T.linear());
  return {T.translation().x(), T.translation().y(), T.translation().z(), q.x(), q.y(), q.z(), q.w()};
}

}  // namespace

int main(int argc, char** argv) {
  Arguments args;
  try {
    args = parse_arguments(argc, argv);
  } catch (const UsageError& e) {
    std::cerr << "error: " << e.what() << "\n\n";
    print_usage();
    return kBadUsage;
  }

  // A summary.json from an earlier run must never survive a run that does not complete,
  // including one that is rejected below. evaluate.py trusts its presence.
  {
    std::error_code ec;
    fs::remove(fs::path(args.out) / "summary.json", ec);
  }

  try {
    // Configuration and initial pose
    check_config_dir(args.config);
    glim::GlobalConfig::instance(args.config, true);
    // The backend (CPU or GPU) follows the so_name of the odometry config, as it does for GLIM.
    LocalizationBackend backend = LocalizationBackend::CPU;
    std::unique_ptr<OdometryEstimationLocalizationParams> params_ptr;
    try {
      backend = localization_backend_from_config();
      params_ptr = make_localization_params(backend);
    } catch (const std::runtime_error& e) {
      throw UsageError(e.what());
    }
    auto& params = *params_ptr;
    params.map_path = args.map;
    std::string pose_source = "config";
    if (args.init_pose) {
      const auto& v = *args.init_pose;
      const Eigen::Quaterniond q(v[6], v[3], v[4], v[5]);
      if (q.norm() < 1e-6) {
        throw UsageError("--init-pose quaternion has zero norm");
      }
      params.init_T_world_imu = Eigen::Isometry3d::Identity();
      params.init_T_world_imu.linear() = q.normalized().toRotationMatrix();
      params.init_T_world_imu.translation() = Eigen::Vector3d(v[0], v[1], v[2]);
      params.estimate_init_state = false;
      params.init_v_without_pose = false;  // The pose comes from the command line, a configured velocity may go with it
      pose_source = "cli";
    }
    if (args.init_vel) {
      params.init_v_world_imu = Eigen::Vector3d((*args.init_vel)[0], (*args.init_vel)[1], (*args.init_vel)[2]);
    }
    if (params.estimate_init_state) {
      throw UsageError(
        "no initial pose: pass --init-pose or set odometry_estimation/init_T_world_imu in the config "
        "(a localization needs a pose in the map frame)");
    }

    // Dataset: validated before the map is read
    const auto dataset = ReplayDataset::open(args.data);
    ReplayOptions options;
    options.max_imu_gap = args.max_imu_gap.value_or(params.max_imu_gap);
    params.max_imu_gap = options.max_imu_gap;  // The odometry applies the same gap rule as the runner
    if (args.on_imu_gap == ReplayOptions::OnImuGap::RESET && params.on_imu_gap != "reset") {
      throw UsageError("--on-imu-gap reset needs the odometry to reset itself, but the config sets on_imu_gap \"" + params.on_imu_gap + "\"");
    }
    options.on_imu_gap = args.on_imu_gap;
    options.imu_gap_reset_max = params.imu_gap_reset_max;
    check_imu_rate(dataset, options.max_imu_gap);
    if (args.wheel_odom) {
      if (*args.wheel_odom && (!dataset.has_wheel_odom() || dataset.wheel_odom().empty())) {
        throw UsageError("--wheel-odom on needs wheel_odom.csv with samples in " + args.data);
      }
      params.wheel_odom.enabled = *args.wheel_odom;
    }
    // Fed only if both are there (ReplayRunner)
    const bool wheel_odom_used = params.wheel_odom.enabled && dataset.has_wheel_odom() && !dataset.wheel_odom().empty();
    if (params.wheel_odom.enabled && !wheel_odom_used) {
      std::cerr << "warning: wheel odometry is enabled, but the dataset has no wheel_odom.csv samples; running without it\n";
    }

    const auto map_begin = std::chrono::steady_clock::now();
    auto map = std::make_shared<MapManager>();
    if (!map->load(args.map)) {
      throw UsageError("cannot load the map " + args.map);
    }
    const double map_load_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - map_begin).count();

    glim::CloudPreprocessor preprocessor;
    const auto odometry_ptr = make_localization_odometry(params, map);
    auto& odometry = *odometry_ptr;

    options.start = args.start;
    options.end = args.end;
    options.max_frames = args.max_frames.value_or(-1);
    if (args.init_stamp) {
      ReplayOptions::InitialReset reset;
      reset.T_world_imu = params.init_T_world_imu;
      reset.v_world_imu = params.init_v_world_imu;
      reset.min_stamp = *args.init_stamp;
      options.initial_reset = reset;
    }

    ReplayOutputWriter writer(args.out);
    {
      nlohmann::json json;
      json["argv"] = std::vector<std::string>(argv, argv + argc);
      json["data"] = args.data;
      json["config"] = args.config;
      json["map"] = args.map;
      json["out"] = args.out;
      json["init_stamp"] = args.init_stamp ? nlohmann::json(*args.init_stamp) : nlohmann::json(nullptr);
      json["start"] = args.start ? nlohmann::json(*args.start) : nlohmann::json(nullptr);
      json["end"] = args.end ? nlohmann::json(*args.end) : nlohmann::json(nullptr);
      json["max_imu_gap"] = options.max_imu_gap;
      json["on_imu_gap"] = args.on_imu_gap == ReplayOptions::OnImuGap::RESET ? "reset" : "abort";
      json["imu_gap_reset_max"] = options.imu_gap_reset_max;
      json["min_tracking_ratio"] = args.min_tracking_ratio;
      json["max_frames"] = args.max_frames ? nlohmann::json(*args.max_frames) : nlohmann::json(nullptr);
      json["effective"] = {
        {"backend", to_string(backend)},
        {"init_pose_source", pose_source},
        {"init_T_world_imu_xyz_qxyzw", pose_vector(params.init_T_world_imu)},
        {"init_v_world_imu", std::vector<double>(params.init_v_world_imu.data(), params.init_v_world_imu.data() + 3)},
        {"use_keyframes", params.use_keyframes},
        {"map_search_radius", params.map_factors.search_radius},
        {"map_max_submaps", params.map_factors.max_submaps},
        {"map_factor_weight", params.map_factor_weight},
        {"num_threads", params.num_threads},
        {"wheel_odom", wheel_odom_used},
      };
      std::ofstream ofs(fs::path(args.out) / "replay_args.json", std::ios::trunc);
      if (!(ofs << json.dump(2) << "\n")) {
        throw std::runtime_error("cannot write replay_args.json in " + args.out);
      }
    }

    ReplayRunner runner(dataset, odometry, preprocessor, options);
    TrackingRatio tracking;
    auto summary = runner.run([&](const ReplayFrameRecord& record) {
      writer.write(record);
      tracking.add(record);
      if (record.state == LocalizationState::LOST) {
        throw std::runtime_error("the localization is LOST at frame " + format_stamp_ns(record.stamp_ns) + " s (see the log); replay stopped");
      }
    });
    summary.tracking_ratio = tracking.ratio();
    summary.min_tracking_ratio = args.min_tracking_ratio;
    writer.write_summary(summary, map_load_seconds);

    std::cout << "frames: " << summary.num_frames;
    for (const auto& [result, count] : summary.frames_by_result) {
      std::cout << ", " << result << ": " << count;
    }
    std::cout << "\nposes written: " << writer.num_poses() << " (" << args.out << "/traj_localization.txt)\n"
              << "wall time: " << summary.wall_seconds << " s (map load " << map_load_seconds << " s)" << std::endl;
    if (writer.num_poses() == 0) {
      std::cerr << "error: finished without any pose" << std::endl;
      return kNoPoses;
    }
    if (!tracking.passes(args.min_tracking_ratio)) {
      std::cerr << "error: tracking ratio " << tracking.ratio() << " is below --min-tracking-ratio " << args.min_tracking_ratio << std::endl;
      return kRuntimeFailure;
    }
    return kOk;
  } catch (const UsageError& e) {
    std::cerr << "error: " << e.what() << std::endl;
    return kBadUsage;
  } catch (const std::invalid_argument& e) {  // Rejected parameters or initial state
    std::cerr << "error: " << e.what() << std::endl;
    return kBadUsage;
  } catch (const InvalidDatasetError& e) {
    std::cerr << "error: invalid dataset: " << e.what() << std::endl;
    return kBadData;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << std::endl;
    return kRuntimeFailure;
  }
}
