#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <sys/wait.h>
#include <unistd.h>

#include <glim_localization/map/map_io.hpp>
#include <glim_localization/map/pcd_map.hpp>
#include <glim_localization/replay/replay_dataset.hpp>

#include "synthetic/synthetic_map.hpp"
#include "synthetic/synthetic_replay.hpp"
#include "synthetic/synthetic_sensor.hpp"

namespace fs = std::filesystem;
using namespace glim_localization;
using namespace glim_localization::test;

namespace {

std::string quote(const fs::path& path) {
  return "'" + path.string() + "'";
}

std::string init_pose_args() {
  const Eigen::Isometry3d T = synthetic_T_world_imu(0.0);
  const Eigen::Quaterniond q(T.linear());
  char text[256];
  std::snprintf(
    text, sizeof(text), "--init-pose %.17g %.17g %.17g %.17g %.17g %.17g %.17g", T.translation().x(), T.translation().y(), T.translation().z(), q.x(), q.y(),
    q.z(), q.w());
  return text;
}

std::vector<std::string> read_lines(const fs::path& path) {
  std::ifstream ifs(path);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(ifs, line)) {
    lines.push_back(line);
  }
  return lines;
}

}  // namespace

class ReplayToolTest : public testing::Test {
protected:
  static void SetUpTestSuite() {
    root_ = new fs::path(fs::temp_directory_path() / ("glim_loc_replay_tool_" + std::to_string(::getpid())));
    fs::remove_all(*root_);
    const auto map_params = replay_map_params();
    write_synthetic_dump((*root_ / "map").string(), map_params);
    write_synthetic_replay((*root_ / "data").string(), 3.0, map_params);
  }
  static void TearDownTestSuite() {
    fs::remove_all(*root_);
    delete root_;
  }

  void SetUp() override { out_ = *root_ / ("out_" + std::string(testing::UnitTest::GetInstance()->current_test_info()->name())); }

  /// Runs the tool and returns its exit code.
  int run(const std::string& extra, const fs::path& data = fs::path(), const fs::path& config = fs::path(GLIM_LOCALIZATION_CONFIG_DIR),
          const fs::path& map = fs::path()) const {
    const std::string command = std::string(quote(REPLAY_TOOL_PATH)) + " --data " + quote(data.empty() ? *root_ / "data" : data) + " --config " + quote(config) +
                                " --map " + quote(map.empty() ? *root_ / "map" : map) + " --out " + quote(out_) + " " + extra + " > " +
                                quote(out_.string() + ".log") + " 2>&1";
    const int status = std::system(command.c_str());
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  }

  /// A copy of the dataset that a test may damage.
  fs::path copy_data(const std::string& name) const {
    const fs::path copy = *root_ / name;
    fs::remove_all(copy);
    fs::copy(*root_ / "data", copy, fs::copy_options::recursive);
    return copy;
  }

  /// A copy of the dataset whose IMU rows are filtered: keep(row_index, stamp_ns)
  fs::path copy_data_with_imu(const std::string& name, const std::function<bool(size_t, int64_t)>& keep) const {
    const fs::path copy = copy_data(name);
    std::ifstream ifs(copy / "imu.csv");
    std::string header, line;
    std::getline(ifs, header);
    std::vector<std::string> rows;
    for (size_t i = 0; std::getline(ifs, line); i++) {
      if (keep(i, std::stoll(line.substr(0, line.find(','))))) {
        rows.push_back(line);
      }
    }
    ifs.close();
    {
      std::ofstream ofs(copy / "imu.csv", std::ios::trunc);
      ofs << header << "\n";
      for (const auto& row : rows) {
        ofs << row << "\n";
      }
    }
    auto meta = nlohmann::json::parse(std::ifstream(copy / "meta.json"));
    meta["num_imu"] = rows.size();
    std::ofstream(copy / "meta.json", std::ios::trunc) << meta.dump();
    return copy;
  }

  static fs::path* root_;
  fs::path out_;
};

fs::path* ReplayToolTest::root_ = nullptr;

TEST_F(ReplayToolTest, RunsAndWritesOutputs) {
  ASSERT_EQ(run(init_pose_args()), 0);

  const auto frames = read_lines(out_ / "frames.csv");
  ASSERT_EQ(frames.size(), 1u + 31u);
  EXPECT_EQ(frames[0].substr(0, 40), "stamp_ns,result,frame_id,state,reset_cou");
  EXPECT_EQ(frames[1].substr(0, 20), "0,estimated,0,INITIA");  // TRACKING needs several frames
  const auto traj = read_lines(out_ / "traj_localization.txt");
  ASSERT_EQ(traj.size(), 31u);
  EXPECT_EQ(traj[0].substr(0, 12), "0.000000000 ");
  EXPECT_EQ(traj[10].substr(0, 12), "1.000000000 ");

  const auto summary = nlohmann::json::parse(std::ifstream(out_ / "summary.json"));
  EXPECT_EQ(summary.at("frames_by_result").at("estimated"), 31);
  EXPECT_EQ(summary.at("num_poses"), 31);
  EXPECT_TRUE(summary.contains("map_load_seconds"));

  const auto args = nlohmann::json::parse(std::ifstream(out_ / "replay_args.json"));
  EXPECT_EQ(args.at("effective").at("init_pose_source"), "cli");
  EXPECT_EQ(args.at("effective").at("init_T_world_imu_xyz_qxyzw").size(), 7u);
}

TEST_F(ReplayToolTest, InitStampDropsEarlierFrames) {
  ASSERT_EQ(run(init_pose_args() + " --init-stamp 2.0"), 0);
  const auto frames = read_lines(out_ / "frames.csv");
  ASSERT_EQ(frames.size(), 32u);
  for (size_t i = 1; i < frames.size(); i++) {
    const bool dropped = i - 1 < 20;
    EXPECT_NE(frames[i].find(dropped ? ",dropped," : ",estimated,"), std::string::npos) << frames[i];
  }
  const auto traj = read_lines(out_ / "traj_localization.txt");
  ASSERT_EQ(traj.size(), 11u);
  EXPECT_EQ(traj[0].substr(0, 12), "2.000000000 ");
  const auto summary = nlohmann::json::parse(std::ifstream(out_ / "summary.json"));
  EXPECT_EQ(summary.at("frames_by_result").at("dropped"), 20);
}

TEST_F(ReplayToolTest, InitStampAfterTheEndExitsWithNoPoses) {
  EXPECT_EQ(run(init_pose_args() + " --init-stamp 100"), 4);
  EXPECT_TRUE(fs::exists(out_ / "summary.json"));  // A complete run, only without poses
  EXPECT_EQ(read_lines(out_ / "traj_localization.txt").size(), 0u);
}

TEST_F(ReplayToolTest, SelectsFrames) {
  ASSERT_EQ(run(init_pose_args() + " --start 0 --end 1.0 --max-frames 5"), 0);
  EXPECT_EQ(read_lines(out_ / "frames.csv").size(), 6u);
}

TEST_F(ReplayToolTest, FailsWithoutInitialPose) {
  EXPECT_EQ(run(""), 2);
  EXPECT_FALSE(fs::exists(out_ / "summary.json"));
}

TEST_F(ReplayToolTest, RejectsBadArguments) {
  EXPECT_EQ(run(init_pose_args() + " --bogus"), 2);
  EXPECT_EQ(run("--init-pose 1 2 3"), 2);
  EXPECT_EQ(run("--init-pose 0 0 0 0 0 0 0"), 2);
  EXPECT_EQ(run("--init-pose 0 0 0 0 0 0 nan"), 2);
  EXPECT_EQ(run(init_pose_args() + " --max-frames -1"), 2);
  EXPECT_EQ(run(init_pose_args() + " --start abc"), 2);
  EXPECT_EQ(run(init_pose_args() + " --init-stamp"), 2);
  EXPECT_NE(std::system((quote(REPLAY_TOOL_PATH) + " --data x > /dev/null 2>&1").c_str()), 0);  // Missing options
}

TEST_F(ReplayToolTest, RejectsBadConfigAndMap) {
  EXPECT_EQ(run(init_pose_args(), fs::path(), *root_ / "no_such_config"), 2);
  EXPECT_EQ(run(init_pose_args(), fs::path(), fs::path(), *root_ / "no_such_map"), 2);

  const fs::path config = *root_ / "partial_config";
  fs::remove_all(config);
  fs::copy(GLIM_LOCALIZATION_CONFIG_DIR, config, fs::copy_options::recursive);
  fs::remove(config / "config_odometry_localization_cpu.json");
  EXPECT_EQ(run(init_pose_args(), fs::path(), config), 2);
}

TEST_F(ReplayToolTest, ConfiguredVelocityNeedsAPose) {
  const fs::path config = *root_ / "config_velocity_only";
  fs::remove_all(config);
  fs::copy(GLIM_LOCALIZATION_CONFIG_DIR, config, fs::copy_options::recursive);
  {
    const auto path = config / "config_odometry_localization_cpu.json";
    std::ifstream ifs(path);
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    std::string text = buffer.str();
    const std::string key = "\"on_imu_gap\": \"reset\",";
    const auto pos = text.find(key);
    ASSERT_NE(pos, std::string::npos);
    text.insert(pos + key.size(), " \"init_v_world_imu\": [0.0, 0.0, 0.0],");
    std::ofstream(path, std::ios::trunc) << text;
  }
  // Upstream would start at the identity pose in the map: rejected as a bad config
  EXPECT_EQ(run("--max-frames 5", fs::path(), config), 2);
  // With the pose from the command line, the configured velocity goes with it
  EXPECT_EQ(run(init_pose_args() + " --max-frames 5", fs::path(), config), 0);
}

TEST_F(ReplayToolTest, RejectsInvalidDataWithExitCode3) {
  EXPECT_EQ(run(init_pose_args(), *root_ / "no_such_data"), 3);

  auto data = copy_data("data_no_meta");
  fs::remove(data / "meta.json");
  EXPECT_EQ(run(init_pose_args(), data), 3);

  data = copy_data("data_bad_imu");
  {
    std::ofstream ofs(data / "imu.csv", std::ios::app);
    ofs << "5,0,0,0,0,0,0\n";  // Not increasing
  }
  EXPECT_EQ(run(init_pose_args(), data), 3);

  data = copy_data("data_bad_cloud");
  {
    const auto bin = data / "points" / "0000000000500000000.bin";
    ASSERT_TRUE(fs::exists(bin));
    fs::resize_file(bin, fs::file_size(bin) - 4);
  }
  EXPECT_EQ(run(init_pose_args(), data), 3);
  EXPECT_FALSE(fs::exists(out_ / "frames.csv"));  // Nothing ran

  data = copy_data("data_ply");
  std::ofstream(data / "points" / "cloud.ply") << "ply\n";
  EXPECT_EQ(run(init_pose_args(), data), 3);
}

TEST_F(ReplayToolTest, RejectsSlowImuBeforeLoadingTheMap) {
  const fs::path no_map = *root_ / "no_such_map";
  // 5 Hz and 10 Hz: exit 3 even though the map does not exist, so the check comes first
  EXPECT_EQ(run(init_pose_args(), copy_data_with_imu("imu5", [](size_t i, int64_t) { return i % 20 == 0; }), fs::path(GLIM_LOCALIZATION_CONFIG_DIR), no_map), 3);
  const auto imu10 = copy_data_with_imu("imu10", [](size_t i, int64_t) { return i % 10 == 0; });
  EXPECT_EQ(run(init_pose_args(), imu10, fs::path(GLIM_LOCALIZATION_CONFIG_DIR), no_map), 3);
  EXPECT_FALSE(fs::exists(out_ / "frames.csv"));
  // A user who raises the limit gets past the check (and then fails on the missing map)
  EXPECT_EQ(run(init_pose_args() + " --max-imu-gap 0.2", imu10, fs::path(GLIM_LOCALIZATION_CONFIG_DIR), no_map), 2);
  EXPECT_EQ(run(init_pose_args() + " --max-imu-gap 0", imu10), 2);
  // 25 Hz is accepted
  EXPECT_EQ(
    run(init_pose_args() + " --end 1.0", copy_data_with_imu("imu25", [](size_t i, int64_t) { return i % 4 == 0; })), 0);
  const auto args = nlohmann::json::parse(std::ifstream(out_ / "replay_args.json"));
  EXPECT_DOUBLE_EQ(args.at("max_imu_gap").get<double>(), 0.05);
}

TEST_F(ReplayToolTest, ImuDropoutExitsWithCode3AndNoSummary) {
  // No IMU strictly between 1.0 s and 2.0 s of the 3 s dataset
  const auto data = copy_data_with_imu("imu_dropout", [](size_t, int64_t stamp) { return stamp <= 1000000000 || stamp >= 2000000000; });
  EXPECT_EQ(run(init_pose_args(), data), 3);
  EXPECT_FALSE(fs::exists(out_ / "summary.json"));
  EXPECT_GT(read_lines(out_ / "frames.csv").size(), 1u);  // The frames before the gap are on disk
}

TEST_F(ReplayToolTest, RemovesAnOldSummaryEvenWhenTheDatasetIsRejected) {
  fs::create_directories(out_);
  std::ofstream(out_ / "summary.json") << "{}\n";
  ASSERT_TRUE(fs::exists(out_ / "summary.json"));
  EXPECT_EQ(run(init_pose_args(), *root_ / "no_such_data"), 3);
  EXPECT_FALSE(fs::exists(out_ / "summary.json"));

  std::ofstream(out_ / "summary.json") << "{}\n";
  EXPECT_EQ(run(""), 2);  // No initial pose
  EXPECT_FALSE(fs::exists(out_ / "summary.json"));
}

TEST_F(ReplayToolTest, NonFiniteBodyFailsAfterPartialOutput) {
  auto data = copy_data("data_nan_body");
  const auto bin = data / "points" / "0000000001000000000.bin";
  {
    const float nan = std::nanf("");
    std::fstream file(bin, std::ios::in | std::ios::out | std::ios::binary);
    file.seekp(kReplayCloudHeaderSize + 7 * kReplayPointStep);
    file.write(reinterpret_cast<const char*>(&nan), sizeof(nan));
  }
  EXPECT_EQ(run(init_pose_args(), data), 3);
  EXPECT_EQ(read_lines(out_ / "frames.csv").size(), 11u);  // The header and the 10 frames before it
  EXPECT_FALSE(fs::exists(out_ / "summary.json"));         // An aborted run has no summary
}

// ---- IMU gap policy and quality gate ----

namespace {

/// init_pose_args() moved 200 m up: far outside the map, so the map factors find no inliers
std::string far_init_pose_args() {
  const Eigen::Isometry3d T = synthetic_T_world_imu(0.0);
  const Eigen::Quaterniond q(T.linear());
  char text[256];
  std::snprintf(
    text, sizeof(text), "--init-pose %.17g %.17g %.17g %.17g %.17g %.17g %.17g", T.translation().x(), T.translation().y(), T.translation().z() + 200.0, q.x(),
    q.y(), q.z(), q.w());
  return text;
}

/// The ground truth start pose turned by yaw_deg about world z
std::string yawed_init_pose_args(double yaw_deg) {
  Eigen::Isometry3d T = synthetic_T_world_imu(0.0);
  T.linear() = Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix() * T.linear();
  const Eigen::Quaterniond q(T.linear());
  char text[256];
  std::snprintf(
    text, sizeof(text), "--init-pose %.17g %.17g %.17g %.17g %.17g %.17g %.17g", T.translation().x(), T.translation().y(), T.translation().z(), q.x(), q.y(),
    q.z(), q.w());
  return text;
}

}  // namespace

TEST_F(ReplayToolTest, OnImuGapResetRecoversFromAGapJustUnderTheLimit) {
  // 0.49 s without IMU in the 3 s dataset
  const auto data = copy_data_with_imu("gap_under", [](size_t, int64_t stamp) { return stamp <= 1000000000 || stamp >= 1490000000; });
  EXPECT_EQ(run(init_pose_args(), data), 3);  // The default is abort
  EXPECT_FALSE(fs::exists(out_ / "summary.json"));
  EXPECT_EQ(run(init_pose_args() + " --on-imu-gap abort", data), 3);

  ASSERT_EQ(run(init_pose_args() + " --on-imu-gap reset --min-tracking-ratio 0", data), 0);
  const auto summary = nlohmann::json::parse(std::ifstream(out_ / "summary.json"));
  EXPECT_EQ(summary.at("imu_gap_resets"), 1);
  EXPECT_GE(summary.at("frames_by_result").at("skipped_no_imu").get<int>(), 3);
  const auto args = nlohmann::json::parse(std::ifstream(out_ / "replay_args.json"));
  EXPECT_EQ(args.at("on_imu_gap"), "reset");
  const auto frames = read_lines(out_ / "frames.csv");
  EXPECT_NE(frames.back().find("TRACKING"), std::string::npos) << frames.back();
}

TEST_F(ReplayToolTest, OnImuGapResetStillExitsWithCode3ForAGapJustOverTheLimit) {
  const auto data = copy_data_with_imu("gap_over", [](size_t, int64_t stamp) { return stamp <= 1000000000 || stamp >= 1510000000; });
  EXPECT_EQ(run(init_pose_args() + " --on-imu-gap reset", data), 3);
  EXPECT_FALSE(fs::exists(out_ / "summary.json"));
}

TEST_F(ReplayToolTest, OnImuGapResetNeedsTheOdometryToResetItself) {
  const fs::path config = *root_ / "config_gap_off";
  fs::remove_all(config);
  fs::copy(GLIM_LOCALIZATION_CONFIG_DIR, config, fs::copy_options::recursive);
  {
    const auto path = config / "config_odometry_localization_cpu.json";
    std::ifstream ifs(path);
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    std::string text = buffer.str();
    const std::string key = "\"on_imu_gap\": \"reset\"";
    const auto pos = text.find(key);
    ASSERT_NE(pos, std::string::npos);
    text.replace(pos, key.size(), "\"on_imu_gap\": \"off\"");
    std::ofstream(path, std::ios::trunc) << text;
  }
  const auto data = copy_data_with_imu("gap_off", [](size_t, int64_t stamp) { return stamp <= 1000000000 || stamp >= 1200000000; });
  EXPECT_EQ(run(init_pose_args() + " --on-imu-gap reset", data, config), 2);
  {
    std::ifstream log(out_.string() + ".log");
    std::stringstream buffer;
    buffer << log.rdbuf();
    EXPECT_NE(buffer.str().find("on_imu_gap"), std::string::npos) << buffer.str();
  }
  EXPECT_EQ(run(init_pose_args() + " --on-imu-gap abort", data, config), 3);  // Replay's default still protects the run
}

TEST_F(ReplayToolTest, RejectsBadOnImuGapAndTrackingRatio) {
  EXPECT_EQ(run(init_pose_args() + " --on-imu-gap skip"), 2);
  EXPECT_EQ(run(init_pose_args() + " --on-imu-gap"), 2);
  EXPECT_EQ(run(init_pose_args() + " --min-tracking-ratio 1.5"), 2);
  EXPECT_EQ(run(init_pose_args() + " --min-tracking-ratio -0.1"), 2);
}

TEST_F(ReplayToolTest, WritesTheTrackingRatioToTheSummary) {
  ASSERT_EQ(run(init_pose_args()), 0);
  const auto summary = nlohmann::json::parse(std::ifstream(out_ / "summary.json"));
  EXPECT_DOUBLE_EQ(summary.at("tracking_ratio").get<double>(), 1.0);
  EXPECT_DOUBLE_EQ(summary.at("min_tracking_ratio").get<double>(), 0.9);  // The default gate
  EXPECT_EQ(summary.at("imu_gap_resets"), 0);
}

TEST_F(ReplayToolTest, TrackingRatioGateAtAndBelowTheThreshold) {
  // A reset after a gap sends the state back to INITIALIZING, which lowers the ratio below 1
  const auto data = copy_data_with_imu("gap_ratio", [](size_t, int64_t stamp) { return stamp <= 1500000000 || stamp >= 1790000000; });
  const std::string reset = init_pose_args() + " --on-imu-gap reset";
  ASSERT_EQ(run(reset + " --min-tracking-ratio 0", data), 0);
  const double ratio = nlohmann::json::parse(std::ifstream(out_ / "summary.json")).at("tracking_ratio").get<double>();
  ASSERT_GT(ratio, 0.0);
  ASSERT_LT(ratio, 1.0);

  char text[64];
  std::snprintf(text, sizeof(text), " --min-tracking-ratio %.17g", ratio);
  EXPECT_EQ(run(reset + text, data), 0);  // At the threshold
  std::snprintf(text, sizeof(text), " --min-tracking-ratio %.17g", std::min(1.0, ratio + 1e-6));
  EXPECT_EQ(run(reset + text, data), 1);  // Just below it
  const auto summary = nlohmann::json::parse(std::ifstream(out_ / "summary.json"));  // The run was complete: the summary is there
  EXPECT_NEAR(summary.at("tracking_ratio").get<double>(), ratio, 1e-12);
  EXPECT_NEAR(summary.at("min_tracking_ratio").get<double>(), ratio + 1e-6, 1e-12);
  EXPECT_NE(read_lines(out_ / "traj_localization.txt").size(), 0u);
}

TEST_F(ReplayToolTest, LeavingTheMapNeverTracksAndFailsTheRatioGateUnlessItIsDisabled) {
  EXPECT_EQ(run(far_init_pose_args()), 1);                              // Never TRACKING: ratio 0 < 0.9
  EXPECT_TRUE(fs::exists(out_ / "summary.json"));
  EXPECT_EQ(run(far_init_pose_args() + " --min-tracking-ratio 0"), 0);  // The gate is off, and no frame is LOST yet
}

TEST_F(ReplayToolTest, AbortsWithExitCode1OnALostFrame) {
  const fs::path config = *root_ / "config_lost3";
  fs::remove_all(config);
  fs::copy(GLIM_LOCALIZATION_CONFIG_DIR, config, fs::copy_options::recursive);
  {
    const auto path = config / "config_odometry_localization_cpu.json";
    std::ifstream ifs(path);
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    std::string text = buffer.str();
    const auto pos = text.find("\"lost_frames\": 50");
    ASSERT_NE(pos, std::string::npos);
    text.replace(pos, std::string("\"lost_frames\": 50").size(), "\"lost_frames\": 3");
    std::ofstream(path, std::ios::trunc) << text;
  }
  EXPECT_EQ(run(far_init_pose_args() + " --min-tracking-ratio 0", fs::path(), config), 1);  // The ratio gate is off: only LOST can fail this
  EXPECT_FALSE(fs::exists(out_ / "summary.json"));                                          // An aborted run has no summary
  std::ifstream log(out_.string() + ".log");
  std::stringstream buffer;
  buffer << log.rdbuf();
  EXPECT_NE(buffer.str().find("LOST"), std::string::npos) << buffer.str();
  const auto frames = read_lines(out_ / "frames.csv");
  EXPECT_LT(frames.size(), 32u);  // It stopped at the first LOST frame
  EXPECT_NE(frames.back().find("LOST"), std::string::npos) << frames.back();
}

TEST_F(ReplayToolTest, LocalizesOnAMapBuiltFromPcd) {
  // The corridor as a PCD, through the same path as `map_compiler <pcd>`: read, tile, estimate covariances, write.
  const fs::path pcd = *root_ / "corridor.pcd";
  const fs::path map = *root_ / "pcd_map";
  fs::remove_all(map);
  write_synthetic_pcd(pcd.string(), make_synthetic_world_points(replay_map_params()), PcdEncoding::Binary);
  PcdPoints points;
  ASSERT_TRUE(read_pcd(pcd.string(), points));
  LoadedSubmaps submaps;
  ASSERT_TRUE(build_pcd_submaps(points, PcdMapParams(), submaps));
  ASSERT_TRUE(save_compiled_map(map.string(), submaps, "pcd"));

  // Start 0.3 m and 3 degrees off the ground truth.
  Eigen::Isometry3d T = synthetic_T_world_imu(0.0);
  T.translation() += Eigen::Vector3d(0.3, -0.2, 0.1);
  T.linear() = Eigen::AngleAxisd(3.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix() * T.linear();
  const Eigen::Quaterniond q(T.linear());
  char init[256];
  std::snprintf(
    init, sizeof(init), "--init-pose %.17g %.17g %.17g %.17g %.17g %.17g %.17g", T.translation().x(), T.translation().y(), T.translation().z(), q.x(),
    q.y(), q.z(), q.w());
  ASSERT_EQ(run(init, fs::path(), fs::path(GLIM_LOCALIZATION_CONFIG_DIR), map), 0);

  const auto summary = nlohmann::json::parse(std::ifstream(out_ / "summary.json"));
  EXPECT_EQ(summary.at("frames_by_result").at("estimated"), 31);
  const auto traj = read_lines(out_ / "traj_localization.txt");
  ASSERT_EQ(traj.size(), 31u);
  int checked = 0;
  for (const auto& line : traj) {
    std::istringstream iss(line);
    double t;
    Eigen::Vector3d p;
    Eigen::Quaterniond r;
    iss >> t >> p.x() >> p.y() >> p.z() >> r.x() >> r.y() >> r.z() >> r.w();
    ASSERT_FALSE(iss.fail()) << line;
    if (t < 1.0 - 1e-6) {
      continue;  // Settling from the offset initial pose
    }
    const Eigen::Isometry3d truth = synthetic_T_world_imu(t);
    EXPECT_LT((p - truth.translation()).norm(), 0.1) << t;
    EXPECT_LT(Eigen::AngleAxisd(Eigen::Quaterniond(truth.linear()).inverse() * r.normalized()).angle() * 180.0 / M_PI, 1.0) << t;
    checked++;
  }
  EXPECT_EQ(checked, 21);
}

// A wrong initial yaw: the plain run never tracks, the yaw search corrects it, and a pose off the map is rejected
TEST_F(ReplayToolTest, InitYawSearchCorrectsAWrongInitialYaw) {
  const fs::path config = *root_ / "config_yaw_search";
  fs::remove_all(config);
  fs::copy(GLIM_LOCALIZATION_CONFIG_DIR, config, fs::copy_options::recursive);
  {
    const auto path = config / "config_odometry_localization_cpu.json";
    std::ifstream ifs(path);
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    std::string text = buffer.str();
    const std::string key = "\"init_yaw_search\": false";
    const auto pos = text.find(key);
    ASSERT_NE(pos, std::string::npos);
    text.replace(pos, key.size(), "\"init_yaw_search\": true");
    // A loaded CI machine must not cut the search short (parallel ctest once slowed it to 11 of 24 candidates in 10 s);
    // the ctest TIMEOUT bounds the run instead
    const std::string budget = "\"init_yaw_search_time_budget\": 10.0";
    const auto budget_pos = text.find(budget);
    ASSERT_NE(budget_pos, std::string::npos);
    text.replace(budget_pos, budget.size(), "\"init_yaw_search_time_budget\": 1000000.0");
    std::ofstream(path, std::ios::trunc) << text;
  }
  const std::string wrong = yawed_init_pose_args(75.0);
  // Error of the last pose against the ground truth: translation [m], rotation [deg]
  struct PoseError {
    double translation;
    double rotation;
  };
  const auto last_error = [&]() -> PoseError {
    const auto traj = read_lines(out_ / "traj_localization.txt");
    if (traj.empty()) {
      return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    }
    std::istringstream last(traj.back());
    double stamp, x, y, z, qx, qy, qz, qw;
    last >> stamp >> x >> y >> z >> qx >> qy >> qz >> qw;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() << x, y, z;
    T.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
    const Eigen::Isometry3d error = synthetic_T_world_imu(stamp).inverse() * T;
    return {error.translation().norm(), Eigen::AngleAxisd(error.linear()).angle() * 180.0 / M_PI};
  };

  // The default config does not search: the run fails its gates, or ends far from the ground truth
  const int plain = run(wrong);
  const auto [plain_translation, plain_rotation] = last_error();
  std::cout << "no search: exit " << plain << ", last pose error " << plain_translation << " m, " << plain_rotation << " deg" << std::endl;
  EXPECT_TRUE(plain != 0 || plain_rotation > 10.0 || plain_translation > 1.0);

  ASSERT_EQ(run(wrong, fs::path(), config), 0);
  const auto summary = nlohmann::json::parse(std::ifstream(out_ / "summary.json"));
  EXPECT_DOUBLE_EQ(summary.at("tracking_ratio").get<double>(), 1.0);
  const auto& search = summary.at("init_search");
  EXPECT_EQ(search.at("result"), "ACCEPTED");
  EXPECT_EQ(search.at("reason"), "NONE");
  EXPECT_EQ(search.at("candidates"), 24);
  EXPECT_EQ(search.at("candidates_total"), 24);
  EXPECT_EQ(search.at("truncated"), false);
  EXPECT_GT(search.at("score").get<double>(), 0.9);
  EXPECT_LT(search.at("runner_up_score").get<double>(), search.at("score").get<double>() - 0.05);
  EXPECT_NEAR(search.at("yaw_offset_deg").get<double>(), -75.0, 1.0);
  EXPECT_EQ(read_lines(out_ / "traj_localization.txt").size(), 31u);
  const auto [translation, rotation] = last_error();
  std::cout << "search: last pose error " << translation << " m, " << rotation << " deg, search " << search.at("seconds").get<double>() << " s" << std::endl;
  EXPECT_LT(translation, 0.1);
  EXPECT_LT(rotation, 1.0);

  // Off the map, no candidate passes: the pose is rejected and the run stops at the first LOST frame
  EXPECT_EQ(run(far_init_pose_args() + " --min-tracking-ratio 0", fs::path(), config), 1);
  std::ifstream log(out_.string() + ".log");
  std::stringstream buffer;
  buffer << log.rdbuf();
  EXPECT_NE(buffer.str().find("rejected the given pose"), std::string::npos) << buffer.str();
}
