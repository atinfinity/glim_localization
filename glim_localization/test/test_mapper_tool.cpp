#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <sys/wait.h>
#include <unistd.h>

#include <glim_localization/map/map_io.hpp>

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

struct Pose {
  double stamp;
  Eigen::Vector3d p;
};

std::vector<Pose> read_tum(const fs::path& path) {
  std::ifstream ifs(path);
  std::vector<Pose> poses;
  double stamp, x, y, z, qx, qy, qz, qw;
  while (ifs >> stamp >> x >> y >> z >> qx >> qy >> qz >> qw) {
    poses.push_back({stamp, Eigen::Vector3d(x, y, z)});
  }
  return poses;
}

}  // namespace

class MapperToolTest : public testing::Test {
protected:
  static void SetUpTestSuite() {
    root_ = new fs::path(fs::temp_directory_path() / ("glim_loc_mapper_tool_" + std::to_string(::getpid())));
    fs::remove_all(*root_);
    write_synthetic_replay((*root_ / "data").string(), 12.0, replay_map_params());

    // The synthetic scene keeps a high overlap, so few keyframes are created. Cut submaps early to get several of them.
    fs::copy(MAPPING_CONFIG_DIR, *root_ / "config", fs::copy_options::recursive);
    const fs::path file = *root_ / "config" / "config_sub_mapping_cpu.json";
    std::ifstream ifs(file);
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    ifs.close();
    const std::string text = std::regex_replace(buffer.str(), std::regex(R"("max_num_keyframes"\s*:\s*\d+)"), "\"max_num_keyframes\": 3");
    std::ofstream(file, std::ios::trunc) << text;
  }
  static void TearDownTestSuite() {
    fs::remove_all(*root_);
    delete root_;
  }

  /// Returns the exit code of the mapper
  int run(const std::string& extra, const fs::path& out, const fs::path& config = fs::path(), const fs::path& data = fs::path()) const {
    const std::string command = "timeout 90 " + std::string(quote(MAPPER_TOOL_PATH)) + " --data " + quote(data.empty() ? *root_ / "data" : data) + " --config " + quote(config.empty() ? *root_ / "config" : config) + " --out " +
                                quote(out) + " " + extra + " > " + quote(out.string() + ".log") + " 2>&1";
    fs::create_directories(out.parent_path());
    const int status = std::system(command.c_str());
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  }


  /// A copy of the dataset whose IMU ends `seconds` after its first sample
  fs::path copy_data_with_imu_until(const std::string& name, double seconds) const {
    const fs::path data = *root_ / name;
    fs::remove_all(data);
    fs::copy(*root_ / "data", data, fs::copy_options::recursive);
    std::vector<std::string> rows;
    std::string header, line;
    {
      std::ifstream ifs(data / "imu.csv");
      std::getline(ifs, header);
      int64_t first = -1;
      while (std::getline(ifs, line)) {
        const int64_t stamp = std::stoll(line.substr(0, line.find(',')));
        first = first < 0 ? stamp : first;
        if (stamp - first <= static_cast<int64_t>(seconds * 1e9)) {
          rows.push_back(line);
        }
      }
    }
    {
      std::ofstream ofs(data / "imu.csv", std::ios::trunc);
      ofs << header << "\n";
      for (const auto& row : rows) {
        ofs << row << "\n";
      }
    }
    auto meta = nlohmann::json::parse(std::ifstream(data / "meta.json"));
    meta["num_imu"] = rows.size();
    std::ofstream(data / "meta.json", std::ios::trunc) << meta.dump();
  
    return data;
  }

  static fs::path* root_;
};

fs::path* MapperToolTest::root_ = nullptr;

TEST_F(MapperToolTest, BuildsALoadableMapAndATrajectory) {
  const fs::path out = *root_ / "out";
  ASSERT_EQ(run("", out), 0);

  LoadedSubmaps submaps;
  ASSERT_TRUE(load_map((out / "dump").string(), submaps));
  EXPECT_GE(submaps.size(), 2u);

  const auto summary = nlohmann::json::parse(std::ifstream(out / "mapper_summary.json"));
  EXPECT_GE(summary.at("num_submaps").get<int>(), 2);
  EXPECT_GE(summary.at("trajectory_coverage_of_estimated").get<double>(), 0.9);
  EXPECT_GT(summary.at("peak_rss_gb").get<double>(), 0.0);

  // GLIM's world frame is its own (gravity-aligned at the first IMU sample), so compare the travelled distance only
  const auto traj = read_tum(out / "traj_glim.txt");
  ASSERT_GE(traj.size(), 25u);
  // Poses of different submaps must join up: a wrong composition of the submap poses shows as a jump between frames
  for (size_t i = 1; i < traj.size(); i++) {
    EXPECT_LT((traj[i].p - traj[i - 1].p).norm(), 0.6) << "jump at stamp " << traj[i].stamp;
  }
  const Pose& last = traj.back();
  const Eigen::Vector3d truth = synthetic_T_world_imu(last.stamp).translation() - synthetic_T_world_imu(traj.front().stamp).translation();
  EXPECT_NEAR((last.p - traj.front().p).norm(), truth.norm(), 0.3);
}

// LiDAR frames after the last IMU sample used to block the odometry and the feeder for good (exit 124 from timeout)
TEST_F(MapperToolTest, FailsWhenTooManyFramesHaveNoImu) {
  const fs::path data = copy_data_with_imu_until("data_short_imu", 8.0);
  const fs::path out = *root_ / "short_imu";
  EXPECT_EQ(run("", out, fs::path(), data), 3);  // about a third of the frames are not covered; 124 would be the old hang
  const auto summary = nlohmann::json::parse(std::ifstream(out / "mapper_summary.json"));
  EXPECT_TRUE(summary.at("failed").get<bool>());
  EXPECT_GE(summary.at("frames_skipped_no_imu").get<int>(), 30);
  // Skipped frames count as missing
  EXPECT_NEAR(summary.at("trajectory_coverage").get<double>(), static_cast<double>(summary.at("num_trajectory_poses").get<int>()) / summary.at("frames_selected").get<int>(), 1e-9);
  EXPECT_LT(summary.at("trajectory_coverage").get<double>(), 0.7);
}

TEST_F(MapperToolTest, SkipsAFewFramesBeyondTheImu) {
  const fs::path data = copy_data_with_imu_until("data_slightly_short_imu", 11.7);
  const fs::path out = *root_ / "slightly_short_imu";
  ASSERT_EQ(run("", out, fs::path(), data), 0);
  const auto summary = nlohmann::json::parse(std::ifstream(out / "mapper_summary.json"));
  EXPECT_FALSE(summary.at("failed").get<bool>());
  const int skipped = summary.at("frames_skipped_no_imu").get<int>();
  const int selected = summary.at("frames_selected").get<int>();
  EXPECT_GE(skipped, 1);
  EXPECT_LE(skipped, 0.05 * selected);
  EXPECT_EQ(summary.at("frames_fed").get<int>() + skipped, selected);
  const double expected = static_cast<double>(summary.at("num_trajectory_poses").get<int>()) / selected;
  EXPECT_NEAR(summary.at("trajectory_coverage").get<double>(), expected, 1e-9);
  EXPECT_LT(summary.at("trajectory_coverage").get<double>(), static_cast<double>(summary.at("num_trajectory_poses").get<int>()) / summary.at("frames_fed").get<int>());
}

TEST_F(MapperToolTest, RejectsBadArguments) {
  const fs::path out = *root_ / "bad";
  // A stale summary must not survive a run that fails on its arguments, config or data
  fs::create_directories(out);
  std::ofstream(out / "mapper_summary.json") << "{\"failed\": false}\n";
  EXPECT_EQ(run("--bogus", out), 2);
  EXPECT_FALSE(fs::exists(out / "mapper_summary.json"));
  std::ofstream(out / "mapper_summary.json") << "{\"failed\": false}\n";
  EXPECT_EQ(run("", out, *root_ / "no_config"), 2);
  EXPECT_FALSE(fs::exists(out / "mapper_summary.json"));
  EXPECT_EQ(run("--start nan", out), 2);
  EXPECT_EQ(run("", out, *root_ / "no_config"), 2);
  // A window with no frame creates no submap
  EXPECT_EQ(run("--start 100 --end 101", out), 1);
}
