#include <filesystem>
#include <fstream>
#include <sstream>

#include <gtest/gtest.h>
#include <unistd.h>

#include <glim_localization/replay/replay_dataset.hpp>

#include "synthetic/synthetic_replay.hpp"
#include "synthetic/synthetic_sensor.hpp"

namespace fs = std::filesystem;
using namespace glim_localization;
using namespace glim_localization::test;

TEST(SyntheticReplayTest, DatasetIsReadableAndConsistent) {
  const fs::path dir = fs::temp_directory_path() / ("glim_loc_synthetic_replay_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  const int frames = write_synthetic_replay(dir.string(), 2.0);

  const auto dataset = ReplayDataset::open(dir.string());
  EXPECT_EQ(frames, 21);
  ASSERT_EQ(dataset.frames().size(), 21u);
  ASSERT_EQ(dataset.imu().size(), 206u);  // 0 s to 2.05 s at 100 Hz
  EXPECT_TRUE(dataset.has_ground_truth());
  EXPECT_FALSE(dataset.meta().per_point_time);
  EXPECT_EQ(dataset.frames()[10].stamp_ns, 1000000000);
  EXPECT_EQ(dataset.imu().back().stamp_ns, 2050000000);

  const auto cloud = dataset.load_cloud(10);
  EXPECT_GT(cloud->size(), 1000);
  EXPECT_DOUBLE_EQ(cloud->stamp, 1.0);

  std::ifstream gt(dataset.ground_truth_path());
  std::string line;
  int rows = 0;
  while (std::getline(gt, line)) {
    std::istringstream stream(line);
    std::string stamp;
    double x, y, z, qx, qy, qz, qw;
    ASSERT_TRUE(stream >> stamp >> x >> y >> z >> qx >> qy >> qz >> qw) << line;
    EXPECT_EQ(stamp, format_stamp_ns(dataset.frames()[rows].stamp_ns));
    const Eigen::Isometry3d T = synthetic_T_world_imu(dataset.frames()[rows].stamp_ns * 1e-9);
    EXPECT_NEAR((T.translation() - Eigen::Vector3d(x, y, z)).norm(), 0.0, 1e-8);
    EXPECT_NEAR(Eigen::Quaterniond(T.linear()).angularDistance(Eigen::Quaterniond(qw, qx, qy, qz)), 0.0, 1e-8);
    rows++;
  }
  EXPECT_EQ(rows, 21);
  fs::remove_all(dir);
}
