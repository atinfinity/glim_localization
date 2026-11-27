#include <memory>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <glim_localization/version.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

TEST(Version, DeliberateFailureForCiPartsCheck) { FAIL() << "deliberate failure (PR #75 verification)"; }

TEST(Version, ReturnsProjectVersion) {
  EXPECT_EQ(glim_localization::version(), "0.1.0");  // x-release-please-version
}

TEST(Dependencies, GlimIsLinked) {
  EXPECT_TRUE(glim_localization::is_glim_linked());
}

TEST(Dependencies, CreatesPointCloud) {
  const std::vector<Eigen::Vector4d> points = {{0.0, 0.0, 0.0, 1.0}, {1.0, 0.0, 0.0, 1.0}};
  const auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
  EXPECT_EQ(cloud->size(), points.size());
}
