// GyroBuffer pairs a frame with the gyro sample of its stamp, and refuses a sample that is too far away.
#include <gtest/gtest.h>

#include <cmath>

#include <glim_localization_ros2/gyro_buffer.hpp>

using glim_localization_ros2::GyroBuffer;

namespace {
// 200 Hz samples from t0 for n samples; the gyro x value is the sample index.
GyroBuffer filled(double t0, int n) {
  GyroBuffer buffer;
  for (int i = 0; i < n; i++) {
    buffer.add(t0 + i * 0.005, Eigen::Vector3d(i, 0.0, 0.0));
  }
  return buffer;
}
}  // namespace

TEST(GyroBuffer, ReturnsTheNearestSample) {
  const GyroBuffer buffer = filled(10.0, 100);
  Eigen::Vector3d g;
  ASSERT_TRUE(buffer.nearest(10.2, g));
  EXPECT_DOUBLE_EQ(g.x(), 40.0);
  ASSERT_TRUE(buffer.nearest(10.2012, g));  // between 40 and 41, nearer to 40
  EXPECT_DOUBLE_EQ(g.x(), 40.0);
  ASSERT_TRUE(buffer.nearest(10.2038, g));  // nearer to 41
  EXPECT_DOUBLE_EQ(g.x(), 41.0);
}

TEST(GyroBuffer, EmptyOrNonFiniteStampGivesNothing) {
  GyroBuffer empty;
  Eigen::Vector3d g;
  EXPECT_FALSE(empty.nearest(1.0, g));
  const GyroBuffer buffer = filled(10.0, 10);
  EXPECT_FALSE(buffer.nearest(std::nan(""), g));
}

// The window is counted back from the newest sample. A frame that the odometry handles late finds only samples far from
// its stamp, and must not get one of them.
TEST(GyroBuffer, RejectsAFrameFarFromEverySample) {
  const GyroBuffer buffer = filled(10.0, 400);  // 10.000 to 11.995; the window keeps 10.995 and later
  Eigen::Vector3d g;
  EXPECT_FALSE(buffer.nearest(10.2, g));  // lags by about 0.8 s: the nearest kept sample is 0.8 s away
  EXPECT_FALSE(buffer.nearest(12.5, g));  // ahead of the newest sample
  ASSERT_TRUE(buffer.nearest(11.9, g));
  EXPECT_DOUBLE_EQ(g.x(), 380.0);
  EXPECT_LE(buffer.size(), 201u);
}

TEST(GyroBuffer, ToleranceIsAboutTwoPeriodsOf100Hz) {
  GyroBuffer buffer;
  buffer.add(5.0, Eigen::Vector3d::Zero());
  Eigen::Vector3d g;
  EXPECT_TRUE(buffer.nearest(5.019, g));
  EXPECT_FALSE(buffer.nearest(5.021, g));
  EXPECT_TRUE(buffer.nearest(4.981, g));
  EXPECT_FALSE(buffer.nearest(4.979, g));
}

TEST(GyroBuffer, StampsGoingBackwardsStartAgain) {
  GyroBuffer buffer = filled(10.0, 100);
  buffer.add(2.0, Eigen::Vector3d(7.0, 0.0, 0.0));
  EXPECT_EQ(buffer.size(), 1u);
  Eigen::Vector3d g;
  ASSERT_TRUE(buffer.nearest(2.001, g));
  EXPECT_DOUBLE_EQ(g.x(), 7.0);
}
