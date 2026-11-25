// Copyright (c) 2026 atinfinity
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// The online check of a late initial pose (R2 ~/initialize and /initialpose). No ROS.
#include <gtest/gtest.h>

#include <limits>
#include <optional>

#include "glim_localization_ros2/init_pose_delay.hpp"

using glim_localization_ros2::check_init_pose_delay;

namespace
{
constexpr double kMax = 0.2;  // max_init_pose_delay default
// a realistic stamp, where a double has about 0.24 us resolution
constexpr double kT = 1700000000.0;
}  // namespace

TEST(InitPoseDelay, UnstampedIsAlwaysAccepted) {
  const auto r = check_init_pose_delay(0.0, kT + 100.0, kMax);
  EXPECT_TRUE(r.accept);
  EXPECT_FALSE(r.late_by.has_value());
}

TEST(InitPoseDelay, NoFrameYetIsAccepted) {
  const auto r = check_init_pose_delay(kT, std::nullopt, kMax);
  EXPECT_TRUE(r.accept);
  EXPECT_FALSE(r.late_by.has_value());
}

TEST(InitPoseDelay, StampAfterTheLastFrameIsOnTime) {
  const auto r = check_init_pose_delay(kT + 0.05, kT, kMax);
  EXPECT_TRUE(r.accept);
  EXPECT_FALSE(r.late_by.has_value());
}

TEST(InitPoseDelay, ShortDelayIsAcceptedWithItsLowerBound) {
  for (const double late : {0.0, 0.1, 0.19, 0.2}) {
    const auto r = check_init_pose_delay(kT, kT + late, kMax);
    EXPECT_TRUE(r.accept) << late;
    ASSERT_TRUE(r.late_by.has_value()) << late;
    EXPECT_NEAR(*r.late_by, late, 1e-6) << late;
  }
}

TEST(InitPoseDelay, LongDelayIsRejected) {
  for (const double late : {0.21, 0.3, 5.0}) {
    const auto r = check_init_pose_delay(kT, kT + late, kMax);
    EXPECT_FALSE(r.accept) << late;
    ASSERT_TRUE(r.late_by.has_value());
    EXPECT_NEAR(*r.late_by, late, 1e-6) << late;
  }
}

TEST(InitPoseDelay, ZeroMaxDelayAcceptsOnlyFuturePoses) {
  EXPECT_TRUE(check_init_pose_delay(kT + 0.01, kT, 0.0).accept);
  EXPECT_FALSE(check_init_pose_delay(kT, kT + 0.01, 0.0).accept);
}

TEST(InitPoseDelay, NonFiniteLastFrameIsIgnored) {
  EXPECT_TRUE(check_init_pose_delay(kT, std::numeric_limits<double>::quiet_NaN(), kMax).accept);
}
