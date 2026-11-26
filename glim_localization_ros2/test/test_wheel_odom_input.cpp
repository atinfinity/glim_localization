// Copyright (c) 2026 atinfinity
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <limits>

#include "glim_localization_ros2/wheel_odom_input.hpp"

using glim_localization_ros2::convert_wheel_odom;
using glim_localization_ros2::to_string;
using glim_localization_ros2::WheelOdomInputParams;
using glim_localization_ros2::WheelOdomReject;
using glim_localization_ros2::WheelOdomSample;

namespace
{

nav_msgs::msg::Odometry make_message()
{
  nav_msgs::msg::Odometry msg;
  msg.header.stamp.sec = 100;
  msg.header.stamp.nanosec = 250000000;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";
  msg.twist.twist.linear.x = 1.0;
  msg.twist.twist.linear.y = 0.1;
  msg.twist.twist.linear.z = -0.2;
  msg.twist.twist.angular.x = 0.01;
  msg.twist.twist.angular.y = -0.02;
  msg.twist.twist.angular.z = 0.3;
  return msg;
}

}  // namespace

TEST(WheelOdomInput, TakesTheTwistAndTheStampWithOffset) {
  WheelOdomInputParams params;
  params.child_frame_id = "base_link";
  params.time_offset = -0.05;
  WheelOdomSample sample;
  ASSERT_EQ(convert_wheel_odom(make_message(), params, sample), WheelOdomReject::NONE);
  EXPECT_DOUBLE_EQ(sample.stamp, 100.2);
  EXPECT_EQ(sample.linear_vel, Eigen::Vector3d(1.0, 0.1, -0.2));
  EXPECT_EQ(sample.angular_vel, Eigen::Vector3d(0.01, -0.02, 0.3));
}

TEST(WheelOdomInput, IgnoresThePoseAndTheParentFrame) {
  auto msg = make_message();
  msg.header.frame_id = "somewhere_else";
  msg.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  msg.pose.pose.orientation.w = 0.0;
  WheelOdomInputParams params;
  params.child_frame_id = "base_link";
  WheelOdomSample sample;
  EXPECT_EQ(convert_wheel_odom(msg, params, sample), WheelOdomReject::NONE);
}

TEST(WheelOdomInput, EmptyExpectedFrameAcceptsAny) {
  auto msg = make_message();
  msg.child_frame_id = "wheels";
  WheelOdomSample sample;
  EXPECT_EQ(convert_wheel_odom(msg, WheelOdomInputParams(), sample), WheelOdomReject::NONE);
}

TEST(WheelOdomInput, RejectsAnotherChildFrame) {
  auto msg = make_message();
  msg.child_frame_id = "wheels";
  WheelOdomInputParams params;
  params.child_frame_id = "base_link";
  WheelOdomSample sample;
  EXPECT_EQ(convert_wheel_odom(msg, params, sample), WheelOdomReject::FRAME_MISMATCH);
}

TEST(WheelOdomInput, RejectsAZeroStamp) {
  auto msg = make_message();
  msg.header.stamp.sec = 0;
  msg.header.stamp.nanosec = 0;
  WheelOdomSample sample;
  EXPECT_EQ(convert_wheel_odom(msg, WheelOdomInputParams(), sample), WheelOdomReject::ZERO_STAMP);
  // Only an exact 0 is unstamped
  msg.header.stamp.nanosec = 1;
  EXPECT_EQ(convert_wheel_odom(msg, WheelOdomInputParams(), sample), WheelOdomReject::NONE);
}

TEST(WheelOdomInput, RejectsNonFiniteValues) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  WheelOdomSample sample;
  for (int i = 0; i < 6; i++) {
    auto msg = make_message();
    auto & t = msg.twist.twist;
    double * fields[] = {&t.linear.x, &t.linear.y, &t.linear.z, &t.angular.x, &t.angular.y,
      &t.angular.z};
    *fields[i] = (i % 2) ? inf : nan;
    EXPECT_EQ(convert_wheel_odom(msg, WheelOdomInputParams(), sample),
      WheelOdomReject::NON_FINITE) << "field " << i;
  }
  WheelOdomInputParams params;
  params.time_offset = nan;
  EXPECT_EQ(convert_wheel_odom(make_message(), params, sample), WheelOdomReject::NON_FINITE);
}

TEST(WheelOdomInput, NamesEveryReason) {
  EXPECT_STREQ(to_string(WheelOdomReject::NONE), "none");
  EXPECT_STREQ(to_string(WheelOdomReject::ZERO_STAMP), "zero stamp");
  EXPECT_STREQ(to_string(WheelOdomReject::NON_FINITE), "non-finite value");
  EXPECT_STREQ(to_string(WheelOdomReject::FRAME_MISMATCH), "child_frame_id mismatch");
}
