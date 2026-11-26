// Copyright (c) 2026 atinfinity
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <cmath>
#include <string>

#include "Eigen/Core"
#include "nav_msgs/msg/odometry.hpp"

namespace glim_localization_ros2
{

/// How a nav_msgs/Odometry message becomes a wheel odometry sample of the core
/// (OdometryEstimationLocalization::insert_wheel_odom). Read from localization_ros in
/// config_ros.json.
struct WheelOdomInputParams
{
  /// Expected child_frame_id, the frame the twist is expressed in. It must be the base frame of
  /// wheel_odom_T_imu_base in the odometry config. Empty accepts any.
  std::string child_frame_id;
  double time_offset = 0.0;  ///< Added to the header stamp [s], like imu_time_offset
};

enum class WheelOdomReject
{
  NONE,
  ZERO_STAMP,      ///< The header stamp is 0: the sender did not stamp the message
  NON_FINITE,      ///< The stamp or a twist component is not finite
  FRAME_MISMATCH,  ///< child_frame_id differs from the expected one
};

inline const char * to_string(WheelOdomReject reason)
{
  switch (reason) {
    case WheelOdomReject::NONE:
      return "none";
    case WheelOdomReject::ZERO_STAMP:
      return "zero stamp";
    case WheelOdomReject::NON_FINITE:
      return "non-finite value";
    case WheelOdomReject::FRAME_MISMATCH:
      return "child_frame_id mismatch";
  }
  return "unknown";
}

struct WheelOdomSample
{
  double stamp = 0.0;  ///< Header stamp plus time_offset [s]
  Eigen::Vector3d linear_vel = Eigen::Vector3d::Zero();   ///< In child_frame_id [m/s]
  Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();  ///< In child_frame_id [rad/s]
};

/// Take the twist of a nav_msgs/Odometry message, which is expressed in child_frame_id. The pose,
/// header.frame_id and the covariances are not used. Header only and without a ROS graph, so that
/// it can be tested alone.
/// @return NONE and fill `sample`, or why the message is rejected (`sample` is then unspecified)
inline WheelOdomReject convert_wheel_odom(
  const nav_msgs::msg::Odometry & msg, const WheelOdomInputParams & params,
  WheelOdomSample & sample)
{
  if (!params.child_frame_id.empty() && msg.child_frame_id != params.child_frame_id) {
    return WheelOdomReject::FRAME_MISMATCH;
  }
  if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) {
    return WheelOdomReject::ZERO_STAMP;
  }
  const auto & twist = msg.twist.twist;
  sample.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9 + params.time_offset;
  sample.linear_vel = Eigen::Vector3d(twist.linear.x, twist.linear.y, twist.linear.z);
  sample.angular_vel = Eigen::Vector3d(twist.angular.x, twist.angular.y, twist.angular.z);
  if (!std::isfinite(sample.stamp) || !sample.linear_vel.allFinite() ||
    !sample.angular_vel.allFinite())
  {
    return WheelOdomReject::NON_FINITE;
  }
  return WheelOdomReject::NONE;
}

}  // namespace glim_localization_ros2
