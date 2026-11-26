// Copyright (c) 2026 atinfinity
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "glim_localization_ros2/localization_publisher.hpp"

#include <cmath>
#include <cstdio>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace glim_localization_ros2
{

namespace
{

using glim_localization::LocalizationState;
using glim_localization_msgs::msg::LocalizationStatus;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

builtin_interfaces::msg::Time to_time_msg(double stamp)
{
  builtin_interfaces::msg::Time msg;
  const double sec = std::floor(stamp);
  msg.sec = static_cast<int32_t>(sec);
  msg.nanosec = static_cast<uint32_t>(std::min(std::round((stamp - sec) * 1e9), 999999999.0));
  return msg;
}

geometry_msgs::msg::Pose to_pose_msg(const Eigen::Isometry3d & T)
{
  const Eigen::Quaterniond q(Eigen::Quaterniond(T.rotation()).normalized());
  geometry_msgs::msg::Pose pose;
  pose.position.x = T.translation().x();
  pose.position.y = T.translation().y();
  pose.position.z = T.translation().z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

geometry_msgs::msg::TransformStamped to_transform_msg(
  const builtin_interfaces::msg::Time & stamp, const std::string & parent,
  const std::string & child, const Eigen::Isometry3d & T)
{
  const auto pose = to_pose_msg(T);
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent;
  msg.child_frame_id = child;
  msg.transform.translation.x = pose.position.x;
  msg.transform.translation.y = pose.position.y;
  msg.transform.translation.z = pose.position.z;
  msg.transform.rotation = pose.orientation;
  return msg;
}

int64_t key_ns(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
}

Eigen::Isometry3d from_transform_msg(const geometry_msgs::msg::Transform & msg)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::Quaterniond(msg.rotation.w, msg.rotation.x, msg.rotation.y,
        msg.rotation.z).normalized().toRotationMatrix();
  T.translation() = Eigen::Vector3d(msg.translation.x, msg.translation.y, msg.translation.z);
  return T;
}

bool is_finite(const Eigen::Isometry3d & T)
{
  return T.matrix().allFinite();
}

// The values are pinned to the core enums by test_message_constants.
uint8_t to_message_state(LocalizationState state)
{
  switch (state) {
    case LocalizationState::INITIALIZING:
      return LocalizationStatus::INITIALIZING;
    case LocalizationState::TRACKING:
      return LocalizationStatus::TRACKING;
    case LocalizationState::DEGRADED:
      return LocalizationStatus::DEGRADED;
    case LocalizationState::LOST:
      return LocalizationStatus::LOST;
  }
  return LocalizationStatus::LOST;
}

uint8_t to_message_reason(glim_localization::ResetReason reason)
{
  switch (reason) {
    case glim_localization::ResetReason::NONE:
      return LocalizationStatus::RESET_NONE;
    case glim_localization::ResetReason::EXTERNAL:
      return LocalizationStatus::RESET_EXTERNAL;
    case glim_localization::ResetReason::IMU_GAP:
      return LocalizationStatus::RESET_IMU_GAP;
  }
  return LocalizationStatus::RESET_NONE;
}

// WARN on DEGRADED, ERROR on LOST. INITIALIZING is a normal start-up state and is OK.
uint8_t diagnostic_level(uint8_t state)
{
  switch (state) {
    case LocalizationStatus::DEGRADED:
      return DiagnosticStatus::WARN;
    case LocalizationStatus::LOST:
      return DiagnosticStatus::ERROR;
    default:
      return DiagnosticStatus::OK;
  }
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  return kv;
}

/// Why the search rejected the pose, with the numbers that matter for that reason (the thresholds
/// are in the odometry config, not in the message).
std::string init_search_rejection_reason(
  const glim_localization_msgs::msg::LocalizationStatus & msg)
{
  using glim_localization_msgs::msg::LocalizationStatus;
  char text[160];
  switch (msg.init_search_reason) {
    case LocalizationStatus::INIT_SEARCH_REASON_TRUNCATED:
      std::snprintf(text, sizeof(text), "time budget ran out after %d of %d candidates",
          msg.init_search_candidates, msg.init_search_candidates_total);
      break;
    case LocalizationStatus::INIT_SEARCH_REASON_LOW_SCORE:
      std::snprintf(text, sizeof(text), "best inlier ratio %.3f is below the minimum",
          msg.init_search_score);
      break;
    case LocalizationStatus::INIT_SEARCH_REASON_AMBIGUOUS:
      std::snprintf(text, sizeof(text), "ambiguous: best inlier ratio %.3f, runner-up %.3f",
          msg.init_search_score, msg.init_search_runner_up_score);
      break;
    case LocalizationStatus::INIT_SEARCH_REASON_NO_CANDIDATE:
      std::snprintf(text, sizeof(text), "no candidate to score");
      break;
    case LocalizationStatus::INIT_SEARCH_REASON_OUT_OF_RANGE:
      std::snprintf(text, sizeof(text),
          "out of range: inlier ratio %.3f beyond the maximum translation from the given pose, best within %.3f",
          msg.init_search_score, msg.init_search_runner_up_score);
      break;
    default:
      std::snprintf(text, sizeof(text), "reason %d", static_cast<int>(msg.init_search_reason));
      break;
  }
  return text;
}

}  // namespace

TfMode parse_tf_mode(const std::string & name)
{
  if (name == "internal_odom") {
    return TfMode::INTERNAL_ODOM;
  }
  if (name == "external_odom") {
    return TfMode::EXTERNAL_ODOM;
  }
  if (name == "none") {
    return TfMode::NONE;
  }
  throw std::invalid_argument("unknown tf_mode '" + name +
      "' (internal_odom, external_odom or none)");
}

const char * to_string(TfMode mode)
{
  switch (mode) {
    case TfMode::INTERNAL_ODOM:
      return "internal_odom";
    case TfMode::EXTERNAL_ODOM:
      return "external_odom";
    case TfMode::NONE:
      return "none";
  }
  return "unknown";
}

LocalizationPublisher::LocalizationPublisher(
  NodeInterfaces node,
  const LocalizationPublisherParams & params)
: params_(params),
  node_(node),
  clock_(node.get_node_clock_interface()->get_clock())
{
  if (params_.map_frame.empty() || params_.base_frame.empty() ||
    (params_.tf_mode != TfMode::NONE && params_.odom_frame.empty()))
  {
    throw std::invalid_argument("frame names must not be empty");
  }
  if (!is_finite(params_.T_base_imu) || !std::isfinite(params_.diagnostics_period) ||
    params_.diagnostics_period < 0.0)
  {
    throw std::invalid_argument(
        "T_base_imu and diagnostics_period must be finite and not negative");
  }

  auto topics = node_.get_node_topics_interface();
  pose_pub_ = rclcpp::create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topics,
      "~/pose", rclcpp::QoS(10));
  odometry_pub_ = rclcpp::create_publisher<nav_msgs::msg::Odometry>(topics, "~/odometry",
      rclcpp::QoS(10));
  status_pub_ = rclcpp::create_publisher<LocalizationStatus>(topics, "~/status", rclcpp::QoS(10));
  diagnostics_pub_ = rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(topics,
      "/diagnostics", rclcpp::QoS(10));

  if (params_.tf_mode != TfMode::NONE) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(&node_);
  }
  if (params_.tf_mode == TfMode::EXTERNAL_ODOM) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, &node_, false);
  }
  last_diag_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
}

LocalizationPublisher::~LocalizationPublisher() = default;

size_t LocalizationPublisher::tf_lookup_failures() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return tf_lookup_failures_;
}

size_t LocalizationPublisher::unpaired_frames() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return unpaired_frames_;
}

size_t LocalizationPublisher::stale_frames() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stale_frames_;
}

void LocalizationPublisher::on_frame(const FrameEstimate & frame)
{
  if (!std::isfinite(frame.stamp) || frame.stamp < 0.0 || !is_finite(frame.T_map_imu) ||
    !frame.v_map_imu.allFinite() || !frame.w_imu.allFinite())
  {
    RCLCPP_WARN_THROTTLE(node_.get_node_logging_interface()->get_logger(), *clock_, 5000,
        "dropped a frame with a non-finite stamp, pose or velocity");
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (last_stamp_ && frame.stamp <= *last_stamp_ && !allow_older_stamp_) {
    ++stale_frames_;
    RCLCPP_WARN_THROTTLE(node_.get_node_logging_interface()->get_logger(), *clock_, 5000,
        "dropped a frame whose stamp is not newer than the last one");
    return;
  }
  last_stamp_ = frame.stamp;
  allow_older_stamp_ = false;
  if (pending_) {
    ++unpaired_frames_;
  }
  pending_ = frame;
}

void LocalizationPublisher::publish_frame(const FrameEstimate & frame, uint8_t state)
{
  const bool lost = state == LocalizationStatus::LOST;
  const Eigen::Isometry3d T_map_base = frame.T_map_imu * params_.T_base_imu.inverse();
  const auto stamp = to_time_msg(frame.stamp);

  // A lost frame is published nowhere but odom->base (internal_odom): the pose covariance is zero,
  // which means "not estimated", and a consumer must not see a pose it cannot tell is lost.
  if (!lost) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = params_.map_frame;
    pose.pose.pose = to_pose_msg(T_map_base);
    pose_pub_->publish(pose);

    // The twist is in the child frame (REP-105). The estimate is of the IMU, so the velocity of the
    // base origin is the IMU velocity plus w x p, with p the base origin in the IMU frame.
    const Eigen::Matrix3d R_base_imu = params_.T_base_imu.rotation();
    const Eigen::Vector3d p_base_in_imu = params_.T_base_imu.inverse().translation();
    const Eigen::Vector3d v_imu = frame.T_map_imu.rotation().transpose() * frame.v_map_imu;
    const Eigen::Vector3d v_base = R_base_imu * (v_imu + frame.w_imu.cross(p_base_in_imu));
    const Eigen::Vector3d w_base = R_base_imu * frame.w_imu;
    nav_msgs::msg::Odometry odometry;
    odometry.header = pose.header;
    odometry.child_frame_id = params_.base_frame;
    odometry.pose.pose = pose.pose.pose;
    odometry.twist.twist.linear.x = v_base.x();
    odometry.twist.twist.linear.y = v_base.y();
    odometry.twist.twist.linear.z = v_base.z();
    odometry.twist.twist.angular.x = w_base.x();
    odometry.twist.twist.angular.y = w_base.y();
    odometry.twist.twist.angular.z = w_base.z();
    odometry_pub_->publish(odometry);
  }

  if (params_.tf_mode == TfMode::NONE || (lost && params_.tf_mode == TfMode::EXTERNAL_ODOM)) {
    return;
  }
  Eigen::Isometry3d T_odom_base = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_map_odom = Eigen::Isometry3d::Identity();
  if (compute_odom_base(stamp, T_map_base, T_odom_base, T_map_odom)) {
    publish_tf(stamp, T_map_odom, T_odom_base, !lost);
  }
}

bool LocalizationPublisher::compute_odom_base(
  const builtin_interfaces::msg::Time & stamp, const Eigen::Isometry3d & T_map_base,
  Eigen::Isometry3d & T_odom_base, Eigen::Isometry3d & T_map_odom)
{
  if (params_.tf_mode == TfMode::EXTERNAL_ODOM) {
    try {
      // A double stamp near 1.7e9 s has 238 ns resolution, so the message stamp can be a few ns
      // after an odom sample at the true stamp. Look up at the stamp floored to 1 us.
      const int64_t ns = key_ns(stamp);
      const auto transform = tf_buffer_->lookupTransform(params_.odom_frame, params_.base_frame,
          tf2::TimePoint(std::chrono::nanoseconds(ns - ns % 1000)));
      T_odom_base = from_transform_msg(transform.transform);
      T_map_odom = T_map_base * T_odom_base.inverse();
      return true;
    } catch (const tf2::TransformException & e) {
      ++tf_lookup_failures_;
      RCLCPP_WARN_THROTTLE(
        node_.get_node_logging_interface()->get_logger(), *clock_, 5000,
          "no %s->%s at the frame stamp, map->odom is not published: %s",
        params_.odom_frame.c_str(), params_.base_frame.c_str(), e.what());
      return false;
    }
  }

  // internal_odom: odom->base = anchor * pose. The anchor is identity at the first frame and
  // changes only at the first frame after an external reset, where odom->base is kept at its last
  // value.
  if (reanchor_ || !has_odom_base_) {
    anchor_ = has_odom_base_ ? Eigen::Isometry3d(last_odom_base_ *
        T_map_base.inverse()) : Eigen::Isometry3d::Identity();
    anchor_inverse_ = anchor_.inverse();
    reanchor_ = false;
  }
  T_odom_base = anchor_ * T_map_base;
  T_map_odom = anchor_inverse_;
  last_odom_base_ = T_odom_base;
  has_odom_base_ = true;
  return true;
}

void LocalizationPublisher::publish_tf(
  const builtin_interfaces::msg::Time & stamp,
  const Eigen::Isometry3d & T_map_odom, const Eigen::Isometry3d & T_odom_base, bool with_map_odom)
{
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  if (with_map_odom) {
    transforms.push_back(to_transform_msg(stamp, params_.map_frame, params_.odom_frame,
        T_map_odom));
  }
  if (params_.tf_mode == TfMode::INTERNAL_ODOM) {
    transforms.push_back(to_transform_msg(stamp, params_.odom_frame, params_.base_frame,
        T_odom_base));
  }
  if (!transforms.empty()) {
    tf_broadcaster_->sendTransform(transforms);
  }
}

void LocalizationPublisher::on_status(const glim_localization::LocalizationStatus & status)
{
  const bool for_frame = status.frame_id >= 0 && std::isfinite(status.stamp);

  LocalizationStatus msg;
  msg.state = to_message_state(status.state);
  msg.reset_count = status.reset_count;
  msg.num_tiles = status.num_tiles;
  msg.num_map_factors = status.num_map_factors;
  msg.inlier_ratio = status.inlier_ratio;
  msg.residual = status.residual;
  msg.processing_time = status.processing_time;
  msg.last_reset_reason = to_message_reason(status.last_reset_reason);
  // Same values, pinned by test_message_constants
  msg.init_search = static_cast<uint8_t>(status.init_search);
  msg.init_search_reason = static_cast<uint8_t>(status.init_search_reason);
  msg.init_search_candidates = status.init_search_candidates;
  msg.init_search_candidates_total = status.init_search_candidates_total;
  msg.init_search_score = status.init_search_score;
  msg.init_search_yaw_offset = status.init_search_yaw_offset;
  msg.init_search_runner_up_score = status.init_search_runner_up_score;
  msg.init_search_runner_up_yaw_offset = status.init_search_runner_up_yaw_offset;
  msg.init_search_time = status.init_search_time;
  msg.wheel_factors = status.wheel_factors;
  msg.wheel_gated = status.wheel_gated;
  msg.wheel_readmitted = status.wheel_readmitted;
  msg.wheel_missing = status.wheel_missing;
  msg.header.frame_id = params_.map_frame;

  std::lock_guard<std::mutex> lock(mutex_);
  // A reset status is stamped with min_stamp, which may be 0 and is not a frame time: use the last
  // frame stamp, or the current time before the first frame
  msg.header.stamp =
    for_frame ? to_time_msg(status.stamp) :
    (last_stamp_ ? to_time_msg(*last_stamp_) : builtin_interfaces::msg::Time(clock_->now()));
  if (status.reset_count != last_reset_count_) {
    last_reset_count_ = status.reset_count;
    if (status.last_reset_reason == glim_localization::ResetReason::EXTERNAL) {
      reanchor_ = true;
      allow_older_stamp_ = true;
    }
  }

  if (for_frame && pending_ && pending_->stamp == status.stamp) {
    const FrameEstimate frame = *pending_;
    pending_.reset();
    publish_frame(frame, msg.state);
  }
  status_pub_->publish(msg);
  publish_diagnostics(msg);
}

void LocalizationPublisher::publish_diagnostics(const LocalizationStatus & msg)
{
  const uint8_t level = diagnostic_level(msg.state);
  const auto now = clock_->now();
  const bool changed = !last_diag_level_ || *last_diag_level_ != level;
  if (!changed && last_diag_time_.nanoseconds() != 0 &&
    (now - last_diag_time_).seconds() < params_.diagnostics_period)
  {
    return;
  }
  last_diag_level_ = level;
  last_diag_time_ = now;

  const auto state_name = glim_localization::to_string(static_cast<LocalizationState>(msg.state));
  DiagnosticStatus status;
  status.level = level;
  status.name = params_.diagnostics_name;
  status.hardware_id = params_.diagnostics_hardware_id;
  status.message = state_name;
  if (msg.init_search == LocalizationStatus::INIT_SEARCH_REJECTED) {
    status.message += ": the initial pose search rejected the given pose (" +
      init_search_rejection_reason(msg) + ")";
  }
  status.values.push_back(key_value("state", state_name));
  status.values.push_back(key_value("reset_count", std::to_string(msg.reset_count)));
  status.values.push_back(key_value("last_reset_reason", std::to_string(msg.last_reset_reason)));
  status.values.push_back(key_value("num_tiles", std::to_string(msg.num_tiles)));
  status.values.push_back(key_value("num_map_factors", std::to_string(msg.num_map_factors)));
  status.values.push_back(key_value("inlier_ratio", std::to_string(msg.inlier_ratio)));
  status.values.push_back(key_value("residual", std::to_string(msg.residual)));
  status.values.push_back(key_value("processing_time", std::to_string(msg.processing_time)));
  status.values.push_back(key_value("init_search",
      glim_localization::to_string(
      static_cast<glim_localization::InitSearchResult>(msg.init_search))));
  status.values.push_back(
    key_value("init_search_reason",
      glim_localization::to_string(
      static_cast<glim_localization::InitSearchReason>(msg.init_search_reason))));
  status.values.push_back(key_value("init_search_candidates",
      std::to_string(msg.init_search_candidates)));
  status.values.push_back(key_value("init_search_candidates_total",
      std::to_string(msg.init_search_candidates_total)));
  status.values.push_back(key_value("init_search_score", std::to_string(msg.init_search_score)));
  status.values.push_back(key_value("init_search_yaw_offset_deg",
      std::to_string(msg.init_search_yaw_offset * 180.0 / M_PI)));
  status.values.push_back(key_value("init_search_runner_up_score",
      std::to_string(msg.init_search_runner_up_score)));
  status.values.push_back(key_value("init_search_runner_up_yaw_offset_deg",
      std::to_string(msg.init_search_runner_up_yaw_offset * 180.0 / M_PI)));
  status.values.push_back(key_value("init_search_time", std::to_string(msg.init_search_time)));
  status.values.push_back(key_value("wheel_factors", std::to_string(msg.wheel_factors)));
  status.values.push_back(key_value("wheel_gated", std::to_string(msg.wheel_gated)));
  status.values.push_back(key_value("wheel_readmitted", std::to_string(msg.wheel_readmitted)));
  status.values.push_back(key_value("wheel_missing", std::to_string(msg.wheel_missing)));

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now;
  array.status.push_back(status);
  diagnostics_pub_->publish(array);
}

}  // namespace glim_localization_ros2
