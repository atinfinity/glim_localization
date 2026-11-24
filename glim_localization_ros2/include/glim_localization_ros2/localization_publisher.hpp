#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <glim_localization_msgs/msg/localization_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <glim_localization/odometry/odometry_estimation_localization_cpu.hpp>

namespace glim_localization_ros2 {

/// Which transforms are published.
enum class TfMode {
  INTERNAL_ODOM,  ///< map->odom (an anchor, changed only by an external reset) and odom->base = anchor * pose
  EXTERNAL_ODOM,  ///< map->odom only, computed from an odom->base published by someone else
  NONE            ///< No TF
};

/// @throw std::invalid_argument for a name other than internal_odom, external_odom, none
TfMode parse_tf_mode(const std::string& name);
const char* to_string(TfMode mode);

struct LocalizationPublisherParams {
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";
  TfMode tf_mode = TfMode::INTERNAL_ODOM;
  /// Pose of the IMU in the base frame. The estimate is of the IMU, so T_map_base = T_map_imu * inverse(T_base_imu)
  Eigen::Isometry3d T_base_imu = Eigen::Isometry3d::Identity();
  /// Minimum time between /diagnostics messages [s]. A change of level is published at once
  double diagnostics_period = 1.0;
  std::string diagnostics_name = "glim_localization: localization";
  std::string diagnostics_hardware_id = "glim_localization";
};

/// The estimate of one frame, in the map frame. Built by the caller from the core's EstimationFrame.
struct FrameEstimate {
  double stamp = 0.0;                                     ///< Frame time [s]
  Eigen::Isometry3d T_map_imu = Eigen::Isometry3d::Identity();
  Eigen::Vector3d v_map_imu = Eigen::Vector3d::Zero();  ///< Linear velocity of the IMU in the map frame
  Eigen::Vector3d w_imu = Eigen::Vector3d::Zero();      ///< Bias-corrected angular velocity in the IMU frame
};

/// Publishes ~/pose, ~/odometry, ~/status, /diagnostics and TF. Shared by the R1 extension and the R2 node.
/// ROS only; the core does not depend on it. Methods are thread-safe. In external_odom the node must be spun
/// (it listens to /tf).
class LocalizationPublisher {
public:
  using NodeInterfaces = rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface,
    rclcpp::node_interfaces::NodeClockInterface,
    rclcpp::node_interfaces::NodeLoggingInterface,
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface>;

  /// @throw std::invalid_argument if a frame name is empty, or if the parameters are not finite or negative
  LocalizationPublisher(NodeInterfaces node, const LocalizationPublisherParams& params = LocalizationPublisherParams());
  ~LocalizationPublisher();

  /// Store a frame. Nothing is published until on_status gets the status of the same frame (same stamp), which
  /// carries its state. Frames with a non-finite stamp, pose, velocity or angular velocity are dropped. A frame
  /// whose stamp is not newer than the last accepted one is dropped, unless an external reset came since. A
  /// stored frame that is replaced by a newer one before its status arrives is dropped.
  void on_frame(const FrameEstimate& frame);

  /// Publish ~/status and /diagnostics, and the pending frame if the status is for it (frame_id >= 0 and the same
  /// stamp). A status that is not for a frame (frame_id < 0, as emitted when a reset is applied) carries
  /// stamp = the reset's min_stamp, which may be 0. It is stamped with the last accepted frame stamp, or with the
  /// current time if there is none, and never used as a pose time. An external reset re-anchors odom->base at
  /// the next frame.
  void on_status(const glim_localization::LocalizationStatus& status);

  /// Frames for which external odom->base was not available, so no map->odom was published (for tests and logs)
  size_t tf_lookup_failures() const;
  /// Frames dropped because a newer frame replaced them before their status arrived
  size_t unpaired_frames() const;
  /// Frames dropped because their stamp was not newer than the last accepted one
  size_t stale_frames() const;
  /// The buffer that listens to /tf in external_odom, otherwise null (for tests)
  const tf2_ros::Buffer* tf_buffer() const { return tf_buffer_.get(); }

private:
  void publish_frame(const FrameEstimate& frame, uint8_t state);
  bool compute_odom_base(const builtin_interfaces::msg::Time& stamp, const Eigen::Isometry3d& T_map_base, Eigen::Isometry3d& T_odom_base, Eigen::Isometry3d& T_map_odom);
  void publish_tf(const builtin_interfaces::msg::Time& stamp, const Eigen::Isometry3d& T_map_odom, const Eigen::Isometry3d& T_odom_base, bool with_map_odom);
  void publish_diagnostics(const glim_localization_msgs::msg::LocalizationStatus& msg);

  const LocalizationPublisherParams params_;
  NodeInterfaces node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<glim_localization_msgs::msg::LocalizationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  // Pairing and stamp order
  std::optional<FrameEstimate> pending_;
  std::optional<double> last_stamp_;  ///< Stamp of the last accepted frame
  bool allow_older_stamp_ = false;    ///< An external reset came since the last accepted frame
  size_t unpaired_frames_ = 0;
  size_t stale_frames_ = 0;
  // internal_odom anchor: T_odom_base = anchor * T_map_base, T_map_odom = inverse(anchor)
  bool has_odom_base_ = false;
  bool reanchor_ = false;
  Eigen::Isometry3d anchor_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d anchor_inverse_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d last_odom_base_ = Eigen::Isometry3d::Identity();
  size_t tf_lookup_failures_ = 0;
  // diagnostics state
  std::optional<uint8_t> last_diag_level_;
  rclcpp::Time last_diag_time_{0, 0, RCL_ROS_TIME};
  int last_reset_count_ = 0;
};

}  // namespace glim_localization_ros2
