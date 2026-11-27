// Copyright (c) 2026 atinfinity
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "Eigen/Geometry"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "glim/odometry/estimation_frame.hpp"
#include "glim/util/config.hpp"
#include "glim_localization/odometry/odometry_estimation_localization.hpp"
#include "glim_localization_ros2/gyro_buffer.hpp"
#include "glim_localization_ros2/localization_publisher.hpp"

namespace glim_localization_ros2
{

/// [x, y, z, qx, qy, qz, qw]. Same layout as T_lidar_imu in config_sensors.json.
/// @throw std::invalid_argument if the size is not 7 or the values are not a valid pose
Eigen::Isometry3d parse_pose7(const std::vector<double> & v, const std::string & name);

/// Publisher settings from config_ros.json: map and odom frame names from "glim_ros", the rest from
/// "localization_ros" (tf_mode, base_frame_id, T_base_imu, diagnostics_period,
/// publish_aligned_points, aligned_points_voxel). Shared by R1 and R2.
/// @throw std::invalid_argument for a bad tf_mode, T_base_imu or aligned_points_voxel
LocalizationPublisherParams read_publisher_params(const glim::Config & config_ros);

/// Connects the core's callbacks to a LocalizationPublisher. Shared by the R1 extension and the R2
/// node.
///
/// The core reports through static CallbackSlots, which are not thread-safe to modify while an
/// odometry thread runs. So a bridge is connected once (connect()) and stays registered for the
/// rest of the process; attach() and detach() switch the forwarding on and off under a mutex. Every
/// connected bridge receives the callbacks of every odometry instance in the process, so run one
/// localization (R1 or R2) per process.
class EstimateBridge {
public:
  using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

  /// Register bridge in OdometryEstimationCallbacks::on_insert_imu, on_update_new_frame and
  /// LocalizationCallbacks::on_status. Call before the odometry starts.
  static void connect(const std::shared_ptr<EstimateBridge> & bridge);

  /// Start forwarding to publisher. Clears the gyro buffer.
  void attach(
    std::unique_ptr<LocalizationPublisher> publisher,
    const LocalizationPublisherParams & params);
  /// Stop forwarding and release the publisher. Idempotent.
  void detach();
  bool attached() const;

  /// Raw gyro of a sample given to the odometry (FrameEstimate::w_imu needs it; the core does not
  /// keep it on the frame)
  void add_imu(double stamp, const Eigen::Vector3d & angular_vel);
  /// A corrected frame (on_update_new_frame). Stored by the publisher until its status arrives.
  void add_frame(const glim::EstimationFrame & frame);
  /// A status (on_status). Publishes the stored frame of the same stamp.
  void add_status(const glim_localization::LocalizationStatus & status);


  /// Convert an initial pose of the base frame in the map frame to the IMU pose for the core's
  /// reset(). min_stamp is the message stamp (0 when unstamped, meaning the next frame). It is
  /// frame time (cloud header + points_time_offset), the time base of ~/pose, TF and RViz's now(),
  /// and is compared with the frames without any shift.
  /// @return an empty string on success, otherwise why the pose is rejected (not attached, wrong
  /// frame, invalid pose)
  std::string to_T_map_imu(
    const PoseMsg & msg, Eigen::Isometry3d & T_map_imu,
    double & min_stamp) const;

  /// Frames that were not published because no gyro sample was near their stamp
  size_t frames_without_gyro() const;

  /// What the attached publisher dropped, for a log line when it is detached
  struct DropCounts
  {
    size_t frames_without_gyro = 0;
    size_t unpaired_frames = 0;
    size_t stale_frames = 0;
    size_t tf_lookup_failures = 0;
  };
  /// The counts of the attached publisher; all zero when none is attached
  DropCounts drop_counts() const;

private:
  mutable std::mutex mutex_;
  std::unique_ptr<LocalizationPublisher> publisher_;
  std::string map_frame_;
  Eigen::Isometry3d T_base_imu_ = Eigen::Isometry3d::Identity();
  GyroBuffer gyro_;
  size_t frames_without_gyro_ = 0;
  // A frame with points not in the IMU frame was seen (warned once; no ~/aligned_points then)
  std::atomic_bool warned_non_imu_frame_{false};
};

}  // namespace glim_localization_ros2
