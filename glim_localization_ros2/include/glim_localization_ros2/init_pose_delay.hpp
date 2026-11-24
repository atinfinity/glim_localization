#pragma once

#include <cmath>
#include <optional>

namespace glim_localization_ros2 {

/// The online form of the replay rule for a late initial pose (DESIGN 6.5, max_init_pose_delay).
///
/// The core applies a stamped pose at the first frame at or after its stamp that it has not taken yet. Online, the
/// node compares the pose stamp with the stamp of the last frame the odometry thread took off its queue
/// (OdometryEstimationLocalizationCPU::last_input_frame_stamp). That frame counts whatever happened to it: estimated,
/// or dropped while waiting for the first pose, in an IMU gap or while LOST. The pose applies at that frame or a later
/// one, so it is at least last_frame_stamp - stamp late. That lower bound is checked when the request arrives: a pose
/// that is already more than max_delay late is rejected; a shorter delay is accepted with a warning. Frames still in
/// the input queue are not counted (they get the pose too), so a frame that arrives later than expected after the
/// check is not caught. Plain C++, tested without ROS.
struct InitPoseDelay {
  bool accept = true;
  /// Lower bound of how late the pose applies [s]; empty when it can apply on time (the stamp is newer than the
  /// last frame taken, the pose is unstamped, or no frame was taken yet)
  std::optional<double> late_by;
};

/// @param stamp             stamp of the pose [s]; 0 means "the next frame" and is always accepted
/// @param last_frame_stamp  stamp of the last frame the odometry took off its input queue, estimated or dropped
/// @param max_delay         max_init_pose_delay [s]
inline InitPoseDelay check_init_pose_delay(double stamp, std::optional<double> last_frame_stamp, double max_delay) {
  InitPoseDelay result;
  if (stamp == 0.0 || !last_frame_stamp || !std::isfinite(*last_frame_stamp) || *last_frame_stamp < stamp) {
    return result;
  }
  result.late_by = *last_frame_stamp - stamp;
  // 1 us of tolerance keeps a delay of exactly max_delay on the accepted side: a double has about 0.24 us resolution
  // at 1.7e9 s, so the difference of two such stamps is off by that much
  result.accept = *result.late_by <= max_delay + 1e-6;
  return result;
}

}  // namespace glim_localization_ros2
