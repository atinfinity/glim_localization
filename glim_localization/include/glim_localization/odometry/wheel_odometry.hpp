#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace glim_localization {

/// Optional wheel odometry input (config keys wheel_odom_*). Off by default: without it nothing changes.
struct WheelOdometryParams {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool enabled = false;
  /// Pose of the wheel base frame (the frame the twist is given in) in the IMU frame
  Eigen::Isometry3d T_imu_base = Eigen::Isometry3d::Identity();
  /// Drop the lateral and vertical velocity of the base (a non-holonomic ground robot)
  bool nonholonomic = false;
  /// Largest allowed interval between samples, and between a frame and the nearest sample [s]
  double max_gap = 0.2;
  /// Translation sigma per axis of one relative pose: min + ratio * distance travelled [m]
  double translation_sigma_min = 0.01;
  double translation_sigma_ratio = 0.05;
  /// Rotation sigma per axis of one relative pose: min + ratio * angle turned [rad]. Weak by default: the gyro is better.
  double rotation_sigma_min = 0.05;
  double rotation_sigma_ratio = 0.2;
  /// Huber threshold on the whitened error; 0 disables the robust kernel
  double huber_threshold = 1.0;
  /// Consistency gate against the IMU prediction: the factor is skipped when the mean speed or angular speed of the
  /// wheel motion over the frame interval differs from the IMU prediction by more than this [m/s], [rad/s]. 0 disables.
  double gate_velocity = 1.0;
  double gate_angular_velocity = 0.5;

  /// @throw std::invalid_argument if a value is invalid
  void validate() const;
};

struct WheelOdometrySample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double stamp = 0.0;
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();   ///< Linear velocity of the base in the base frame [m/s]
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();  ///< Angular velocity of the base in the base frame [rad/s]
};

/// Time-ordered buffer of wheel twists. Thread-safe: samples may come from another thread than the frames.
class WheelOdometryBuffer {
public:
  /// Bound on the samples kept, in case no frame trims the buffer (input without LiDAR)
  static constexpr size_t kMaxSamples = 10000;

  explicit WheelOdometryBuffer(const WheelOdometryParams& params = WheelOdometryParams());

  /// @return false if the sample was dropped (non-finite, or not newer than the last one)
  bool insert(double stamp, const Eigen::Vector3d& linear, const Eigen::Vector3d& angular);

  /// Motion of the base frame from t0 to t1 (T_base(t0)_base(t1)), integrating the twist linearly interpolated between
  /// samples and held constant past the ends. Empty if the samples do not cover [t0, t1] within max_gap.
  std::optional<Eigen::Isometry3d> integrate(double t0, double t1) const;

  /// Drop samples before the last one at or before keep_from
  void trim(double keep_from);

  size_t size() const;

private:
  WheelOdometrySample interpolate(double t) const;  ///< Caller holds mutex_

  WheelOdometryParams params_;
  mutable std::mutex mutex_;
  std::deque<WheelOdometrySample, Eigen::aligned_allocator<WheelOdometrySample>> samples_;
};

/// Why no wheel factor was added for a frame
enum class WheelFactorResult { ADDED, NO_DATA, GATED };

/// Relative pose factor between two IMU poses from the wheel motion of the base.
/// @param T_last_current_imu  the IMU prediction of the same motion, for the consistency gate
/// @param T_base  the wheel motion of the base over [t_last, t_current] (WheelOdometryBuffer::integrate)
/// @param result  ADDED, or GATED if the factor graph is empty because of the gate
gtsam::NonlinearFactorGraph create_wheel_factor(
  const WheelOdometryParams& params,
  gtsam::Key key_last,
  gtsam::Key key_current,
  double dt,
  const Eigen::Isometry3d& T_base,
  const Eigen::Isometry3d& T_last_current_imu,
  WheelFactorResult& result);

}  // namespace glim_localization
