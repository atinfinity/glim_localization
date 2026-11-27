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
  /// Drop the lateral and vertical velocity of the base (a non-holonomic ground robot). Assumes the base origin is at
  /// the centre of the rear (fixed) axle, the point that does not slide sideways; elsewhere a turn has a lateral
  /// velocity that this would wrongly remove.
  bool nonholonomic = false;
  /// Largest allowed interval between samples, and between a frame and the nearest sample [s]
  double max_gap = 0.2;
  /// Translation sigma per axis of one relative pose: min + ratio * distance travelled [m]
  double translation_sigma_min = 0.01;
  double translation_sigma_ratio = 0.05;
  /// Rotation sigma per axis of one relative pose: min + ratio * angle turned [rad]. Weak by default: the gyro is better.
  double rotation_sigma_min = 0.05;
  double rotation_sigma_ratio = 0.2;
  /// Huber threshold on the whitened error of the whole factor (one kernel on the 6-DoF Mahalanobis distance, not per
  /// axis); 0 disables the robust kernel
  double huber_threshold = 1.0;
  /// Consistency gate against the IMU prediction over the frame interval [t0, t1] (dt = t1 - t0). The factor is skipped
  /// if |t_wheel - t_imu| / dt > max(gate_velocity, gate_velocity_ratio * max(|t_wheel|, |t_imu|) / dt), where t_* are
  /// the translation vectors of the two relative motions in the IMU frame (a vector difference: a wrong direction
  /// counts as well as a wrong speed), or if the angle of R_wheel^T R_imu / dt > gate_angular_velocity.
  /// gate_velocity is an absolute floor [m/s] that a stuck wheel at a crawl still exceeds, and the ratio term
  /// [-] scales the gate with the speed so that IMU prediction noise does not gate fast motion. 0 disables a term.
  double gate_velocity = 0.3;
  double gate_velocity_ratio = 0.3;
  double gate_angular_velocity = 0.5;  ///< [rad/s]
  /// Without a map constraint, a wrong IMU prediction (a reset with a wrong velocity, a long slip) could make the gate
  /// reject every wheel factor for good. After this many consecutive gated frames, the next factor is added without
  /// the gate (the Huber kernel still bounds its pull), and the count starts again. 0 never re-admits. Without a map
  /// the two cases cannot be told apart, so a slip longer than this many frames lets one slipping factor through: keep
  /// it above the longest expected slip (the default is 2 s at 10 Hz).
  int gate_readmit_frames = 20;
  /// IMU gap self-reset: move the last pose to the reset frame with the wheel motion and take the velocity from the
  /// wheel twist, instead of the constant velocity and held orientation. Falls back to those (with a warning) when the
  /// samples do not cover the interval within max_gap or the result is not a valid reset. No effect unless enabled.
  bool reset_extrapolation = false;

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

  /// Consecutive samples not newer than the last one after which the buffer accepts the clock as restarted (a bag
  /// loop, a simulation clock reset): it is cleared and the sample kept
  static constexpr int kRestartRejects = 5;

  /// @return false if the sample was dropped (non-finite, or not newer than the last one and no clock restart yet)
  bool insert(double stamp, const Eigen::Vector3d& linear, const Eigen::Vector3d& angular);

  /// Motion of the base frame from t0 to t1 (T_base(t0)_base(t1)), integrating the twist linearly interpolated between
  /// samples and held constant past the ends. Empty if the samples do not cover [t0, t1] within max_gap.
  std::optional<Eigen::Isometry3d> integrate(double t0, double t1) const;

  /// Twist at t, linearly interpolated between samples (as integrate() uses it; stamp is t). Empty if t is more than
  /// max_gap outside the samples or inside a hole longer than max_gap.
  std::optional<WheelOdometrySample> twist_at(double t) const;

  /// Drop samples before the last one at or before keep_from
  void trim(double keep_from);

  /// Drop all samples (a reset of the estimation)
  void clear();

  size_t size() const;

private:
  WheelOdometrySample interpolate(double t) const;  ///< Caller holds mutex_

  WheelOdometryParams params_;
  mutable std::mutex mutex_;
  std::deque<WheelOdometrySample, Eigen::aligned_allocator<WheelOdometrySample>> samples_;
  int rejected_ = 0;  ///< Consecutive samples not newer than the last one
};

/// Velocity of the IMU origin in the IMU frame for a twist of the base: R_imu_base * (v + w x p), with p the IMU origin
/// in the base frame (the lever arm of params.T_imu_base)
Eigen::Vector3d wheel_velocity_in_imu(const WheelOdometryParams& params, const WheelOdometrySample& twist);

/// Why no wheel factor was added for a frame
enum class WheelFactorResult { ADDED, NO_DATA, GATED };

/// Relative pose factor between two IMU poses from the wheel motion of the base.
/// @param T_last_current_imu  the IMU prediction of the same motion, for the consistency gate
/// @param T_base  the wheel motion of the base over [t_last, t_current] (WheelOdometryBuffer::integrate)
/// @param apply_gate  false adds the factor whatever the gate says (re-admission, see gate_readmit_frames)
/// @param result  ADDED, or GATED if the factor graph is empty because of the gate
gtsam::NonlinearFactorGraph create_wheel_factor(
  const WheelOdometryParams& params,
  gtsam::Key key_last,
  gtsam::Key key_current,
  double dt,
  const Eigen::Isometry3d& T_base,
  const Eigen::Isometry3d& T_last_current_imu,
  WheelFactorResult& result,
  bool apply_gate = true);

}  // namespace glim_localization
