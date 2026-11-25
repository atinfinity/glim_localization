#include <glim_localization/odometry/wheel_odometry.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>

namespace glim_localization {

namespace {

bool is_rigid(const Eigen::Isometry3d& T) {
  if (!T.matrix().allFinite()) {
    return false;
  }
  const Eigen::Matrix3d R = T.linear();
  return (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < 1e-3 && R.determinant() > 0.0;
}

bool non_negative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

}  // namespace

void WheelOdometryParams::validate() const {
  if (!is_rigid(T_imu_base)) {
    throw std::invalid_argument("wheel_odom_T_imu_base must be a rigid transform");
  }
  if (!(std::isfinite(max_gap) && max_gap > 0.0)) {
    throw std::invalid_argument("wheel_odom_max_gap must be positive");
  }
  if (!(std::isfinite(translation_sigma_min) && translation_sigma_min > 0.0) || !(std::isfinite(rotation_sigma_min) && rotation_sigma_min > 0.0)) {
    throw std::invalid_argument("wheel_odom_translation_sigma_min and wheel_odom_rotation_sigma_min must be positive");
  }
  if (!non_negative(translation_sigma_ratio) || !non_negative(rotation_sigma_ratio) || !non_negative(huber_threshold) || !non_negative(gate_velocity) ||
      !non_negative(gate_velocity_ratio) || !non_negative(gate_angular_velocity) || gate_readmit_frames < 0) {
    throw std::invalid_argument("the wheel_odom sigma ratios, huber threshold and gates must not be negative");
  }
}

WheelOdometryBuffer::WheelOdometryBuffer(const WheelOdometryParams& params) : params_(params) {}

bool WheelOdometryBuffer::insert(double stamp, const Eigen::Vector3d& linear, const Eigen::Vector3d& angular) {
  if (!std::isfinite(stamp) || !linear.allFinite() || !angular.allFinite()) {
    return false;
  }
  WheelOdometrySample sample;
  sample.stamp = stamp;
  sample.linear = linear;
  sample.angular = angular;
  if (params_.nonholonomic) {
    sample.linear.y() = 0.0;
    sample.linear.z() = 0.0;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!samples_.empty() && stamp <= samples_.back().stamp) {
    if (++rejected_ < kRestartRejects) {
      return false;
    }
    // Not a stray sample: the clock went back (a bag loop, a simulation reset). Start over from this sample.
    samples_.clear();
  }
  rejected_ = 0;
  samples_.push_back(sample);
  while (samples_.size() > kMaxSamples) {
    samples_.pop_front();
  }
  return true;
}

WheelOdometrySample WheelOdometryBuffer::interpolate(double t) const {
  if (t <= samples_.front().stamp) {
    return samples_.front();
  }
  if (t >= samples_.back().stamp) {
    return samples_.back();
  }
  const auto next = std::upper_bound(samples_.begin(), samples_.end(), t, [](double lhs, const WheelOdometrySample& rhs) { return lhs < rhs.stamp; });
  const auto& b = *next;
  const auto& a = *(next - 1);
  const double w = (t - a.stamp) / (b.stamp - a.stamp);
  WheelOdometrySample sample;
  sample.stamp = t;
  sample.linear = (1.0 - w) * a.linear + w * b.linear;
  sample.angular = (1.0 - w) * a.angular + w * b.angular;
  return sample;
}

std::optional<Eigen::Isometry3d> WheelOdometryBuffer::integrate(double t0, double t1) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (samples_.empty() || !(t1 > t0)) {
    return std::nullopt;
  }
  const double tol = params_.max_gap + 1e-9;
  if (samples_.front().stamp > t0 + tol || samples_.back().stamp < t1 - tol) {
    return std::nullopt;
  }

  // Breakpoints: t0, every sample strictly inside (t0, t1), t1. Between two samples the twist is linear, so the twist
  // at the middle of a segment is its mean.
  std::vector<double> breaks = {t0};
  for (size_t i = 0; i < samples_.size(); i++) {
    if (i > 0 && samples_[i].stamp > t0 && samples_[i - 1].stamp < t1 && samples_[i].stamp - samples_[i - 1].stamp > tol) {
      return std::nullopt;  // A hole inside the interval
    }
    if (samples_[i].stamp > t0 && samples_[i].stamp < t1) {
      breaks.push_back(samples_[i].stamp);
    }
  }
  breaks.push_back(t1);

  gtsam::Pose3 T;
  for (size_t i = 0; i + 1 < breaks.size(); i++) {
    const double dt = breaks[i + 1] - breaks[i];
    if (dt <= 0.0) {
      continue;
    }
    const WheelOdometrySample mid = interpolate(0.5 * (breaks[i] + breaks[i + 1]));
    gtsam::Vector6 xi;
    xi << mid.angular * dt, mid.linear * dt;  // Pose3 tangent order: rotation, then translation (body frame)
    T = T * gtsam::Pose3::Expmap(xi);
  }
  return Eigen::Isometry3d(T.matrix());
}

void WheelOdometryBuffer::trim(double keep_from) {
  std::lock_guard<std::mutex> lock(mutex_);
  while (samples_.size() >= 2 && samples_[1].stamp <= keep_from) {
    samples_.pop_front();
  }
}

void WheelOdometryBuffer::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.clear();
  rejected_ = 0;
}

size_t WheelOdometryBuffer::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.size();
}

gtsam::NonlinearFactorGraph create_wheel_factor(
  const WheelOdometryParams& params,
  gtsam::Key key_last,
  gtsam::Key key_current,
  double dt,
  const Eigen::Isometry3d& T_base,
  const Eigen::Isometry3d& T_last_current_imu,
  WheelFactorResult& result,
  bool apply_gate) {
  // The same rigid motion seen from the IMU: T_imu(t0)_imu(t1) = T_imu_base * T_base * T_base_imu
  Eigen::Isometry3d T_imu = params.T_imu_base * T_base * params.T_imu_base.inverse();
  T_imu.linear() = Eigen::Quaterniond(T_imu.linear()).normalized().toRotationMatrix();

  if (apply_gate && dt > 0.0) {
    // The vector difference of the two translations, so a wrong direction counts as well as a wrong speed
    const double velocity_diff = (T_imu.translation() - T_last_current_imu.translation()).norm() / dt;
    const double speed = std::max(T_imu.translation().norm(), T_last_current_imu.translation().norm()) / dt;
    const double velocity_gate = std::max(params.gate_velocity, params.gate_velocity_ratio * speed);
    const double angular_diff = Eigen::AngleAxisd(T_imu.linear().transpose() * T_last_current_imu.linear()).angle() / dt;
    if ((velocity_gate > 0.0 && velocity_diff > velocity_gate) || (params.gate_angular_velocity > 0.0 && angular_diff > params.gate_angular_velocity)) {
      result = WheelFactorResult::GATED;
      return gtsam::NonlinearFactorGraph();
    }
  }

  const double distance = T_imu.translation().norm();
  const double angle = Eigen::AngleAxisd(T_imu.linear()).angle();
  const double sigma_t = params.translation_sigma_min + params.translation_sigma_ratio * distance;
  const double sigma_r = params.rotation_sigma_min + params.rotation_sigma_ratio * angle;
  gtsam::Vector6 sigmas;
  sigmas << gtsam::Vector3::Constant(sigma_r), gtsam::Vector3::Constant(sigma_t);
  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  if (params.huber_threshold > 0.0) {
    noise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(params.huber_threshold), noise);
  }

  gtsam::NonlinearFactorGraph factors;
  factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(key_last, key_current, gtsam::Pose3(T_imu.matrix()), noise);
  result = WheelFactorResult::ADDED;
  return factors;
}

}  // namespace glim_localization
