#include "synthetic_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace glim_localization::test {
namespace {

constexpr double kGravity = 9.81;
constexpr double kDiffStep = 1e-3;
constexpr double kStartX = 5.0;
constexpr double kRestTime = 1.0;
constexpr double kAccelTime = 2.0;
constexpr double kCruiseSpeed = 2.0;
constexpr double kWeaveAmplitude = 1.0;
constexpr double kWeaveLength = 10.0;
constexpr double kTiltAmplitude = 2.0 * M_PI / 180.0;

double ramp(double t) {
  const double u = std::clamp((t - kRestTime) / kAccelTime, 0.0, 1.0);
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

// Integral of kCruiseSpeed * ramp(t) from the start of the motion.
double travel(double t) {
  if (t <= kRestTime) {
    return 0.0;
  }
  if (t >= kRestTime + kAccelTime) {
    return kCruiseSpeed * (0.5 * kAccelTime + t - kRestTime - kAccelTime);
  }
  const double u = (t - kRestTime) / kAccelTime;
  return kCruiseSpeed * kAccelTime * u * u * u * u * (2.5 + u * (-3.0 + u));
}

Eigen::Matrix3d rotation_log_input(const Eigen::Isometry3d& lhs, const Eigen::Isometry3d& rhs) {
  return lhs.linear().transpose() * rhs.linear();
}

// Slab test. Returns the nearest positive hit distance along a unit ray.
double intersect(const SceneBox& box, const Eigen::Vector3d& origin, const Eigen::Vector3d& dir) {
  double t_near = 0.0;
  double t_far = std::numeric_limits<double>::infinity();
  for (int axis = 0; axis < 3; axis++) {
    if (std::abs(dir[axis]) < 1e-12) {
      if (origin[axis] < box.min[axis] || origin[axis] > box.max[axis]) {
        return std::numeric_limits<double>::infinity();
      }
      continue;
    }
    double t0 = (box.min[axis] - origin[axis]) / dir[axis];
    double t1 = (box.max[axis] - origin[axis]) / dir[axis];
    if (t0 > t1) {
      std::swap(t0, t1);
    }
    t_near = std::max(t_near, t0);
    t_far = std::min(t_far, t1);
    if (t_near > t_far) {
      return std::numeric_limits<double>::infinity();
    }
  }
  return t_near > 0.0 ? t_near : std::numeric_limits<double>::infinity();
}

}  // namespace

Eigen::Isometry3d synthetic_T_world_imu(double t) {
  const double s = travel(t);
  const double k = 2.0 * M_PI / kWeaveLength;
  const double yaw = std::atan(kWeaveAmplitude * k * std::cos(k * s) * ramp(t));
  const double roll = kTiltAmplitude * ramp(t) * std::sin(0.8 * t);
  const double pitch = kTiltAmplitude * ramp(t) * std::sin(0.5 * t);

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                 .toRotationMatrix();
  T.translation() = Eigen::Vector3d(kStartX + s, kWeaveAmplitude * std::sin(k * s), 1.0);
  return T;
}

Eigen::Vector3d synthetic_v_world_imu(double t) {
  return (synthetic_T_world_imu(t + kDiffStep).translation() - synthetic_T_world_imu(t - kDiffStep).translation()) / (2.0 * kDiffStep);
}

std::vector<SyntheticImuSample> make_synthetic_imu(double t_begin, double t_end, const SyntheticImuParams& params) {
  std::mt19937 rng(params.seed);
  std::normal_distribution<double> unit(0.0, 1.0);
  const auto noise = [&](double sigma) { return Eigen::Vector3d(sigma * unit(rng), sigma * unit(rng), sigma * unit(rng)); };

  std::vector<SyntheticImuSample> samples;
  const int count = static_cast<int>(std::floor((t_end - t_begin) * params.rate + 1e-9)) + 1;
  samples.reserve(count);
  for (int i = 0; i < count; i++) {
    const double t = t_begin + i / params.rate;
    const Eigen::Isometry3d T_prev = synthetic_T_world_imu(t - kDiffStep);
    const Eigen::Isometry3d T_curr = synthetic_T_world_imu(t);
    const Eigen::Isometry3d T_next = synthetic_T_world_imu(t + kDiffStep);

    const Eigen::Vector3d a_world =
      (T_next.translation() - 2.0 * T_curr.translation() + T_prev.translation()) / (kDiffStep * kDiffStep);
    const Eigen::AngleAxisd delta(rotation_log_input(T_prev, T_next));

    SyntheticImuSample sample;
    sample.stamp = t;
    sample.linear_acc = T_curr.linear().transpose() * (a_world + Eigen::Vector3d(0.0, 0.0, kGravity)) + params.acc_bias + noise(params.acc_noise);
    sample.angular_vel = delta.axis() * delta.angle() / (2.0 * kDiffStep) + params.gyro_bias + noise(params.gyro_noise);
    samples.push_back(sample);
  }
  return samples;
}

std::vector<glim::RawPoints::Ptr> make_synthetic_scans(
  const std::vector<SceneBox>& scene,
  double t_begin,
  double t_end,
  const SyntheticLidarParams& params) {
  std::mt19937 rng(params.seed);
  std::normal_distribution<double> range_noise(0.0, params.range_noise);

  std::vector<glim::RawPoints::Ptr> scans;
  const int count = static_cast<int>(std::floor((t_end - t_begin) * params.rate + 1e-9)) + 1;
  scans.reserve(count);
  for (int i = 0; i < count; i++) {
    const double t = t_begin + i / params.rate;

    auto scan = std::make_shared<glim::RawPoints>();
    scan->stamp = t;
    for (int ring = 0; ring < params.num_rings; ring++) {
      const double elevation = -params.max_elevation + 2.0 * params.max_elevation * ring / (params.num_rings - 1);
      for (int az = 0; az < params.num_azimuths; az++) {
        const double offset = params.scan_duration * az / params.num_azimuths;
        const Eigen::Isometry3d T_world_imu = synthetic_T_world_imu(t + offset);  // The pose when this point is observed
        const Eigen::Vector3d origin = T_world_imu.translation();
        const double azimuth = 2.0 * M_PI * az / params.num_azimuths;
        const Eigen::Vector3d dir_imu(std::cos(elevation) * std::cos(azimuth), std::cos(elevation) * std::sin(azimuth), std::sin(elevation));
        const Eigen::Vector3d dir_world = T_world_imu.linear() * dir_imu;

        double range = std::numeric_limits<double>::infinity();
        for (const auto& box : scene) {
          range = std::min(range, intersect(box, origin, dir_world));
        }
        if (range < params.min_range || range > params.max_range) {
          continue;
        }
        const Eigen::Vector3d p = dir_imu * (range + range_noise(rng));
        scan->points.emplace_back(p.x(), p.y(), p.z(), 1.0);
        scan->times.push_back(offset);
      }
    }
    scans.push_back(scan);
  }
  return scans;
}

}  // namespace glim_localization::test
