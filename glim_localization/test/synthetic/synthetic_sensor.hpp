#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/util/raw_points.hpp>

#include "synthetic_scene.hpp"

namespace glim_localization::test {

struct SyntheticImuSample {
  double stamp = 0.0;
  Eigen::Vector3d linear_acc = Eigen::Vector3d::Zero();   // Specific force in the IMU frame [m/s^2]
  Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();  // Body rate in the IMU frame [rad/s]
};

struct SyntheticImuParams {
  double rate = 100.0;
  double acc_noise = 0.05;
  double gyro_noise = 0.005;
  Eigen::Vector3d acc_bias = Eigen::Vector3d(0.02, -0.01, 0.015);
  Eigen::Vector3d gyro_bias = Eigen::Vector3d(0.002, -0.001, 0.0015);
  unsigned int seed = 7;
};

struct SyntheticWheelSample {
  double stamp = 0.0;
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();   // Velocity of the base frame in the base frame [m/s]
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();  // Body rate of the base frame in the base frame [rad/s]
};

struct SyntheticWheelParams {
  double rate = 50.0;
  Eigen::Isometry3d T_imu_base = Eigen::Isometry3d::Identity();  // The wheel base in the IMU frame
  double linear_noise = 0.0;   // Gaussian noise on each axis [m/s]
  double angular_noise = 0.0;  // Gaussian noise on each axis [rad/s]
  unsigned int seed = 13;
};

struct SyntheticLidarParams {
  double rate = 10.0;
  int num_rings = 24;
  int num_azimuths = 240;
  double max_elevation = 30.0 * M_PI / 180.0;
  double min_range = 1.0;
  double max_range = 30.0;
  double range_noise = 0.02;
  unsigned int seed = 11;
  /// Duration of one sweep [s]. 0 makes the scan instantaneous (every point at the scan stamp). Otherwise azimuth
  /// column k is observed at stamp + scan_duration * k / num_azimuths, along the true trajectory.
  double scan_duration = 0.0;
};

/// Analytic trajectory: 1 s at rest, then about 20 m along x with a weave, ending at 12 s.
Eigen::Isometry3d synthetic_T_world_imu(double t);

/// World-frame velocity of the analytic trajectory (central difference).
Eigen::Vector3d synthetic_v_world_imu(double t);

/// IMU samples in [t_begin, t_end]. Gravity is +9.81 along world z in the specific force.
std::vector<SyntheticImuSample> make_synthetic_imu(double t_begin, double t_end, const SyntheticImuParams& params = SyntheticImuParams());

/// Wheel odometry twists in [t_begin, t_end]: the twist of the base frame (T_world_imu * T_imu_base) along the analytic
/// trajectory, in the base frame, plus the configured noise.
std::vector<SyntheticWheelSample> make_synthetic_wheel(double t_begin, double t_end, const SyntheticWheelParams& params = SyntheticWheelParams());

/// Scans at the sensor rate in [t_begin, t_end]. Each point is expressed in the IMU frame at its own observation time, as
/// a real rolling-shutter sensor reports it (not deskewed), and `times` holds its offset from the scan stamp.
std::vector<glim::RawPoints::Ptr> make_synthetic_scans(
  const std::vector<SceneBox>& scene,
  double t_begin,
  double t_end,
  const SyntheticLidarParams& params = SyntheticLidarParams());

}  // namespace glim_localization::test
