#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <glim/util/raw_points.hpp>

namespace glim_localization {

/// Replay dataset format (version 1):
///   meta.json                      required
///   imu.csv                        required, header `stamp_ns,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z`
///   points/<stamp_ns, 19 digits>.bin   one file per LiDAR frame
///   traj_gt.txt                    optional, TUM format, T_world_imu
///   wheel_odom.csv                 optional, header `stamp_ns,vx,vy,vz,wx,wy,wz`: the twist of the wheel base
///                                  in the base frame (wheel_odom_T_imu_base). Required when meta.json has num_wheel_odom
/// See docs/DESIGN.md for the full specification.

constexpr uint32_t kReplayCloudVersion = 1;
constexpr uint32_t kReplayCloudHeaderSize = 32;
constexpr uint32_t kReplayPointStep = 20;
constexpr uint32_t kReplayFlagPointTime = 1u << 0;
constexpr uint32_t kReplayFlagIntensity = 1u << 1;

/// Thrown for malformed dataset content. The message starts with the offending path.
class InvalidDatasetError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

struct ReplayPoint {
  float x = 0.0f, y = 0.0f, z = 0.0f;  ///< LiDAR frame [m]
  float t = 0.0f;                      ///< Seconds since the frame stamp, in [0, 1)
  float intensity = 0.0f;
};

/// Specific force [m/s^2] and angular rate [rad/s] in the IMU frame.
struct ReplayImu {
  int64_t stamp_ns = 0;
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
};

/// Twist of the wheel base [m/s, rad/s], in the base frame (the base of wheel_odom_T_imu_base).
struct ReplayWheelOdom {
  int64_t stamp_ns = 0;
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();
};

struct ReplayMeta {
  bool per_point_time = false;
  size_t num_frames = 0;
  size_t num_imu = 0;
  bool has_num_wheel_odom = false;  ///< meta.json has num_wheel_odom, so wheel_odom.csv is required
  size_t num_wheel_odom = 0;
  std::string conversion_json = "{}";  ///< Free-form JSON object describing the conversion
};

struct ReplayFrameInfo {
  int64_t stamp_ns = 0;
  std::string path;
  uint32_t num_points = 0;
  uint32_t flags = 0;
};

/// Format a nanosecond stamp as `seconds.nanoseconds` ("%d.%09d").
std::string format_stamp_ns(int64_t stamp_ns);

/// Writes a dataset. `finish` writes meta.json from the counts written so far.
class ReplayDatasetWriter {
public:
  /// @throw std::runtime_error if the directory cannot be created
  explicit ReplayDatasetWriter(const std::string& dir);

  /// Point values are written as given (the reader checks them), except that `t` and `intensity` are zeroed when their flag is clear.
  /// @throw std::invalid_argument if the cloud is empty
  void write_cloud(int64_t stamp_ns, const std::vector<ReplayPoint>& points, uint32_t flags);

  /// Replace imu.csv. Stamps must be strictly increasing.
  void write_imu(const std::vector<ReplayImu>& imu);

  /// Write wheel_odom.csv; `finish` then records num_wheel_odom. Stamps must be strictly increasing.
  void write_wheel_odom(const std::vector<ReplayWheelOdom>& wheel_odom);

  void finish(bool per_point_time, const std::string& conversion_json = "{}");

private:
  std::string dir_;
  size_t num_frames_ = 0;
  size_t num_imu_ = 0;
  bool wrote_wheel_odom_ = false;
  size_t num_wheel_odom_ = 0;
};

/// A validated dataset. `open` checks meta.json, every imu.csv row, and every .bin header
/// and file size, so that errors surface before any expensive work. Point bodies are
/// checked in `load_cloud`.
class ReplayDataset {
public:
  /// @throw InvalidDatasetError on any malformed content, including a missing required file
  static ReplayDataset open(const std::string& dir);

  const std::string& dir() const { return dir_; }
  const ReplayMeta& meta() const { return meta_; }
  const std::vector<ReplayImu>& imu() const { return imu_; }
  /// Empty if the dataset has no wheel_odom.csv.
  const std::vector<ReplayWheelOdom>& wheel_odom() const { return wheel_odom_; }
  bool has_wheel_odom() const { return has_wheel_odom_; }
  /// Sorted by stamp.
  const std::vector<ReplayFrameInfo>& frames() const { return frames_; }
  bool has_ground_truth() const;
  std::string ground_truth_path() const;

  /// @throw InvalidDatasetError if the body has non-finite values or invalid times
  glim::RawPoints::Ptr load_cloud(size_t index) const;

private:
  std::string dir_;
  ReplayMeta meta_;
  std::vector<ReplayImu> imu_;
  bool has_wheel_odom_ = false;
  std::vector<ReplayWheelOdom> wheel_odom_;
  std::vector<ReplayFrameInfo> frames_;
};

}  // namespace glim_localization
