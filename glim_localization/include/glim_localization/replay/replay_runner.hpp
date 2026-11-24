// Derived from koide3/glim src/glim/odometry/async_odometry_estimation.cpp at commit 1a197c999e4865f5cab7c9cd25474128413ebbca (MIT License, Copyright (c) 2024 koide3)
#pragma once

#include <cstdint>
#include <fstream>
#include <functional>
#include <map>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/preprocess/cloud_preprocessor.hpp>

#include <glim_localization/odometry/imu_gap.hpp>
#include <glim_localization/odometry/odometry_estimation_localization_cpu.hpp>
#include <glim_localization/replay/replay_dataset.hpp>

namespace glim_localization {

enum class ReplayResult {
  ESTIMATED,        ///< insert_frame returned a pose
  DROPPED,          ///< No pose: waiting for an initial pose, before min_stamp, or no points left after preprocessing
  DIVERGED,         ///< A pose came back, but it or its velocity or bias is non-finite, or |v| > kMaxPlausibleSpeed; run() throws after this row
  SKIPPED_NO_IMU    ///< The IMU data does not cover [stamp, scan_end_time] (leading hole or the end of the IMU data); the frame was not inserted
};

const char* to_string(ReplayResult result);

/// Speed [m/s] above which an estimate is treated as a numerical blow-up. A guard only: finite but wrong poses are for evaluate.py.
constexpr double kMaxPlausibleSpeed = 1000.0;

/// True if the pose, velocity and IMU bias are finite and the speed is below kMaxPlausibleSpeed
bool estimate_is_sane(const glim::EstimationFrame& frame);

struct ReplayOptions {
  /// Seconds on the dataset time axis. Frames outside [start, end] are ignored entirely.
  std::optional<double> start;
  std::optional<double> end;
  int64_t max_frames = -1;  ///< Negative means no limit; counted after the start/end filter
  /// Largest allowed gap between consecutive IMU samples [s]. The upstream IMU integration holds one sample across
  /// a gap, which corrupts the estimate; 0.1 s (exactly 10 Hz) was measured to diverge, 0.05 s accepts 25 Hz and above.
  /// A frame whose IMU window [last estimated frame, scan_end_time] has a larger gap, or fewer than 2 samples since the
  /// last estimated frame, makes run() throw InvalidDatasetError. Before the first pose, a hole only skips the frame.
  double max_imu_gap = 0.05;

  /// What to do when an IMU gap is found after the first pose (see imu_gap.hpp).
  enum class OnImuGap {
    ABORT,  ///< run() throws InvalidDatasetError before the odometry sees the gap
    RESET   ///< Skip the frames that span the gap (skipped_no_imu); the odometry resets itself at the first clean frame
  };
  OnImuGap on_imu_gap = OnImuGap::ABORT;
  /// With OnImuGap::RESET, a gap longer than this still throws InvalidDatasetError [s]. Keep it equal to the odometry's imu_gap_reset_max.
  double imu_gap_reset_max = 0.5;
  /// The initial pose applies at the first frame at or after initial_reset->min_stamp that the IMU covers. If that
  /// frame is more than this much later, run() throws std::invalid_argument [s]. Does not apply to min_stamp 0 (unstamped).
  double max_init_pose_delay = 0.2;

  struct InitialReset {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
    Eigen::Vector3d v_world_imu = Eigen::Vector3d::Zero();
    double min_stamp = 0.0;  ///< Frames before this are dropped
  };
  /// Applied once before the first frame.
  std::optional<InitialReset> initial_reset;
};

struct ReplayFrameRecord {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int64_t stamp_ns = 0;
  ReplayResult result = ReplayResult::DROPPED;
  int frame_id = -1;  ///< -1 unless estimated
  LocalizationState state = LocalizationState::INITIALIZING;
  int reset_count = 0;
  int num_points_raw = 0;
  int num_points_preprocessed = 0;
  int num_imu_fed = 0;
  double t_load_ms = 0.0;
  double t_preprocess_ms = 0.0;
  double t_odometry_ms = 0.0;  ///< insert_frame only
  double inlier_ratio = 0.0;
  double residual = 0.0;
  int num_map_factors = 0;
  int num_tiles = 0;
  std::optional<Eigen::Isometry3d> T_world_imu;  ///< Set when estimated
};

struct ReplaySummary {
  int num_frames = 0;
  std::map<std::string, int> frames_by_result;
  std::map<std::string, int> estimated_by_state;
  double wall_seconds = 0.0;
  int imu_gap_resets = 0;  ///< Resets the odometry made after an IMU gap (OnImuGap::RESET)
  /// Set by the replay tool, not by ReplayRunner
  std::optional<double> tracking_ratio;      ///< TRACKING share of the estimated frames from the first TRACKING frame on
  std::optional<double> min_tracking_ratio;  ///< The gate that tracking_ratio was checked against
};

/// Feeds a dataset to an odometry instance from one thread, in a fixed order, so that a run is repeatable.
/// For each frame (ascending stamp): load, preprocess, insert every IMU sample with stamp <= scan_end_time plus
/// the next one, then insert_frame. The extra sample matters because the upstream IMU integration integrates
/// the tail of the interval with the first sample after it. Frames that the IMU data does not cover are skipped.
class ReplayRunner {
public:
  ReplayRunner(
    const ReplayDataset& dataset,
    OdometryEstimationLocalizationCPU& odometry,
    glim::CloudPreprocessor& preprocessor,
    const ReplayOptions& options = ReplayOptions());

  /// @param on_frame Called after every frame, in order
  /// @throw std::invalid_argument if the initial reset is rejected
  /// @throw std::runtime_error if an estimate is not sane (the row is reported with result diverged first)
  /// @throw InvalidDatasetError if a point cloud body is malformed, or the IMU has a gap after the first pose (OnImuGap::ABORT,
  ///        or a gap above imu_gap_reset_max)
  /// @throw std::invalid_argument if the initial pose would apply more than max_init_pose_delay late
  ReplaySummary run(const std::function<void(const ReplayFrameRecord&)>& on_frame = nullptr);

private:
  const ReplayDataset& dataset_;
  OdometryEstimationLocalizationCPU& odometry_;
  glim::CloudPreprocessor& preprocessor_;
  ReplayOptions options_;
};

/// Share of frames in TRACKING among the estimated frames from the first TRACKING frame on. A reset that sends the
/// state back to INITIALIZING, and DEGRADED or LOST frames, lower it. Frames before the first TRACKING frame are the
/// start-up and do not count.
class TrackingRatio {
public:
  void add(const ReplayFrameRecord& record) {
    if (record.result != ReplayResult::ESTIMATED) {
      return;
    }
    started_ = started_ || record.state == LocalizationState::TRACKING;
    if (started_) {
      total_++;
      tracking_ += record.state == LocalizationState::TRACKING ? 1 : 0;
    }
  }
  /// 0 if no frame was ever TRACKING
  double ratio() const { return total_ > 0 ? static_cast<double>(tracking_) / total_ : 0.0; }
  /// True if ratio() >= min_ratio. Always true for min_ratio 0, which disables the gate.
  bool passes(double min_ratio) const { return min_ratio <= 0.0 || ratio() >= min_ratio; }

private:
  bool started_ = false;
  int total_ = 0;
  int tracking_ = 0;
};

/// Writes the outputs of a run under one directory, flushing after every frame so that
/// an aborted run keeps its partial results.
///   traj_localization.txt  TUM, T_world_imu, estimated frames only
///   frames.csv             one row per processed frame
///   summary.json           only after a complete run
class ReplayOutputWriter {
public:
  /// Removes a summary.json left by an earlier run, so that an aborted run never looks complete.
  /// @throw std::runtime_error if the files cannot be created
  explicit ReplayOutputWriter(const std::string& out_dir);

  /// @throw std::runtime_error on a write failure
  void write(const ReplayFrameRecord& record);

  /// @throw std::runtime_error on a write failure
  void write_summary(const ReplaySummary& summary, double map_load_seconds);

  int num_poses() const { return num_poses_; }

private:
  std::string out_dir_;
  std::ofstream traj_;
  std::ofstream frames_;
  int num_poses_ = 0;
};

}  // namespace glim_localization
