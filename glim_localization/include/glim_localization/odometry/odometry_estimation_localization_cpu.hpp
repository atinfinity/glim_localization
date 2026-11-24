// Derived from koide3/glim include/glim/odometry/odometry_estimation_cpu.hpp at commit 1a197c999e4865f5cab7c9cd25474128413ebbca (MIT License, Copyright (c) 2024 koide3)
#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>

#include <glim/odometry/odometry_estimation_imu.hpp>
#include <glim/util/callback_slot.hpp>

#include <glim_localization/map/map_manager.hpp>
#include <glim_localization/odometry/map_factor_builder.hpp>
#include <glim_localization/odometry/quality_monitor.hpp>

namespace glim_localization {

/// Why the last pose reset happened. The values are part of the ROS message
/// glim_localization_msgs/LocalizationStatus (RESET_*); the core does not include it.
enum class ResetReason : uint8_t {
  NONE = 0,      ///< No reset yet
  EXTERNAL = 1,  ///< A pose was given from outside (reset())
  IMU_GAP = 2    ///< The odometry reset itself after an IMU gap
};

const char* to_string(ResetReason reason);

struct LocalizationStatus {
  LocalizationState state = LocalizationState::INITIALIZING;
  double stamp = 0.0;
  int frame_id = -1;
  int reset_count = 0;
  int num_tiles = 0;
  int num_map_factors = 0;
  double inlier_ratio = 0.0;  ///< Fraction of points in the finest voxels of the used tiles
  double residual = 0.0;      ///< Finest-level error per inlier point. Reported only
  double processing_time = 0.0;  ///< Wall-clock seconds spent in insert_frame for this frame; 0 on a reset
  ResetReason last_reset_reason = ResetReason::NONE;  ///< NONE while reset_count is 0
};

struct LocalizationCallbacks {
  /// Called on the odometry thread after each estimated frame and each applied reset.
  /// CallbackSlot is not thread-safe, so subscribe before the odometry starts.
  static CallbackSlot<void(const LocalizationStatus&)> on_status;
  /// Forward a pose reset to every live odometry instance. Safe from any thread,
  /// including while an instance is being constructed or destroyed.
  static void request_reset(const Eigen::Isometry3d& T_world_imu, const Eigen::Vector3d& v_world_imu, double min_stamp);
};

struct OdometryEstimationLocalizationCPUParams : public glim::OdometryEstimationIMUParams {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationLocalizationCPUParams();
  ~OdometryEstimationLocalizationCPUParams() override;

  std::string map_path;
  std::vector<double> map_voxel_resolutions;
  MapFactorParams map_factors;
  double map_factor_weight;
  double degraded_weight_scale;
  QualityMonitorParams quality;

  /// What to do when the IMU data has a gap (see imu_gap.hpp). "reset": drop the frames that span the gap,
  /// then reset the pose at the first clean frame from the last pose and velocity (INITIALIZING, reset_count + 1,
  /// reason IMU_GAP); a gap longer than imu_gap_reset_max makes the state LOST until an external reset().
  /// "off": integrate across the gap as upstream does.
  std::string on_imu_gap;
  double max_imu_gap;        ///< Largest allowed gap between consecutive IMU samples [s]
  double imu_gap_reset_max;  ///< Longest gap that is recovered by a self-reset [s]

  /// Also match each scan to a local model and add only a relative pose factor.
  /// This supports estimation outside the map. The key name is shared with the GPU module.
  bool use_keyframes;
  int max_iterations;
  int lru_thresh;
  double target_downsampling_rate;
  double vgicp_resolution;
  int vgicp_voxelmap_levels;
  double vgicp_voxelmap_scaling_factor;
};

/// LiDAR-IMU odometry tightly coupled with unary VGICP factors against a fixed map.
class OdometryEstimationLocalizationCPU : public glim::OdometryEstimationIMU {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Params = OdometryEstimationLocalizationCPUParams;

  /// Load the map at params.map_path.
  /// @throw std::runtime_error if the map cannot be loaded
  explicit OdometryEstimationLocalizationCPU(const Params& params = Params());

  /// Use an already loaded map.
  /// @throw std::invalid_argument if the map is null
  OdometryEstimationLocalizationCPU(const Params& params, std::shared_ptr<const MapManager> map);

  ~OdometryEstimationLocalizationCPU() override;

  void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;

  glim::EstimationFrame::ConstPtr insert_frame(
    const glim::PreprocessedFrame::Ptr& frame,
    std::vector<glim::EstimationFrame::ConstPtr>& marginalized_frames) override;

  /// Restart estimation from the given state at the next insert_frame. Thread-safe.
  /// Frames older than min_stamp are dropped. Frame ids restart from 0, so use
  /// LocalizationStatus::reset_count to tell runs apart. The latest pending request wins.
  /// Frames still in the smoother are discarded without being marginalized to callbacks.
  /// @return false, keeping the current state, if the state is non-finite or not a rigid transform
  bool reset(const Eigen::Isometry3d& T_world_imu, const Eigen::Vector3d& v_world_imu = Eigen::Vector3d::Zero(), double min_stamp = 0.0);

  /// Thread-safe.
  LocalizationStatus status() const;

  /// Stamp of the last frame passed to insert_frame, whatever happened to it: estimated, or dropped while waiting for
  /// the first pose, before min_stamp, in an IMU gap or while LOST. A reset requested now applies at a later frame.
  /// Empty before the first frame. Thread-safe.
  std::optional<double> last_input_frame_stamp() const;

  /// True while a reset (reset(), relocalize() or an IMU gap self-reset) is queued and not yet applied. Thread-safe.
  bool reset_pending() const;

  /// Restart estimation at the next frame from the newest estimate, moved to that frame by its velocity and turned
  /// at the rate between the last two estimates, with reason EXTERNAL. Thread-safe. Refused, keeping the current state, when the state
  /// is not TRACKING or DEGRADED, when a reset is pending (it would silently replace it), or when the newest estimate
  /// is more than max_age older than the last input frame (frames were dropped since, for example in an IMU gap).
  /// The state and age are checked again when the reset is applied: if the odometry is LOST by then, or the frame is
  /// more than max_age after the newest estimate (a stall), the request is dropped with a warning and the frame is
  /// handled as without it (the IMU gap policy and LOST stay in force).
  /// @return an empty string if queued, otherwise why it was refused
  std::string relocalize(double max_age);

  /// Number of IMU stamps kept for the gap check. Bounded: trimmed after every frame, also while LOST (for tests, not thread-safe).
  size_t imu_stamp_history_size() const { return imu_stamps_.size(); }
  const std::deque<double>& imu_stamp_history() const { return imu_stamps_; }
  /// How often the hold check on upstream's queue disagreed with the gap history (0 on any legal stream; for tests and logs)
  size_t hold_check_trips() const { return hold_check_trips_; }

protected:
  gtsam::NonlinearFactorGraph create_factors(
    const int current,
    const gtsam_points::shared_ptr<gtsam::ImuFactor>& imu_factor,
    gtsam::Values& new_values) override;

private:
  struct PendingReset {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Isometry3d T_world_imu;
    Eigen::Vector3d v_world_imu;
    double min_stamp;
    ResetReason reason;
    bool from_current = false;  ///< relocalize(): the pose and velocity are taken from the newest frame when applied
    double max_age = 0.0;       ///< relocalize(): the newest estimate may be at most this much older than the frame
  };

  const Params& localization_params() const;
  bool queue_reset(const Eigen::Isometry3d& T_world_imu, const Eigen::Vector3d& v_world_imu, double min_stamp, ResetReason reason);
  /// @param frame_stamp  the frame the reset applies at
  void apply_pending_reset(double frame_stamp);
  /// @return true if the frame must be dropped because of an IMU gap
  bool handle_imu_gap(const glim::PreprocessedFrame::Ptr& frame);
  void trim_imu_stamps(double keep_from);
  void publish_lost();
  void trim_imu_queue(double stamp);
  void init_local_model();
  gtsam::NonlinearFactorGraph create_local_model_factors(int current, gtsam::Values& new_values);
  void update_local_model(int current, const Eigen::Isometry3d& T_target_imu);
  void update_status(const glim::EstimationFrame::ConstPtr& frame, double processing_time);

  std::shared_ptr<const MapManager> map_;
  QualityMonitor quality_;
  MapFactors current_map_factors_;

  Eigen::Isometry3d last_T_target_imu_ = Eigen::Isometry3d::Identity();
  std::vector<gtsam_points::GaussianVoxelMapCPU::Ptr> target_voxelmaps_;
  std::mt19937 mt_;

  double min_stamp_ = 0.0;

  // IMU gap handling. imu_stamps_ is every IMU stamp since the last sample at or before the last frame,
  // because the upstream queue drops the samples that the last frame consumed.
  std::deque<double> imu_stamps_;
  std::optional<double> newest_frame_stamp_;  ///< The newest frame stamp seen in this IMU epoch; not cleared by a reset, cleared by an IMU fault
  std::optional<double> imu_trimmed_stamp_;  ///< The newest stamp trimmed off the front of imu_stamps_; empty after a fault
  std::optional<double> last_imu_stamp_;  ///< Stamp of the last IMU sample that was forwarded (on_imu_gap: reset)
  int imu_rejected_ = 0;                  ///< Consecutive IMU samples that the gate rejected
  bool gap_pending_ = false;  ///< A frame was dropped for a gap and no clean frame has come since
  double gap_longest_ = 0.0;  ///< Longest gap seen in this episode [s]
  size_t hold_check_trips_ = 0;  ///< Frames dropped as LOST by the hold check
  bool imu_lost_ = false;     ///< The gap was too long to recover; frames are dropped until reset()

  mutable std::mutex reset_mutex_;
  std::unique_ptr<PendingReset> pending_reset_;
  std::optional<double> last_input_frame_stamp_;  ///< Guarded by reset_mutex_

  mutable std::mutex status_mutex_;
  LocalizationStatus status_;
};

}  // namespace glim_localization
