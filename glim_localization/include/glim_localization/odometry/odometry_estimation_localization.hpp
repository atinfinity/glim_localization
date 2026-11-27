// Derived from koide3/glim include/glim/odometry/odometry_estimation_cpu.hpp at commit 1a197c999e4865f5cab7c9cd25474128413ebbca (MIT License, Copyright (c) 2024 koide3)
#pragma once

#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <glim/odometry/odometry_estimation_imu.hpp>
#include <glim/util/callback_slot.hpp>

#include <glim_localization/map/map_manager.hpp>
#include <glim_localization/odometry/map_factor_builder.hpp>
#include <glim_localization/odometry/quality_monitor.hpp>
#include <glim_localization/odometry/wheel_odometry.hpp>
#include <glim_localization/relocalization/pose_candidates.hpp>

namespace glim_localization {

/// Why the last pose reset happened. The values are part of the ROS message
/// glim_localization_msgs/LocalizationStatus (RESET_*); the core does not include it.
enum class ResetReason : uint8_t {
  NONE = 0,      ///< No reset yet
  EXTERNAL = 1,  ///< A pose was given from outside (reset())
  IMU_GAP = 2    ///< The odometry reset itself after an IMU gap
};

/// How the last IMU gap self-reset extrapolated the pose (wheel_odom_reset_extrapolation). Every value but WHEEL used
/// the constant velocity with the orientation held; the others than NONE name why the wheels were not used.
enum class ResetExtrapolation : uint8_t {
  NONE = 0,          ///< Not an IMU gap reset, or the wheels were not tried (key off or wheel odometry disabled)
  WHEEL = 1,         ///< The wheel motion and twist
  NO_COVERAGE = 2,   ///< The wheel samples do not cover the gap or its ends within wheel_odom_max_gap
  INVALID = 3,       ///< The wheel state is not a valid reset (non-finite, or faster than kMaxResetSpeed)
  SLIP_START = 4,    ///< The wheel velocity at the last estimate disagrees with the estimated one
  SLIP_ANGULAR = 5,  ///< The wheel angular velocity at the last estimate disagrees with the bias-corrected gyro
  SLIP_GAP = 6       ///< The wheel displacement over the gap disagrees with the velocities at both ends
};

const char* to_string(ResetReason reason);

/// Outcome of the yaw search on the first frame after a given pose. The values are part of the ROS message
/// glim_localization_msgs/LocalizationStatus (INIT_SEARCH_*).
enum class InitSearchResult : uint8_t {
  NONE = 0,      ///< No search since the last reset (disabled, not run yet, or the pose was not given from outside)
  ACCEPTED = 1,  ///< The best candidate passed and estimation started from it
  REJECTED = 2   ///< Truncated by the time budget, below the minimum score or ambiguous: LOST until another pose is given
};

const char* to_string(InitSearchResult result);

/// Why the yaw search rejected the given pose. The values are part of the ROS message
/// glim_localization_msgs/LocalizationStatus (INIT_SEARCH_REASON_*).
enum class InitSearchReason : uint8_t {
  NONE = 0,          ///< Not searched, or accepted
  TRUNCATED = 1,     ///< The time budget ran out before every candidate was scored
  LOW_SCORE = 2,     ///< The best inlier ratio is below init_yaw_search_min_inlier_ratio
  AMBIGUOUS = 3,     ///< The runner-up is within init_yaw_search_min_score_gap of the best
  NO_CANDIDATE = 4,  ///< The candidate generator returned nothing to score
  OUT_OF_RANGE = 5   ///< A registered pose beyond init_yaw_search_max_translation scores at least the best within it
};

const char* to_string(InitSearchReason reason);

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

  // Yaw search of the current run (init_yaw_search). Cleared by a reset, set when the search runs on the first frame.
  InitSearchResult init_search = InitSearchResult::NONE;
  InitSearchReason init_search_reason = InitSearchReason::NONE;  ///< Set when init_search is REJECTED
  int init_search_candidates = 0;       ///< Candidates scored; fewer than init_search_candidates_total means truncated
  int init_search_candidates_total = 0;  ///< Candidates generated
  double init_search_score = 0.0;        ///< Inlier ratio of the best candidate, also when it was rejected
  double init_search_yaw_offset = 0.0;   ///< Yaw of the best registered pose relative to the given pose [rad]
  /// Inlier ratio of the best candidate whose registered yaw is more than distinct_yaw_for_step(step) (two steps, at
  /// most 90 deg) from the best one (0 if none)
  double init_search_runner_up_score = 0.0;
  double init_search_runner_up_yaw_offset = 0.0;  ///< Its yaw relative to the given pose [rad]
  double init_search_time = 0.0;                  ///< Wall-clock seconds of the search

  // Wheel odometry (wheel_odom_enabled), counted per frame since the last reset. All 0 while it is disabled.
  int wheel_factors = 0;  ///< Frames that got a wheel relative pose factor
  int wheel_gated = 0;    ///< Frames whose wheel motion the consistency gate rejected (slip, a stuck wheel)
  int wheel_readmitted = 0;  ///< Of wheel_factors, those added without the gate after gate_readmit_frames gated frames
  int wheel_missing = 0;  ///< Frames whose interval the wheel samples did not cover
};

struct LocalizationCallbacks {
  /// Called on the odometry thread after each estimated frame and each applied reset.
  /// CallbackSlot is not thread-safe, so subscribe before the odometry starts.
  static CallbackSlot<void(const LocalizationStatus&)> on_status;
  /// Forward a pose reset to every live odometry instance. Safe from any thread,
  /// including while an instance is being constructed or destroyed.
  static void request_reset(const Eigen::Isometry3d& T_world_imu, const Eigen::Vector3d& v_world_imu, double min_stamp);
  /// The map of the oldest live odometry instance, or null if there is none. Safe from any thread. Lets a module that
  /// does not own the odometry (the R1 extension, loaded after it by glim_ros2) publish the map without loading it again.
  static std::shared_ptr<const MapManager> loaded_map();
  /// Forward a wheel odometry twist to every live odometry instance (insert_wheel_odom). Safe from any thread. Lets a
  /// module that does not own the odometry (the R1 extension) feed it.
  /// @return true if at least one instance took the sample
  static bool insert_wheel_odom(double stamp, const Eigen::Vector3d& linear_vel, const Eigen::Vector3d& angular_vel);
  /// true if a live odometry instance has wheel odometry enabled (wheel_odom_enabled). Safe from any thread.
  static bool wheel_odom_enabled();
};

/// Parameters shared by the CPU and GPU modules: the same keys with the same meaning in both configs.
struct OdometryEstimationLocalizationParams : public glim::OdometryEstimationIMUParams {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationLocalizationParams();
  ~OdometryEstimationLocalizationParams() override;

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

  /// init_v_world_imu was configured without init_T_world_imu. Upstream then starts at the identity pose in the map,
  /// so the constructor rejects it.
  bool init_v_without_pose = false;

  /// Also match each scan to earlier scans, for estimation outside the map. The CPU module matches a local model and
  /// adds a relative pose factor; the GPU module adds upstream's keyframe factors.
  bool use_keyframes;

  /// Yaw search on the first frame after a pose given from outside (the configured pose, reset()): score yaw
  /// candidates around the given pose by scan-to-map registration and start from the best one. Not applied to
  /// relocalize() or an IMU gap self-reset, which start from the estimate. The pose is rejected (LOST) if the time
  /// budget runs out before every candidate is scored, if the best inlier ratio is below min_inlier_ratio, or if a
  /// candidate more than two steps (at most 90 deg) away in yaw scores within min_score_gap of the best. With a range
  /// of 0 or one too small to reach that far, only the inlier ratio threshold applies. With max_translation, only
  /// registered poses within that distance of the given pose (xy) are accepted, and the pose is rejected if one beyond
  /// it reaches min_inlier_ratio and scores at least as high as the best within it.
  struct InitYawSearch {
    bool enabled = false;
    double range = M_PI;                ///< Half width [rad] (config: init_yaw_search_range_deg)
    double step = 15.0 * M_PI / 180.0;  ///< [rad], at least 0.5 deg (config: init_yaw_search_step_deg)
    double min_inlier_ratio = 0.7;      ///< The best candidate must reach this inlier ratio
    double min_score_gap = 0.05;        ///< The best must beat the runner-up at a different yaw by more than this
    double time_budget = 10.0;          ///< Every candidate must be scored within this many seconds [s]
    int max_iterations = 10;            ///< Registration iterations per candidate
    int min_points = 100;               ///< Smaller frames are dropped while the search waits for its first frame
    double max_translation = 0.0;       ///< Position uncertainty of the given pose [m] in xy; 0 is unbounded
  } init_yaw_search;

  /// Optional wheel odometry: a relative pose factor between consecutive frames from the twists given to
  /// insert_wheel_odom(). Off by default.
  WheelOdometryParams wheel_odom;
};

/// Load the map at params.map_path with params.map_voxel_resolutions.
/// @throw std::runtime_error if the map cannot be loaded
std::shared_ptr<const MapManager> load_localization_map(const OdometryEstimationLocalizationParams& params);

/// LiDAR-IMU odometry tightly coupled with unary VGICP factors against a fixed map. The backends (CPU, GPU) create
/// the frame data and the factors; this class holds the reset, IMU gap and status logic they share.
class OdometryEstimationLocalization : public glim::OdometryEstimationIMU {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Params = OdometryEstimationLocalizationParams;

  ~OdometryEstimationLocalization() override;

  /// "cpu" or "gpu"
  virtual const char* backend_name() const = 0;

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

  /// Add a wheel odometry twist of the base frame (wheel_odom_T_imu_base) at the given stamp, in the base frame.
  /// Thread-safe. Samples are associated with frames by time, so they may arrive before or shortly after the frame.
  /// @return false if wheel odometry is disabled or the sample was dropped (non-finite, or not newer than the last one;
  /// a run of WheelOdometryBuffer::kRestartRejects such samples is taken as a clock restart). A reset clears the samples.
  bool insert_wheel_odom(double stamp, const Eigen::Vector3d& linear_vel, const Eigen::Vector3d& angular_vel);
  /// wheel_odom_enabled of the config. Immutable after construction.
  bool wheel_odom_enabled() const;

  /// Thread-safe.
  LocalizationStatus status() const;

  /// The map this instance localizes against (never null). Thread-safe.
  const std::shared_ptr<const MapManager>& map() const { return map_; }

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

  /// Largest speed of a reset [m/s]: reset(), the configured init_v_world_imu, and the estimate that relocalize() and
  /// the IMU gap self-reset start from (beyond it, the self-reset goes LOST). A finite but huge velocity makes the
  /// smoother throw IndeterminantLinearSystemException and abort the process.
  static constexpr double kMaxResetSpeed = 100.0;

  /// IMU bias the current run started from: the configured imu_bias, then at every reset the newest estimate of the
  /// run before, or the bias that run started from if it estimated no frame. Not thread-safe (for tests and logs).
  const Eigen::Matrix<double, 6, 1>& start_imu_bias() const { return start_imu_bias_; }

  /// Number of IMU stamps kept for the gap check. Bounded: trimmed after every frame, also while LOST (for tests, not thread-safe).
  size_t imu_stamp_history_size() const { return imu_stamps_.size(); }
  const std::deque<double>& imu_stamp_history() const { return imu_stamps_; }
  /// How often the hold check on upstream's queue disagreed with the gap history (0 on any legal stream; for tests and logs)
  size_t hold_check_trips() const { return hold_check_trips_; }
  /// Frames dropped without a pose because the IMU data ended before them and upstream would hold the last sample
  /// for more than max_imu_gap (an INCOMPLETE frame outside a gap episode; for tests and logs, not thread-safe)
  size_t incomplete_drops() const { return incomplete_drops_; }
  /// Frames dropped without a pose because their stamp or scan end time is not finite (for tests and logs, not thread-safe)
  size_t nonfinite_frame_drops() const { return nonfinite_frame_drops_; }
  /// true if the last applied reset was an IMU gap self-reset extrapolated with wheel odometry
  /// (wheel_odom_reset_extrapolation), false after any other reset and before the first (for tests and logs, not thread-safe)
  bool last_reset_used_wheel() const { return last_reset_extrapolation_ == ResetExtrapolation::WHEEL; }
  /// How the last applied reset was extrapolated, NONE before the first and after any reset that is not an IMU gap
  /// (for tests and logs, not thread-safe)
  ResetExtrapolation last_reset_extrapolation() const { return last_reset_extrapolation_; }

protected:
  /// @throw std::invalid_argument if the map is null or a shared parameter is invalid
  OdometryEstimationLocalization(std::unique_ptr<Params>&& params, std::shared_ptr<const MapManager> map);

  const Params& localization_params() const;

  /// Weight of the map factors of the next frame: map_factor_weight, times degraded_weight_scale while DEGRADED or LOST
  double map_factor_weight() const;

  /// Replace the candidate generator of the initial pose search (default: YawCandidateGenerator from init_yaw_search).
  /// Not thread-safe: call before the first frame and before any reset(), because the estimation thread reads it.
  /// Lets a global candidate source plug in later.
  void set_candidate_generator(std::unique_ptr<PoseCandidateGenerator> generator);

  /// Called when a reset is applied, after the frames and the smoother are cleared: drop the backend state of the old run.
  virtual void reset_backend() {}

  struct MapQuality {
    double inlier_ratio = 0.0;  ///< Fraction of points in the finest voxels of current_map_factors_.tiles
    double residual = 0.0;      ///< Finest-level error per inlier point
  };
  /// Map quality of an estimated frame, with fresh factors at the estimate: those in the smoother keep the
  /// correspondences of their last linearization.
  virtual MapQuality evaluate_map_quality(const glim::EstimationFrame::ConstPtr& frame) = 0;

  /// The wheel odometry factor between frames current - 1 and current (empty if disabled, the first frame, without
  /// wheel data, or gated). Call from create_factors before the backend changes frames[current]->T_world_imu: the gate
  /// compares the wheel motion with the IMU prediction.
  gtsam::NonlinearFactorGraph create_wheel_factors(int current);

  /// The map factors and tiles of the newest frame. Set by the backend's create_factors, cleared by a reset.
  MapFactors current_map_factors_;

private:
  struct PendingReset {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Isometry3d T_world_imu;
    Eigen::Vector3d v_world_imu;
    double min_stamp;
    ResetReason reason;
    bool from_current = false;  ///< relocalize(): the pose and velocity are taken from the newest frame when applied
    double max_age = 0.0;       ///< relocalize(): the newest estimate may be at most this much older than the frame
    ResetExtrapolation extrapolation = ResetExtrapolation::NONE;  ///< IMU gap: how the pose and velocity were extrapolated
  };

  bool queue_reset(
    const Eigen::Isometry3d& T_world_imu,
    const Eigen::Vector3d& v_world_imu,
    double min_stamp,
    ResetReason reason,
    ResetExtrapolation extrapolation = ResetExtrapolation::NONE);
  /// The IMU gap self-reset state at stamp from the wheel motion since the last estimate (wheel_odom_reset_extrapolation):
  /// T = last.T_world_imu * T_imu_base * T_base * T_imu_base^-1, v = R * wheel_velocity_in_imu(twist at stamp).
  /// @return WHEEL, or why the wheels cannot be used (with a warning; T and v are then unchanged): the samples do not
  ///         cover [last.stamp, stamp] within max_gap, the result is not a valid reset, or a slip check fails
  ResetExtrapolation extrapolate_with_wheel(const glim::EstimationFrame& last, double stamp, Eigen::Isometry3d& T_world_imu, Eigen::Vector3d& v_world_imu) const;
  /// @param frame_stamp  the frame the reset applies at
  void apply_pending_reset(double frame_stamp);
  /// What insert_frame does with a frame after the IMU gap policy
  enum class GapVerdict {
    PROCEED,     ///< Integrate the frame
    DROP,        ///< Drop the frame; the IMU window restarts at it (a gap episode, or LOST)
    DROP_KEEP,   ///< Drop the frame and keep the IMU data since the last pose: the next frame integrates from that pose
  };
  GapVerdict handle_imu_gap(const glim::PreprocessedFrame::Ptr& frame);
  void trim_imu_stamps(double keep_from);
  void publish_lost();
  void trim_imu_queue(double stamp);
  void update_status(const glim::EstimationFrame::ConstPtr& frame, double processing_time);
  /// Run the initial pose search on the first frame and set the initial state to the best candidate.
  /// @return false if no candidate passed (the state is then LOST until a new pose)
  bool run_init_search(const glim::PreprocessedFrame::Ptr& frame);

  std::shared_ptr<const MapManager> map_;
  QualityMonitor quality_;
  WheelOdometryBuffer wheel_;
  int wheel_gated_streak_ = 0;  ///< Consecutive frames whose wheel factor was gated

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
  size_t incomplete_drops_ = 0;  ///< See incomplete_drops()
  size_t nonfinite_frame_drops_ = 0;  ///< See nonfinite_frame_drops()
  bool imu_lost_ = false;     ///< The gap was too long to recover; frames are dropped until reset()
  ResetExtrapolation last_reset_extrapolation_ = ResetExtrapolation::NONE;  ///< See last_reset_extrapolation()
  Eigen::Matrix<double, 6, 1> start_imu_bias_;  ///< See start_imu_bias()

  // Initial pose search (init_yaw_search)
  std::unique_ptr<PoseCandidateGenerator> candidate_generator_;
  /// A pose was given from outside and no frame has been estimated from it yet. Frames with fewer than min_points
  /// points are dropped meanwhile: upstream would start from the unsearched pose on an empty frame.
  bool init_search_pending_ = false;
  bool init_search_rejected_ = false;  ///< No candidate passed; frames are dropped until reset()
  bool init_search_sparse_warned_ = false;  ///< A sparse frame was already reported for the pending search
  Eigen::Isometry3d init_hint_T_world_imu_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d init_hint_v_world_imu_ = Eigen::Vector3d::Zero();

  mutable std::mutex reset_mutex_;
  std::unique_ptr<PendingReset> pending_reset_;
  std::optional<double> last_input_frame_stamp_;  ///< Guarded by reset_mutex_

  mutable std::mutex status_mutex_;
  LocalizationStatus status_;
};

}  // namespace glim_localization
