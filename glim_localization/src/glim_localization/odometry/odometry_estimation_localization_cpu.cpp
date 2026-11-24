// Derived from koide3/glim src/glim/odometry/odometry_estimation_cpu.cpp at commit 1a197c999e4865f5cab7c9cd25474128413ebbca (MIT License, Copyright (c) 2024 koide3)
// Derived from koide3/glim src/glim/odometry/odometry_estimation_imu.cpp (L115-122, ISAM2 parameters) at commit 1a197c999e4865f5cab7c9cd25474128413ebbca (MIT License, Copyright (c) 2024 koide3)
#include <glim_localization/odometry/odometry_estimation_localization_cpu.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include <spdlog/spdlog.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/common/cloud_covariance_estimation.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/common/imu_validation.hpp>
#include <glim/util/config.hpp>

#include <glim_localization/odometry/imu_gap.hpp>
#include <glim_localization/odometry/map_initial_state_estimation.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/task_arena.h>
#endif

namespace glim_localization {

using gtsam::symbol_shorthand::X;

CallbackSlot<void(const LocalizationStatus&)> LocalizationCallbacks::on_status;

const char* to_string(ResetReason reason) {
  switch (reason) {
    case ResetReason::NONE:
      return "NONE";
    case ResetReason::EXTERNAL:
      return "EXTERNAL";
    case ResetReason::IMU_GAP:
      return "IMU_GAP";
  }
  return "UNKNOWN";
}

namespace {

// IMU history kept before the first estimated frame, for the scan that starts it.
constexpr double kImuHistoryWhileWaiting = 1.0;

// This many rejected IMU samples in a row (not finite, or not newer than the last accepted one) are a fault of the stream
constexpr int kImuFaultRejects = 5;

// Upstream CallbackSlot has no locking. Holding this lock during dispatch keeps an
// instance alive until its destructor can unregister it. Never destroyed, as modules
// may be torn down during static destruction.
struct ResetRegistry {
  std::mutex mutex;
  std::vector<OdometryEstimationLocalizationCPU*> instances;
};

ResetRegistry& reset_registry() {
  static auto* registry = new ResetRegistry;
  return *registry;
}

bool is_valid_reset(const Eigen::Isometry3d& T, const Eigen::Vector3d& v, double min_stamp) {
  if (!T.matrix().allFinite() || !v.allFinite() || !std::isfinite(min_stamp)) {
    return false;
  }
  const Eigen::Matrix3d R = T.linear();
  return (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < 1e-3 && R.determinant() > 0.0;
}

std::shared_ptr<const MapManager> load_map(const OdometryEstimationLocalizationCPUParams& params) {
  MapManagerParams map_params;
  map_params.voxel_resolutions = params.map_voxel_resolutions;
  auto map = std::make_shared<MapManager>(map_params);
  if (params.map_path.empty() || !map->load(params.map_path)) {
    throw std::runtime_error("failed to load the localization map: " + params.map_path);
  }
  return map;
}

std::shared_ptr<const MapManager> require_map(std::shared_ptr<const MapManager> map) {
  if (!map) {
    throw std::invalid_argument("localization map is null");
  }
  return map;
}

}  // namespace

OdometryEstimationLocalizationCPUParams::OdometryEstimationLocalizationCPUParams() {
  glim::Config config(glim::GlobalConfig::get_config_path("config_odometry"));
  const std::string section = "odometry_estimation";

  init_pose_damping_scale = config.param<double>(section, "init_pose_damping_scale", 1e2);

  map_path = config.param<std::string>(section, "map_path", "");
  map_voxel_resolutions = config.param<std::vector<double>>(section, "map_voxel_resolutions", {0.5, 1.0, 2.0});
  map_factors.search_radius = config.param<double>(section, "map_search_radius", 60.0);
  map_factors.max_submaps = config.param<int>(section, "map_max_submaps", 5);
  map_factors.min_overlap = config.param<double>(section, "map_min_overlap", 0.1);
  map_factors.num_threads = num_threads;
  map_factor_weight = config.param<double>(section, "map_factor_weight", 1.0);
  degraded_weight_scale = config.param<double>(section, "degraded_weight_scale", 0.1);

  quality.degraded_inlier_ratio = config.param<double>(section, "degraded_inlier_ratio", 0.3);
  quality.tracking_frames = config.param<int>(section, "tracking_frames", 5);
  quality.degraded_frames = config.param<int>(section, "degraded_frames", 5);
  quality.lost_frames = config.param<int>(section, "lost_frames", 50);

  on_imu_gap = config.param<std::string>(section, "on_imu_gap", "reset");
  max_imu_gap = config.param<double>(section, "max_imu_gap", 0.05);
  imu_gap_reset_max = config.param<double>(section, "imu_gap_reset_max", 0.5);

  use_keyframes = config.param<bool>(section, "use_keyframes", false);
  max_iterations = config.param<int>(section, "max_iterations", 5);
  lru_thresh = config.param<int>(section, "lru_thresh", 100);
  target_downsampling_rate = config.param<double>(section, "target_downsampling_rate", 0.1);
  vgicp_resolution = config.param<double>(section, "vgicp_resolution", 0.5);
  vgicp_voxelmap_levels = config.param<int>(section, "vgicp_voxelmap_levels", 1);
  vgicp_voxelmap_scaling_factor = config.param<double>(section, "vgicp_voxelmap_scaling_factor", 2.0);
}

OdometryEstimationLocalizationCPUParams::~OdometryEstimationLocalizationCPUParams() {}

OdometryEstimationLocalizationCPU::OdometryEstimationLocalizationCPU(const Params& params)
: OdometryEstimationLocalizationCPU(params, load_map(params)) {}

OdometryEstimationLocalizationCPU::OdometryEstimationLocalizationCPU(const Params& params, std::shared_ptr<const MapManager> map)
: glim::OdometryEstimationIMU(std::make_unique<Params>(params)), map_(require_map(std::move(map))), quality_(params.quality) {
  // Upstream estimators cannot produce a pose in the map frame, so wait for a given one.
  auto init = std::make_unique<MapInitialStateEstimation>(params.T_lidar_imu, params.imu_bias);
  if (!params.estimate_init_state) {
    init->set_state(params.init_T_world_imu, params.init_v_world_imu);
  }
  init_estimation = std::move(init);
  init_local_model();
  if (!(params.degraded_weight_scale > 0.0)) {
    // A zero weight removes the map factors, so DEGRADED could never recover.
    throw std::invalid_argument("degraded_weight_scale must be positive");
  }

  if (params.on_imu_gap != "reset" && params.on_imu_gap != "off") {
    throw std::invalid_argument("on_imu_gap must be \"reset\" or \"off\"");
  }
  if (!(params.max_imu_gap > 0.0) || !(params.imu_gap_reset_max >= 0.0)) {
    throw std::invalid_argument("max_imu_gap must be positive and imu_gap_reset_max must not be negative");
  }

  auto& registry = reset_registry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  registry.instances.push_back(this);
}

void LocalizationCallbacks::request_reset(const Eigen::Isometry3d& T_world_imu, const Eigen::Vector3d& v_world_imu, double min_stamp) {
  auto& registry = reset_registry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  for (auto* instance : registry.instances) {
    instance->reset(T_world_imu, v_world_imu, min_stamp);
  }
}

std::shared_ptr<const MapManager> LocalizationCallbacks::loaded_map() {
  auto& registry = reset_registry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  return registry.instances.empty() ? nullptr : registry.instances.front()->map();
}

OdometryEstimationLocalizationCPU::~OdometryEstimationLocalizationCPU() {
  auto& registry = reset_registry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  registry.instances.erase(std::remove(registry.instances.begin(), registry.instances.end(), this), registry.instances.end());
}

const OdometryEstimationLocalizationCPU::Params& OdometryEstimationLocalizationCPU::localization_params() const {
  return *static_cast<const Params*>(params.get());
}

glim::EstimationFrame::ConstPtr OdometryEstimationLocalizationCPU::insert_frame(
  const glim::PreprocessedFrame::Ptr& frame,
  std::vector<glim::EstimationFrame::ConstPtr>& marginalized_frames) {
  const auto begin = std::chrono::steady_clock::now();
  {
    // Before any drop: a reset requested after a reader saw this stamp applies at this frame or a later one
    std::lock_guard<std::mutex> lock(reset_mutex_);
    last_input_frame_stamp_ = frame->stamp;
  }
  apply_pending_reset(frame->stamp);
  if (localization_params().on_imu_gap == "reset") {
    // Within one IMU epoch the frame stamps only increase, and upstream has consumed the IMU data up to the newest one.
    // A frame at or before it cannot be the first pose after a reset: the next pose would hold the first later sample
    // across everything in between.
    if (frames.empty() && newest_frame_stamp_ && frame->stamp <= *newest_frame_stamp_) {
      spdlog::warn("dropping the frame at {:.3f} s: not after the newest frame {:.3f} s seen since the IMU epoch began", frame->stamp, *newest_frame_stamp_);
      trim_imu_queue(frame->stamp);
      trim_imu_stamps(frame->stamp - kImuHistoryWhileWaiting);
      return nullptr;
    }
    newest_frame_stamp_ = newest_frame_stamp_ ? std::max(*newest_frame_stamp_, frame->stamp) : frame->stamp;  // Never lowered by an old frame
  }
  if (frame->stamp < min_stamp_) {
    trim_imu_queue(frame->stamp);
    trim_imu_stamps(frame->stamp - kImuHistoryWhileWaiting);
    return nullptr;
  }
  if (imu_lost_ || handle_imu_gap(frame)) {
    trim_imu_queue(frame->stamp);
    // While lost or in a gap episode the window restarts at the next frame, so older stamps are not needed
    trim_imu_stamps(frame->stamp);
    return nullptr;
  }

  const auto result = glim::OdometryEstimationIMU::insert_frame(frame, marginalized_frames);
  if (result) {
    update_status(result, std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count());
    trim_imu_stamps(result->stamp);
  } else if (frames.empty()) {
    trim_imu_queue(frame->stamp);
    trim_imu_stamps(frame->stamp - kImuHistoryWhileWaiting);
  }
  return result;
}

void OdometryEstimationLocalizationCPU::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  if (localization_params().on_imu_gap != "reset") {
    glim::OdometryEstimationIMU::insert_imu(stamp, linear_acc, angular_vel);  // No gap policy: forward everything as upstream does
    return;
  }

  // Gate: only finite, strictly increasing stamps reach upstream's queue (which skips dt <= 0 samples anyway), so the
  // history below is an append-only copy of what was forwarded and cannot disagree with that queue.
  if (!std::isfinite(stamp) || (last_imu_stamp_ && stamp <= *last_imu_stamp_)) {
    if (++imu_rejected_ < kImuFaultRejects) {
      spdlog::warn("dropping the IMU sample with stamp {} (last accepted {})", stamp, last_imu_stamp_.value_or(std::nan("")));
      return;
    }
    // A fault, not a stray sample: the stream is on another clock, or a batch arrived late
    imu_rejected_ = 0;
    if (!std::isfinite(stamp)) {
      return;  // Nothing to restart from
    }
    imu_integration->erase_imu_data(static_cast<int>(imu_integration->imu_data_in_queue().size()));
    imu_stamps_.clear();
    imu_trimmed_stamp_.reset();  // The restart sample has no predecessor
    newest_frame_stamp_.reset();  // The IMU clock restarted: the frame clock may have too
    last_imu_stamp_.reset();
    if (!frames.empty() && !imu_lost_) {
      spdlog::error("the IMU stamps are not increasing ({} consecutive samples dropped): the localization is lost until it is initialized again", kImuFaultRejects);
      imu_lost_ = true;
      gap_pending_ = false;
      publish_lost();
    }
  } else {
    imu_rejected_ = 0;
  }
  glim::OdometryEstimationIMU::insert_imu(stamp, linear_acc, angular_vel);
  imu_stamps_.push_back(stamp);
  last_imu_stamp_ = stamp;
}

void OdometryEstimationLocalizationCPU::trim_imu_stamps(const double keep_from) {
  // Keep the last sample at or before keep_from: the gap window starts there.
  while (imu_stamps_.size() >= 2 && imu_stamps_[1] <= keep_from) {
    imu_trimmed_stamp_ = imu_stamps_.front();  // The real predecessor of the new front: a hole there is not a leading one
    imu_stamps_.pop_front();
  }
}

bool OdometryEstimationLocalizationCPU::handle_imu_gap(const glim::PreprocessedFrame::Ptr& frame) {
  const auto& p = localization_params();
  if (p.on_imu_gap != "reset" || frames.empty()) {
    return false;
  }

  if (frame->stamp <= frames.back()->stamp) {
    // The frame clock stepped back or repeated: a self-reset would extrapolate by v * dt <= 0
    spdlog::error("the frame stamp {:.3f} s is not after the last frame {:.3f} s: the localization is lost until it is initialized again", frame->stamp, frames.back()->stamp);
    imu_lost_ = true;
    gap_pending_ = false;
    publish_lost();
    return true;
  }

  // The whole interval since the last pose counts, also when no LiDAR frame fell into it (a stall drops both topics).
  const auto check = [&](std::optional<double> last) {
    return check_imu_coverage(
      imu_stamps_.size(), [&](size_t k) { return imu_stamps_[k]; }, last, frame->stamp, frame->scan_end_time, p.max_imu_gap, 1e-6, imu_trimmed_stamp_);
  };
  const auto since_last_pose = check(frames.back()->stamp);
  // After a dropped frame the old frame is no longer a reference for being clean (its gap stays in the window):
  // wait for a frame with clean IMU data around it. The longest gap since the last pose still decides what happens.
  const auto around_frame = gap_pending_ ? check(std::nullopt) : ImuCoverageResult();

  if (since_last_pose.kind == ImuCoverage::INCOMPLETE) {
    return gap_pending_;  // Cannot tell yet: keep dropping during an episode, otherwise proceed as before
  }
  const auto lose = [&]() {
    spdlog::error("IMU gap of {:.3f} s exceeds {:.3f} s: the localization is lost until it is initialized again", gap_longest_, p.imu_gap_reset_max);
    imu_lost_ = true;
    gap_pending_ = false;
    publish_lost();
  };
  const double gap_since_last_pose = since_last_pose.is_gap() ? since_last_pose.gap_seconds : 0.0;
  if (gap_pending_ ? !around_frame.ok() : since_last_pose.is_gap()) {
    gap_pending_ = true;
    gap_longest_ = std::max({gap_longest_, gap_since_last_pose, around_frame.gap_seconds});
    if (gap_longest_ > p.imu_gap_reset_max) {
      lose();
    } else {
      spdlog::warn("dropping the frame at {:.3f} s: IMU gap of {:.3f} s", frame->stamp, gap_longest_);
    }
    return true;
  }
  if (!gap_pending_) {
    // The history says the interval since the last pose is clean. Check that upstream's real queue agrees, because
    // upstream holds the first later sample across any hole: a disagreement is an internal fault, so fail closed.
    const auto& queue = imu_integration->imu_data_in_queue();
    const double worst = longest_imu_hold(queue.size(), [&](size_t k) { return queue[k][0]; }, frames.back()->stamp, frame->stamp);
    if (worst > p.max_imu_gap) {
      spdlog::error(
        "internal disagreement: the IMU queue would be held for {:.3f} s since the last pose at {:.3f} s (limit {:.3f} s): the localization is lost until it is initialized again",
        worst,
        frames.back()->stamp,
        p.max_imu_gap);
      hold_check_trips_++;
      imu_lost_ = true;
      gap_pending_ = false;
      publish_lost();
      return true;
    }
    return false;
  }
  gap_longest_ = std::max(gap_longest_, gap_since_last_pose);
  if (gap_longest_ > p.imu_gap_reset_max) {
    lose();  // Clean around this frame, but the outage before it was too long
    return true;
  }

  // The first clean frame after a gap: restart from the last pose moved by the last velocity.
  const double gap = gap_longest_;
  gap_pending_ = false;
  gap_longest_ = 0.0;
  Eigen::Isometry3d T = frames.back()->T_world_imu;
  const Eigen::Vector3d v = frames.back()->v_world_imu;
  T.translation() += v * (frame->stamp - frames.back()->stamp);
  spdlog::warn("resetting the pose after an IMU gap of {:.3f} s", gap);
  if (!queue_reset(T, v, frame->stamp, ResetReason::IMU_GAP)) {
    imu_lost_ = true;  // A non-finite state cannot be recovered
    publish_lost();
    return true;
  }
  // Applies the self-reset, or the external reset that arrived first and won (see queue_reset)
  apply_pending_reset(frame->stamp);
  return frame->stamp < min_stamp_;
}

void OdometryEstimationLocalizationCPU::publish_lost() {
  LocalizationStatus snapshot;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.state = LocalizationState::LOST;
    snapshot = status_;
  }
  LocalizationCallbacks::on_status(snapshot);
}

void OdometryEstimationLocalizationCPU::trim_imu_queue(const double stamp) {
  // Upstream erases IMU data only from the second frame on, so waiting would grow the queue.
  const auto& queue = imu_integration->imu_data_in_queue();
  int num_old = 0;
  while (num_old + 1 < static_cast<int>(queue.size()) && queue[num_old + 1][0] < stamp - kImuHistoryWhileWaiting) {
    num_old++;
  }
  if (num_old > 0) {
    imu_integration->erase_imu_data(num_old);
  }
}

bool OdometryEstimationLocalizationCPU::reset(const Eigen::Isometry3d& T_world_imu, const Eigen::Vector3d& v_world_imu, double min_stamp) {
  return queue_reset(T_world_imu, v_world_imu, min_stamp, ResetReason::EXTERNAL);
}

std::optional<double> OdometryEstimationLocalizationCPU::last_input_frame_stamp() const {
  std::lock_guard<std::mutex> lock(reset_mutex_);
  return last_input_frame_stamp_;
}

bool OdometryEstimationLocalizationCPU::reset_pending() const {
  std::lock_guard<std::mutex> lock(reset_mutex_);
  return pending_reset_ != nullptr;
}

std::string OdometryEstimationLocalizationCPU::relocalize(double max_age) {
  const LocalizationStatus current = status();
  auto pending = std::make_unique<PendingReset>();
  pending->T_world_imu.setIdentity();  // Taken from the newest frame when applied
  pending->v_world_imu.setZero();
  pending->min_stamp = 0.0;
  pending->reason = ResetReason::EXTERNAL;
  pending->from_current = true;
  pending->max_age = max_age;

  std::lock_guard<std::mutex> lock(reset_mutex_);
  if (pending_reset_) {
    return "a reset is pending";  // Replacing it would silently drop an accepted pose; the caller waits
  }
  if (current.frame_id < 0 || (current.state != LocalizationState::TRACKING && current.state != LocalizationState::DEGRADED)) {
    return std::string("the state is ") + to_string(current.state) + ", not TRACKING or DEGRADED";
  }
  // Frames dropped after the estimate (LOST, an IMU gap) leave status() behind the input
  if (last_input_frame_stamp_ && *last_input_frame_stamp_ - current.stamp > max_age + 1e-6) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "the newest estimate is %.3f s older than the last input frame (max %.3f s)", *last_input_frame_stamp_ - current.stamp, max_age);
    return buf;
  }
  pending_reset_ = std::move(pending);
  return "";
}

bool OdometryEstimationLocalizationCPU::queue_reset(
  const Eigen::Isometry3d& T_world_imu,
  const Eigen::Vector3d& v_world_imu,
  double min_stamp,
  ResetReason reason) {
  if (!is_valid_reset(T_world_imu, v_world_imu, min_stamp)) {
    spdlog::error("ignoring invalid localization reset request");
    return false;
  }

  auto pending = std::make_unique<PendingReset>();
  pending->T_world_imu = T_world_imu;
  pending->T_world_imu.linear() = Eigen::Quaterniond(T_world_imu.linear()).normalized().toRotationMatrix();
  pending->v_world_imu = v_world_imu;
  pending->min_stamp = min_stamp;
  pending->reason = reason;

  std::lock_guard<std::mutex> lock(reset_mutex_);
  if (reason == ResetReason::IMU_GAP && pending_reset_) {
    // An external pose that arrived after the gap was detected always wins over a self-reset. The check and the
    // write are one step under the lock, so that it can neither be overwritten nor reported as IMU_GAP.
    return true;
  }
  pending_reset_ = std::move(pending);
  return true;
}

void OdometryEstimationLocalizationCPU::apply_pending_reset(double frame_stamp) {
  std::unique_ptr<PendingReset> pending;
  {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    pending = std::move(pending_reset_);
  }
  if (!pending) {
    return;
  }
  if (pending->from_current) {
    // relocalize(): restart from the newest estimate, moved to this frame by its velocity, and turned at the rate
    // between the last two estimates (the first frame after a reset is pinned to it, so a turn would show)
    if (frames.empty() || !(frame_stamp >= frames.back()->stamp)) {
      spdlog::warn("ignoring a relocalization: no estimate before the frame at {:.3f} s", frame_stamp);
      return;
    }
    // The checks of relocalize() again: the odometry may have gone LOST (an IMU fault) or the input may have stalled
    // since the request. Dropping the request leaves imu_lost_ and the IMU gap policy to handle this frame.
    const LocalizationState state = status().state;
    if (imu_lost_ || (state != LocalizationState::TRACKING && state != LocalizationState::DEGRADED)) {
      spdlog::warn("ignoring a relocalization: the state is {} at the frame at {:.3f} s", imu_lost_ ? "LOST" : to_string(state), frame_stamp);
      return;
    }
    const auto& last = frames.back();
    const double dt = frame_stamp - last->stamp;
    if (dt > pending->max_age + 1e-6) {
      spdlog::warn("ignoring a relocalization: the frame at {:.3f} s is {:.3f} s after the newest estimate (max {:.3f} s)", frame_stamp, dt, pending->max_age);
      return;
    }
    pending->T_world_imu = last->T_world_imu;
    pending->v_world_imu = last->v_world_imu;
    pending->T_world_imu.translation() += pending->v_world_imu * dt;
    if (frames.size() >= 2) {
      const auto& prev = frames[frames.size() - 2];
      const double dt_prev = last->stamp - prev->stamp;
      if (dt_prev > 1e-6) {
        const Eigen::AngleAxisd turn(prev->T_world_imu.linear().transpose() * last->T_world_imu.linear());
        const Eigen::AngleAxisd scaled(turn.angle() * dt / dt_prev, turn.axis());
        pending->T_world_imu.linear() = last->T_world_imu.linear() * scaled.toRotationMatrix();
      }
    }
    if (!is_valid_reset(pending->T_world_imu, pending->v_world_imu, pending->min_stamp)) {
      spdlog::warn("ignoring a relocalization: the newest estimate is not valid");
      return;
    }
  }

  const auto& p = localization_params();
  // The sensor is the same, so the latest bias estimate beats the configured one.
  const Eigen::Matrix<double, 6, 1> imu_bias = frames.empty() ? Eigen::Matrix<double, 6, 1>(p.imu_bias) : Eigen::Matrix<double, 6, 1>(frames.back()->imu_bias);
  frames.clear();
  marginalized_cursor = 0;

  gtsam::ISAM2Params isam2_params;
  if (p.use_isam2_dogleg) {
    isam2_params.setOptimizationParams(gtsam::ISAM2DoglegParams());
  }
  isam2_params.findUnusedFactorSlots = true;
  isam2_params.relinearizeSkip = p.isam2_relinearize_skip;
  isam2_params.setRelinearizeThreshold(p.isam2_relinearize_thresh);
  smoother.reset(new gtsam_points::IncrementalFixedLagSmootherExtWithFallback(p.smoother_lag, isam2_params));

  auto init = std::make_unique<MapInitialStateEstimation>(p.T_lidar_imu, imu_bias);
  init->set_state(pending->T_world_imu, pending->v_world_imu);
  init_estimation = std::move(init);

  quality_.reset();
  gap_pending_ = false;
  gap_longest_ = 0.0;
  imu_lost_ = false;
  current_map_factors_ = MapFactors();
  init_local_model();
  min_stamp_ = pending->min_stamp;

  LocalizationStatus snapshot;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    const int reset_count = status_.reset_count + 1;
    status_ = LocalizationStatus();
    status_.reset_count = reset_count;
    status_.last_reset_reason = pending->reason;
    status_.stamp = pending->min_stamp;
    snapshot = status_;
  }
  LocalizationCallbacks::on_status(snapshot);
}

LocalizationStatus OdometryEstimationLocalizationCPU::status() const {
  std::lock_guard<std::mutex> lock(status_mutex_);
  return status_;
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizationCPU::create_factors(
  const int current,
  const gtsam_points::shared_ptr<gtsam::ImuFactor>& /*imu_factor*/,
  gtsam::Values& new_values) {
  const auto& p = localization_params();
  const bool reduced = quality_.state() == LocalizationState::DEGRADED || quality_.state() == LocalizationState::LOST;
  const double weight = p.map_factor_weight * (reduced ? p.degraded_weight_scale : 1.0);

  current_map_factors_ = create_map_factors(*map_, X(current), frames[current]->frame, frames[current]->T_world_imu, p.map_factors, weight);
  if (!p.use_keyframes) {
    return current_map_factors_.factors;
  }

  // Map factors above use the IMU prediction, before the local model refines it.
  gtsam::NonlinearFactorGraph factors = current_map_factors_.factors;
  factors.add(create_local_model_factors(current, new_values));
  return factors;
}

void OdometryEstimationLocalizationCPU::init_local_model() {
  const auto& p = localization_params();
  target_voxelmaps_.clear();
  last_T_target_imu_.setIdentity();
  if (!p.use_keyframes) {
    return;
  }
  if (p.vgicp_voxelmap_levels < 1 || p.vgicp_resolution <= 0.0) {
    throw std::invalid_argument("invalid local model voxel parameters");
  }

  target_voxelmaps_.resize(p.vgicp_voxelmap_levels);
  for (int i = 0; i < p.vgicp_voxelmap_levels; i++) {
    const double resolution = p.vgicp_resolution * std::pow(p.vgicp_voxelmap_scaling_factor, i);
    target_voxelmaps_[i] = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
    target_voxelmaps_[i]->set_lru_horizon(p.lru_thresh);
  }
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizationCPU::create_local_model_factors(const int current, gtsam::Values& new_values) {
  const auto& p = localization_params();
  const int last = current - 1;

  if (current == 0) {
    last_T_target_imu_ = frames[current]->T_world_imu;
    update_local_model(current, frames[current]->T_world_imu);
    return gtsam::NonlinearFactorGraph();
  }

  const Eigen::Isometry3d pred_T_last_current = frames[last]->T_world_imu.inverse() * frames[current]->T_world_imu;
  const Eigen::Isometry3d pred_T_target_imu = last_T_target_imu_ * pred_T_last_current;

  gtsam::Values values;
  values.insert(X(current), gtsam::Pose3(pred_T_target_imu.matrix()));

  gtsam::NonlinearFactorGraph graph;
  for (const auto& voxelmap : target_voxelmaps_) {
    auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), X(current), voxelmap, frames[current]->frame);
    factor->set_num_threads(p.num_threads);
    graph.add(factor);
  }

  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(p.max_iterations);
  lm_params.setAbsoluteErrorTol(0.1);

  gtsam::Pose3 last_estimate = values.at<gtsam::Pose3>(X(current));
  lm_params.termination_criteria = [&](const gtsam::Values& values) {
    const gtsam::Pose3 current_pose = values.at<gtsam::Pose3>(X(current));
    const gtsam::Pose3 delta = last_estimate.inverse() * current_pose;

    const double delta_t = delta.translation().norm();
    const double delta_r = Eigen::AngleAxisd(delta.rotation().matrix()).angle();
    last_estimate = current_pose;

    if (delta_t < 1e-10 && delta_r < 1e-10) {
      // Maybe failed to solve the linear system
      return false;
    }
    return delta_t < 1e-3 && delta_r < 1e-3 * M_PI / 180.0;
  };

  gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);

#ifdef GTSAM_USE_TBB
  auto arena = static_cast<tbb::task_arena*>(this->tbb_task_arena.get());
  arena->execute([&] {
#endif
    values = optimizer.optimize();
#ifdef GTSAM_USE_TBB
  });
#endif

  const Eigen::Isometry3d T_target_imu = Eigen::Isometry3d(values.at<gtsam::Pose3>(X(current)).matrix());
  Eigen::Isometry3d T_last_current = last_T_target_imu_.inverse() * T_target_imu;
  T_last_current.linear() = Eigen::Quaterniond(T_last_current.linear()).normalized().toRotationMatrix();
  frames[current]->T_world_imu = frames[last]->T_world_imu * T_last_current;
  new_values.insert_or_assign(X(current), gtsam::Pose3(frames[current]->T_world_imu.matrix()));

  // No prior on X(current): the local model frame drifts and must not compete with the map.
  gtsam::NonlinearFactorGraph factors;
  factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
    X(last),
    X(current),
    gtsam::Pose3(T_last_current.matrix()),
    gtsam::noiseModel::Isotropic::Precision(6, 1e3));

  update_local_model(current, T_target_imu);
  last_T_target_imu_ = T_target_imu;
  return factors;
}

void OdometryEstimationLocalizationCPU::update_local_model(const int current, const Eigen::Isometry3d& T_target_imu) {
  const auto& p = localization_params();
  auto frame = frames[current]->frame;
  if (current >= 5) {
    frame = gtsam_points::random_sampling(frames[current]->frame, p.target_downsampling_rate, mt_);
  }

  auto transformed = gtsam_points::transform(frame, T_target_imu);
  for (auto& voxelmap : target_voxelmaps_) {
    voxelmap->insert(*transformed);
  }
}

void OdometryEstimationLocalizationCPU::update_status(const glim::EstimationFrame::ConstPtr& frame, const double processing_time) {
  const double inlier_ratio = map_inlier_ratio(current_map_factors_.tiles, frame->frame, frame->T_world_imu);
  const LocalizationState state = quality_.update(inlier_ratio);

  // Use separate factors: those in the smoother keep correspondences from their last linearization.
  double residual = 0.0;
  int num_inliers = 0;
  gtsam::Values values;
  values.insert(X(frame->id), gtsam::Pose3(frame->T_world_imu.matrix()));
  for (const auto& tile : current_map_factors_.tiles) {
    gtsam_points::IntegratedVGICPFactor factor(gtsam::Pose3(tile->T_world_origin.matrix()), X(frame->id), tile->voxelmaps.front(), frame->frame);
    factor.set_num_threads(localization_params().map_factors.num_threads);
    factor.linearize(values);
    residual += factor.error(values);
    num_inliers += factor.num_inliers();
  }
  residual = num_inliers > 0 ? residual / num_inliers : 0.0;

  LocalizationStatus snapshot;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.state = state;
    status_.stamp = frame->stamp;
    status_.frame_id = frame->id;
    status_.num_tiles = static_cast<int>(current_map_factors_.tiles.size());
    status_.num_map_factors = static_cast<int>(current_map_factors_.factors.size());
    status_.inlier_ratio = inlier_ratio;
    status_.residual = residual;
    status_.processing_time = processing_time;
    snapshot = status_;
  }
  LocalizationCallbacks::on_status(snapshot);
}

}  // namespace glim_localization
