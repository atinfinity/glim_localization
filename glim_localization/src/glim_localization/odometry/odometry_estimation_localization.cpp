// Derived from koide3/glim src/glim/odometry/odometry_estimation_cpu.cpp at commit 1a197c999e4865f5cab7c9cd25474128413ebbca (MIT License, Copyright (c) 2024 koide3)
// Derived from koide3/glim src/glim/odometry/odometry_estimation_imu.cpp (L115-122, ISAM2 parameters) at commit 1a197c999e4865f5cab7c9cd25474128413ebbca (MIT License, Copyright (c) 2024 koide3)
#include <glim_localization/odometry/odometry_estimation_localization.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2Params.h>

#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/common/cloud_covariance_estimation.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/common/imu_validation.hpp>
#include <glim/util/config.hpp>

#include <glim_localization/odometry/imu_gap.hpp>
#include <glim_localization/odometry/map_initial_state_estimation.hpp>
#include <glim_localization/relocalization/candidate_search.hpp>

namespace glim_localization {

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

const char* to_string(InitSearchResult result) {
  switch (result) {
    case InitSearchResult::NONE:
      return "NONE";
    case InitSearchResult::ACCEPTED:
      return "ACCEPTED";
    case InitSearchResult::REJECTED:
      return "REJECTED";
  }
  return "UNKNOWN";
}

const char* to_string(InitSearchReason reason) {
  switch (reason) {
    case InitSearchReason::NONE:
      return "NONE";
    case InitSearchReason::TRUNCATED:
      return "TRUNCATED";
    case InitSearchReason::LOW_SCORE:
      return "LOW_SCORE";
    case InitSearchReason::AMBIGUOUS:
      return "AMBIGUOUS";
    case InitSearchReason::NO_CANDIDATE:
      return "NO_CANDIDATE";
    case InitSearchReason::OUT_OF_RANGE:
      return "OUT_OF_RANGE";
  }
  return "UNKNOWN";
}

namespace {

InitSearchReason to_init_search_reason(CandidateSearchOutcome outcome) {
  switch (outcome) {
    case CandidateSearchOutcome::ACCEPTED:
      return InitSearchReason::NONE;
    case CandidateSearchOutcome::NO_CANDIDATE:
      return InitSearchReason::NO_CANDIDATE;
    case CandidateSearchOutcome::TRUNCATED:
      return InitSearchReason::TRUNCATED;
    case CandidateSearchOutcome::LOW_SCORE:
      return InitSearchReason::LOW_SCORE;
    case CandidateSearchOutcome::AMBIGUOUS:
      return InitSearchReason::AMBIGUOUS;
    case CandidateSearchOutcome::OUT_OF_RANGE:
      return InitSearchReason::OUT_OF_RANGE;
  }
  return InitSearchReason::NONE;
}

// IMU history kept before the first estimated frame, for the scan that starts it.
constexpr double kImuHistoryWhileWaiting = 1.0;

// This many rejected IMU samples in a row (not finite, or not newer than the last accepted one) are a fault of the stream
constexpr int kImuFaultRejects = 5;

// Upstream CallbackSlot has no locking. Holding this lock during dispatch keeps an
// instance alive until its destructor can unregister it. Never destroyed, as modules
// may be torn down during static destruction.
struct ResetRegistry {
  std::mutex mutex;
  std::vector<OdometryEstimationLocalization*> instances;
};

ResetRegistry& reset_registry() {
  static auto* registry = new ResetRegistry;
  return *registry;
}

bool is_valid_reset(const Eigen::Isometry3d& T, const Eigen::Vector3d& v, double min_stamp) {
  if (!T.matrix().allFinite() || !v.allFinite() || !std::isfinite(min_stamp)) {
    return false;
  }
  if (!(v.norm() <= OdometryEstimationLocalization::kMaxResetSpeed)) {
    return false;
  }
  const Eigen::Matrix3d R = T.linear();
  return (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < 1e-3 && R.determinant() > 0.0;
}

std::shared_ptr<const MapManager> require_map(std::shared_ptr<const MapManager> map) {
  if (!map) {
    throw std::invalid_argument("localization map is null");
  }
  return map;
}

}  // namespace

std::shared_ptr<const MapManager> load_localization_map(const OdometryEstimationLocalizationParams& params) {
  MapManagerParams map_params;
  map_params.voxel_resolutions = params.map_voxel_resolutions;
  auto map = std::make_shared<MapManager>(map_params);
  if (params.map_path.empty() || !map->load(params.map_path)) {
    throw std::runtime_error("failed to load the localization map: " + params.map_path);
  }
  return map;
}

CandidateSearchParams init_search_params(const OdometryEstimationLocalizationParams& params, const Eigen::Vector3d& hint_position) {
  const auto& search = params.init_yaw_search;
  CandidateSearchParams search_params;
  search_params.scorer.map_factors = params.map_factors;
  if (search.num_threads > 0) {
    search_params.scorer.map_factors.num_threads = search.num_threads;
  }
  search_params.scorer.max_iterations = search.max_iterations;
  search_params.min_score = search.min_inlier_ratio;
  search_params.min_score_gap = search.min_score_gap;
  search_params.distinct_yaw = distinct_yaw_for_step(search.step);
  search_params.time_budget = search.time_budget;
  search_params.hint_position = hint_position;
  search_params.max_translation = search.max_translation;
  return search_params;
}

OdometryEstimationLocalizationParams::OdometryEstimationLocalizationParams() {
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
  init_v_without_pose = config.param<Eigen::Vector3d>(section, "init_v_world_imu") && !config.param<Eigen::Isometry3d>(section, "init_T_world_imu");

  init_yaw_search.enabled = config.param<bool>(section, "init_yaw_search", false);
  init_yaw_search.range = config.param<double>(section, "init_yaw_search_range_deg", 180.0) * M_PI / 180.0;
  init_yaw_search.step = config.param<double>(section, "init_yaw_search_step_deg", 15.0) * M_PI / 180.0;
  init_yaw_search.min_inlier_ratio = config.param<double>(section, "init_yaw_search_min_inlier_ratio", 0.7);
  init_yaw_search.min_score_gap = config.param<double>(section, "init_yaw_search_min_score_gap", 0.05);
  init_yaw_search.time_budget = config.param<double>(section, "init_yaw_search_time_budget", 10.0);
  init_yaw_search.max_iterations = config.param<int>(section, "init_yaw_search_max_iterations", 10);
  init_yaw_search.min_points = config.param<int>(section, "init_yaw_search_min_points", 100);
  init_yaw_search.max_translation = config.param<double>(section, "init_yaw_search_max_translation", 0.0);
  init_yaw_search.num_threads = config.param<int>(section, "init_yaw_search_num_threads", 0);

  wheel_odom.enabled = config.param<bool>(section, "wheel_odom_enabled", false);
  wheel_odom.T_imu_base = config.param<Eigen::Isometry3d>(section, "wheel_odom_T_imu_base", Eigen::Isometry3d::Identity());
  wheel_odom.nonholonomic = config.param<bool>(section, "wheel_odom_nonholonomic", false);
  wheel_odom.max_gap = config.param<double>(section, "wheel_odom_max_gap", 0.2);
  wheel_odom.translation_sigma_min = config.param<double>(section, "wheel_odom_translation_sigma_min", 0.01);
  wheel_odom.translation_sigma_ratio = config.param<double>(section, "wheel_odom_translation_sigma_ratio", 0.05);
  wheel_odom.rotation_sigma_min = config.param<double>(section, "wheel_odom_rotation_sigma_min", 0.05);
  wheel_odom.rotation_sigma_ratio = config.param<double>(section, "wheel_odom_rotation_sigma_ratio", 0.2);
  wheel_odom.huber_threshold = config.param<double>(section, "wheel_odom_huber_threshold", 1.0);
  wheel_odom.gate_velocity = config.param<double>(section, "wheel_odom_gate_velocity", 0.3);
  wheel_odom.gate_velocity_ratio = config.param<double>(section, "wheel_odom_gate_velocity_ratio", 0.3);
  wheel_odom.gate_angular_velocity = config.param<double>(section, "wheel_odom_gate_angular_velocity", 0.5);
  wheel_odom.gate_readmit_frames = config.param<int>(section, "wheel_odom_gate_readmit_frames", 20);
  wheel_odom.reset_extrapolation = config.param<bool>(section, "wheel_odom_reset_extrapolation", false);
}

OdometryEstimationLocalizationParams::~OdometryEstimationLocalizationParams() {}

OdometryEstimationLocalization::OdometryEstimationLocalization(std::unique_ptr<Params>&& params_ptr, std::shared_ptr<const MapManager> map)
: glim::OdometryEstimationIMU(std::move(params_ptr)), map_(require_map(std::move(map))), quality_(localization_params().quality),
  wheel_(localization_params().wheel_odom), start_imu_bias_(localization_params().imu_bias) {
  const auto& p = localization_params();
  if (p.init_v_without_pose) {
    throw std::invalid_argument("init_v_world_imu is set without init_T_world_imu: set both, or neither to wait for a pose");
  }
  if (!p.estimate_init_state && !is_valid_reset(p.init_T_world_imu, p.init_v_world_imu, 0.0)) {
    throw std::invalid_argument(
      "init_T_world_imu must be a finite rigid transform and init_v_world_imu finite and at most " + std::to_string(kMaxResetSpeed) + " m/s");
  }
  // Upstream estimators cannot produce a pose in the map frame, so wait for a given one.
  auto init = std::make_unique<MapInitialStateEstimation>(p.T_lidar_imu, p.imu_bias);
  if (!p.estimate_init_state) {
    init->set_state(p.init_T_world_imu, p.init_v_world_imu);
    init_search_pending_ = p.init_yaw_search.enabled;
    init_hint_T_world_imu_ = p.init_T_world_imu;
    init_hint_v_world_imu_ = p.init_v_world_imu;
  }
  init_estimation = std::move(init);
  if (!(p.degraded_weight_scale > 0.0)) {
    // A zero weight removes the map factors, so DEGRADED could never recover.
    throw std::invalid_argument("degraded_weight_scale must be positive");
  }

  if (p.on_imu_gap != "reset" && p.on_imu_gap != "off") {
    throw std::invalid_argument("on_imu_gap must be \"reset\" or \"off\"");
  }
  if (!(p.max_imu_gap > 0.0) || !(p.imu_gap_reset_max >= 0.0)) {
    throw std::invalid_argument("max_imu_gap must be positive and imu_gap_reset_max must not be negative");
  }
  const auto& search = p.init_yaw_search;
  if (!(search.min_inlier_ratio >= 0.0 && search.min_inlier_ratio <= 1.0) || !(search.min_score_gap >= 0.0 && search.min_score_gap <= 1.0) || !(search.time_budget >= 0.0) ||
      !std::isfinite(search.time_budget) || search.max_iterations < 1 || search.min_points < 1 || !(search.max_translation >= 0.0) || !std::isfinite(search.max_translation)) {
    throw std::invalid_argument(
      "init_yaw_search_min_inlier_ratio and init_yaw_search_min_score_gap must be in [0, 1], init_yaw_search_time_budget and "
      "init_yaw_search_max_translation finite and not negative, and init_yaw_search_max_iterations and init_yaw_search_min_points positive");
  }
  if (search.num_threads < 0) {
    throw std::invalid_argument("init_yaw_search_num_threads must not be negative (0 uses num_threads)");
  }
  candidate_generator_ = std::make_unique<YawCandidateGenerator>(search.range, search.step);  // Validates the range and step
  if (p.wheel_odom.enabled) {
    p.wheel_odom.validate();
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

bool LocalizationCallbacks::insert_wheel_odom(double stamp, const Eigen::Vector3d& linear_vel, const Eigen::Vector3d& angular_vel) {
  auto& registry = reset_registry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  bool inserted = false;
  for (auto* instance : registry.instances) {
    inserted |= instance->insert_wheel_odom(stamp, linear_vel, angular_vel);
  }
  return inserted;
}

bool LocalizationCallbacks::wheel_odom_enabled() {
  auto& registry = reset_registry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  return std::any_of(registry.instances.begin(), registry.instances.end(), [](const OdometryEstimationLocalization* instance) {
    return instance->wheel_odom_enabled();
  });
}

OdometryEstimationLocalization::~OdometryEstimationLocalization() {
  auto& registry = reset_registry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  registry.instances.erase(std::remove(registry.instances.begin(), registry.instances.end(), this), registry.instances.end());
}

const OdometryEstimationLocalization::Params& OdometryEstimationLocalization::localization_params() const {
  return *static_cast<const Params*>(params.get());
}

glim::EstimationFrame::ConstPtr OdometryEstimationLocalization::insert_frame(
  const glim::PreprocessedFrame::Ptr& frame,
  std::vector<glim::EstimationFrame::ConstPtr>& marginalized_frames) {
  const auto begin = std::chrono::steady_clock::now();
  if (!std::isfinite(frame->stamp) || !std::isfinite(frame->scan_end_time)) {
    // A corrupt frame is a missing frame: no comparison or hold check can judge it (a NaN compares false and a NaN
    // span vanishes in std::max), and its stamp must not reach the reset bookkeeping. Nothing is trimmed, so the next
    // finite frame is judged over the whole interval since the last pose, as after a LiDAR dropout. Not LOST: the
    // IMU and the last pose are still valid.
    nonfinite_frame_drops_++;
    spdlog::warn("dropping a frame with a non-finite stamp ({}) or scan end time ({})", frame->stamp, frame->scan_end_time);
    return nullptr;
  }
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
  const GapVerdict verdict = imu_lost_ ? GapVerdict::DROP : handle_imu_gap(frame);
  if (verdict == GapVerdict::DROP) {
    trim_imu_queue(frame->stamp);
    // While lost or in a gap episode the window restarts at the next frame, so older stamps are not needed
    trim_imu_stamps(frame->stamp);
    return nullptr;
  }
  if (verdict == GapVerdict::DROP_KEEP) {
    // The last pose stays the reference: the queue and the history keep everything since it, so the next frame is
    // judged (and integrated) over the whole interval, including the data before this frame
    return nullptr;
  }
  if (init_search_pending_ && frames.empty() && frame->size() < localization_params().init_yaw_search.min_points) {
    // Upstream would build the first frame from the unsearched pose even when it is empty, so the search would never
    // run: keep it pending and wait for a frame that can be scored. Warned once per pending search.
    if (!init_search_sparse_warned_) {
      spdlog::warn("dropping the frame at {:.3f} s: {} points are too few for the initial pose search (init_yaw_search_min_points {}); later sparse frames are dropped silently until the search runs",
        frame->stamp, frame->size(), localization_params().init_yaw_search.min_points);
      init_search_sparse_warned_ = true;
    }
    trim_imu_queue(frame->stamp);
    trim_imu_stamps(frame->stamp - kImuHistoryWhileWaiting);
    return nullptr;
  }
  if (init_search_rejected_ || (init_search_pending_ && frames.empty() && !run_init_search(frame))) {
    // The given pose was rejected: wait for another one, as before the first pose
    trim_imu_queue(frame->stamp);
    trim_imu_stamps(frame->stamp - kImuHistoryWhileWaiting);
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

void OdometryEstimationLocalization::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
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
      // Nothing to restart from, and the stream is broken: keep the queue (it is still consistent) and stop publishing
      if (!frames.empty() && !imu_lost_) {
        spdlog::error("the IMU stamps are not finite ({} consecutive samples dropped): the localization is lost until it is initialized again", kImuFaultRejects);
        imu_lost_ = true;
        gap_pending_ = false;
        publish_lost();
      }
      return;
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

void OdometryEstimationLocalization::trim_imu_stamps(const double keep_from) {
  // Keep the last sample at or before keep_from: the gap window starts there.
  while (imu_stamps_.size() >= 2 && imu_stamps_[1] <= keep_from) {
    imu_trimmed_stamp_ = imu_stamps_.front();  // The real predecessor of the new front: a hole there is not a leading one
    imu_stamps_.pop_front();
  }
}

OdometryEstimationLocalization::GapVerdict OdometryEstimationLocalization::handle_imu_gap(const glim::PreprocessedFrame::Ptr& frame) {
  const auto& p = localization_params();
  if (p.on_imu_gap != "reset" || frames.empty()) {
    return GapVerdict::PROCEED;
  }

  if (frame->stamp <= frames.back()->stamp) {
    // The frame clock stepped back or repeated: a self-reset would extrapolate by v * dt <= 0
    spdlog::error("the frame stamp {:.3f} s is not after the last frame {:.3f} s: the localization is lost until it is initialized again", frame->stamp, frames.back()->stamp);
    imu_lost_ = true;
    gap_pending_ = false;
    publish_lost();
    return GapVerdict::DROP;
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
    if (gap_pending_) {
      return GapVerdict::DROP;  // Cannot tell yet: keep dropping during the episode
    }
    // No sample reaches the scan end. Online the async wrapper releases such a frame only in the end_of_sequence
    // flush, or when last_imu_time is NaN (a non-finite IMU stamp, which the gate drops). Upstream holds the last
    // sample up to the frame, so measure that on its real queue. A short tail (a normal shutdown) proceeds; a longer
    // one is not a pose, but not yet a gap either: the data may still come, so the frame is dropped and the last pose
    // stays the reference.
    const auto& queue = imu_integration->imu_data_in_queue();
    const double hold = longest_imu_hold(queue.size(), [&](size_t k) { return queue[k][0]; }, frames.back()->stamp, frame->stamp);
    // The 2-sample rule of the covered path, over (last pose, frame]: the hold limit alone allows a single sample (or
    // none, for a frame interval of at most max_imu_gap), and upstream then predicts without an IMU factor
    const auto in_interval = std::count_if(queue.begin(), queue.end(), [&](const auto& imu) {
      return imu[0] > frames.back()->stamp && imu[0] <= frame->stamp;
    });
    if (hold <= p.max_imu_gap && in_interval >= 2) {
      return GapVerdict::PROCEED;
    }
    if (hold > p.imu_gap_reset_max) {
      spdlog::error(
        "the IMU data ends before the scan, and upstream would hold a sample for {:.3f} s since the last pose at {:.3f} s (limit {:.3f} s): the localization is lost until it is initialized again",
        hold,
        frames.back()->stamp,
        p.imu_gap_reset_max);
      imu_lost_ = true;
      publish_lost();
      return GapVerdict::DROP;
    }
    incomplete_drops_++;
    spdlog::warn(
      "dropping the frame at {:.3f} s: the IMU data ends before the scan, and upstream would hold a sample for {:.3f} s with {} samples since the last pose",
      frame->stamp,
      hold,
      in_interval);
    return GapVerdict::DROP_KEEP;
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
    return GapVerdict::DROP;
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
      return GapVerdict::DROP;
    }
    return GapVerdict::PROCEED;
  }
  gap_longest_ = std::max(gap_longest_, gap_since_last_pose);
  if (gap_longest_ > p.imu_gap_reset_max) {
    lose();  // Clean around this frame, but the outage before it was too long
    return GapVerdict::DROP;
  }

  // The first clean frame after a gap: restart from the last pose moved by the wheel motion (when enabled and
  // available), else by the last velocity with the orientation held.
  const double gap = gap_longest_;
  gap_pending_ = false;
  gap_longest_ = 0.0;
  Eigen::Isometry3d T = frames.back()->T_world_imu;
  Eigen::Vector3d v = frames.back()->v_world_imu;
  const ResetExtrapolation extrapolation = p.wheel_odom.enabled && p.wheel_odom.reset_extrapolation
                                             ? extrapolate_with_wheel(*frames.back(), frame->stamp, T, v)
                                             : ResetExtrapolation::NONE;
  const bool from_wheel = extrapolation == ResetExtrapolation::WHEEL;
  if (!from_wheel) {
    T.translation() += v * (frame->stamp - frames.back()->stamp);
  }
  spdlog::warn("resetting the pose after an IMU gap of {:.3f} s{}", gap, from_wheel ? " with the wheel odometry motion" : "");
  if (!queue_reset(T, v, frame->stamp, ResetReason::IMU_GAP, extrapolation)) {
    imu_lost_ = true;  // A non-finite state or one above kMaxResetSpeed cannot be recovered
    publish_lost();
    return GapVerdict::DROP;
  }
  // Applies the self-reset, or the external reset that arrived first and won (see queue_reset)
  apply_pending_reset(frame->stamp);
  return frame->stamp < min_stamp_ ? GapVerdict::DROP : GapVerdict::PROCEED;
}

ResetExtrapolation OdometryEstimationLocalization::extrapolate_with_wheel(
  const glim::EstimationFrame& last,
  const double stamp,
  Eigen::Isometry3d& T_world_imu,
  Eigen::Vector3d& v_world_imu) const {
  const auto& params = localization_params().wheel_odom;
  if (!params.enabled || !params.reset_extrapolation) {
    return ResetExtrapolation::NONE;
  }
  // The buffer still holds the samples since the last estimate: it is trimmed only when a frame gets its wheel factor,
  // and only up to the reset frame when the reset is applied
  const auto motion = wheel_.motion(last.stamp, stamp);
  if (!motion) {
    spdlog::warn(
      "the wheel odometry does not cover [{:.3f}, {:.3f}] s within {:.3f} s: resetting with the last velocity instead",
      last.stamp,
      stamp,
      params.max_gap);
    return ResetExtrapolation::NO_COVERAGE;
  }
  Eigen::Isometry3d T = last.T_world_imu * (params.T_imu_base * motion->T_base * params.T_imu_base.inverse());
  T.linear() = Eigen::Quaterniond(T.linear()).normalized().toRotationMatrix();
  const Eigen::Vector3d v = T.linear() * wheel_velocity_in_imu(params, motion->twist1);
  if (!is_valid_reset(T, v, stamp)) {
    spdlog::warn("the wheel odometry gives an invalid reset state (speed {:.3f} m/s): resetting with the last velocity instead", v.norm());
    return ResetExtrapolation::INVALID;
  }

  // Slip check. The reset velocity also becomes the IMU prediction of the next frames, so the wheel factor gate would
  // agree with a slipping wheel from then on: check the wheels here, with the thresholds of that gate.
  // 1. At the last estimate, the wheel velocity must agree with the estimated one (a wheel already slipping).
  // 2. At the last estimate, the wheel angular velocity must agree with the bias-corrected gyro around it.
  // 3. Over the gap, the wheel displacement must agree with the trapezoid of the estimated velocity at the start and the
  //    wheel velocity at the end (a wheel that locks, or slips and recovers, inside the gap). A smooth brake or turn is
  //    close to linear over the gap; a step in the speed is not, and then the constant velocity reset is used.
  // Not caught: a scale slip that starts in the gap and lasts through the reset frame. The end velocity is then wrong
  // too, and there is no other speed reference after the gap; the map factors have to correct it.
  const auto velocity_gate = [&params](double speed) { return std::max(params.gate_velocity, params.gate_velocity_ratio * speed); };
  const Eigen::Vector3d v_wheel_last = last.T_world_imu.linear() * wheel_velocity_in_imu(params, motion->twist0);
  const double start_diff = (v_wheel_last - last.v_world_imu).norm();
  const double start_gate = velocity_gate(std::max(v_wheel_last.norm(), last.v_world_imu.norm()));
  if (start_gate > 0.0 && start_diff > start_gate) {
    spdlog::warn(
      "the wheel velocity disagrees with the estimate at {:.3f} s by {:.3f} m/s (gate {:.3f}): resetting with the last velocity instead",
      last.stamp,
      start_diff,
      start_gate);
    return ResetExtrapolation::SLIP_START;
  }

  if (params.gate_angular_velocity > 0.0) {
    // The upstream queue still holds the samples since the last estimate; without one near it there is no reference
    const double window = localization_params().max_imu_gap;
    Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
    int count = 0;
    for (const auto& imu : imu_integration->imu_data_in_queue()) {
      if (std::abs(imu[0] - last.stamp) <= window) {
        gyro_sum += imu.tail<3>();
        count++;
      }
    }
    if (count > 0) {
      const Eigen::Vector3d gyro = gyro_sum / count - last.imu_bias.tail<3>();
      const Eigen::Vector3d w_wheel = params.T_imu_base.linear() * motion->twist0.angular;
      const double angular_diff = (w_wheel - gyro).norm();
      if (angular_diff > params.gate_angular_velocity) {
        spdlog::warn(
          "the wheel angular velocity disagrees with the gyro at {:.3f} s by {:.3f} rad/s (gate {:.3f}): resetting with the last velocity instead",
          last.stamp,
          angular_diff,
          params.gate_angular_velocity);
        return ResetExtrapolation::SLIP_ANGULAR;
      }
    }
  }

  const double dt = stamp - last.stamp;
  const Eigen::Vector3d displacement = T.translation() - last.T_world_imu.translation();
  const double gap_diff = (displacement - 0.5 * (last.v_world_imu + v) * dt).norm() / dt;
  const double gap_gate = velocity_gate(std::max(last.v_world_imu.norm(), v.norm()));
  if (gap_gate > 0.0 && gap_diff > gap_gate) {
    spdlog::warn(
      "the wheel displacement over the gap [{:.3f}, {:.3f}] s disagrees with the velocities at its ends by {:.3f} m/s (gate {:.3f}): "
      "resetting with the last velocity instead",
      last.stamp,
      stamp,
      gap_diff,
      gap_gate);
    return ResetExtrapolation::SLIP_GAP;
  }
  T_world_imu = T;
  v_world_imu = v;
  return ResetExtrapolation::WHEEL;
}

void OdometryEstimationLocalization::set_candidate_generator(std::unique_ptr<PoseCandidateGenerator> generator) {
  if (!generator) {
    throw std::invalid_argument("the candidate generator is null");
  }
  candidate_generator_ = std::move(generator);
}

bool OdometryEstimationLocalization::run_init_search(const glim::PreprocessedFrame::Ptr& raw_frame) {
  const auto& p = localization_params();
  init_search_pending_ = false;

  // The first frame as upstream builds it (IMU frame, covariances from the preprocessed neighbors), without deskewing
  const Eigen::Isometry3d T_imu_lidar = p.T_lidar_imu.inverse();
  std::vector<Eigen::Vector4d> points_imu(raw_frame->size());
  for (int i = 0; i < raw_frame->size(); i++) {
    points_imu[i] = T_imu_lidar * raw_frame->points[i];
  }
  std::vector<Eigen::Vector4d> normals;
  std::vector<Eigen::Matrix4d> covs;
  covariance_estimation->estimate(points_imu, raw_frame->neighbors, normals, covs);
  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points_imu);
  cloud->add_covs(covs);

  CandidateQuery query;
  query.hint = init_hint_T_world_imu_;
  query.frame = cloud;
  query.map = map_;
  const PoseCandidates candidates = candidate_generator_->generate(query);

  const CandidateSearchResult result = search_candidates(*map_, cloud, candidates, init_search_params(p, init_hint_T_world_imu_.translation()));

  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.init_search = result.accepted ? InitSearchResult::ACCEPTED : InitSearchResult::REJECTED;
    status_.init_search_reason = to_init_search_reason(result.outcome);
    status_.init_search_candidates = result.num_evaluated;
    status_.init_search_candidates_total = result.num_candidates;
    // For OUT_OF_RANGE the score is that of the match beyond the bound, the one that caused the rejection, and the
    // runner-up is the best within the bound, the one it outscored
    const bool out_of_range = result.outcome == CandidateSearchOutcome::OUT_OF_RANGE;
    const auto& reported = out_of_range ? result.out_of_range : result.best;
    const auto& second = out_of_range ? result.best : result.runner_up;
    status_.init_search_score = reported ? reported->score : 0.0;
    status_.init_search_yaw_offset = reported ? reported->yaw_offset : 0.0;
    status_.init_search_runner_up_score = second ? second->score : 0.0;
    status_.init_search_runner_up_yaw_offset = second ? second->yaw_offset : 0.0;
    status_.init_search_time = result.elapsed;
  }

  if (!result.accepted) {
    spdlog::error("initial pose search ({}) at {:.3f} s rejected the given pose: {}; the localization is lost until it is initialized again", candidate_generator_->name(), raw_frame->stamp, result.message);
    init_search_rejected_ = true;
    publish_lost();
    return false;
  }

  spdlog::info("initial pose search ({}) at {:.3f} s: {}", candidate_generator_->name(), raw_frame->stamp, result.message);
  // Start from the registered pose; the velocity turns with the yaw
  const Eigen::Isometry3d& T = result.best->T_world_imu;
  const Eigen::Matrix3d turn = T.linear() * init_hint_T_world_imu_.linear().transpose();
  auto* init = dynamic_cast<MapInitialStateEstimation*>(init_estimation.get());
  if (!init) {
    throw std::logic_error("the initial state estimation is not a MapInitialStateEstimation");
  }
  init->set_state(T, turn * init_hint_v_world_imu_);
  return true;
}

void OdometryEstimationLocalization::publish_lost() {
  LocalizationStatus snapshot;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.state = LocalizationState::LOST;
    snapshot = status_;
  }
  LocalizationCallbacks::on_status(snapshot);
}

void OdometryEstimationLocalization::trim_imu_queue(const double stamp) {
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

bool OdometryEstimationLocalization::reset(const Eigen::Isometry3d& T_world_imu, const Eigen::Vector3d& v_world_imu, double min_stamp) {
  return queue_reset(T_world_imu, v_world_imu, min_stamp, ResetReason::EXTERNAL);
}

std::optional<double> OdometryEstimationLocalization::last_input_frame_stamp() const {
  std::lock_guard<std::mutex> lock(reset_mutex_);
  return last_input_frame_stamp_;
}

bool OdometryEstimationLocalization::reset_pending() const {
  std::lock_guard<std::mutex> lock(reset_mutex_);
  return pending_reset_ != nullptr;
}

std::string OdometryEstimationLocalization::relocalize(double max_age) {
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

bool OdometryEstimationLocalization::queue_reset(
  const Eigen::Isometry3d& T_world_imu,
  const Eigen::Vector3d& v_world_imu,
  double min_stamp,
  ResetReason reason,
  ResetExtrapolation extrapolation) {
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
  pending->extrapolation = extrapolation;

  std::lock_guard<std::mutex> lock(reset_mutex_);
  if (reason == ResetReason::IMU_GAP && pending_reset_) {
    // An external pose that arrived after the gap was detected always wins over a self-reset. The check and the
    // write are one step under the lock, so that it can neither be overwritten nor reported as IMU_GAP.
    return true;
  }
  pending_reset_ = std::move(pending);
  return true;
}

void OdometryEstimationLocalization::apply_pending_reset(double frame_stamp) {
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
  // The sensor is the same, so the latest bias estimate beats the configured one. Without a frame in this run (two
  // resets in a row), keep the bias this run started from.
  // A non-finite estimate is not carried over, so a reset can still recover from it.
  if (!frames.empty() && frames.back()->imu_bias.allFinite()) {
    start_imu_bias_ = frames.back()->imu_bias;
  }
  const Eigen::Matrix<double, 6, 1> imu_bias = start_imu_bias_;
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

  // Only a pose given from outside is searched: relocalize() and an IMU gap self-reset start from the estimate
  init_search_pending_ = p.init_yaw_search.enabled && pending->reason == ResetReason::EXTERNAL && !pending->from_current;
  init_search_rejected_ = false;
  init_search_sparse_warned_ = false;
  init_hint_T_world_imu_ = pending->T_world_imu;
  init_hint_v_world_imu_ = pending->v_world_imu;

  quality_.reset();
  gap_pending_ = false;
  gap_longest_ = 0.0;
  imu_lost_ = false;
  last_reset_extrapolation_ = pending->extrapolation;
  current_map_factors_ = MapFactors();
  if (pending->reason == ResetReason::IMU_GAP) {
    // The same clock goes on: keep the sample at or before the reset frame, so the first interval after it is covered
    wheel_.trim(frame_stamp);
  } else {
    // Samples of the old run may be stamped after the new one (a bag loop): they would block the new samples
    wheel_.clear();
  }
  wheel_gated_streak_ = 0;
  reset_backend();
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

bool OdometryEstimationLocalization::insert_wheel_odom(double stamp, const Eigen::Vector3d& linear_vel, const Eigen::Vector3d& angular_vel) {
  if (!localization_params().wheel_odom.enabled) {
    return false;
  }
  return wheel_.insert(stamp, linear_vel, angular_vel);
}

bool OdometryEstimationLocalization::wheel_odom_enabled() const {
  return localization_params().wheel_odom.enabled;
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalization::create_wheel_factors(const int current) {
  const auto& p = localization_params();
  if (!p.wheel_odom.enabled || current < 1 || !frames[current - 1] || !frames[current]) {
    return gtsam::NonlinearFactorGraph();
  }
  const auto& last = frames[current - 1];
  const auto& now = frames[current];

  gtsam::NonlinearFactorGraph factors;
  WheelFactorResult result = WheelFactorResult::NO_DATA;
  bool readmitted = false;
  const auto T_base = wheel_.integrate(last->stamp, now->stamp);
  if (T_base) {
    // frames[current] still holds the IMU prediction here
    const Eigen::Isometry3d T_last_current_imu = last->T_world_imu.inverse() * now->T_world_imu;
    // A long disagreement may be a wrong prediction rather than slip: without a map constraint nothing else would
    // correct it, so let one factor through every gate_readmit_frames gated frames
    readmitted = p.wheel_odom.gate_readmit_frames > 0 && wheel_gated_streak_ >= p.wheel_odom.gate_readmit_frames;
    using gtsam::symbol_shorthand::X;
    factors = create_wheel_factor(p.wheel_odom, X(current - 1), X(current), now->stamp - last->stamp, *T_base, T_last_current_imu, result, !readmitted);
    if (result == WheelFactorResult::GATED) {
      wheel_gated_streak_++;
    } else {
      if (readmitted) {
        spdlog::warn("wheel odometry disagreed with the IMU prediction for {} frames: adding the wheel factor at {:.3f} s anyway", wheel_gated_streak_, now->stamp);
      }
      wheel_gated_streak_ = 0;
    }
  }
  // The next interval starts at this frame
  wheel_.trim(now->stamp);

  std::lock_guard<std::mutex> lock(status_mutex_);
  switch (result) {
    case WheelFactorResult::ADDED:
      status_.wheel_factors++;
      status_.wheel_readmitted += readmitted ? 1 : 0;
      break;
    case WheelFactorResult::GATED:
      status_.wheel_gated++;
      spdlog::debug("wheel odometry at {:.3f} s disagrees with the IMU prediction: no wheel factor", now->stamp);
      break;
    case WheelFactorResult::NO_DATA:
      status_.wheel_missing++;
      break;
  }
  return factors;
}

double OdometryEstimationLocalization::map_factor_weight() const {
  const auto& p = localization_params();
  const bool reduced = quality_.state() == LocalizationState::DEGRADED || quality_.state() == LocalizationState::LOST;
  return p.map_factor_weight * (reduced ? p.degraded_weight_scale : 1.0);
}

LocalizationStatus OdometryEstimationLocalization::status() const {
  std::lock_guard<std::mutex> lock(status_mutex_);
  return status_;
}

void OdometryEstimationLocalization::update_status(const glim::EstimationFrame::ConstPtr& frame, const double processing_time) {
  const MapQuality quality = evaluate_map_quality(frame);
  const LocalizationState state = quality_.update(quality.inlier_ratio);

  LocalizationStatus snapshot;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.state = state;
    status_.stamp = frame->stamp;
    status_.frame_id = frame->id;
    status_.num_tiles = static_cast<int>(current_map_factors_.tiles.size());
    status_.num_map_factors = static_cast<int>(current_map_factors_.factors.size());
    status_.inlier_ratio = quality.inlier_ratio;
    status_.residual = quality.residual;
    status_.processing_time = processing_time;
    snapshot = status_;
  }
  LocalizationCallbacks::on_status(snapshot);
}

}  // namespace glim_localization
