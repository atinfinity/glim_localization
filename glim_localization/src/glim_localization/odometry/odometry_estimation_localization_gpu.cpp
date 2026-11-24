// Derived from koide3/glim src/glim/odometry/odometry_estimation_gpu.cpp at commit 1a197c999e4865f5cab7c9cd25474128413ebbca (MIT License, Copyright (c) 2024 koide3)
#include <glim_localization/odometry/odometry_estimation_localization_gpu.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include <spdlog/spdlog.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam_points/cuda/cuda_stream.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu.hpp>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>

#include <glim/odometry/callbacks.hpp>
#include <glim/util/config.hpp>

#include <glim_localization/odometry/gpu_linearization_hook.hpp>
#include <glim_localization/odometry/map_factor_builder.hpp>
#include <glim_localization/odometry/weighted_vgicp_factor_gpu.hpp>

namespace glim_localization {

using Callbacks = glim::OdometryEstimationCallbacks;
using gtsam::symbol_shorthand::X;

OdometryEstimationLocalizationGPUParams::OdometryEstimationLocalizationGPUParams() {
  glim::Config config(glim::GlobalConfig::get_config_path("config_odometry"));
  const std::string section = "odometry_estimation";

  map_gpu_max_tiles = config.param<int>(section, "map_gpu_max_tiles", 32);

  voxel_resolution = config.param<double>(section, "voxel_resolution", 0.5);
  voxel_resolution_max = config.param<double>(section, "voxel_resolution_max", voxel_resolution);
  voxel_resolution_dmin = config.param<double>(section, "voxel_resolution_dmin", 4.0);
  voxel_resolution_dmax = config.param<double>(section, "voxel_resolution_dmax", 12.0);

  voxelmap_levels = config.param<int>(section, "voxelmap_levels", 2);
  voxelmap_scaling_factor = config.param<double>(section, "voxelmap_scaling_factor", 2.0);

  max_num_keyframes = config.param<int>(section, "max_num_keyframes", 10);
  full_connection_window_size = config.param<int>(section, "full_connection_window_size", 3);

  const std::string strategy = config.param<std::string>(section, "keyframe_update_strategy", "OVERLAP");
  if (strategy == "OVERLAP") {
    keyframe_strategy = KeyframeUpdateStrategy::OVERLAP;
  } else if (strategy == "DISPLACEMENT") {
    keyframe_strategy = KeyframeUpdateStrategy::DISPLACEMENT;
  } else if (strategy == "ENTROPY") {
    keyframe_strategy = KeyframeUpdateStrategy::ENTROPY;
  } else {
    // Upstream logs and leaves it unset; fail closed instead.
    throw std::invalid_argument("unknown keyframe_update_strategy " + strategy);
  }

  keyframe_min_overlap = config.param<double>(section, "keyframe_min_overlap", 0.1);
  keyframe_max_overlap = config.param<double>(section, "keyframe_max_overlap", 0.9);
  keyframe_delta_trans = config.param<double>(section, "keyframe_delta_trans", 1.0);
  keyframe_delta_rot = config.param<double>(section, "keyframe_delta_rot", 0.25);
  keyframe_entropy_thresh = config.param<double>(section, "keyframe_entropy_thresh", 0.99);
}

OdometryEstimationLocalizationGPUParams::~OdometryEstimationLocalizationGPUParams() {}

namespace {

/// Validate the GPU parameters, then register the GPU linearization hook: the smoother is created in the base
/// constructor, and it picks up the hooks registered at that time.
std::unique_ptr<OdometryEstimationLocalizationGPUParams> gpu_params_with_hook(const OdometryEstimationLocalizationGPUParams& params) {
  if (params.map_gpu_max_tiles < params.map_factors.max_submaps) {
    // A frame would evict its own tiles and upload them again on every frame.
    throw std::invalid_argument("map_gpu_max_tiles must be at least map_max_submaps");
  }
  if (params.use_keyframes && (params.voxelmap_levels < 1 || !(params.voxel_resolution > 0.0) || params.max_num_keyframes < 1)) {
    throw std::invalid_argument("invalid keyframe parameters");
  }
  ensure_gpu_linearization_hook();
  return std::make_unique<OdometryEstimationLocalizationGPUParams>(params);
}

}  // namespace

OdometryEstimationLocalizationGPU::OdometryEstimationLocalizationGPU(const Params& params)
: OdometryEstimationLocalizationGPU(params, load_localization_map(params)) {}

OdometryEstimationLocalizationGPU::OdometryEstimationLocalizationGPU(const Params& params, std::shared_ptr<const MapManager> map)
: OdometryEstimationLocalization(gpu_params_with_hook(params), std::move(map)) {
  stream.reset(new gtsam_points::CUDAStream());
  stream_buffer_roundrobin.reset(new gtsam_points::StreamTempBufferRoundRobin());
  tile_cache_.reset(new GpuTileCache(params.map_gpu_max_tiles, *stream));
}

OdometryEstimationLocalizationGPU::~OdometryEstimationLocalizationGPU() {
  // Drop everything that holds GPU data before the streams go away.
  frames.clear();
  keyframes_.clear();
  smoother.reset();
  current_map_factors_ = MapFactors();
  current_gpu_tiles_.clear();
  tile_cache_.reset();
}

const OdometryEstimationLocalizationGPU::Params& OdometryEstimationLocalizationGPU::gpu_params() const {
  return *static_cast<const Params*>(params.get());
}

void OdometryEstimationLocalizationGPU::create_frame(glim::EstimationFrame::Ptr& new_frame) {
  const auto& p = gpu_params();

  // Map and keyframe factors need the points and covariances on the GPU.
  new_frame->frame = gtsam_points::PointCloudGPU::clone(*new_frame->frame);
  if (!p.use_keyframes) {
    return;
  }

  // Adaptively determine the voxel resolution based on the median distance
  const int max_scan_count = 256;
  const double dist_median = gtsam_points::median_distance(new_frame->frame, max_scan_count);
  const double r = std::max(0.0, std::min(1.0, (dist_median - p.voxel_resolution_dmin) / (p.voxel_resolution_dmax - p.voxel_resolution_dmin)));
  const double base_resolution = p.voxel_resolution + r * (p.voxel_resolution_max - p.voxel_resolution);

  for (int i = 0; i < p.voxelmap_levels; i++) {
    if (!new_frame->frame->size()) {
      break;
    }

    const double resolution = base_resolution * std::pow(p.voxelmap_scaling_factor, i);
    auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(resolution, 8192 * 2, 10, 1e-3, *stream);
    voxelmap->insert(*new_frame->frame);
    new_frame->voxelmaps.push_back(voxelmap);
  }
}

void OdometryEstimationLocalizationGPU::update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors) {
  OdometryEstimationLocalization::update_frames(current, new_factors);

  const auto& p = gpu_params();
  if (!p.use_keyframes) {
    return;
  }

  switch (p.keyframe_strategy) {
    case Params::KeyframeUpdateStrategy::OVERLAP:
      update_keyframes_overlap(current);
      break;
    case Params::KeyframeUpdateStrategy::DISPLACEMENT:
      update_keyframes_displacement(current);
      break;
    case Params::KeyframeUpdateStrategy::ENTROPY:
      update_keyframes_entropy(new_factors, current);
      break;
  }

  Callbacks::on_update_keyframes(keyframes_);
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizationGPU::create_factors(
  const int current,
  const gtsam_points::shared_ptr<gtsam::ImuFactor>& /*imu_factor*/,
  gtsam::Values& /*new_values*/) {
  gtsam::NonlinearFactorGraph factors = create_gpu_map_factors(current);
  if (gpu_params().use_keyframes) {
    factors.add(create_keyframe_factors(current));
  }
  return factors;
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizationGPU::create_gpu_map_factors(const int current) {
  const auto& p = gpu_params();
  current_map_factors_ = MapFactors();
  current_gpu_tiles_.clear();

  const double weight = map_factor_weight();
  const auto& frame = frames[current]->frame;
  if (!(weight > 0.0) || !frame || frame->size() == 0) {
    return gtsam::NonlinearFactorGraph();
  }

  // Tile selection and the overlap filter run on the CPU tiles, so only the selected tiles are uploaded.
  for (const auto& tile : select_map_tiles(*map(), frame, frames[current]->T_world_imu, p.map_factors)) {
    const auto gpu_tile = tile_cache_->get(tile);
    const gtsam::Pose3 T_world_origin(tile->T_world_origin.matrix());

    auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
    const auto& stream = stream_buffer.first;
    const auto& buffer = stream_buffer.second;
    for (const auto& voxelmap : gpu_tile->voxelmaps) {
      if (weight == 1.0) {
        current_map_factors_.factors.add(
          gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(T_world_origin, X(current), voxelmap, frame, stream, buffer));
      } else {
        current_map_factors_.factors.add(
          gtsam::make_shared<WeightedVGICPFactorGPU>(T_world_origin, X(current), voxelmap, frame, weight, stream, buffer));
      }
    }
    current_map_factors_.tiles.push_back(tile);
    current_gpu_tiles_.push_back(gpu_tile);
  }
  return current_map_factors_.factors;
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizationGPU::create_keyframe_factors(const int current) {
  gtsam::NonlinearFactorGraph factors;
  if (current == 0 || !frames[current]->frame->size()) {
    return factors;
  }

  const auto create_binary_factor = [this](
                                      gtsam::NonlinearFactorGraph& factors,
                                      gtsam::Key target_key,
                                      gtsam::Key source_key,
                                      const glim::EstimationFrame::ConstPtr& target,
                                      const glim::EstimationFrame::ConstPtr& source) {
    auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
    const auto& stream = stream_buffer.first;
    const auto& buffer = stream_buffer.second;

    for (const auto& voxelmap : target->voxelmaps) {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(target_key, source_key, voxelmap, source->frame, stream, buffer);
      factor->set_enable_surface_validation(true);
      factors.add(factor);
    }
  };

  const auto create_unary_factor = [this](
                                     gtsam::NonlinearFactorGraph& factors,
                                     const gtsam::Pose3& fixed_target_pose,
                                     gtsam::Key source_key,
                                     const glim::EstimationFrame::ConstPtr& target,
                                     const glim::EstimationFrame::ConstPtr& source) {
    auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
    const auto& stream = stream_buffer.first;
    const auto& buffer = stream_buffer.second;

    for (const auto& voxelmap : target->voxelmaps) {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(fixed_target_pose, source_key, voxelmap, source->frame, stream, buffer);
      factor->set_enable_surface_validation(true);
      factors.add(factor);
    }
  };

  // A frame still in the smoother window. The same range test as has_index, which gtsam_points 1.2.2 lacks
  // (third_party/patches/glim_cuda); marginalized frames are null.
  const auto in_window = [this](const int index) {
    return index >= 0 && index >= static_cast<int>(frames.size() - frames.inner_size()) && index < static_cast<int>(frames.size()) && frames[index];
  };

  const auto& p = gpu_params();

  // There must be at least one factor between consecutive frames
  for (int target = current - p.full_connection_window_size; target < current; target++) {
    if (!in_window(target)) {
      continue;
    }

    create_binary_factor(factors, X(target), X(current), frames[target], frames[current]);
  }

  for (const auto& keyframe : keyframes_) {
    if (keyframe->id >= current - p.full_connection_window_size) {
      // There already exists a factor
      continue;
    }

    double span = frames[current]->stamp - keyframe->stamp;
    if (span > p.smoother_lag - 0.1 || !in_window(keyframe->id)) {
      // Create unary factor
      const gtsam::Pose3 key_T_world_imu(keyframe->T_world_imu.matrix());
      create_unary_factor(factors, key_T_world_imu, X(current), keyframe, frames[current]);
    } else {
      // Create binary factor
      const int target = keyframe->id;
      create_binary_factor(factors, X(target), X(current), frames[target], frames[current]);
    }
  }

  return factors;
}

void OdometryEstimationLocalizationGPU::reset_backend() {
  // Keyframe ids refer to the frames of the old run.
  keyframes_.clear();
  entropy_num_frames = 0;
  entropy_running_average = 0.0;
  current_gpu_tiles_.clear();
}

OdometryEstimationLocalization::MapQuality OdometryEstimationLocalizationGPU::evaluate_map_quality(const glim::EstimationFrame::ConstPtr& frame) {
  MapQuality quality;
  // The same CPU overlap as the CPU module, so the two report the same inlier ratio for the same estimate.
  quality.inlier_ratio = map_inlier_ratio(current_map_factors_.tiles, frame->frame, frame->T_world_imu);
  if (current_gpu_tiles_.empty()) {
    return quality;
  }

  // Use separate factors: those in the smoother keep correspondences from their last linearization.
  // They are evaluated as one batch, as the smoother does.
  gtsam::Values values;
  values.insert(X(frame->id), gtsam::Pose3(frame->T_world_imu.matrix()));
  gtsam::NonlinearFactorGraph graph;
  std::vector<std::shared_ptr<gtsam_points::IntegratedVGICPFactorGPU>> factors;
  for (const auto& gpu_tile : current_gpu_tiles_) {
    auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
    auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(
      gtsam::Pose3(gpu_tile->tile->T_world_origin.matrix()),
      X(frame->id),
      gpu_tile->voxelmaps.front(),
      frame->frame,
      stream_buffer.first,
      stream_buffer.second);
    graph.add(factor);
    factors.push_back(factor);
  }

  gtsam_points::NonlinearFactorSetGPU factor_set;
  factor_set.add(graph);
  factor_set.linearize(values);
  factor_set.error(values);

  double residual = 0.0;
  int num_inliers = 0;
  for (const auto& factor : factors) {
    residual += factor->error(values);
    num_inliers += factor->num_inliers();
  }
  quality.residual = num_inliers > 0 ? residual / num_inliers : 0.0;
  return quality;
}

/**
 * @brief Keyframe management based on an overlap metric
 * @ref   Koide et al., "Globally Consistent and Tightly Coupled 3D LiDAR Inertial Mapping", ICRA2022
 */
void OdometryEstimationLocalizationGPU::update_keyframes_overlap(int current) {
  const auto& p = gpu_params();

  if (!frames[current]->frame->size()) {
    return;
  }

  if (keyframes_.empty()) {
    keyframes_.push_back(frames[current]);
    return;
  }

  std::vector<gtsam_points::GaussianVoxelMap::ConstPtr> keyframes(keyframes_.size());
  std::vector<Eigen::Isometry3d> delta_from_keyframes(keyframes_.size());
  for (int i = 0; i < keyframes_.size(); i++) {
    keyframes[i] = keyframes_[i]->voxelmaps.back();
    delta_from_keyframes[i] = keyframes_[i]->T_world_imu.inverse() * frames[current]->T_world_imu;
  }

  const double overlap = gtsam_points::overlap_gpu(keyframes, frames[current]->frame, delta_from_keyframes, *stream);
  if (overlap > p.keyframe_max_overlap) {
    return;
  }

  const auto& new_keyframe = frames[current];
  keyframes_.push_back(new_keyframe);

  if (keyframes_.size() <= p.max_num_keyframes) {
    return;
  }

  std::vector<glim::EstimationFrame::ConstPtr> marginalized_keyframes;

  // Remove keyframes without overlap to the new keyframe
  for (int i = 0; i < keyframes_.size(); i++) {
    const Eigen::Isometry3d delta = keyframes_[i]->T_world_imu.inverse() * new_keyframe->T_world_imu;
    const double overlap = gtsam_points::overlap_gpu(keyframes_[i]->voxelmaps.back(), new_keyframe->frame, delta, *stream);
    if (overlap < p.keyframe_min_overlap) {
      marginalized_keyframes.push_back(keyframes_[i]);
      keyframes_.erase(keyframes_.begin() + i);
      i--;
    }
  }

  if (keyframes_.size() <= p.max_num_keyframes) {
    Callbacks::on_marginalized_keyframes(marginalized_keyframes);
    return;
  }

  // Remove the keyframe with the minimum score
  std::vector<double> scores(keyframes_.size() - 1, 0.0);
  for (int i = 0; i < keyframes_.size() - 1; i++) {
    const auto& keyframe = keyframes_[i];
    const double overlap_latest =
      gtsam_points::overlap_gpu(keyframe->voxelmaps.back(), new_keyframe->frame, keyframe->T_world_imu.inverse() * new_keyframe->T_world_imu, *stream);

    std::vector<gtsam_points::GaussianVoxelMap::ConstPtr> other_keyframes;
    std::vector<Eigen::Isometry3d> delta_from_others;
    for (int j = 0; j < keyframes_.size() - 1; j++) {
      if (i == j) {
        continue;
      }

      const auto& other = keyframes_[j];
      other_keyframes.push_back(other->voxelmaps.back());
      delta_from_others.push_back(other->T_world_imu.inverse() * keyframe->T_world_imu);
    }

    const double overlap_others = gtsam_points::overlap_gpu(other_keyframes, keyframe->frame, delta_from_others, *stream);
    scores[i] = overlap_latest * (1.0 - overlap_others);
  }

  double min_score = scores[0];
  int frame_to_eliminate = 0;
  for (int i = 1; i < scores.size(); i++) {
    if (scores[i] < min_score) {
      min_score = scores[i];
      frame_to_eliminate = i;
    }
  }

  marginalized_keyframes.push_back(keyframes_[frame_to_eliminate]);
  keyframes_.erase(keyframes_.begin() + frame_to_eliminate);
  Callbacks::on_marginalized_keyframes(marginalized_keyframes);
}

/**
 * @brief Keyframe management based on displacement criteria
 * @ref   Engel et al., "Direct Sparse Odometry", IEEE Trans. PAMI, 2018
 */
void OdometryEstimationLocalizationGPU::update_keyframes_displacement(int current) {
  const auto& p = gpu_params();

  if (keyframes_.empty()) {
    keyframes_.push_back(frames[current]);
    return;
  }

  const Eigen::Isometry3d delta_from_last = keyframes_.back()->T_world_imu.inverse() * frames[current]->T_world_imu;
  const double delta_trans = delta_from_last.translation().norm();
  const double delta_rot = Eigen::AngleAxisd(delta_from_last.linear()).angle();

  if (delta_trans < p.keyframe_delta_trans && delta_rot < p.keyframe_delta_rot) {
    return;
  }

  const auto& new_keyframe = frames[current];
  keyframes_.push_back(new_keyframe);

  if (keyframes_.size() <= p.max_num_keyframes) {
    return;
  }

  for (int i = 0; i < keyframes_.size() - 1; i++) {
    const Eigen::Isometry3d delta = keyframes_[i]->T_world_imu.inverse() * new_keyframe->T_world_imu;
    const double overlap = gtsam_points::overlap_gpu(keyframes_[i]->voxelmaps.back(), new_keyframe->frame, delta, *stream);

    if (overlap < 0.01) {
      std::vector<glim::EstimationFrame::ConstPtr> marginalized_keyframes;
      marginalized_keyframes.push_back(keyframes_[i]);
      keyframes_.erase(keyframes_.begin() + i);
      Callbacks::on_marginalized_keyframes(marginalized_keyframes);
      return;
    }
  }

  const int leave_window = 2;
  const double eps = 1e-3;
  std::vector<double> scores(keyframes_.size() - 1, 0.0);
  for (int i = leave_window; i < keyframes_.size() - 1; i++) {
    double sum_inv_dist = 0.0;
    for (int j = 0; j < keyframes_.size() - 1; j++) {
      if (i == j) {
        continue;
      }

      const double dist = (keyframes_[i]->T_world_imu.translation() - keyframes_[j]->T_world_imu.translation()).norm();
      sum_inv_dist += 1.0 / (dist + eps);
    }

    const double d0 = (keyframes_[i]->T_world_imu.translation() - new_keyframe->T_world_imu.translation()).norm();
    scores[i] = std::sqrt(d0) * sum_inv_dist;
  }

  const auto max_score_loc = std::max_element(scores.begin(), scores.end());
  const int max_score_index = std::distance(scores.begin(), max_score_loc);

  std::vector<glim::EstimationFrame::ConstPtr> marginalized_keyframes;
  marginalized_keyframes.push_back(keyframes_[max_score_index]);
  keyframes_.erase(keyframes_.begin() + max_score_index);
  Callbacks::on_marginalized_keyframes(marginalized_keyframes);
}

/**
 * @brief Keyframe management based on entropy measure
 * @ref   Kuo et al., "Redesigning SLAM for Arbitrary Multi-Camera Systems", ICRA2020
 */
void OdometryEstimationLocalizationGPU::update_keyframes_entropy(const gtsam::NonlinearFactorGraph& matching_cost_factors, int current) {
  const auto& p = gpu_params();

  gtsam::Values values = smoother->calculateEstimate();

  gtsam::NonlinearFactorGraph valid_factors;
  for (const auto& factor : matching_cost_factors) {
    bool valid = std::all_of(factor->keys().begin(), factor->keys().end(), [&](const gtsam::Key key) { return values.exists(key); });
    if (!valid) {
      continue;
    }

    valid_factors.push_back(factor->clone());
  }

  gtsam_points::NonlinearFactorSetGPU factor_set;
  factor_set.add(valid_factors);
  factor_set.linearize(values);
  auto linearized = valid_factors.linearize(values);

  gtsam::Matrix6 H = linearized->hessianBlockDiagonal()[X(current)];
  double negative_entropy = std::log(H.determinant());

  entropy_num_frames++;
  entropy_running_average += (negative_entropy - entropy_running_average) / entropy_num_frames;

  if (!keyframes_.empty() && negative_entropy > entropy_running_average * p.keyframe_entropy_thresh) {
    return;
  }

  entropy_num_frames = 0;
  entropy_running_average = 0.0;

  const auto& new_keyframe = frames[current];
  keyframes_.push_back(new_keyframe);

  if (keyframes_.size() <= p.max_num_keyframes) {
    return;
  }

  std::vector<glim::EstimationFrame::ConstPtr> marginalized_keyframes;
  marginalized_keyframes.push_back(keyframes_.front());
  keyframes_.erase(keyframes_.begin());
  Callbacks::on_marginalized_keyframes(marginalized_keyframes);
}

}  // namespace glim_localization
