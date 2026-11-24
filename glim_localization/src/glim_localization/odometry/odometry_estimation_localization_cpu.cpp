// Derived from koide3/glim src/glim/odometry/odometry_estimation_cpu.cpp at commit 1a197c999e4865f5cab7c9cd25474128413ebbca (MIT License, Copyright (c) 2024 koide3)
#include <glim_localization/odometry/odometry_estimation_localization_cpu.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/util/config.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/task_arena.h>
#endif

namespace glim_localization {

using gtsam::symbol_shorthand::X;

OdometryEstimationLocalizationCPUParams::OdometryEstimationLocalizationCPUParams() {
  glim::Config config(glim::GlobalConfig::get_config_path("config_odometry"));
  const std::string section = "odometry_estimation";

  max_iterations = config.param<int>(section, "max_iterations", 5);
  lru_thresh = config.param<int>(section, "lru_thresh", 100);
  target_downsampling_rate = config.param<double>(section, "target_downsampling_rate", 0.1);
  vgicp_resolution = config.param<double>(section, "vgicp_resolution", 0.5);
  vgicp_voxelmap_levels = config.param<int>(section, "vgicp_voxelmap_levels", 1);
  vgicp_voxelmap_scaling_factor = config.param<double>(section, "vgicp_voxelmap_scaling_factor", 2.0);
}

OdometryEstimationLocalizationCPUParams::~OdometryEstimationLocalizationCPUParams() {}

OdometryEstimationLocalizationCPU::OdometryEstimationLocalizationCPU(const Params& params)
: OdometryEstimationLocalizationCPU(params, load_localization_map(params)) {}

OdometryEstimationLocalizationCPU::OdometryEstimationLocalizationCPU(const Params& params, std::shared_ptr<const MapManager> map)
: OdometryEstimationLocalization(std::make_unique<Params>(params), std::move(map)) {
  init_local_model();
}

OdometryEstimationLocalizationCPU::~OdometryEstimationLocalizationCPU() {}

const OdometryEstimationLocalizationCPU::Params& OdometryEstimationLocalizationCPU::cpu_params() const {
  return *static_cast<const Params*>(params.get());
}

void OdometryEstimationLocalizationCPU::reset_backend() {
  init_local_model();
}

OdometryEstimationLocalization::MapQuality OdometryEstimationLocalizationCPU::evaluate_map_quality(const glim::EstimationFrame::ConstPtr& frame) {
  MapQuality quality;
  quality.inlier_ratio = map_inlier_ratio(current_map_factors_.tiles, frame->frame, frame->T_world_imu);

  // Use separate factors: those in the smoother keep correspondences from their last linearization.
  double residual = 0.0;
  int num_inliers = 0;
  gtsam::Values values;
  values.insert(X(frame->id), gtsam::Pose3(frame->T_world_imu.matrix()));
  for (const auto& tile : current_map_factors_.tiles) {
    gtsam_points::IntegratedVGICPFactor factor(gtsam::Pose3(tile->T_world_origin.matrix()), X(frame->id), tile->voxelmaps.front(), frame->frame);
    factor.set_num_threads(cpu_params().map_factors.num_threads);
    factor.linearize(values);
    residual += factor.error(values);
    num_inliers += factor.num_inliers();
  }
  quality.residual = num_inliers > 0 ? residual / num_inliers : 0.0;
  return quality;
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizationCPU::create_factors(
  const int current,
  const gtsam_points::shared_ptr<gtsam::ImuFactor>& /*imu_factor*/,
  gtsam::Values& new_values) {
  const auto& p = cpu_params();
  current_map_factors_ = create_map_factors(*map(), X(current), frames[current]->frame, frames[current]->T_world_imu, p.map_factors, map_factor_weight());
  if (!p.use_keyframes) {
    return current_map_factors_.factors;
  }

  // Map factors above use the IMU prediction, before the local model refines it.
  gtsam::NonlinearFactorGraph factors = current_map_factors_.factors;
  factors.add(create_local_model_factors(current, new_values));
  return factors;
}

void OdometryEstimationLocalizationCPU::init_local_model() {
  const auto& p = cpu_params();
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
  const auto& p = cpu_params();
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
  const auto& p = cpu_params();
  auto frame = frames[current]->frame;
  if (current >= 5) {
    frame = gtsam_points::random_sampling(frames[current]->frame, p.target_downsampling_rate, mt_);
  }

  auto transformed = gtsam_points::transform(frame, T_target_imu);
  for (auto& voxelmap : target_voxelmaps_) {
    voxelmap->insert(*transformed);
  }
}

}  // namespace glim_localization
