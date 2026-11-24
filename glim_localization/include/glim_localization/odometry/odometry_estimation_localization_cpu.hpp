// Derived from koide3/glim include/glim/odometry/odometry_estimation_cpu.hpp at commit 1a197c999e4865f5cab7c9cd25474128413ebbca (MIT License, Copyright (c) 2024 koide3)
#pragma once

#include <memory>
#include <random>
#include <vector>

#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>

#include <glim_localization/odometry/odometry_estimation_localization.hpp>

namespace glim_localization {

struct OdometryEstimationLocalizationCPUParams : public OdometryEstimationLocalizationParams {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationLocalizationCPUParams();
  ~OdometryEstimationLocalizationCPUParams() override;

  // Local model (use_keyframes)
  int max_iterations;
  int lru_thresh;
  double target_downsampling_rate;
  double vgicp_resolution;
  int vgicp_voxelmap_levels;
  double vgicp_voxelmap_scaling_factor;
};

/// CPU backend: unary CPU VGICP factors against the map tiles (libodometry_localization_cpu.so).
class OdometryEstimationLocalizationCPU : public OdometryEstimationLocalization {
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

  const char* backend_name() const override { return "cpu"; }

protected:
  gtsam::NonlinearFactorGraph create_factors(
    const int current,
    const gtsam_points::shared_ptr<gtsam::ImuFactor>& imu_factor,
    gtsam::Values& new_values) override;

  void reset_backend() override;
  MapQuality evaluate_map_quality(const glim::EstimationFrame::ConstPtr& frame) override;

private:
  const Params& cpu_params() const;
  void init_local_model();
  gtsam::NonlinearFactorGraph create_local_model_factors(int current, gtsam::Values& new_values);
  void update_local_model(int current, const Eigen::Isometry3d& T_target_imu);

  Eigen::Isometry3d last_T_target_imu_ = Eigen::Isometry3d::Identity();
  std::vector<gtsam_points::GaussianVoxelMapCPU::Ptr> target_voxelmaps_;
  std::mt19937 mt_;
};

}  // namespace glim_localization
