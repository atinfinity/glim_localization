// Derived from koide3/glim include/glim/odometry/odometry_estimation_gpu.hpp at commit 1a197c999e4865f5cab7c9cd25474128413ebbca (MIT License, Copyright (c) 2024 koide3)
#pragma once

#include <memory>
#include <vector>

#include <glim_localization/map/gpu_tile_cache.hpp>
#include <glim_localization/odometry/odometry_estimation_localization.hpp>

namespace gtsam_points {
class StreamTempBufferRoundRobin;
class CUDAStream;
}  // namespace gtsam_points

namespace glim_localization {

/// GPU module parameters (CUDA builds only): the shared keys, the GPU tile cache, and upstream's
/// OdometryEstimationGPU keys for the scan-to-keyframe factors (use_keyframes).
struct OdometryEstimationLocalizationGPUParams : public OdometryEstimationLocalizationParams {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationLocalizationGPUParams();
  ~OdometryEstimationLocalizationGPUParams() override;

  enum class KeyframeUpdateStrategy { OVERLAP, DISPLACEMENT, ENTROPY };

  /// Map tiles kept on the GPU (least recently used are dropped). Must be at least map_max_submaps.
  int map_gpu_max_tiles;

  // Keyframes (use_keyframes), as upstream
  double voxel_resolution;
  double voxel_resolution_max;
  double voxel_resolution_dmin;
  double voxel_resolution_dmax;
  int voxelmap_levels;
  double voxelmap_scaling_factor;

  int max_num_keyframes;
  int full_connection_window_size;

  KeyframeUpdateStrategy keyframe_strategy;
  double keyframe_min_overlap;
  double keyframe_max_overlap;
  double keyframe_delta_trans;
  double keyframe_delta_rot;
  double keyframe_entropy_thresh;
};

/// GPU backend (libodometry_localization_gpu.so): unary IntegratedVGICPFactorGPU factors against map tiles uploaded
/// to the GPU on demand, batched by the smoother on the GPU. With use_keyframes it also adds upstream's
/// scan-to-keyframe factors. Status, resets and the IMU gap policy are those of the CPU module (the base class).
class OdometryEstimationLocalizationGPU : public OdometryEstimationLocalization {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Params = OdometryEstimationLocalizationGPUParams;

  /// Load the map at params.map_path.
  /// @throw std::runtime_error if the map cannot be loaded
  explicit OdometryEstimationLocalizationGPU(const Params& params = Params());

  /// Use an already loaded map.
  /// @throw std::invalid_argument if the map is null or a parameter is invalid
  OdometryEstimationLocalizationGPU(const Params& params, std::shared_ptr<const MapManager> map);

  ~OdometryEstimationLocalizationGPU() override;

  const char* backend_name() const override { return "gpu"; }

  /// Map tiles on the GPU (for tests and logs, not thread-safe)
  const GpuTileCache& tile_cache() const { return *tile_cache_; }

  /// Current keyframes; empty unless use_keyframes (for tests, not thread-safe)
  const std::vector<glim::EstimationFrame::ConstPtr>& keyframes() const { return keyframes_; }

protected:
  void create_frame(glim::EstimationFrame::Ptr& frame) override;
  gtsam::NonlinearFactorGraph create_factors(
    const int current,
    const gtsam_points::shared_ptr<gtsam::ImuFactor>& imu_factor,
    gtsam::Values& new_values) override;
  void update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors) override;

  void reset_backend() override;
  MapQuality evaluate_map_quality(const glim::EstimationFrame::ConstPtr& frame) override;

private:
  const Params& gpu_params() const;
  gtsam::NonlinearFactorGraph create_gpu_map_factors(int current);
  gtsam::NonlinearFactorGraph create_keyframe_factors(int current);

  void update_keyframes_overlap(int current);
  void update_keyframes_displacement(int current);
  void update_keyframes_entropy(const gtsam::NonlinearFactorGraph& matching_cost_factors, int current);

  // CUDA-related
  std::unique_ptr<gtsam_points::CUDAStream> stream;
  std::unique_ptr<gtsam_points::StreamTempBufferRoundRobin> stream_buffer_roundrobin;
  std::unique_ptr<GpuTileCache> tile_cache_;

  /// GPU tiles of current_map_factors_.tiles, same order
  std::vector<GpuTile::ConstPtr> current_gpu_tiles_;

  // Keyframe params
  int entropy_num_frames = 0;
  double entropy_running_average = 0.0;
  std::vector<glim::EstimationFrame::ConstPtr> keyframes_;
};

}  // namespace glim_localization
