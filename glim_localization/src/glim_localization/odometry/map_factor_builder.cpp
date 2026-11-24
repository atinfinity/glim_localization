#include <glim_localization/odometry/map_factor_builder.hpp>

#include <memory>

#include <gtsam/geometry/Pose3.h>

#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/types/gaussian_voxelmap.hpp>

#include <glim_localization/odometry/weighted_factor.hpp>

namespace glim_localization {

std::vector<MapTile::ConstPtr> select_map_tiles(
  const MapManager& map,
  const gtsam_points::PointCloud::ConstPtr& frame,
  const Eigen::Isometry3d& T_world_imu,
  const MapFactorParams& params) {
  std::vector<MapTile::ConstPtr> selected;
  if (!frame || frame->size() == 0) {
    return selected;
  }

  for (const auto& tile : map.query(T_world_imu, params.search_radius, params.max_submaps)) {
    if (tile->voxelmaps.empty()) {
      continue;
    }
    // The coarsest level keeps tiles alive under a large initial pose error.
    const Eigen::Isometry3d T_origin_imu = tile->T_world_origin.inverse() * T_world_imu;
    if (gtsam_points::overlap(tile->voxelmaps.back(), frame, T_origin_imu) < params.min_overlap) {
      continue;
    }
    selected.push_back(tile);
  }
  return selected;
}

MapFactors create_map_factors(
  const MapManager& map,
  gtsam::Key key,
  const gtsam_points::PointCloud::ConstPtr& frame,
  const Eigen::Isometry3d& T_world_imu,
  const MapFactorParams& params,
  double weight) {
  MapFactors result;
  if (!(weight > 0.0) || !frame || frame->size() == 0) {
    return result;
  }

  for (const auto& tile : select_map_tiles(map, frame, T_world_imu, params)) {
    const gtsam::Pose3 T_world_origin(tile->T_world_origin.matrix());
    for (const auto& voxelmap : tile->voxelmaps) {
      auto factor = std::make_shared<gtsam_points::IntegratedVGICPFactor>(T_world_origin, key, voxelmap, frame);
      factor->set_num_threads(params.num_threads);
      if (weight == 1.0) {
        result.factors.add(factor);
      } else {
        result.factors.add(std::make_shared<WeightedFactor>(factor, weight));
      }
    }
    result.tiles.push_back(tile);
  }
  return result;
}

double map_inlier_ratio(
  const std::vector<MapTile::ConstPtr>& tiles,
  const gtsam_points::PointCloud::ConstPtr& frame,
  const Eigen::Isometry3d& T_world_imu) {
  std::vector<gtsam_points::GaussianVoxelMap::ConstPtr> voxelmaps;
  std::vector<Eigen::Isometry3d> transforms;
  for (const auto& tile : tiles) {
    if (!tile->voxelmaps.empty()) {
      voxelmaps.push_back(tile->voxelmaps.front());
      transforms.push_back(tile->T_world_origin.inverse() * T_world_imu);
    }
  }
  if (voxelmaps.empty() || !frame || frame->size() == 0) {
    return 0.0;
  }
  return gtsam_points::overlap(voxelmaps, frame, transforms);
}

}  // namespace glim_localization
