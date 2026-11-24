#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_points/types/point_cloud.hpp>

#include <glim_localization/map/map_manager.hpp>

namespace glim_localization {

struct MapFactorParams {
  double search_radius = 60.0;  ///< Tile search radius around the predicted pose [m]
  int max_submaps = 5;          ///< Maximum number of tiles
  double min_overlap = 0.1;     ///< Minimum overlap with the coarsest voxelmap of a tile
  int num_threads = 2;          ///< Threads per factor
};

struct MapFactors {
  gtsam::NonlinearFactorGraph factors;    ///< Unary VGICP factors on the pose key
  std::vector<MapTile::ConstPtr> tiles;   ///< Tiles that passed the overlap filter
};

/// The tiles near the predicted pose whose coarsest voxelmap overlaps the frame by at least params.min_overlap
/// (CPU voxelmaps; the GPU module uploads only these).
std::vector<MapTile::ConstPtr> select_map_tiles(
  const MapManager& map,
  const gtsam_points::PointCloud::ConstPtr& frame,
  const Eigen::Isometry3d& T_world_imu,
  const MapFactorParams& params);

/// Create unary VGICP factors against fixed map tiles. Holds no state.
/// @param map          Map to query
/// @param key          Pose key of the frame
/// @param frame        Frame in the IMU frame, with covariances
/// @param T_world_imu  Predicted pose, used for tile search and overlap filtering
/// @param params       Search parameters
/// @param weight       Factor weight. No factors are created if it is not positive
MapFactors create_map_factors(
  const MapManager& map,
  gtsam::Key key,
  const gtsam_points::PointCloud::ConstPtr& frame,
  const Eigen::Isometry3d& T_world_imu,
  const MapFactorParams& params,
  double weight = 1.0);

/// Fraction of frame points that fall in the finest voxelmap of any tile.
/// @return Overlap in [0, 1], or 0 if there are no tiles
double map_inlier_ratio(
  const std::vector<MapTile::ConstPtr>& tiles,
  const gtsam_points::PointCloud::ConstPtr& frame,
  const Eigen::Isometry3d& T_world_imu);

}  // namespace glim_localization
