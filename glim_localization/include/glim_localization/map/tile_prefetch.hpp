#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim_localization/map/map_manager.hpp>

namespace glim_localization {

struct TilePrefetchParams {
  double search_radius = 60.0;  ///< The tile search radius of the map factors [m]
  int max_submaps = 5;          ///< Tiles per query, as the map factors take them
  double lookahead = 1.0;       ///< Time ahead of the current pose to prefetch for [s]
  int max_tiles = 10;           ///< At most this many candidates
};

/// The tiles the map factors are likely to need soon, most urgent first.
///
/// The predicted path is the segment from the current position to the position `lookahead` seconds ahead at the
/// current velocity (a non-finite velocity counts as zero). It is sampled at its start, its end and every half search
/// radius in between; at each sample, the `max_submaps` tiles nearest within `search_radius` are candidates, as the
/// map factors select them. Candidates are ordered by the distance of their origin to the segment, then to the current
/// position, then by id, and the first `max_tiles` are returned. CPU only.
std::vector<MapTile::ConstPtr> select_prefetch_tiles(
  const MapManager& map,
  const Eigen::Isometry3d& T_world_imu,
  const Eigen::Vector3d& v_world_imu,
  const TilePrefetchParams& params);

}  // namespace glim_localization
