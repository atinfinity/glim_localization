#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam_points/types/point_cloud.hpp>

namespace glim_localization {

/// A submap read from storage, reduced to what localization needs.
struct LoadedSubmap {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int id = -1;                                      ///< Index in the dump, X(id) in values.bin
  Eigen::Isometry3d T_world_origin = Eigen::Isometry3d::Identity();
  gtsam_points::PointCloud::Ptr points;             ///< Points with covariances in the origin frame
};

using LoadedSubmaps = std::vector<LoadedSubmap, Eigen::aligned_allocator<LoadedSubmap>>;

/// Load a GLIM dump or a compiled map (a directory with map_index.json).
/// Any invalid submap fails the whole load, leaving `submaps` unchanged.
/// Submaps without points are skipped with a warning and their ids are kept.
bool load_map(const std::string& path, LoadedSubmaps& submaps);

/// Write a compiled map holding only what localization needs.
/// Fails without writing if `path` exists and is not empty. The result appears atomically.
/// `source` is recorded in map_index.json ("glim_dump", "pcd" or "ply") and is not read back.
bool save_compiled_map(const std::string& path, const LoadedSubmaps& submaps, const std::string& source = "glim_dump");

}  // namespace glim_localization
