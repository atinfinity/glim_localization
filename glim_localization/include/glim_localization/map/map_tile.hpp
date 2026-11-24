#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam_points/types/gaussian_voxelmap.hpp>
#include <gtsam_points/types/point_cloud.hpp>

namespace glim_localization {

/// An immutable map submap with its multi-resolution voxelmaps.
struct MapTile {
  using ConstPtr = std::shared_ptr<const MapTile>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int id = -1;                                                      ///< Index in the dump, X(id)
  Eigen::Isometry3d T_world_origin = Eigen::Isometry3d::Identity();
  gtsam_points::PointCloud::ConstPtr points;                        ///< Points in the origin frame
  std::vector<gtsam_points::GaussianVoxelMap::ConstPtr> voxelmaps;  ///< Fine to coarse
  Eigen::Vector3d bbox_min = Eigen::Vector3d::Zero();               ///< World frame AABB
  Eigen::Vector3d bbox_max = Eigen::Vector3d::Zero();               ///< World frame AABB
};

}  // namespace glim_localization
