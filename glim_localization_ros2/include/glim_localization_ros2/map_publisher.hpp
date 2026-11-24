#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <glim/util/config.hpp>
#include <glim_localization/map/map_tile.hpp>

namespace glim_localization_ros2 {

/// Settings of ~/map, from the "localization_ros" section of config_ros.json (publish_map, map_cloud_voxel)
struct MapPublisherParams {
  bool enabled = true;      ///< Publish ~/map
  double voxel_size = 0.5;  ///< Downsampling voxel [m]: one point (the centroid) per voxel. 0 keeps every point
};

/// @throw std::invalid_argument if map_cloud_voxel is negative or not finite
MapPublisherParams read_map_publisher_params(const glim::Config& config_ros);

/// Bytes per point of ~/map (float32 x, y, z)
constexpr size_t kMapCloudPointStep = 12;
/// Default cap on the ~/map payload: 1 GiB (about 89 million points). The message itself allows at most 4 GiB (uint32 row_step).
constexpr size_t kMaxMapCloudBytes = size_t(1) << 30;
/// With map_cloud_voxel 0, maps above this many points get a warning (memory and time of the build and of the send)
constexpr size_t kLargeMapCloudPoints = 10'000'000;

/// The number of points of the tiles (null tiles count as empty)
size_t count_map_points(const std::vector<glim_localization::MapTile::ConstPtr>& tiles);

/// @throw std::length_error if a cloud of num_points does not fit in max_bytes (capped at the uint32 limit of the message)
void check_map_cloud_size(size_t num_points, size_t max_bytes = kMaxMapCloudBytes);

/// The points of the tiles in the map frame (T_world_origin * p) as an unorganized PointCloud2 with float32 x, y, z.
/// With voxel_size > 0, each voxel of that size keeps one point, the centroid of its points, in the order the voxels
/// are first met (tiles in the given order). Non-finite points are skipped. The header stamp is left zero.
/// With voxel_size 0 the points are written straight into the message (12 bytes per point); with downsampling the
/// build also holds about 40 bytes per occupied voxel.
/// @throw std::invalid_argument if voxel_size is negative or not finite
/// @throw std::length_error if the cloud would exceed max_bytes (see check_map_cloud_size); nothing large is allocated then
sensor_msgs::msg::PointCloud2 make_map_cloud(
  const std::vector<glim_localization::MapTile::ConstPtr>& tiles,
  double voxel_size,
  const std::string& frame_id,
  size_t max_bytes = kMaxMapCloudBytes);

/// Publishes the loaded map once on ~/map, latched (reliable, transient local, depth 1), so that a late subscriber
/// such as RViz still gets it. ROS only; the core does not depend on it.
class MapPublisher {
public:
  /// @throw std::invalid_argument if frame_id is empty or voxel_size is negative or not finite
  MapPublisher(
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr& topics,
    const rclcpp::Clock::SharedPtr& clock,
    const std::string& frame_id,
    double voxel_size);

  /// Build and publish the cloud of the tiles. @return the number of published points
  /// @throw std::length_error if the cloud is too large (nothing is published then)
  size_t publish(const std::vector<glim_localization::MapTile::ConstPtr>& tiles);

private:
  rclcpp::Clock::SharedPtr clock_;
  std::string frame_id_;
  double voxel_size_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace glim_localization_ros2
