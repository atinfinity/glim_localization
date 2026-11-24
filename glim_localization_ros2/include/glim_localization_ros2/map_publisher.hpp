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

/// The points of the tiles in the map frame (T_world_origin * p) as an unorganized PointCloud2 with float32 x, y, z.
/// With voxel_size > 0, each voxel of that size keeps one point, the centroid of its points, in the order the voxels
/// are first met (tiles in the given order). Non-finite points are skipped. The header stamp is left zero.
sensor_msgs::msg::PointCloud2 make_map_cloud(const std::vector<glim_localization::MapTile::ConstPtr>& tiles, double voxel_size, const std::string& frame_id);

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
  size_t publish(const std::vector<glim_localization::MapTile::ConstPtr>& tiles);

private:
  rclcpp::Clock::SharedPtr clock_;
  std::string frame_id_;
  double voxel_size_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace glim_localization_ros2
