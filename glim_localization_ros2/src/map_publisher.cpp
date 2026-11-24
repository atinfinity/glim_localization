#include <glim_localization_ros2/map_publisher.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <unordered_map>

#include <Eigen/Core>

#include <sensor_msgs/msg/point_field.hpp>

namespace glim_localization_ros2 {

namespace {

// Voxel coordinates beyond this are not representable as int64 after the division; such points are skipped.
constexpr double kMaxVoxelCoord = 1e15;

struct VoxelKey {
  int64_t x, y, z;
  bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct VoxelKeyHash {
  size_t operator()(const VoxelKey& k) const {
    // Large primes as in the usual spatial hash; collisions only cost a comparison
    // (unsigned arithmetic: wraps instead of overflowing)
    const uint64_t h = (static_cast<uint64_t>(k.x) * 73856093ULL) ^ (static_cast<uint64_t>(k.y) * 19349669ULL) ^ (static_cast<uint64_t>(k.z) * 83492791ULL);
    return static_cast<size_t>(h);
  }
};

void check_voxel_size(double voxel_size) {
  if (!std::isfinite(voxel_size) || voxel_size < 0.0) {
    throw std::invalid_argument("map_cloud_voxel must be finite and not negative");
  }
}

}  // namespace

MapPublisherParams read_map_publisher_params(const glim::Config& config) {
  MapPublisherParams params;
  params.enabled = config.param<bool>("localization_ros", "publish_map", params.enabled);
  params.voxel_size = config.param<double>("localization_ros", "map_cloud_voxel", params.voxel_size);
  check_voxel_size(params.voxel_size);
  return params;
}

sensor_msgs::msg::PointCloud2 make_map_cloud(const std::vector<glim_localization::MapTile::ConstPtr>& tiles, double voxel_size, const std::string& frame_id) {
  check_voxel_size(voxel_size);
  const bool downsample = voxel_size > 0.0;

  // Sum and count per voxel, in the order the voxels are first met (deterministic output)
  std::vector<Eigen::Vector3d> sums;
  std::vector<size_t> counts;
  std::unordered_map<VoxelKey, size_t, VoxelKeyHash> index;
  for (const auto& tile : tiles) {
    if (!tile || !tile->points) {
      continue;
    }
    const auto& cloud = *tile->points;
    for (size_t i = 0; i < cloud.size(); i++) {
      const Eigen::Vector3d p = tile->T_world_origin * cloud.points[i].head<3>();
      if (!p.allFinite()) {
        continue;
      }
      if (!downsample) {
        sums.push_back(p);
        counts.push_back(1);
        continue;
      }
      const Eigen::Vector3d c = (p / voxel_size).array().floor();
      if ((c.array().abs() > kMaxVoxelCoord).any()) {
        continue;
      }
      const VoxelKey key{static_cast<int64_t>(c.x()), static_cast<int64_t>(c.y()), static_cast<int64_t>(c.z())};
      const auto [it, inserted] = index.emplace(key, sums.size());
      if (inserted) {
        sums.push_back(p);
        counts.push_back(1);
      } else {
        sums[it->second] += p;
        counts[it->second]++;
      }
    }
  }

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(sums.size());
  const char* names[] = {"x", "y", "z"};
  for (uint32_t k = 0; k < 3; k++) {
    sensor_msgs::msg::PointField field;
    field.name = names[k];
    field.offset = 4 * k;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    msg.fields.push_back(field);
  }
  msg.is_bigendian = false;
  msg.point_step = 12;
  msg.row_step = msg.point_step * msg.width;
  msg.is_dense = true;
  msg.data.resize(static_cast<size_t>(msg.row_step));
  for (size_t i = 0; i < sums.size(); i++) {
    const Eigen::Vector3f p = (sums[i] / static_cast<double>(counts[i])).cast<float>();
    std::memcpy(&msg.data[i * msg.point_step], p.data(), 3 * sizeof(float));
  }
  return msg;
}

MapPublisher::MapPublisher(
  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr& topics,
  const rclcpp::Clock::SharedPtr& clock,
  const std::string& frame_id,
  double voxel_size)
: clock_(clock),
  frame_id_(frame_id),
  voxel_size_(voxel_size) {
  if (frame_id_.empty()) {
    throw std::invalid_argument("the map frame name must not be empty");
  }
  check_voxel_size(voxel_size_);
  auto node_topics = topics;  // create_publisher does not take a const shared_ptr
  pub_ = rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(node_topics, "~/map",rclcpp::QoS(1).reliable().transient_local());
}

size_t MapPublisher::publish(const std::vector<glim_localization::MapTile::ConstPtr>& tiles) {
  auto msg = make_map_cloud(tiles, voxel_size_, frame_id_);
  msg.header.stamp = clock_->now();
  const size_t num_points = msg.width;
  pub_->publish(msg);
  return num_points;
}

}  // namespace glim_localization_ros2
