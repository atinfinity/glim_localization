// In-process test of ~/map: the map cloud conversion and its latched publication. No bag, no display.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <glim_localization_ros2/map_publisher.hpp>

namespace {

using glim_localization::MapTile;
using glim_localization_ros2::make_map_cloud;
using glim_localization_ros2::MapPublisher;
using sensor_msgs::msg::PointCloud2;
using namespace std::chrono_literals;

MapTile::ConstPtr make_tile(int id, const Eigen::Isometry3d& T_world_origin, const std::vector<Eigen::Vector3d>& points) {
  std::vector<Eigen::Vector4d> homogeneous;
  for (const auto& p : points) {
    homogeneous.emplace_back(p.x(), p.y(), p.z(), 1.0);
  }
  auto tile = std::make_shared<MapTile>();
  tile->id = id;
  tile->T_world_origin = T_world_origin;
  tile->points = std::make_shared<gtsam_points::PointCloudCPU>(homogeneous);
  return tile;
}

Eigen::Isometry3d translation(double x, double y, double z) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

std::vector<Eigen::Vector3f> read_points(const PointCloud2& msg) {
  std::vector<Eigen::Vector3f> points(msg.width * msg.height);
  for (size_t i = 0; i < points.size(); i++) {
    std::memcpy(points[i].data(), &msg.data[i * msg.point_step], 3 * sizeof(float));
  }
  return points;
}

}  // namespace

TEST(MapCloud, KeepsEveryPointInTheMapFrameWithoutDownsampling) {
  Eigen::Isometry3d T = translation(10.0, -5.0, 1.0);
  T.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto tiles = {make_tile(0, T, {{1.0, 0.0, 0.0}, {0.0, 2.0, 0.5}}), make_tile(1, translation(0.0, 0.0, 0.0), {{3.0, 3.0, 3.0}})};

  const PointCloud2 msg = make_map_cloud(tiles, 0.0, "map");
  EXPECT_EQ(msg.header.frame_id, "map");
  ASSERT_EQ(msg.height, 1u);
  ASSERT_EQ(msg.width, 3u);
  ASSERT_EQ(msg.fields.size(), 3u);
  EXPECT_EQ(msg.fields[0].name, "x");
  EXPECT_EQ(msg.fields[1].name, "y");
  EXPECT_EQ(msg.fields[2].name, "z");
  for (const auto& field : msg.fields) {
    EXPECT_EQ(field.datatype, sensor_msgs::msg::PointField::FLOAT32);
  }
  EXPECT_EQ(msg.data.size(), static_cast<size_t>(msg.row_step));
  EXPECT_EQ(msg.row_step, msg.point_step * msg.width);

  const auto points = read_points(msg);
  // T_world_origin * p: a quarter turn about z, then the translation
  EXPECT_TRUE(points[0].isApprox(Eigen::Vector3f(10.0f, -4.0f, 1.0f), 1e-6f));
  EXPECT_TRUE(points[1].isApprox(Eigen::Vector3f(8.0f, -5.0f, 1.5f), 1e-6f));
  EXPECT_TRUE(points[2].isApprox(Eigen::Vector3f(3.0f, 3.0f, 3.0f), 1e-6f));
}

TEST(MapCloud, KeepsOneCentroidPerVoxel) {
  // Two points in voxel (0, 0, 0) of size 1, one in voxel (2, 0, 0), one in (-1, 0, 0) and one non-finite point
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto tiles = {
    make_tile(0, translation(0.0, 0.0, 0.0), {{0.2, 0.2, 0.2}, {2.5, 0.5, 0.5}, {nan, 0.0, 0.0}}),
    make_tile(1, translation(0.0, 0.0, 0.0), {{0.6, 0.4, 0.8}, {-0.5, 0.5, 0.5}}),
  };
  const auto points = read_points(make_map_cloud(tiles, 1.0, "map"));
  ASSERT_EQ(points.size(), 3u);
  EXPECT_TRUE(points[0].isApprox(Eigen::Vector3f(0.4f, 0.3f, 0.5f), 1e-6f));
  EXPECT_TRUE(points[1].isApprox(Eigen::Vector3f(2.5f, 0.5f, 0.5f), 1e-6f));
  EXPECT_TRUE(points[2].isApprox(Eigen::Vector3f(-0.5f, 0.5f, 0.5f), 1e-6f));
}

TEST(MapCloud, RejectsABadVoxelSizeAndHandlesNoTiles) {
  EXPECT_THROW(make_map_cloud({}, -1.0, "map"), std::invalid_argument);
  EXPECT_THROW(make_map_cloud({}, std::numeric_limits<double>::infinity(), "map"), std::invalid_argument);
  const PointCloud2 empty = make_map_cloud({nullptr}, 0.5, "map");
  EXPECT_EQ(empty.width, 0u);
  EXPECT_TRUE(empty.data.empty());
}

TEST(MapCloud, SkipsNonFinitePointsWithoutDownsampling) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const PointCloud2 msg = make_map_cloud({make_tile(0, translation(0.0, 0.0, 0.0), {{1.0, 0.0, 0.0}, {nan, 0.0, 0.0}, {2.0, 0.0, 0.0}})}, 0.0, "map");
  ASSERT_EQ(msg.width, 2u);
  EXPECT_EQ(msg.row_step, 24u);
  EXPECT_EQ(msg.data.size(), 24u);
  const auto points = read_points(msg);
  EXPECT_TRUE(points[1].isApprox(Eigen::Vector3f(2.0f, 0.0f, 0.0f), 1e-6f));
}

TEST(MapCloud, RefusesACloudBeyondTheSizeLimit) {
  using glim_localization_ros2::check_map_cloud_size;
  using glim_localization_ros2::kMapCloudPointStep;
  // The uint32 row_step of the message: 357913941 points of 12 bytes fit, one more does not, even with no byte cap
  const size_t max_uint32_points = std::numeric_limits<uint32_t>::max() / kMapCloudPointStep;
  const size_t no_cap = std::numeric_limits<size_t>::max();
  EXPECT_NO_THROW(check_map_cloud_size(max_uint32_points, no_cap));
  EXPECT_THROW(check_map_cloud_size(max_uint32_points + 1, no_cap), std::length_error);
  EXPECT_THROW(check_map_cloud_size(no_cap, no_cap), std::length_error);
  // The default 1 GiB cap
  EXPECT_NO_THROW(check_map_cloud_size(glim_localization_ros2::kMaxMapCloudBytes / kMapCloudPointStep));
  EXPECT_THROW(check_map_cloud_size(glim_localization_ros2::kMaxMapCloudBytes / kMapCloudPointStep + 1), std::length_error);

  // make_map_cloud applies it, with and without downsampling (a small cap stands in for a huge map)
  const auto tiles = {make_tile(0, translation(0.0, 0.0, 0.0), {{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, {10.0, 0.0, 0.0}})};
  EXPECT_THROW(make_map_cloud(tiles, 0.0, "map", 2 * kMapCloudPointStep), std::length_error);
  EXPECT_THROW(make_map_cloud(tiles, 1.0, "map", 2 * kMapCloudPointStep), std::length_error);
  EXPECT_EQ(make_map_cloud(tiles, 1.0, "map", 3 * kMapCloudPointStep).width, 3u);
  EXPECT_EQ(make_map_cloud(tiles, 100.0, "map", 2 * kMapCloudPointStep).width, 1u);
  EXPECT_EQ(glim_localization_ros2::count_map_points({tiles.begin(), tiles.end()}), 3u);
  EXPECT_EQ(glim_localization_ros2::count_map_points({nullptr}), 0u);
}

class MapPublisherTest : public testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(MapPublisherTest, LateSubscriberGetsTheLatchedMap) {
  auto node = std::make_shared<rclcpp::Node>("map_publisher_test");
  EXPECT_THROW(MapPublisher(node->get_node_topics_interface(), node->get_clock(), "", 0.5), std::invalid_argument);
  EXPECT_THROW(MapPublisher(node->get_node_topics_interface(), node->get_clock(), "map", -0.5), std::invalid_argument);

  MapPublisher publisher(node->get_node_topics_interface(), node->get_clock(), "map", 0.5);
  const size_t published = publisher.publish({make_tile(0, translation(1.0, 2.0, 3.0), {{0.0, 0.0, 0.0}, {0.1, 0.1, 0.1}, {5.0, 0.0, 0.0}})});
  EXPECT_EQ(published, 2u);

  // Subscribed after the publication: only transient local durability delivers it
  auto observer = std::make_shared<rclcpp::Node>("map_publisher_observer");
  std::vector<PointCloud2> received;
  auto sub = observer->create_subscription<PointCloud2>(
    "/map_publisher_test/map", rclcpp::QoS(1).reliable().transient_local(), [&](const PointCloud2::ConstSharedPtr msg) { received.push_back(*msg); });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(observer);
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while (received.empty() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(10ms);
  }
  ASSERT_EQ(received.size(), 1u);
  EXPECT_EQ(received[0].header.frame_id, "map");
  EXPECT_EQ(received[0].width, 2u);
}
