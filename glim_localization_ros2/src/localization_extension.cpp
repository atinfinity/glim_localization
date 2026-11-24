// R1: a GLIM extension module that publishes the localization result from inside glim_rosnode / glim_rosbag.
//
// EstimateBridge adapts glim::EstimationFrame (via OdometryEstimationCallbacks) to FrameEstimate and drives
// LocalizationPublisher. A frame is published together with its status (LocalizationCallbacks::on_status), so a LOST
// frame gives no ~/pose. It subscribes to /initialpose and forwards it to the odometry with
// LocalizationCallbacks::request_reset, and publishes the map that the odometry module loaded on ~/map (latched).
// Settings come from the "localization_ros" section of config_ros.json (see config/r1/config_ros.json).
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>

#include <glim_localization/odometry/odometry_estimation_localization_cpu.hpp>
#include <glim_localization_ros2/estimate_bridge.hpp>
#include <glim_localization_ros2/localization_publisher.hpp>
#include <glim_localization_ros2/map_publisher.hpp>

namespace glim_localization_ros2 {

class LocalizationExtension : public glim::ExtensionModuleROS2 {
public:
  using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

  LocalizationExtension() : bridge_(std::make_shared<EstimateBridge>()) {}

  // The callbacks stay registered in the static slots after the extension is destroyed (see EstimateBridge)
  ~LocalizationExtension() override { bridge_->detach(); }

  std::vector<glim::GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override {
    const glim::Config config(glim::GlobalConfig::get_config_path("config_ros"));
    const LocalizationPublisherParams params = read_publisher_params(config);
    bridge_->attach(
      std::make_unique<LocalizationPublisher>(
        LocalizationPublisher::NodeInterfaces(
          node.get_node_base_interface(), node.get_node_clock_interface(), node.get_node_logging_interface(), node.get_node_parameters_interface(),
          node.get_node_topics_interface()),
        params),
      params);

    // The slots are not thread-safe: register before the first frame, which cannot come before this constructor chain is done.
    EstimateBridge::connect(bridge_);

    // ~/map: glim_ros2 creates the odometry module (which loads the map) before the extensions, so the map is there
    const MapPublisherParams map_params = read_map_publisher_params(config);
    if (map_params.enabled) {
      if (const auto map = glim_localization::LocalizationCallbacks::loaded_map()) {
        map_publisher_ = std::make_unique<MapPublisher>(node.get_node_topics_interface(), node.get_clock(), params.map_frame, map_params.voxel_size);
        const size_t num_points = map_publisher_->publish(map->tiles());
        spdlog::info("[glim_localization] published ~/map: {} points from {} tiles (voxel {} m)", num_points, map->size(), map_params.voxel_size);
      } else {
        spdlog::warn("[glim_localization] no localization odometry is loaded, so ~/map is not published");
      }
    }

    const auto bridge = bridge_;
    return {std::make_shared<glim::TopicSubscription<PoseMsg>>(
      "/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped", [bridge](const std::shared_ptr<const PoseMsg>& msg) { handle_initial_pose(*bridge, *msg); })};
  }

private:
  static void handle_initial_pose(const EstimateBridge& bridge, const PoseMsg& msg) {
    Eigen::Isometry3d T_map_imu;
    double min_stamp = 0.0;
    const std::string error = bridge.to_T_map_imu(msg, T_map_imu, min_stamp);
    if (!error.empty()) {
      spdlog::warn("[glim_localization] ignoring /initialpose: {}", error);
      return;
    }
    spdlog::info("[glim_localization] /initialpose received (min_stamp={:.3f})", min_stamp);
    glim_localization::LocalizationCallbacks::request_reset(T_map_imu, Eigen::Vector3d::Zero(), min_stamp);
  }

  std::shared_ptr<EstimateBridge> bridge_;
  std::unique_ptr<MapPublisher> map_publisher_;
};

}  // namespace glim_localization_ros2

extern "C" glim::ExtensionModule* create_extension_module() {
  // An exception must not cross the C entry point that GLIM calls through dlsym.
  try {
    return new glim_localization_ros2::LocalizationExtension();
  } catch (const std::exception& e) {
    spdlog::error("failed to create the localization extension: {}", e.what());
    return nullptr;
  }
}
