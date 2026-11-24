// R1: a GLIM extension module that publishes the localization result from inside glim_rosnode / glim_rosbag.
//
// EstimateBridge adapts glim::EstimationFrame (via OdometryEstimationCallbacks) to FrameEstimate and drives
// LocalizationPublisher. A frame is published together with its status (LocalizationCallbacks::on_status), so a LOST
// frame gives no ~/pose. It subscribes to /initialpose and forwards it to the odometry with
// LocalizationCallbacks::request_reset. Settings come from the "localization_ros" section of config_ros.json
// (see config/r1/config_ros.json).
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
