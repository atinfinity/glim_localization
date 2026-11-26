// Copyright (c) 2026 atinfinity
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// R1: a GLIM extension module that publishes the localization result from inside glim_rosnode /
// glim_rosbag.
//
// EstimateBridge adapts glim::EstimationFrame (via OdometryEstimationCallbacks) to FrameEstimate
// and drives LocalizationPublisher. A frame is published together with its status
// (LocalizationCallbacks::on_status), so a LOST frame gives no ~/pose. It subscribes to
// /initialpose and forwards it to the odometry with LocalizationCallbacks::request_reset, and
// publishes the map that the odometry module loaded on ~/map (latched). With wheel_odom_topic set,
// it feeds nav_msgs/Odometry twists to the odometry (LocalizationCallbacks::insert_wheel_odom).
// Settings come from the "localization_ros" section of config_ros.json (see
// config/r1/config_ros.json).
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "Eigen/Geometry"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"

#include "glim/util/config.hpp"
#include "glim/util/extension_module_ros2.hpp"

#include "glim_localization/odometry/odometry_estimation_localization.hpp"
#include "glim_localization_ros2/estimate_bridge.hpp"
#include "glim_localization_ros2/localization_publisher.hpp"
#include "glim_localization_ros2/map_publisher.hpp"
#include "glim_localization_ros2/wheel_odom_input.hpp"

namespace glim_localization_ros2
{

class LocalizationExtension : public glim::ExtensionModuleROS2 {
public:
  using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
  using OdometryMsg = nav_msgs::msg::Odometry;

  LocalizationExtension()
  : bridge_(std::make_shared<EstimateBridge>()) {}

  // The callbacks stay registered in the static slots after the extension is destroyed (see
  // EstimateBridge)
  ~LocalizationExtension() override
  {
    bridge_->detach();
    if (wheel_) {
      spdlog::info(
        "[glim_localization] wheel odometry received {}, rejected {}, dropped by the odometry {}",
        wheel_->received.load(), wheel_->rejected.load(), wheel_->dropped.load());
    }
  }

  std::vector<glim::GenericTopicSubscription::Ptr> create_subscriptions(
    rclcpp::Node & node) override
  {
    const glim::Config config(glim::GlobalConfig::get_config_path("config_ros"));
    const LocalizationPublisherParams params = read_publisher_params(config);
    bridge_->attach(
      std::make_unique<LocalizationPublisher>(
        LocalizationPublisher::NodeInterfaces(
          node.get_node_base_interface(), node.get_node_clock_interface(),
        node.get_node_logging_interface(), node.get_node_parameters_interface(),
          node.get_node_topics_interface()),
        params),
      params);

    // The slots are not thread-safe: register before the first frame, which cannot come before this
    // constructor chain is done.
    EstimateBridge::connect(bridge_);

    // ~/map: glim_ros2 creates the odometry module (which loads the map) before the extensions, so
    // the map is there
    const MapPublisherParams map_params = read_map_publisher_params(config);
    if (map_params.enabled) {
      if (const auto map = glim_localization::LocalizationCallbacks::loaded_map()) {
        const auto tiles = map->tiles();
        const size_t num_map_points = count_map_points(tiles);
        if (map_params.voxel_size <= 0.0 && num_map_points > kLargeMapCloudPoints) {
          spdlog::warn(
            "[glim_localization] map_cloud_voxel is 0 and the map has {} points: "
            "~/map is large and slow to build and send; set map_cloud_voxel",
            num_map_points);
        }
        map_publisher_ = std::make_unique<MapPublisher>(node.get_node_topics_interface(),
            node.get_clock(), params.map_frame, map_params.voxel_size);
        try {
          const size_t num_points = map_publisher_->publish(tiles);
          spdlog::info("[glim_localization] published ~/map: {} points from {} tiles (voxel {} m)",
              num_points, map->size(), map_params.voxel_size);
        } catch (const std::length_error & e) {
          spdlog::warn("[glim_localization] ~/map is not published: {}", e.what());
        }
      } else {
        spdlog::warn(
            "[glim_localization] no localization odometry is loaded, so ~/map is not published");
      }
    }

    const auto bridge = bridge_;
    std::vector<glim::GenericTopicSubscription::Ptr> subscriptions = {
      std::make_shared<glim::TopicSubscription<PoseMsg>>(
        "/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped",
        [bridge](const std::shared_ptr<const PoseMsg> & msg) {
          handle_initial_pose(*bridge, *msg);
        })};

    // Wheel odometry. GLIM subscribes with its own QoS (reliable, depth 100), so wheel_odom_qos of
    // R2 does not apply here.
    const auto wheel_topic = config.param<std::string>("localization_ros", "wheel_odom_topic", "");
    if (!wheel_topic.empty()) {
      WheelOdomInputParams wheel_params;
      wheel_params.child_frame_id = config.param<std::string>("localization_ros",
          "wheel_odom_child_frame_id", "");
      wheel_params.time_offset = config.param<double>("localization_ros", "wheel_odom_time_offset",
          0.0);
      if (!std::isfinite(wheel_params.time_offset)) {
        spdlog::error(
          "[glim_localization] localization_ros.wheel_odom_time_offset must be finite: "
          "not subscribing to {}", wheel_topic);
      } else if (!glim_localization::LocalizationCallbacks::wheel_odom_enabled()) {
        spdlog::warn(
          "[glim_localization] localization_ros.wheel_odom_topic is set but "
          "odometry_estimation.wheel_odom_enabled is false (or no localization odometry is "
          "loaded): not subscribing to {}", wheel_topic);
      } else {
        auto wheel = std::make_shared<WheelInput>();
        wheel->topic = wheel_topic;
        wheel->params = wheel_params;
        wheel_ = wheel;
        spdlog::info("[glim_localization] wheel odometry on {} (child_frame_id '{}')", wheel_topic,
            wheel_params.child_frame_id.empty() ? "any" : wheel_params.child_frame_id);
        subscriptions.push_back(std::make_shared<glim::TopicSubscription<OdometryMsg>>(
          wheel_topic, "nav_msgs/msg/Odometry",
            [wheel](const std::shared_ptr<const OdometryMsg> & msg) {
              handle_wheel_odom(*wheel, *msg);
          }));
      }
    }
    return subscriptions;
  }

private:
  /// State of the wheel odometry subscription, shared with its callback
  struct WheelInput
  {
    std::string topic;
    WheelOdomInputParams params;
    std::atomic<size_t> received{0};
    std::atomic<size_t> rejected{0};  ///< By the conversion (frame, stamp, value)
    std::atomic<size_t> dropped{0};   ///< By the odometry (not newer than the last)
    std::atomic<int64_t> last_warning_ns{0};
    std::atomic<bool> frame_logged{false};  ///< The accepted child_frame_id was logged (any frame)
  };

  static void handle_wheel_odom(WheelInput & wheel, const OdometryMsg & msg)
  {
    ++wheel.received;
    WheelOdomSample sample;
    const auto reject = convert_wheel_odom(msg, wheel.params, sample);
    if (reject != WheelOdomReject::NONE) {
      ++wheel.rejected;
      // At most one warning per 5 s (wall clock)
      const int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      int64_t last = wheel.last_warning_ns.load();
      if ((last == 0 || now - last >= 5000000000LL) &&
        wheel.last_warning_ns.compare_exchange_strong(last, now))
      {
        spdlog::warn(
          "[glim_localization] rejecting wheel odometry on {}: {} (child_frame_id '{}', "
          "expected '{}'; {} rejected so far)", wheel.topic, to_string(reject),
          msg.child_frame_id, wheel.params.child_frame_id, wheel.rejected.load());
      }
      return;
    }
    if (wheel.params.child_frame_id.empty() && !wheel.frame_logged.exchange(true)) {
      spdlog::info("[glim_localization] wheel odometry on {}: child_frame_id '{}'", wheel.topic,
          msg.child_frame_id);
    }
    if (!glim_localization::LocalizationCallbacks::insert_wheel_odom(sample.stamp,
      sample.linear_vel, sample.angular_vel))
    {
      ++wheel.dropped;
    }
  }

  static void handle_initial_pose(const EstimateBridge & bridge, const PoseMsg & msg)
  {
    Eigen::Isometry3d T_map_imu;
    double min_stamp = 0.0;
    const std::string error = bridge.to_T_map_imu(msg, T_map_imu, min_stamp);
    if (!error.empty()) {
      spdlog::warn("[glim_localization] ignoring /initialpose: {}", error);
      return;
    }
    spdlog::info("[glim_localization] /initialpose received (min_stamp={:.3f})", min_stamp);
    glim_localization::LocalizationCallbacks::request_reset(T_map_imu, Eigen::Vector3d::Zero(),
        min_stamp);
  }

  std::shared_ptr<EstimateBridge> bridge_;
  std::unique_ptr<MapPublisher> map_publisher_;
  std::shared_ptr<WheelInput> wheel_;
};

}  // namespace glim_localization_ros2

extern "C" glim::ExtensionModule * create_extension_module()
{
  // An exception must not cross the C entry point that GLIM calls through dlsym.
  try {
    return new glim_localization_ros2::LocalizationExtension();
  } catch (const std::exception & e) {
    spdlog::error("failed to create the localization extension: {}", e.what());
    return nullptr;
  }
}
