// Derived from koide3/glim_ros2 src/glim_ros/glim_ros.cpp at commit 4d4ec524ccf1b02aa09b0af2af767ecc54343798 (MIT License, Copyright (c) 2024 koide3)
// Derived from koide3/glim_ros2 src/glim_ros/ros_qos.cpp at commit 4d4ec524ccf1b02aa09b0af2af767ecc54343798 (MIT License, Copyright (c) 2024 koide3)
//
// Copyright (c) 2026 atinfinity
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// R2: glim_localization_node, an rclcpp_lifecycle node (also a component).
//
//   configure   reads the GLIM JSON config (config_path), loads the map (map_path) and publishes
//               it on ~/map (latched); a bad map fails the transition
//   activate    creates the odometry, the publisher and the subscriptions; estimation starts with
//               the first data
//   deactivate  drops the subscriptions and stops the odometry thread (pending input is discarded)
//   cleanup     releases the map and the config
//
// The input handling (IMU and point cloud conversion, TimeKeeper, the QoS settings in
// config_ros.json) follows GlimROS. Unlike GlimROS, the node holds the odometry module itself (no
// dlopen), so ~/initialize, ~/relocalize and /initialpose call its reset() directly. Results go
// through EstimateBridge and LocalizationPublisher, as in R1.
//
// With autostart:=true the node configures and activates itself once it is spinning (for launch
// files, also when it is loaded as a component, where launch_ros cannot drive the lifecycle). A
// failed transition is logged and leaves the node where it stopped; drive it by hand from there.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "Eigen/Geometry"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

#define GLIM_ROS2  // selects the ROS 2 message types in ros_cloud_converter.hpp
#include "glim/odometry/async_odometry_estimation.hpp"
#include "glim/preprocess/cloud_preprocessor.hpp"
#include "glim/util/config.hpp"
#include "glim/util/ros_cloud_converter.hpp"
#include "glim/util/time_keeper.hpp"

#include "glim_localization/map/map_manager.hpp"
#include "glim_localization/odometry/localization_backend.hpp"
#include "glim_localization/odometry/odometry_estimation_localization.hpp"
#include "glim_localization_msgs/srv/initialize.hpp"
#include "glim_localization_ros2/estimate_bridge.hpp"
#include "glim_localization_ros2/init_pose_delay.hpp"
#include "glim_localization_ros2/localization_publisher.hpp"
#include "glim_localization_ros2/map_publisher.hpp"
#include "glim_localization_ros2/wheel_odom_input.hpp"

namespace glim_localization_ros2
{

namespace
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
using ImuMsg = sensor_msgs::msg::Imu;
using OdometryMsg = nav_msgs::msg::Odometry;
using PointsMsg = sensor_msgs::msg::PointCloud2;
using InitializeSrv = glim_localization_msgs::srv::Initialize;
using TriggerSrv = std_srvs::srv::Trigger;
using OdometryParams = glim_localization::OdometryEstimationLocalizationParams;

bool init_qos_profile(const std::string & name, rmw_qos_profile_t & profile)
{
  if (name == "default") {
    profile = rmw_qos_profile_default;
  } else if (name == "system_default") {
    profile = rmw_qos_profile_system_default;
  } else if (name == "sensor_data") {
    profile = rmw_qos_profile_sensor_data;
  } else if (name == "services_default") {
    profile = rmw_qos_profile_services_default;
  } else if (name == "parameters") {
    profile = rmw_qos_profile_parameters;
  } else if (name == "parameter_events") {
    profile = rmw_qos_profile_parameter_events;
  } else {
    profile = rmw_qos_profile_default;
    return false;
  }
  return true;
}

/// The QoS of <section>.<param_name> in config_ros.json ({profile, depth, durability, reliability,
/// history}), as GlimROS reads it: the profile defaults to sensor_data, and default_qos is used
/// when nothing is configured.
rclcpp::QoS read_qos(
  const glim::Config & config, const std::string & param_name,
  const rclcpp::Logger & logger, const std::optional<rclcpp::QoS> & default_qos = {},
  const std::string & section = "glim_ros")
{
  const std::vector<std::string> path = {section, param_name};
  bool configured = false;
  rmw_qos_profile_t profile;
  const auto profile_name = config.param_nested<std::string>(path, "profile");
  configured |= profile_name.has_value();
  if (!init_qos_profile(profile_name.value_or("sensor_data"), profile)) {
    RCLCPP_WARN(logger, "unknown QoS profile '%s' for %s, falling back to 'default'",
          profile_name->c_str(), param_name.c_str());
  }
  if (const auto depth = config.param_nested<int>(path, "depth")) {
    profile.depth = *depth;
    configured = true;
  }
  if (const auto str = config.param_nested<std::string>(path, "durability")) {
    const auto value = rmw_qos_durability_policy_from_str(str->c_str());
    if (value == RMW_QOS_POLICY_DURABILITY_UNKNOWN) {
      RCLCPP_WARN(logger, "ignoring unknown durability policy '%s' for %s", str->c_str(),
            param_name.c_str());
    } else {
      profile.durability = value;
      configured = true;
    }
  }
  if (const auto str = config.param_nested<std::string>(path, "reliability")) {
    const auto value = rmw_qos_reliability_policy_from_str(str->c_str());
    if (value == RMW_QOS_POLICY_RELIABILITY_UNKNOWN) {
      RCLCPP_WARN(logger, "ignoring unknown reliability policy '%s' for %s", str->c_str(),
            param_name.c_str());
    } else {
      profile.reliability = value;
      configured = true;
    }
  }
  if (const auto str = config.param_nested<std::string>(path, "history")) {
    const auto value = rmw_qos_history_policy_from_str(str->c_str());
    if (value == RMW_QOS_POLICY_HISTORY_UNKNOWN) {
      RCLCPP_WARN(logger, "ignoring unknown history policy '%s' for %s", str->c_str(),
            param_name.c_str());
    } else {
      profile.history = value;
      configured = true;
    }
  }
  if (!configured && default_qos) {
    return *default_qos;
  }
  return rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(profile), profile);
}

/// Input settings from the glim_ros section of config_ros.json (the same keys as GlimROS)
struct InputSettings
{
  std::string imu_topic;
  std::string points_topic;
  double imu_time_offset = 0.0;
  double points_time_offset = 0.0;
  double acc_scale = 0.0;  ///< 0: detect [m/s^2] or [g] from the first sample
  double ang_scale = 1.0;
  std::string intensity_field;
  std::string ring_field;
  // localization_ros: our own keys
  std::string wheel_odom_topic;  ///< Empty: no wheel odometry subscription
  WheelOdomInputParams wheel_odom;
};

}  // namespace

class LocalizationNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit LocalizationNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("glim_localization", options),
    bridge_(std::make_shared<EstimateBridge>())
  {
    declare_parameter<std::string>("config_path", "config/r2");
    declare_parameter<std::string>("map_path", "");
    declare_parameter<double>("max_init_pose_delay", 0.2);
    if (declare_parameter<bool>("autostart", false)) {
      // One shot, from the executor: the transitions must not run inside the constructor
      autostart_timer_ = create_wall_timer(std::chrono::milliseconds(0), [this] {autostart();});
    }
    // The core reports through static slots: connect once, before any odometry of this node starts
    // (see EstimateBridge)
    EstimateBridge::connect(bridge_);
  }

  ~LocalizationNode() override
  {
    stop();
    release();
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    try {
      if (!configure()) {
        release();
        return CallbackReturn::FAILURE;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "configure failed: %s", e.what());
      release();
      return CallbackReturn::FAILURE;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    try {
      activate();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "activate failed: %s", e.what());
      stop();
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "active: waiting for %s and %s", input_.imu_topic.c_str(),
        input_.points_topic.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    stop();
    RCLCPP_INFO(get_logger(), "inactive");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    release();
    RCLCPP_INFO(get_logger(), "unconfigured");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    stop();
    release();
    return CallbackReturn::SUCCESS;
  }

private:
  void autostart()
  {
    autostart_timer_->cancel();
    // Someone may have changed the state before the timer fired: then leave the node to them
    const auto & start = get_current_state();
    if (start.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
      RCLCPP_INFO(get_logger(), "autostart: skipped, the node is already %s",
          start.label().c_str());
      return;
    }
    if (rclcpp_lifecycle::LifecycleNode::configure().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      RCLCPP_ERROR(get_logger(), "autostart: configure failed; the node is %s",
          get_current_state().label().c_str());
      return;
    }
    if (rclcpp_lifecycle::LifecycleNode::activate().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_ERROR(get_logger(), "autostart: activate failed; the node is %s",
          get_current_state().label().c_str());
    }
  }

  bool configure()
  {
    std::string config_path = get_parameter("config_path").as_string();
    if (config_path.empty() || config_path[0] != '/') {
      config_path = ament_index_cpp::get_package_share_directory("glim_localization_ros2") + "/" +
        config_path;
    }
    if (!std::filesystem::is_regular_file(config_path + "/config.json")) {
      RCLCPP_ERROR(get_logger(), "config_path '%s' has no config.json", config_path.c_str());
      return false;
    }
    max_init_pose_delay_ = get_parameter("max_init_pose_delay").as_double();
    if (!(max_init_pose_delay_ >= 0.0) || !std::isfinite(max_init_pose_delay_)) {
      RCLCPP_ERROR(get_logger(), "max_init_pose_delay must be finite and not negative");
      return false;
    }

    // GLIM reads its settings from the process-wide GlobalConfig; replace it so that a new
    // configure sees new files
    glim::GlobalConfig::instance(config_path, true);
    RCLCPP_INFO(get_logger(), "config_path: %s", config_path.c_str());
    const glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));
    const glim::Config config_sensors(glim::GlobalConfig::get_config_path("config_sensors"));

    input_ = InputSettings();
    input_.imu_topic = config_ros.param<std::string>("glim_ros", "imu_topic", "");
    input_.points_topic = config_ros.param<std::string>("glim_ros", "points_topic", "");
    input_.imu_time_offset = config_ros.param<double>("glim_ros", "imu_time_offset", 0.0);
    input_.points_time_offset = config_ros.param<double>("glim_ros", "points_time_offset", 0.0);
    if (!std::isfinite(input_.imu_time_offset) || !std::isfinite(input_.points_time_offset)) {
      // A non-finite offset would make every stamp non-finite: the IMU gate would drop every
      // sample, and upstream would release every frame
      RCLCPP_ERROR(get_logger(),
          "glim_ros.imu_time_offset and glim_ros.points_time_offset must be finite");
      return false;
    }
    input_.acc_scale = config_ros.param<double>("glim_ros", "acc_scale", 0.0);
    input_.ang_scale = config_ros.param<double>("glim_ros", "ang_scale", 1.0);
    input_.intensity_field = config_sensors.param<std::string>("sensors", "intensity_field",
        "intensity");
    input_.ring_field = config_sensors.param<std::string>("sensors", "ring_field", "");
    input_.wheel_odom_topic = config_ros.param<std::string>("localization_ros", "wheel_odom_topic",
        "");
    input_.wheel_odom.child_frame_id = config_ros.param<std::string>("localization_ros",
        "wheel_odom_child_frame_id", "");
    input_.wheel_odom.time_offset = config_ros.param<double>("localization_ros",
        "wheel_odom_time_offset", 0.0);
    if (!std::isfinite(input_.wheel_odom.time_offset)) {
      RCLCPP_ERROR(get_logger(), "localization_ros.wheel_odom_time_offset must be finite");
      return false;
    }
    if (input_.imu_topic.empty() || input_.points_topic.empty()) {
      RCLCPP_ERROR(get_logger(),
          "glim_ros.imu_topic and glim_ros.points_topic must be set in config_ros.json");
      return false;
    }

    rclcpp::SensorDataQoS default_imu_qos;
    default_imu_qos.get_rmw_qos_profile().depth = 1000;
    imu_qos_ = read_qos(config_ros, "imu_qos", get_logger(), default_imu_qos);
    points_qos_ = read_qos(config_ros, "points_qos", get_logger());
    rclcpp::SensorDataQoS default_wheel_odom_qos;
    default_wheel_odom_qos.get_rmw_qos_profile().depth = 200;
    wheel_odom_qos_ = read_qos(config_ros, "wheel_odom_qos", get_logger(), default_wheel_odom_qos,
        "localization_ros");
    publisher_params_ = read_publisher_params(config_ros);
    const MapPublisherParams map_publisher_params = read_map_publisher_params(config_ros);

    // CPU or GPU, as selected by the so_name of the odometry config
    try {
      const auto backend = glim_localization::localization_backend_from_config();
      odometry_params_ = glim_localization::make_localization_params(backend);
      RCLCPP_INFO(get_logger(), "odometry backend: %s",
          glim_localization::to_string(backend).c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      return false;
    }
    const std::string map_path = get_parameter("map_path").as_string();
    if (!map_path.empty()) {
      odometry_params_->map_path = map_path;
    }
    if (odometry_params_->map_path.empty()) {
      RCLCPP_ERROR(get_logger(),
          "no map: set the map_path parameter or odometry_estimation.map_path");
      return false;
    }
    glim_localization::MapManagerParams map_params;
    map_params.voxel_resolutions = odometry_params_->map_voxel_resolutions;
    auto map = std::make_shared<glim_localization::MapManager>(map_params);
    if (!map->load(odometry_params_->map_path)) {
      RCLCPP_ERROR(get_logger(), "failed to load the map '%s'", odometry_params_->map_path.c_str());
      return false;
    }
    map_ = map;
    RCLCPP_INFO(get_logger(), "loaded the map '%s' (%zu tiles)", odometry_params_->map_path.c_str(),
        map_->size());
    if (map_publisher_params.enabled) {
      const auto tiles = map_->tiles();
      const size_t num_map_points = count_map_points(tiles);
      if (map_publisher_params.voxel_size <= 0.0 && num_map_points > kLargeMapCloudPoints) {
        RCLCPP_WARN(
          get_logger(),
          "map_cloud_voxel is 0 and the map has %zu points: "
          "~/map is large and slow to build and send; set map_cloud_voxel",
          num_map_points);
      }
      map_publisher_ = std::make_unique<MapPublisher>(get_node_topics_interface(), get_clock(),
          publisher_params_.map_frame, map_publisher_params.voxel_size);
      try {
        const size_t num_points = map_publisher_->publish(tiles);
        RCLCPP_INFO(get_logger(), "published ~/map: %zu points (voxel %.3f m)", num_points,
            map_publisher_params.voxel_size);
      } catch (const std::length_error & e) {
        RCLCPP_WARN(get_logger(), "~/map is not published: %s", e.what());
      }
    }

    initialize_srv_ = create_service<InitializeSrv>(
      "~/initialize",
      [this](const std::shared_ptr<InitializeSrv::Request> req,
      std::shared_ptr<InitializeSrv::Response> res) {on_initialize(*req, *res);});
    relocalize_srv_ = create_service<TriggerSrv>(
      "~/relocalize",
      [this](const std::shared_ptr<TriggerSrv::Request>,
      std::shared_ptr<TriggerSrv::Response> res) {
        on_relocalize(*res);
      });
    return true;
  }

  void activate()
  {
    // TimeKeeper and the acceleration unit keep state of the stream: start them afresh
    time_keeper_ = std::make_unique<glim::TimeKeeper>();
    preprocessor_ = std::make_unique<glim::CloudPreprocessor>();
    acc_scale_ = input_.acc_scale;
    num_imu_ = 0;
    num_points_ = 0;
    num_wheel_odom_ = 0;
    num_wheel_odom_rejected_ = 0;
    num_wheel_odom_dropped_ = 0;
    wheel_odom_frame_logged_ = false;
    first_points_stamp_ = std::nan("");

    odometry_ = glim_localization::make_localization_odometry(*odometry_params_, map_);
    bridge_->attach(
      std::make_unique<LocalizationPublisher>(
        LocalizationPublisher::NodeInterfaces(
          get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
        get_node_parameters_interface(), get_node_topics_interface()),
        publisher_params_),
      publisher_params_);
    async_odometry_ = std::make_unique<glim::AsyncOdometryEstimation>(odometry_,
        odometry_->requires_imu());

    imu_sub_ = subscribe<ImuMsg>(input_.imu_topic, *imu_qos_,
        [this](const ImuMsg::ConstSharedPtr msg) {on_imu(*msg);});
    points_sub_ = subscribe<PointsMsg>(input_.points_topic, *points_qos_,
        [this](const PointsMsg::ConstSharedPtr msg) {on_points(*msg);});
    initial_pose_sub_ = subscribe<PoseMsg>("/initialpose", rclcpp::QoS(10),
        [this](const PoseMsg::ConstSharedPtr msg) {on_initial_pose(*msg);});
    if (!input_.wheel_odom_topic.empty()) {
      if (odometry_params_->wheel_odom.enabled) {
        wheel_odom_sub_ = subscribe<OdometryMsg>(input_.wheel_odom_topic, *wheel_odom_qos_,
            [this](const OdometryMsg::ConstSharedPtr msg) {on_wheel_odom(*msg);});
      } else {
        RCLCPP_WARN(get_logger(),
            "localization_ros.wheel_odom_topic is set but odometry_estimation.wheel_odom_enabled "
            "is false: not subscribing to %s", input_.wheel_odom_topic.c_str());
      }
    }
    // AsyncOdometryEstimation also queues its results for get_results(); the node publishes from
    // the callbacks, so drop them
    drain_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
          std::vector<glim::EstimationFrame::ConstPtr> results, marginalized;
          if (async_odometry_) {
            async_odometry_->get_results(results, marginalized);
          }
    });
  }

  /// A subscription that logs each change of its matched publishers. A message sent before its
  /// publisher is matched is lost (volatile), so "matched publishers on <topic>: N" tells whether a
  /// sender is connected yet; the launch tests wait for it before playing a bag. Without matched
  /// events in the RMW, a plain subscription.
  template<typename MsgT, typename CallbackT>
  typename rclcpp::Subscription<MsgT>::SharedPtr subscribe(
    const std::string & topic,
    const rclcpp::QoS & qos, CallbackT && callback)
  {
    rclcpp::SubscriptionOptions options;
    options.event_callbacks.matched_callback = [logger = get_logger(),
        topic](const rclcpp::MatchedInfo & info) {
        RCLCPP_INFO(logger, "matched publishers on %s: %zu", topic.c_str(), info.current_count);
      };
    try {
      return create_subscription<MsgT>(topic, qos, callback, options);
    } catch (const rclcpp::UnsupportedEventTypeException &) {
      RCLCPP_INFO(get_logger(),
          "matched events not supported on %s by this RMW; matched publishers are not logged",
          topic.c_str());
      return create_subscription<MsgT>(topic, qos, std::forward<CallbackT>(callback));
    }
  }

  /// Stop estimation. Idempotent.
  void stop()
  {
    drain_timer_.reset();
    initial_pose_sub_.reset();
    wheel_odom_sub_.reset();
    points_sub_.reset();
    imu_sub_.reset();
    // stops and joins the odometry thread, so no callback comes after this
    async_odometry_.reset();
    odometry_.reset();
    if (bridge_->attached()) {
      // One line per active period, so that a test log tells which path lost frames. After the
      // join, so the frames still queued in the odometry thread are counted too
      const auto drops = bridge_->drop_counts();
      RCLCPP_INFO(
        get_logger(),
        "stopping: received %zu IMU samples and %zu point clouds (first cloud stamp %.3f); "
        "dropped frames: %zu without gyro, %zu unpaired, %zu stale; %zu TF lookup failures",
        num_imu_.load(), num_points_.load(), first_points_stamp_.load(), drops.frames_without_gyro,
          drops.unpaired_frames, drops.stale_frames,
        drops.tf_lookup_failures);
      if (!input_.wheel_odom_topic.empty()) {
        RCLCPP_INFO(get_logger(),
            "stopping: wheel odometry received %zu, rejected %zu, dropped by the odometry %zu",
            num_wheel_odom_.load(), num_wheel_odom_rejected_.load(),
            num_wheel_odom_dropped_.load());
      }
    }
    bridge_->detach();
    preprocessor_.reset();
    time_keeper_.reset();
  }

  /// Release what configure made. Idempotent.
  void release()
  {
    initialize_srv_.reset();
    relocalize_srv_.reset();
    map_publisher_.reset();
    map_.reset();
    odometry_params_.reset();
    imu_qos_.reset();
    points_qos_.reset();
    wheel_odom_qos_.reset();
  }

  void on_imu(const ImuMsg & msg)
  {
    if (!async_odometry_) {
      return;
    }
    ++num_imu_;
    const Eigen::Vector3d acc(msg.linear_acceleration.x, msg.linear_acceleration.y,
      msg.linear_acceleration.z);
    if (std::abs(acc_scale_) < 1e-6) {
      const double norm = acc.norm();
      if (norm > 7.0 && norm < 12.0) {
        acc_scale_ = 1.0;
      } else if (norm > 0.8 && norm < 1.2) {
        acc_scale_ = 9.80665;
      } else {
        acc_scale_ = 1.0;
        RCLCPP_WARN(get_logger(), "unexpected acceleration norm %f: assuming [m/s^2]", norm);
      }
      RCLCPP_INFO(get_logger(), "acc_scale=%f (from the first IMU sample, stamp %.3f)", acc_scale_,
          msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9);
    }

    const double stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9 +
      input_.imu_time_offset;
    const Eigen::Vector3d angular_vel = input_.ang_scale * Eigen::Vector3d(msg.angular_velocity.x,
        msg.angular_velocity.y, msg.angular_velocity.z);
    if (!time_keeper_->validate_imu_stamp(stamp)) {
      RCLCPP_WARN(get_logger(), "skip an invalid IMU sample (stamp=%f)", stamp);
      return;
    }
    async_odometry_->insert_imu(stamp, acc_scale_ * acc, angular_vel);
  }

  void on_wheel_odom(const OdometryMsg & msg)
  {
    if (!odometry_) {
      return;
    }
    ++num_wheel_odom_;
    WheelOdomSample sample;
    const auto reject = convert_wheel_odom(msg, input_.wheel_odom, sample);
    if (reject != WheelOdomReject::NONE) {
      ++num_wheel_odom_rejected_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "rejecting wheel odometry on %s: %s (child_frame_id '%s', expected '%s')",
          input_.wheel_odom_topic.c_str(), to_string(reject), msg.child_frame_id.c_str(),
          input_.wheel_odom.child_frame_id.c_str());
      return;
    }
    if (input_.wheel_odom.child_frame_id.empty() && !wheel_odom_frame_logged_.exchange(true)) {
      RCLCPP_INFO(get_logger(),
          "wheel odometry twist in child_frame_id '%s' (not checked: "
          "localization_ros.wheel_odom_child_frame_id is empty)", msg.child_frame_id.c_str());
    }
    // Thread-safe; bypasses the frame queue, so the samples are buffered before their frame runs
    if (!odometry_->insert_wheel_odom(sample.stamp, sample.linear_vel, sample.angular_vel)) {
      ++num_wheel_odom_dropped_;
    }
  }

  void on_points(const PointsMsg & msg)
  {
    if (!async_odometry_) {
      return;
    }
    auto raw_points = glim::extract_raw_points(msg, input_.intensity_field, input_.ring_field);
    if (!raw_points) {
      RCLCPP_WARN(get_logger(), "failed to extract points from a message");
      return;
    }
    raw_points->stamp += input_.points_time_offset;
    if (num_points_++ == 0) {
      first_points_stamp_ = raw_points->stamp;
      RCLCPP_INFO(get_logger(), "first point cloud (stamp %.3f)", raw_points->stamp);
    }
    if (!time_keeper_->process(raw_points)) {
      RCLCPP_WARN(get_logger(), "skip an invalid point cloud (stamp=%f)", raw_points->stamp);
      return;
    }
    async_odometry_->insert_frame(preprocessor_->preprocess(raw_points));
  }

  /// Queue a reset to the given pose. @return an empty string if accepted, otherwise the reason
  std::string request_reset(const PoseMsg & msg, const char * source)
  {
    if (!odometry_) {
      return "the localization is not active";
    }
    Eigen::Isometry3d T_map_imu;
    double min_stamp = 0.0;
    std::string error = bridge_->to_T_map_imu(msg, T_map_imu, min_stamp);
    if (!error.empty()) {
      return error;
    }
    // Compared with the last frame the odometry thread took, estimated or dropped (not the last
    // estimated frame: none is estimated before the first pose or while LOST). The pose stamp is
    // frame time, like the frame stamps (points_time_offset already in), so it is not shifted.
    const InitPoseDelay delay = check_init_pose_delay(min_stamp,
        odometry_->last_input_frame_stamp(), max_init_pose_delay_);
    if (!delay.accept) {
      char buf[192];
      std::snprintf(
        buf, sizeof(buf),
        "too late: the pose is at least %.3f s older than the last frame taken by the odometry "
        "(max_init_pose_delay %.3f s)",
        *delay.late_by,
        max_init_pose_delay_);
      return buf;
    }
    if (!odometry_->reset(T_map_imu, Eigen::Vector3d::Zero(), min_stamp)) {
      return "the odometry rejected the pose";
    }
    if (delay.late_by) {
      RCLCPP_WARN(get_logger(), "%s: the pose applies at least %.3f s after its stamp", source,
          *delay.late_by);
    }
    RCLCPP_INFO(get_logger(), "%s: reset requested (stamp=%.3f)", source, min_stamp);
    return "";
  }

  void on_initial_pose(const PoseMsg & msg)
  {
    const std::string error = request_reset(msg, "/initialpose");
    if (!error.empty()) {
      RCLCPP_WARN(get_logger(), "ignoring /initialpose: %s", error.c_str());
    }
  }

  void on_initialize(const InitializeSrv::Request & req, InitializeSrv::Response & res)
  {
    res.score = 0.0;
    if (req.global_search) {
      res.success = false;
      res.message = "global relocalization is not implemented";
    } else {
      const std::string error = request_reset(req.pose, "~/initialize");
      res.success = error.empty();
      res.message =
        error.empty() ? "accepted: the reset applies at the next frame at or after the stamp" :
        error;
    }
    if (!res.success) {
      RCLCPP_WARN(get_logger(), "~/initialize rejected: %s", res.message.c_str());
    }
  }

  /// Restart from the newest estimate, moved to the next frame by its velocity (the core's
  /// relocalize()). Refused unless TRACKING or DEGRADED, while a reset is pending (it would
  /// silently replace an accepted ~/initialize: the latest request wins), and when the estimate is
  /// more than max_init_pose_delay older than the last input frame.
  void on_relocalize(TriggerSrv::Response & res)
  {
    res.success = false;
    if (!odometry_) {
      res.message = "the localization is not active";
    } else {
      const std::string error = odometry_->relocalize(max_init_pose_delay_);
      if (!error.empty()) {
        res.message = error;
      } else {
        res.success = true;
        res.message = "accepted: restarting from the newest estimate at the next frame";
        RCLCPP_INFO(get_logger(), "~/relocalize: reset requested from the newest estimate");
      }
    }
    if (!res.success) {
      RCLCPP_WARN(get_logger(), "~/relocalize rejected: %s", res.message.c_str());
    }
  }

  const std::shared_ptr<EstimateBridge> bridge_;

  // Set by configure
  double max_init_pose_delay_ = 0.2;
  InputSettings input_;
  std::optional<rclcpp::QoS> imu_qos_;
  std::optional<rclcpp::QoS> points_qos_;
  std::optional<rclcpp::QoS> wheel_odom_qos_;
  LocalizationPublisherParams publisher_params_;
  std::unique_ptr<OdometryParams> odometry_params_;
  std::shared_ptr<const glim_localization::MapManager> map_;
  rclcpp::Service<InitializeSrv>::SharedPtr initialize_srv_;
  rclcpp::Service<TriggerSrv>::SharedPtr relocalize_srv_;
  std::unique_ptr<MapPublisher> map_publisher_;

  // Set by activate
  double acc_scale_ = 0.0;
  // Written by the subscription callbacks, read by stop(), which can also run from on_shutdown or
  // the destructor on another thread
  std::atomic<size_t> num_imu_{0};
  std::atomic<size_t> num_points_{0};
  std::atomic<double> first_points_stamp_{0.0};
  std::atomic<size_t> num_wheel_odom_{0};
  std::atomic<size_t> num_wheel_odom_rejected_{0};  ///< By the conversion (frame, stamp, value)
  std::atomic<size_t> num_wheel_odom_dropped_{0};   ///< By the odometry (not newer than the last)
  std::atomic<bool> wheel_odom_frame_logged_{false};
  std::unique_ptr<glim::TimeKeeper> time_keeper_;
  std::unique_ptr<glim::CloudPreprocessor> preprocessor_;
  std::shared_ptr<glim_localization::OdometryEstimationLocalization> odometry_;
  std::unique_ptr<glim::AsyncOdometryEstimation> async_odometry_;
  rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<PointsMsg>::SharedPtr points_sub_;
  rclcpp::Subscription<PoseMsg>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<OdometryMsg>::SharedPtr wheel_odom_sub_;
  rclcpp::TimerBase::SharedPtr drain_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;
};

}  // namespace glim_localization_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(glim_localization_ros2::LocalizationNode)
