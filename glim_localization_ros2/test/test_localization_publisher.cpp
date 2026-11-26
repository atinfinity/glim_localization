// Copyright (c) 2026 atinfinity
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// In-process test of LocalizationPublisher with synthetic frames and one map jump. No bag, no
// display.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <map>
#include <thread>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "glim_localization_ros2/localization_publisher.hpp"

namespace
{

using glim_localization_ros2::FrameEstimate;
using glim_localization_ros2::LocalizationPublisher;
using glim_localization_ros2::LocalizationPublisherParams;
using glim_localization_ros2::TfMode;
using StatusMsg = glim_localization_msgs::msg::LocalizationStatus;
using glim_localization::LocalizationState;
using glim_localization::ResetReason;
using namespace std::chrono_literals;

constexpr int kFrames = 20;
constexpr int kJumpAt = 10;
constexpr double kDt = 0.1;
constexpr double kStart = 100.0;

int64_t key(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
}

Eigen::Isometry3d to_eigen(const geometry_msgs::msg::Pose & p)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y,
      p.orientation.z).normalized().toRotationMatrix();
  T.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  return T;
}

Eigen::Isometry3d to_eigen(const geometry_msgs::msg::Transform & t)
{
  geometry_msgs::msg::Pose p;
  p.position.x = t.translation.x;
  p.position.y = t.translation.y;
  p.position.z = t.translation.z;
  p.orientation = t.rotation;
  return to_eigen(p);
}

Eigen::Isometry3d make_pose(double x, double y, double yaw)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = Eigen::Vector3d(x, y, 0.5);
  return T;
}

/// Moves at 1 m/s along x while turning. The map pose jumps by `jump` along x and y at frame
/// kJumpAt.
FrameEstimate make_frame(int k, double jump)
{
  FrameEstimate frame;
  frame.stamp = kStart + k * kDt;
  const double offset = k >= kJumpAt ? jump : 0.0;
  frame.T_map_imu = make_pose(k * kDt + offset, 2.0 * offset, 0.05 * k);
  frame.v_map_imu = Eigen::Vector3d(1.0, 0.0, 0.0);
  return frame;
}

FrameEstimate make_frame_at(
  double stamp, const Eigen::Isometry3d & T_map_imu,
  const Eigen::Vector3d & v = Eigen::Vector3d::Zero(),
  const Eigen::Vector3d & w = Eigen::Vector3d::Zero())
{
  FrameEstimate frame;
  frame.stamp = stamp;
  frame.T_map_imu = T_map_imu;
  frame.v_map_imu = v;
  frame.w_imu = w;
  return frame;
}

// Same conversion as the publisher's, so that a test can stamp a message at exactly a frame's stamp
builtin_interfaces::msg::Time to_msg(double stamp)
{
  builtin_interfaces::msg::Time msg;
  const double sec = std::floor(stamp);
  msg.sec = static_cast<int32_t>(sec);
  msg.nanosec = static_cast<uint32_t>(std::min(std::round((stamp - sec) * 1e9), 999999999.0));
  return msg;
}

/// Translation difference and rotation angle between two poses, whichever is larger
double pose_step(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  const Eigen::Isometry3d d = a.inverse() * b;
  return std::max(d.translation().norm(), Eigen::AngleAxisd(d.rotation()).angle());
}

/// Hosts the publisher under test plus an observer, in one executor.
class Fixture {
public:
  explicit Fixture(
    const LocalizationPublisherParams & params,
    const std::string & name = "publisher_test")
  {
    node_ = std::make_shared<rclcpp::Node>(name);
    observer_ = std::make_shared<rclcpp::Node>(name + "_observer");
    executor_.add_node(node_);
    executor_.add_node(observer_);
    tf_sub_ = observer_->create_subscription<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100),
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
          for (const auto & t : msg->transforms) {
            transforms.push_back(t);
          }
    });
    pose_sub_ = observer_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/" + name + "/pose", rclcpp::QoS(100),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        poses.push_back(*msg);
      });
    odometry_sub_ = observer_->create_subscription<nav_msgs::msg::Odometry>(
      "/" + name + "/odometry", rclcpp::QoS(100),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {odometries.push_back(*msg);});
    status_sub_ = observer_->create_subscription<StatusMsg>("/" + name + "/status",
        rclcpp::QoS(100), [this](StatusMsg::ConstSharedPtr msg) {statuses.push_back(*msg);});
    diag_sub_ = observer_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(100),
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
        diagnostics.push_back(*msg);
      });
    publisher = std::make_unique<LocalizationPublisher>(*node_, params);
  }

  ~Fixture()
  {
    executor_.remove_node(node_);
    executor_.remove_node(observer_);
  }

  template<typename Predicate>
  bool spin_until(Predicate done, std::chrono::milliseconds timeout = 10s)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (!done()) {
      if (std::chrono::steady_clock::now() > end) {
        return false;
      }
      executor_.spin_some(10ms);
    }
    return true;
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some(10ms);
    }
  }

  /// Wait until the publisher's endpoints are matched with the observer, so that no message is lost
  void wait_for_discovery(size_t tf_subscribers)
  {
    const std::string base = std::string("/") + node_->get_name();
    ASSERT_TRUE(spin_until([&] {
        return node_->count_subscribers(base + "/pose") >= 1 &&
               node_->count_subscribers(base + "/status") >= 1 &&
               node_->count_subscribers(base + "/odometry") >= 1 &&
               node_->count_subscribers("/diagnostics") >= 1 &&
               node_->count_subscribers("/tf") >= tf_subscribers;
    })) << "discovery timed out";
    spin_for(200ms);
  }

  /// Find the transform for a frame pair at a stamp
  std::optional<Eigen::Isometry3d> find(
    const std::string & parent, const std::string & child,
    int64_t stamp) const
  {
    for (const auto & t : transforms) {
      if (t.header.frame_id == parent && t.child_frame_id == child &&
        key(t.header.stamp) == stamp)
      {
        return to_eigen(t.transform);
      }
    }
    return std::nullopt;
  }

  size_t count(const std::string & parent, const std::string & child) const
  {
    size_t n = 0;
    for (const auto & t : transforms) {
      n += t.header.frame_id == parent && t.child_frame_id == child;
    }
    return n;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr observer_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::unique_ptr<LocalizationPublisher> publisher;
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> poses;
  std::vector<nav_msgs::msg::Odometry> odometries;
  std::vector<StatusMsg> statuses;
  int reset_count = 0;
  ResetReason reset_reason = ResetReason::NONE;
  int next_frame_id = 0;

  /// What the core does for a frame: the frame arrives first, then its status
  void feed(const FrameEstimate & frame, LocalizationState state = LocalizationState::TRACKING)
  {
    publisher->on_frame(frame);
    glim_localization::LocalizationStatus status;
    status.state = state;
    status.stamp = frame.stamp;
    status.frame_id = next_frame_id++;
    status.reset_count = reset_count;
    status.last_reset_reason = reset_reason;
    publisher->on_status(status);
  }

  /// A reset as the core emits it: no frame, stamp = min_stamp = 0
  void reset(ResetReason reason)
  {
    ++reset_count;
    reset_reason = reason;
    glim_localization::LocalizationStatus status;
    status.reset_count = reset_count;
    status.last_reset_reason = reason;
    publisher->on_status(status);
  }
  std::vector<diagnostic_msgs::msg::DiagnosticArray> diagnostics;

private:
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<StatusMsg>::SharedPtr status_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
};

glim_localization::LocalizationStatus frame_status(
  int k,
  glim_localization::LocalizationState state, int reset_count = 0)
{
  glim_localization::LocalizationStatus status;
  status.state = state;
  status.stamp = kStart + k * kDt;
  status.frame_id = k;
  status.reset_count = reset_count;
  return status;
}

class RclcppEnvironment : public ::testing::Environment {
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};
const auto * const environment = ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

}  // namespace

TEST(TfMode, ParsesNames) {
  EXPECT_EQ(glim_localization_ros2::parse_tf_mode("internal_odom"), TfMode::INTERNAL_ODOM);
  EXPECT_EQ(glim_localization_ros2::parse_tf_mode("external_odom"), TfMode::EXTERNAL_ODOM);
  EXPECT_EQ(glim_localization_ros2::parse_tf_mode("none"), TfMode::NONE);
  EXPECT_THROW(glim_localization_ros2::parse_tf_mode("odom"), std::invalid_argument);
}

// odom->base has no jump across an external reset, and map->odom * odom->base is the pose at every
// frame.
TEST(LocalizationPublisher, InternalOdomHasNoOdomJumpAtAnExternalResetAndComposesToPose) {
  LocalizationPublisherParams params;
  params.tf_mode = TfMode::INTERNAL_ODOM;
  Fixture fixture(params);
  fixture.wait_for_discovery(1);

  for (int k = 0; k < kFrames; ++k) {
    if (k == kJumpAt) {
      fixture.reset(ResetReason::EXTERNAL);
    }
    fixture.feed(make_frame(k, 3.0));
  }
  ASSERT_TRUE(fixture.spin_until([&] {
      return fixture.poses.size() >= kFrames &&
             fixture.count("odom", "base_link") >= kFrames &&
             fixture.count("map", "odom") >= kFrames;
    }));
  ASSERT_EQ(fixture.poses.size(), static_cast<size_t>(kFrames));

  std::optional<Eigen::Isometry3d> previous_odom;
  double largest_odom_step = 0.0;
  double map_odom_step_at_jump = 0.0;
  std::optional<Eigen::Isometry3d> previous_map_odom;
  for (int k = 0; k < kFrames; ++k) {
    const auto stamp = key(fixture.poses[k].header.stamp);
    const auto map_odom = fixture.find("map", "odom", stamp);
    const auto odom_base = fixture.find("odom", "base_link", stamp);
    ASSERT_TRUE(map_odom && odom_base) << "frame " << k;
    const Eigen::Isometry3d composed = *map_odom * *odom_base;
    const Eigen::Isometry3d pose = to_eigen(fixture.poses[k].pose.pose);
    EXPECT_LT((composed.translation() - pose.translation()).norm(), 1e-6) << "frame " << k;
    EXPECT_LT(Eigen::Quaterniond(composed.rotation()).angularDistance(Eigen::Quaterniond(
      pose.rotation())), 1e-6) << "frame " << k;
    if (previous_odom) {
      largest_odom_step = std::max(largest_odom_step,
          (odom_base->translation() - previous_odom->translation()).norm());
    }
    if (previous_map_odom && k == kJumpAt) {
      map_odom_step_at_jump = (map_odom->translation() - previous_map_odom->translation()).norm();
    }
    previous_odom = odom_base;
    previous_map_odom = map_odom;
  }
  // 1 m/s at 10 Hz is 0.1 m per frame. The map pose jumps by 3 m x and 6 m y
  EXPECT_LT(largest_odom_step, 0.15);
  EXPECT_GT(map_odom_step_at_jump, 6.0);
}

// A small jump of an external reset is absorbed by map->odom too.
TEST(LocalizationPublisher, InternalOdomAbsorbsSmallJumpOfAnExternalReset) {
  LocalizationPublisherParams params;
  Fixture fixture(params);
  fixture.wait_for_discovery(1);

  for (int k = 0; k < kFrames; ++k) {
    if (k == kJumpAt) {
      fixture.reset(ResetReason::EXTERNAL);
    }
    fixture.feed(make_frame(k, 0.3));
  }
  ASSERT_TRUE(fixture.spin_until([&] {
      return fixture.poses.size() >= kFrames && fixture.count("odom", "base_link") >= kFrames;
    }));

  std::optional<Eigen::Isometry3d> previous;
  double largest_step = 0.0;
  for (int k = 0; k < kFrames; ++k) {
    const auto odom_base = fixture.find("odom", "base_link", key(fixture.poses[k].header.stamp));
    ASSERT_TRUE(odom_base);
    if (previous) {
      largest_step = std::max(largest_step,
          (odom_base->translation() - previous->translation()).norm());
    }
    previous = odom_base;
  }
  EXPECT_LT(largest_step, 0.15);
}

// map->odom = T_map_base * inv(T_odom_base) with an odom->base from someone else.
TEST(LocalizationPublisher, ExternalOdomUsesTheGivenOdomBase) {
  LocalizationPublisherParams params;
  params.tf_mode = TfMode::EXTERNAL_ODOM;
  Fixture fixture(params);
  auto fake_node = std::make_shared<rclcpp::Node>("fake_odom");
  fixture.executor_.add_node(fake_node);
  tf2_ros::TransformBroadcaster fake(fake_node);
  // The observer and the publisher's listener
  ASSERT_TRUE(fixture.spin_until([&] {return fake_node->count_subscribers("/tf") >= 2;}));
  fixture.wait_for_discovery(2);

  std::map<int, Eigen::Isometry3d> odom_base;
  for (int k = 0; k < kFrames; ++k) {
    const auto frame = make_frame(k, 3.0);
    // A wheel odometry that drifts from the map pose and never jumps
    odom_base[k] = make_pose(0.9 * k * kDt - 7.0, 0.01 * k, 0.04 * k - 1.0);
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.header.stamp.sec = static_cast<int32_t>(std::floor(frame.stamp));
    t.header.stamp.nanosec = static_cast<uint32_t>(std::round((frame.stamp -
      std::floor(frame.stamp)) * 1e9));
    const Eigen::Quaterniond q(odom_base[k].rotation());
    t.transform.translation.x = odom_base[k].translation().x();
    t.transform.translation.y = odom_base[k].translation().y();
    t.transform.translation.z = odom_base[k].translation().z();
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    fake.sendTransform(t);
    const tf2::TimePoint tp(std::chrono::nanoseconds(key(t.header.stamp)));
    ASSERT_TRUE(fixture.spin_until([&] {
        return fixture.publisher->tf_buffer()->canTransform("odom", "base_link", tp);
      }))
      << "frame " << k;
    fixture.feed(frame);
  }
  ASSERT_TRUE(fixture.spin_until([&] {
      return fixture.poses.size() >= kFrames && fixture.count("map", "odom") >= kFrames;
    }));
  EXPECT_EQ(fixture.publisher->tf_lookup_failures(), 0u);
  // Only the fake publishes odom->base
  EXPECT_EQ(fixture.count("odom", "base_link"), static_cast<size_t>(kFrames));

  for (int k = 0; k < kFrames; ++k) {
    const Eigen::Isometry3d expected = to_eigen(fixture.poses[k].pose.pose) *
      odom_base[k].inverse();
    const auto map_odom = fixture.find("map", "odom", key(fixture.poses[k].header.stamp));
    ASSERT_TRUE(map_odom) << "frame " << k;
    EXPECT_LT((map_odom->translation() - expected.translation()).norm(), 1e-6) << "frame " << k;
    EXPECT_LT(Eigen::Quaterniond(map_odom->rotation()).angularDistance(Eigen::Quaterniond(
      expected.rotation())), 1e-6) << "frame " << k;
  }
  fixture.executor_.remove_node(fake_node);
}

TEST(LocalizationPublisher, ExternalOdomWithoutOdomBasePublishesPoseButNoTf) {
  LocalizationPublisherParams params;
  params.tf_mode = TfMode::EXTERNAL_ODOM;
  Fixture fixture(params, "publisher_external_missing");
  fixture.wait_for_discovery(1);
  for (int k = 0; k < 3; ++k) {
    fixture.feed(make_frame(k, 0.0));
  }
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.poses.size() >= 3;}));
  fixture.spin_for(200ms);
  EXPECT_EQ(fixture.publisher->tf_lookup_failures(), 3u);
  EXPECT_TRUE(fixture.transforms.empty());
}

TEST(LocalizationPublisher, NoneModePublishesNothingOnTf) {
  LocalizationPublisherParams params;
  params.tf_mode = TfMode::NONE;
  Fixture fixture(params);
  // The observer's /tf subscription must be up, so that silence means something
  fixture.wait_for_discovery(1);
  for (int k = 0; k < kFrames; ++k) {
    fixture.feed(make_frame(k, 3.0));
  }
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.poses.size() >= kFrames;}));
  fixture.spin_for(300ms);
  EXPECT_TRUE(fixture.transforms.empty());
  EXPECT_EQ(fixture.node_->count_publishers("/tf"), 0u);
}

// A reset status carries stamp = min_stamp = 0. It must not become a pose or a status time.
TEST(LocalizationPublisher, ResetStatusWithZeroStampIsNotAFrameTime) {
  LocalizationPublisherParams params;
  Fixture fixture(params);
  fixture.wait_for_discovery(1);

  fixture.reset(ResetReason::IMU_GAP);
  fixture.publisher->on_status(frame_status(3, glim_localization::LocalizationState::TRACKING, 1));
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.statuses.size() >= 2;}));

  EXPECT_TRUE(fixture.poses.empty());
  EXPECT_TRUE(fixture.transforms.empty());
  const auto & reset_msg = fixture.statuses[0];
  EXPECT_NE(key(reset_msg.header.stamp), 0);
  EXPECT_EQ(reset_msg.reset_count, 1);
  EXPECT_EQ(reset_msg.last_reset_reason, StatusMsg::RESET_IMU_GAP);
  EXPECT_EQ(reset_msg.state, StatusMsg::INITIALIZING);
  // A frame status keeps its frame stamp
  EXPECT_NEAR(rclcpp::Time(fixture.statuses[1].header.stamp).seconds(), kStart + 3 * kDt, 1e-6);
  EXPECT_EQ(fixture.statuses[1].state, StatusMsg::TRACKING);
}

TEST(LocalizationPublisher, DiagnosticsWarnOnDegradedAndErrorOnLost) {
  LocalizationPublisherParams params;
  params.diagnostics_period = 0.0;
  Fixture fixture(params);
  fixture.wait_for_discovery(1);

  const std::vector<std::pair<LocalizationState, uint8_t>> cases = {
    {LocalizationState::INITIALIZING, diagnostic_msgs::msg::DiagnosticStatus::OK},
    {LocalizationState::TRACKING, diagnostic_msgs::msg::DiagnosticStatus::OK},
    {LocalizationState::DEGRADED, diagnostic_msgs::msg::DiagnosticStatus::WARN},
    {LocalizationState::LOST, diagnostic_msgs::msg::DiagnosticStatus::ERROR}};
  int k = 0;
  for (const auto & c : cases) {
    fixture.publisher->on_status(frame_status(k++, c.first));
  }
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.diagnostics.size() >= cases.size();}));
  for (size_t i = 0; i < cases.size(); ++i) {
    ASSERT_EQ(fixture.diagnostics[i].status.size(), 1u);
    EXPECT_EQ(fixture.diagnostics[i].status[0].level, cases[i].second) << "case " << i;
  }
}

// The result of the initial pose search reaches the status message and the diagnostics.
TEST(LocalizationPublisher, InitSearchResultIsReported) {
  LocalizationPublisherParams params;
  params.diagnostics_period = 0.0;
  Fixture fixture(params, "publisher_init_search");
  fixture.wait_for_discovery(1);

  auto accepted = frame_status(0, LocalizationState::INITIALIZING, 1);
  accepted.init_search = glim_localization::InitSearchResult::ACCEPTED;
  accepted.init_search_candidates = 24;
  accepted.init_search_candidates_total = 24;
  accepted.init_search_score = 0.95;
  accepted.init_search_yaw_offset = -M_PI / 2.0;
  accepted.init_search_runner_up_score = 0.8;
  accepted.init_search_runner_up_yaw_offset = M_PI / 2.0;
  accepted.init_search_time = 0.8;
  fixture.publisher->on_status(accepted);
  auto truncated = frame_status(1, LocalizationState::LOST, 2);
  truncated.init_search = glim_localization::InitSearchResult::REJECTED;
  truncated.init_search_reason = glim_localization::InitSearchReason::TRUNCATED;
  truncated.init_search_candidates = 1;
  truncated.init_search_candidates_total = 24;
  truncated.init_search_score = 0.6;
  fixture.publisher->on_status(truncated);
  auto ambiguous = frame_status(2, LocalizationState::LOST, 3);
  ambiguous.init_search = glim_localization::InitSearchResult::REJECTED;
  ambiguous.init_search_reason = glim_localization::InitSearchReason::AMBIGUOUS;
  ambiguous.init_search_candidates = 24;
  ambiguous.init_search_candidates_total = 24;
  ambiguous.init_search_score = 0.93;
  ambiguous.init_search_runner_up_score = 0.92;
  fixture.publisher->on_status(ambiguous);
  auto low_score = frame_status(3, LocalizationState::LOST, 4);
  low_score.init_search = glim_localization::InitSearchResult::REJECTED;
  low_score.init_search_reason = glim_localization::InitSearchReason::LOW_SCORE;
  low_score.init_search_candidates = 24;
  low_score.init_search_candidates_total = 24;
  low_score.init_search_score = 0.41;
  fixture.publisher->on_status(low_score);
  auto no_candidate = frame_status(4, LocalizationState::LOST, 5);
  no_candidate.init_search = glim_localization::InitSearchResult::REJECTED;
  no_candidate.init_search_reason = glim_localization::InitSearchReason::NO_CANDIDATE;
  fixture.publisher->on_status(no_candidate);
  auto out_of_range = frame_status(5, LocalizationState::LOST, 6);
  out_of_range.init_search = glim_localization::InitSearchResult::REJECTED;
  out_of_range.init_search_reason = glim_localization::InitSearchReason::OUT_OF_RANGE;
  out_of_range.init_search_score = 0.959;
  fixture.publisher->on_status(out_of_range);
  ASSERT_TRUE(fixture.spin_until([&] {
      return fixture.statuses.size() >= 6 && fixture.diagnostics.size() >= 6;
    }));

  const auto & msg = fixture.statuses[0];
  EXPECT_EQ(msg.init_search, StatusMsg::INIT_SEARCH_ACCEPTED);
  EXPECT_EQ(msg.init_search_candidates, 24);
  EXPECT_EQ(msg.init_search_candidates_total, 24);
  EXPECT_DOUBLE_EQ(msg.init_search_score, 0.95);
  EXPECT_DOUBLE_EQ(msg.init_search_yaw_offset, -M_PI / 2.0);
  EXPECT_DOUBLE_EQ(msg.init_search_runner_up_score, 0.8);
  EXPECT_DOUBLE_EQ(msg.init_search_runner_up_yaw_offset, M_PI / 2.0);
  EXPECT_DOUBLE_EQ(msg.init_search_time, 0.8);
  EXPECT_EQ(fixture.statuses[1].init_search, StatusMsg::INIT_SEARCH_REJECTED);
  EXPECT_EQ(fixture.statuses[1].init_search_candidates, 1);
  EXPECT_EQ(msg.init_search_reason, StatusMsg::INIT_SEARCH_REASON_NONE);
  EXPECT_EQ(fixture.statuses[1].init_search_reason, StatusMsg::INIT_SEARCH_REASON_TRUNCATED);
  EXPECT_EQ(fixture.statuses[2].init_search_reason, StatusMsg::INIT_SEARCH_REASON_AMBIGUOUS);
  EXPECT_EQ(fixture.statuses[3].init_search_reason, StatusMsg::INIT_SEARCH_REASON_LOW_SCORE);
  EXPECT_EQ(fixture.statuses[4].init_search_reason, StatusMsg::INIT_SEARCH_REASON_NO_CANDIDATE);
  EXPECT_EQ(fixture.statuses[5].init_search_reason, StatusMsg::INIT_SEARCH_REASON_OUT_OF_RANGE);
  EXPECT_EQ(fixture.statuses[1].init_search_candidates_total, 24);

  const auto value_of = [](const diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & key) {
      for (const auto & kv : status.values) {
        if (kv.key == key) {
          return kv.value;
        }
      }
      return std::string("<missing>");
    };
  const auto & first = fixture.diagnostics[0].status.at(0);
  EXPECT_EQ(value_of(first, "init_search"), "ACCEPTED");
  EXPECT_EQ(value_of(first, "init_search_reason"), "NONE");
  EXPECT_EQ(value_of(first, "init_search_candidates"), "24");
  EXPECT_EQ(value_of(first, "init_search_candidates_total"), "24");
  EXPECT_NEAR(std::stod(value_of(first, "init_search_yaw_offset_deg")), -90.0, 1e-3);
  EXPECT_NEAR(std::stod(value_of(first, "init_search_runner_up_score")), 0.8, 1e-6);
  EXPECT_NEAR(std::stod(value_of(first, "init_search_runner_up_yaw_offset_deg")), 90.0, 1e-3);
  const auto & second = fixture.diagnostics[1].status.at(0);
  EXPECT_EQ(second.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(value_of(second, "init_search"), "REJECTED");
  EXPECT_NE(second.message.find("rejected"), std::string::npos) << second.message;
  EXPECT_NE(second.message.find("time budget ran out after 1 of 24 candidates"),
    std::string::npos) << second.message;
  EXPECT_EQ(value_of(second, "init_search_reason"), "TRUNCATED");
  const auto & third = fixture.diagnostics[2].status.at(0);
  EXPECT_NE(third.message.find("best inlier ratio 0.930, runner-up 0.920"),
    std::string::npos) << third.message;
  EXPECT_EQ(value_of(third, "init_search_reason"), "AMBIGUOUS");
  const auto & fourth = fixture.diagnostics[3].status.at(0);
  EXPECT_NE(fourth.message.find("best inlier ratio 0.410 is below the minimum"),
    std::string::npos) << fourth.message;
  EXPECT_EQ(fourth.message.find("runner-up"), std::string::npos) << fourth.message;
  EXPECT_EQ(value_of(fourth, "init_search_reason"), "LOW_SCORE");
  const auto & fifth = fixture.diagnostics[4].status.at(0);
  EXPECT_NE(fifth.message.find("no candidate to score"), std::string::npos) << fifth.message;
  EXPECT_EQ(fifth.message.find("0.000"), std::string::npos) << fifth.message;
  EXPECT_EQ(fifth.message.find("runner-up"), std::string::npos) << fifth.message;
  EXPECT_EQ(value_of(fifth, "init_search_reason"), "NO_CANDIDATE");
  const auto & sixth = fixture.diagnostics[5].status.at(0);
  EXPECT_NE(sixth.message.find("out of range: inlier ratio 0.959 beyond the maximum translation"),
    std::string::npos) << sixth.message;
  EXPECT_EQ(value_of(sixth, "init_search_reason"), "OUT_OF_RANGE");
}

// The wheel odometry counters reach the status message and the diagnostics.
TEST(LocalizationPublisher, WheelCountersAreReported) {
  LocalizationPublisherParams params;
  params.diagnostics_period = 0.0;
  Fixture fixture(params, "publisher_wheel");
  fixture.wait_for_discovery(1);

  auto status = frame_status(0, LocalizationState::TRACKING, 1);
  status.wheel_factors = 7;
  status.wheel_gated = 3;
  status.wheel_readmitted = 2;
  status.wheel_missing = 5;
  fixture.publisher->on_status(status);
  ASSERT_TRUE(fixture.spin_until([&] {
      return fixture.statuses.size() >= 1 && fixture.diagnostics.size() >= 1;
    }));

  const auto & msg = fixture.statuses[0];
  EXPECT_EQ(msg.wheel_factors, 7);
  EXPECT_EQ(msg.wheel_gated, 3);
  EXPECT_EQ(msg.wheel_readmitted, 2);
  EXPECT_EQ(msg.wheel_missing, 5);
  std::map<std::string, std::string> values;
  for (const auto & kv : fixture.diagnostics[0].status.at(0).values) {
    values[kv.key] = kv.value;
  }
  EXPECT_EQ(values["wheel_factors"], "7");
  EXPECT_EQ(values["wheel_gated"], "3");
  EXPECT_EQ(values["wheel_readmitted"], "2");
  EXPECT_EQ(values["wheel_missing"], "5");
}

// The pose of a frame is published only with the status of the same frame.
TEST(LocalizationPublisher, FrameAloneIsNotPublished) {
  Fixture fixture(LocalizationPublisherParams(), "publisher_pairing");
  fixture.wait_for_discovery(1);
  fixture.publisher->on_frame(make_frame(0, 0.0));
  fixture.spin_for(300ms);
  EXPECT_TRUE(fixture.poses.empty());
  EXPECT_TRUE(fixture.odometries.empty());
  EXPECT_TRUE(fixture.transforms.empty());
}

TEST(LocalizationPublisher, StatusWithoutMatchingFramePublishesStatusOnly) {
  Fixture fixture(LocalizationPublisherParams(), "publisher_unmatched");
  fixture.wait_for_discovery(1);
  fixture.publisher->on_frame(make_frame(0, 0.0));
  fixture.publisher->on_status(frame_status(5, LocalizationState::TRACKING));
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.statuses.size() >= 1;}));
  fixture.spin_for(300ms);
  EXPECT_TRUE(fixture.poses.empty());
  EXPECT_TRUE(fixture.transforms.empty());
  // The pending frame is still paired with its own status
  fixture.publisher->on_status(frame_status(0, LocalizationState::TRACKING));
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.poses.size() >= 1;}));
}

TEST(LocalizationPublisher, FrameReplacedBeforeItsStatusIsDroppedAndCounted) {
  Fixture fixture(LocalizationPublisherParams(), "publisher_replaced");
  fixture.wait_for_discovery(1);
  fixture.publisher->on_frame(make_frame(0, 0.0));
  fixture.publisher->on_frame(make_frame(1, 0.0));
  fixture.publisher->on_status(frame_status(1, LocalizationState::TRACKING));
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.poses.size() >= 1;}));
  fixture.spin_for(200ms);
  ASSERT_EQ(fixture.poses.size(), 1u);
  EXPECT_EQ(key(fixture.poses[0].header.stamp), key(to_msg(kStart + kDt)));
  EXPECT_EQ(fixture.publisher->unpaired_frames(), 1u);
}

// internal_odom: a LOST frame gives no pose, odometry or map->odom, only odom->base. The first LOST
// frame is already suppressed, and the frame after it is published again.
TEST(LocalizationPublisher, LostFrameIsSuppressedExceptOdomBase) {
  Fixture fixture(LocalizationPublisherParams(), "publisher_lost");
  fixture.wait_for_discovery(1);
  fixture.feed(make_frame(0, 0.0));
  fixture.feed(make_frame(1, 0.0), LocalizationState::LOST);
  fixture.feed(make_frame(2, 0.0));
  ASSERT_TRUE(fixture.spin_until([&] {
      return fixture.statuses.size() >= 3 && fixture.count("odom", "base_link") >= 3 &&
             fixture.odometries.size() >= 2;
    }));
  fixture.spin_for(200ms);
  ASSERT_EQ(fixture.poses.size(), 2u);
  EXPECT_EQ(key(fixture.poses[0].header.stamp), key(to_msg(kStart)));
  EXPECT_EQ(key(fixture.poses[1].header.stamp), key(to_msg(kStart + 2 * kDt)));
  EXPECT_EQ(fixture.odometries.size(), 2u);
  EXPECT_EQ(fixture.count("map", "odom"), 2u);
  EXPECT_EQ(fixture.count("odom", "base_link"), 3u);
  EXPECT_TRUE(fixture.find("odom", "base_link", key(to_msg(kStart + kDt))));
  EXPECT_FALSE(fixture.find("map", "odom", key(to_msg(kStart + kDt))));
  // ~/status still reports the lost state
  EXPECT_EQ(fixture.statuses[1].state, StatusMsg::LOST);
}

TEST(LocalizationPublisher, ExternalOdomPublishesNothingInLost) {
  LocalizationPublisherParams params;
  params.tf_mode = TfMode::EXTERNAL_ODOM;
  Fixture fixture(params, "publisher_external_lost");
  fixture.wait_for_discovery(1);
  fixture.feed(make_frame(0, 0.0), LocalizationState::LOST);
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.statuses.size() >= 1;}));
  fixture.spin_for(300ms);
  EXPECT_TRUE(fixture.poses.empty());
  EXPECT_TRUE(fixture.transforms.empty());
  EXPECT_EQ(fixture.publisher->tf_lookup_failures(), 0u);
}

// TRACKING, LOST, then an external reset: odom->base resumes where the last one was.
TEST(LocalizationPublisher, ResumesAfterLostAndExternalResetWithoutOdomBaseStep) {
  Fixture fixture(LocalizationPublisherParams(), "publisher_resume");
  fixture.wait_for_discovery(1);
  for (int k = 0; k < 3; ++k) {
    fixture.feed(make_frame(k, 0.0));
  }
  fixture.feed(make_frame_at(kStart + 3 * kDt, make_pose(1.0, 0.0, 0.0)), LocalizationState::LOST);
  fixture.feed(make_frame_at(kStart + 4 * kDt, make_pose(5.0, 0.0, 0.0)), LocalizationState::LOST);
  fixture.reset(ResetReason::EXTERNAL);
  fixture.feed(make_frame_at(kStart + 5 * kDt, make_pose(40.0, 0.0, 0.0)));
  ASSERT_TRUE(fixture.spin_until([&] {
      return fixture.count("odom", "base_link") >= 6 && fixture.count("map", "odom") >= 4;
    }));
  const auto last_lost = fixture.find("odom", "base_link", key(to_msg(kStart + 4 * kDt)));
  const auto resumed = fixture.find("odom", "base_link", key(to_msg(kStart + 5 * kDt)));
  ASSERT_TRUE(last_lost && resumed);
  EXPECT_LT(pose_step(*last_lost, *resumed), 1e-9);
  const auto map_odom = fixture.find("map", "odom", key(to_msg(kStart + 5 * kDt)));
  ASSERT_TRUE(map_odom);
  EXPECT_NEAR(map_odom->translation().x(), 35.0, 1e-6);
}

// M1: an external reset after a long pause does not move odom->base, wherever the new pose is.
TEST(LocalizationPublisher, ExternalResetAfterALongPauseDoesNotStepOdomBase) {
  const std::vector<std::pair<std::string, Eigen::Isometry3d>> cases = {{"publisher_pause_same",
    make_pose(0.5, 0.0, 0.0)}, {"publisher_pause_far", make_pose(20.0, 5.0, 0.3)}};
  for (const auto & [name, after] : cases) {
    Fixture fixture(LocalizationPublisherParams(), name);
    fixture.wait_for_discovery(1);
    fixture.feed(make_frame_at(100.0, make_pose(0.0, 0.0, 0.0), Eigen::Vector3d(5.0, 0.0, 0.0)));
    fixture.feed(make_frame_at(100.1, make_pose(0.5, 0.0, 0.0), Eigen::Vector3d(5.0, 0.0, 0.0)));
    fixture.reset(ResetReason::EXTERNAL);
    fixture.feed(make_frame_at(160.1, after));
    ASSERT_TRUE(fixture.spin_until([&] {
        return fixture.count("odom", "base_link") >= 3 && fixture.count("map", "odom") >= 3;
      }))
      << name;
    const auto before_odom = fixture.find("odom", "base_link", key(to_msg(100.1)));
    const auto after_odom = fixture.find("odom", "base_link", key(to_msg(160.1)));
    const auto before_map = fixture.find("map", "odom", key(to_msg(100.1)));
    const auto after_map = fixture.find("map", "odom", key(to_msg(160.1)));
    ASSERT_TRUE(before_odom && after_odom && before_map && after_map) << name;
    EXPECT_LT(pose_step(*before_odom, *after_odom), 1e-9) << name;
    const Eigen::Isometry3d composed = *after_map * *after_odom;
    EXPECT_LT(pose_step(composed, after), 1e-6) << name;
    if (name == "publisher_pause_far") {
      EXPECT_GT(pose_step(*before_map, *after_map), 1.0);
    }
  }
}

// M3: the motion between two frames is the pose's, however long the interval is.
TEST(LocalizationPublisher, OdomBaseMotionAfterDroppedFramesEqualsThePoseMotion) {
  Fixture fixture(LocalizationPublisherParams(), "publisher_dropped");
  fixture.wait_for_discovery(1);
  // 10 m/s and 1 rad/s over 1 s: an arc of radius 10 m
  const Eigen::Isometry3d end = make_pose(10.0 * std::sin(1.0), 10.0 * (1.0 - std::cos(1.0)), 1.0);
  fixture.feed(make_frame_at(100.0, make_pose(0.0, 0.0, 0.0), Eigen::Vector3d(10.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 1.0)));
  fixture.feed(make_frame_at(101.0, end,
    Eigen::Vector3d(10.0 * std::cos(1.0), 10.0 * std::sin(1.0), 0.0),
    Eigen::Vector3d(0.0, 0.0, 1.0)));
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.count("odom", "base_link") >= 2;}));
  const auto first = fixture.find("odom", "base_link", key(to_msg(100.0)));
  const auto second = fixture.find("odom", "base_link", key(to_msg(101.0)));
  ASSERT_TRUE(first && second);
  const Eigen::Isometry3d relative = first->inverse() * *second;
  EXPECT_NEAR(relative.translation().norm(), 9.589, 1e-3);
  EXPECT_NEAR(Eigen::AngleAxisd(relative.rotation()).angle(), 1.0, 1e-9);
}

// M2/M3: without a reset the map->odom anchor is bit-identical over many frames.
TEST(LocalizationPublisher, MapOdomIsConstantWithoutAReset) {
  Fixture fixture(LocalizationPublisherParams(), "publisher_anchor");
  fixture.wait_for_discovery(1);
  for (int k = 0; k < 100; ++k) {
    fixture.feed(make_frame_at(100.0 + 0.1 * k,
      make_pose(0.37 * k, 0.11 * k * k * 0.01, 0.013 * k)));
  }
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.count("map", "odom") >= 100;}));
  std::vector<geometry_msgs::msg::Transform> map_odom;
  for (const auto & t : fixture.transforms) {
    if (t.header.frame_id == "map" && t.child_frame_id == "odom") {
      map_odom.push_back(t.transform);
    }
  }
  ASSERT_EQ(map_odom.size(), 100u);
  for (const auto & t : map_odom) {
    EXPECT_EQ(t.translation.x, map_odom[0].translation.x);
    EXPECT_EQ(t.translation.y, map_odom[0].translation.y);
    EXPECT_EQ(t.translation.z, map_odom[0].translation.z);
    EXPECT_EQ(t.rotation.x, map_odom[0].rotation.x);
    EXPECT_EQ(t.rotation.y, map_odom[0].rotation.y);
    EXPECT_EQ(t.rotation.z, map_odom[0].rotation.z);
    EXPECT_EQ(t.rotation.w, map_odom[0].rotation.w);
  }
}

// An IMU gap reset does not re-anchor: map->odom stays and the new pose goes to odom->base.
TEST(LocalizationPublisher, ImuGapResetDoesNotChangeMapOdom) {
  Fixture fixture(LocalizationPublisherParams(), "publisher_imu_gap");
  fixture.wait_for_discovery(1);
  for (int k = 0; k < 5; ++k) {
    fixture.feed(make_frame(k, 0.0));
  }
  fixture.reset(ResetReason::IMU_GAP);
  for (int k = 5; k < 10; ++k) {
    fixture.feed(make_frame(k, 0.0));
  }
  fixture.reset(ResetReason::IMU_GAP);
  fixture.feed(make_frame_at(kStart + 1.0, make_pose(2.0, 0.0, 0.0)));
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.count("map", "odom") >= 11;}));
  const auto first = fixture.find("map", "odom", key(to_msg(kStart)));
  const auto last = fixture.find("map", "odom", key(to_msg(kStart + 1.0)));
  ASSERT_TRUE(first && last);
  EXPECT_EQ(pose_step(*first, *last), 0.0);
}

// Pins the definition: a change of the estimate without a reset, such as a 90 deg correction, is in
// odom->base.
TEST(LocalizationPublisher, YawChangeWithoutResetGoesToOdomBase) {
  Fixture fixture(LocalizationPublisherParams(), "publisher_yaw");
  fixture.wait_for_discovery(1);
  fixture.feed(make_frame_at(100.0, make_pose(0.0, 0.0, 0.0)));
  fixture.feed(make_frame_at(100.1, make_pose(0.0, 0.0, M_PI / 2)));
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.count("odom", "base_link") >= 2;}));
  const auto a = fixture.find("odom", "base_link", key(to_msg(100.0)));
  const auto b = fixture.find("odom", "base_link", key(to_msg(100.1)));
  const auto map_a = fixture.find("map", "odom", key(to_msg(100.0)));
  const auto map_b = fixture.find("map", "odom", key(to_msg(100.1)));
  ASSERT_TRUE(a && b && map_a && map_b);
  EXPECT_NEAR(pose_step(*a, *b), M_PI / 2, 1e-9);
  EXPECT_EQ(pose_step(*map_a, *map_b), 0.0);
}

// M4, N2: the twist of the base from the IMU's velocity and angular velocity, with a lever arm and
// a rotated T_base_imu.
TEST(LocalizationPublisher, TwistOfASpinInPlaceHasNoLinearPartWithALeverArm) {
  LocalizationPublisherParams params;
  params.tf_mode = TfMode::NONE;
  // the IMU is 1 m ahead of the base
  params.T_base_imu.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  Fixture fixture(params, "publisher_spin");
  fixture.wait_for_discovery(1);
  // The base spins at the origin at 1 rad/s, so the IMU moves along y at 1 m/s
  fixture.feed(make_frame_at(100.0, Eigen::Isometry3d(Eigen::Translation3d(1.0, 0.0, 0.0)),
    Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0)));
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.odometries.size() >= 1;}));
  const auto & twist = fixture.odometries[0].twist.twist;
  EXPECT_NEAR(twist.linear.x, 0.0, 1e-12);
  EXPECT_NEAR(twist.linear.y, 0.0, 1e-12);
  EXPECT_NEAR(twist.linear.z, 0.0, 1e-12);
  EXPECT_NEAR(twist.angular.z, 1.0, 1e-12);
}

TEST(LocalizationPublisher, TwistMatchesTheBaseMotionWithARotatedBaseImu) {
  LocalizationPublisherParams params;
  params.tf_mode = TfMode::NONE;
  params.T_base_imu = Eigen::Isometry3d::Identity();
  params.T_base_imu.linear() = (Eigen::AngleAxisd(M_PI / 2,
    Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.3,
    Eigen::Vector3d::UnitX())).toRotationMatrix();
  params.T_base_imu.translation() = Eigen::Vector3d(1.0, 0.5, 0.2);
  Fixture fixture(params, "publisher_twist");
  fixture.wait_for_discovery(1);

  // A rigid body: the base moves with v and rotates with omega (both in the map frame)
  Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();
  T_map_base.linear() = (Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.1,
    Eigen::Vector3d::UnitY())).toRotationMatrix();
  T_map_base.translation() = Eigen::Vector3d(3.0, -2.0, 0.5);
  const Eigen::Vector3d v_base_map(1.0, 2.0, 0.5);
  const Eigen::Vector3d omega_map(0.1, -0.2, 0.8);
  const Eigen::Isometry3d T_map_imu = T_map_base * params.T_base_imu;
  const Eigen::Vector3d v_imu_map = v_base_map +
    omega_map.cross(T_map_base.rotation() * params.T_base_imu.translation());
  const Eigen::Vector3d w_imu = T_map_imu.rotation().transpose() * omega_map;

  fixture.feed(make_frame_at(100.0, T_map_imu, v_imu_map, w_imu));
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.odometries.size() >= 1;}));
  const Eigen::Vector3d linear = T_map_base.rotation().transpose() * v_base_map;
  const Eigen::Vector3d angular = T_map_base.rotation().transpose() * omega_map;
  const auto & twist = fixture.odometries[0].twist.twist;
  EXPECT_NEAR(twist.linear.x, linear.x(), 1e-9);
  EXPECT_NEAR(twist.linear.y, linear.y(), 1e-9);
  EXPECT_NEAR(twist.linear.z, linear.z(), 1e-9);
  EXPECT_NEAR(twist.angular.x, angular.x(), 1e-9);
  EXPECT_NEAR(twist.angular.y, angular.y(), 1e-9);
  EXPECT_NEAR(twist.angular.z, angular.z(), 1e-9);
  // The pose is the base's, not the IMU's
  EXPECT_LT(pose_step(to_eigen(fixture.poses[0].pose.pose), T_map_base), 1e-9);
}

TEST(LocalizationPublisher, FrameWithNonFiniteAngularVelocityIsDropped) {
  Fixture fixture(LocalizationPublisherParams(), "publisher_nan_w");
  fixture.wait_for_discovery(1);
  fixture.feed(make_frame_at(100.0, make_pose(0, 0, 0), Eigen::Vector3d::Zero(),
    Eigen::Vector3d(std::nan(""), 0.0, 0.0)));
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.statuses.size() >= 1;}));
  fixture.spin_for(200ms);
  EXPECT_TRUE(fixture.poses.empty());
}

// N6: an older frame publishes nothing, unless an external reset came since.
TEST(LocalizationPublisher, OlderFrameIsDroppedUnlessAnExternalResetCameFirst) {
  Fixture fixture(LocalizationPublisherParams(), "publisher_stale");
  fixture.wait_for_discovery(1);
  for (int k = 0; k < 3; ++k) {
    fixture.feed(make_frame(k, 0.0));
  }
  fixture.feed(make_frame(1, 0.0));  // older
  fixture.feed(make_frame(2, 0.0));  // same stamp
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.statuses.size() >= 5;}));
  fixture.spin_for(200ms);
  EXPECT_EQ(fixture.poses.size(), 3u);
  EXPECT_EQ(fixture.count("odom", "base_link"), 3u);
  EXPECT_EQ(fixture.publisher->stale_frames(), 2u);

  fixture.reset(ResetReason::IMU_GAP);
  fixture.feed(make_frame(0, 0.0));
  fixture.spin_for(200ms);
  EXPECT_EQ(fixture.poses.size(), 3u) << "an IMU gap reset does not allow an older stamp";

  fixture.reset(ResetReason::EXTERNAL);
  fixture.feed(make_frame(0, 0.0));
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.poses.size() >= 4;}));
  EXPECT_EQ(key(fixture.poses[3].header.stamp), key(to_msg(kStart)));
}

// N4: a status that is not for a frame is stamped with the last frame stamp.
TEST(LocalizationPublisher, ResetStatusIsStampedWithTheLastFrameStamp) {
  Fixture fixture(LocalizationPublisherParams(), "publisher_reset_stamp");
  fixture.wait_for_discovery(1);
  fixture.feed(make_frame(2, 0.0));
  fixture.reset(ResetReason::EXTERNAL);
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.statuses.size() >= 2;}));
  EXPECT_EQ(key(fixture.statuses[1].header.stamp), key(to_msg(kStart + 2 * kDt)));
}

// N1: the stamp loses ns at epoch scale; an odom sample at exactly the frame stamp must still be
// found.
TEST(LocalizationPublisher, ExternalOdomFindsSamplesAtEpochScaleStamps) {
  LocalizationPublisherParams params;
  params.tf_mode = TfMode::EXTERNAL_ODOM;
  Fixture fixture(params, "publisher_epoch");
  auto fake_node = std::make_shared<rclcpp::Node>("fake_odom_epoch");
  fixture.executor_.add_node(fake_node);
  tf2_ros::TransformBroadcaster fake(fake_node);
  ASSERT_TRUE(fixture.spin_until([&] {return fake_node->count_subscribers("/tf") >= 2;}));
  fixture.wait_for_discovery(2);

  constexpr int kEpochFrames = 200;
  const double start = 1.7e9 + 0.123456789;
  auto send = [&](double stamp, double x) {
      geometry_msgs::msg::TransformStamped t;
      t.header.frame_id = "odom";
      t.child_frame_id = "base_link";
      t.header.stamp = to_msg(stamp);
      t.transform.translation.x = x;
      t.transform.rotation.w = 1.0;
      fake.sendTransform(t);
      const tf2::TimePoint tp(std::chrono::nanoseconds(key(t.header.stamp)));
      ASSERT_TRUE(fixture.spin_until([&] {
          return fixture.publisher->tf_buffer()->canTransform("odom", "base_link", tp);
        }));
    };
  // The sample before the first frame, so that no lookup needs extrapolation
  send(start - 0.1, -0.1);
  for (int k = 0; k < kEpochFrames; ++k) {
    const double stamp = start + k * 0.0731;
    send(stamp, 0.1 * k);
    fixture.feed(make_frame_at(stamp, make_pose(0.1 * k, 0.0, 0.0)));
  }
  ASSERT_TRUE(fixture.spin_until([&] {return fixture.count("map", "odom") >= kEpochFrames;}));
  EXPECT_EQ(fixture.publisher->tf_lookup_failures(), 0u);
  fixture.executor_.remove_node(fake_node);
}
