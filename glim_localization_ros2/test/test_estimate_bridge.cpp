// Copyright (c) 2026 atinfinity
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// One localization per process: EstimateBridge::connect() takes the process claim and
// disconnect() removes the callbacks from GLIM's static slots. No odometry thread, no bag: the test
// calls the slots itself, as the odometry thread would.
#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "glim/odometry/callbacks.hpp"
#include "glim/odometry/estimation_frame.hpp"
#include "glim_localization/util/process_claim.hpp"
#include "glim_localization_ros2/estimate_bridge.hpp"
#include "glim_localization_ros2/localization_publisher.hpp"

namespace
{

using glim_localization::ProcessClaim;
using glim_localization_ros2::EstimateBridge;
using glim_localization_ros2::LocalizationPublisher;
using glim_localization_ros2::LocalizationPublisherParams;

class EstimateBridgeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("estimate_bridge_test");
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
    // A failed test must not leave the claim to the next one
    ASSERT_EQ(ProcessClaim::holder(), "");
  }

  /// A bridge attached to a publisher, so that its frame counters move
  std::shared_ptr<EstimateBridge> make_attached_bridge()
  {
    auto bridge = std::make_shared<EstimateBridge>();
    const LocalizationPublisherParams params;
    bridge->attach(std::make_unique<LocalizationPublisher>(*node_, params), params);
    return bridge;
  }

  /// What the odometry thread does with a corrected frame. No IMU sample was given, so a bridge
  /// that receives it counts one frame without gyro.
  static void dispatch_frame(double stamp)
  {
    auto frame = std::make_shared<glim::EstimationFrame>();
    frame->stamp = stamp;
    frame->T_world_imu.setIdentity();
    frame->v_world_imu.setZero();
    frame->imu_bias.setZero();
    frame->frame_id = glim::FrameID::IMU;
    glim::OdometryEstimationCallbacks::on_update_new_frame(
      glim::EstimationFrame::ConstPtr(frame));
  }

  rclcpp::Node::SharedPtr node_;
};

}  // namespace

TEST_F(EstimateBridgeTest, SecondConnectThrowsAndNamesTheFirst) {
  auto first = make_attached_bridge();
  EstimateBridge::connect(first, "first bridge");
  EXPECT_TRUE(first->connected());

  auto second = make_attached_bridge();
  try {
    EstimateBridge::connect(second, "second bridge");
    FAIL() << "the second connect did not throw";
  } catch (const std::runtime_error & e) {
    EXPECT_STREQ(e.what(), "one localization per process: already held by first bridge");
  }
  EXPECT_FALSE(second->connected());

  // The refused bridge did not reach the slots
  dispatch_frame(1.0);
  EXPECT_EQ(first->frames_without_gyro(), 1u);
  EXPECT_EQ(second->frames_without_gyro(), 0u);

  // The same bridge twice is a programming error, and keeps its connection
  EXPECT_THROW(EstimateBridge::connect(first, "first bridge again"), std::logic_error);
  EXPECT_EQ(ProcessClaim::holder(), "first bridge");

  first->disconnect();
}

TEST_F(EstimateBridgeTest, DisconnectRemovesTheCallbacksAndAllowsANewConnect) {
  auto first = make_attached_bridge();
  EstimateBridge::connect(first, "first bridge");
  dispatch_frame(1.0);
  ASSERT_EQ(first->frames_without_gyro(), 1u);

  first->disconnect();
  first->disconnect();  // idempotent
  EXPECT_FALSE(first->connected());
  EXPECT_EQ(ProcessClaim::holder(), "");
  // The slots no longer reach the bridge, although it is still attached
  dispatch_frame(2.0);
  glim::OdometryEstimationCallbacks::on_insert_imu(2.0, Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero());
  EXPECT_TRUE(first->attached());
  EXPECT_EQ(first->frames_without_gyro(), 1u);

  // An unload followed by a load: a new bridge connects and receives the callbacks
  auto second = make_attached_bridge();
  EXPECT_NO_THROW(EstimateBridge::connect(second, "second bridge"));
  EXPECT_EQ(ProcessClaim::holder(), "second bridge");
  dispatch_frame(3.0);
  EXPECT_EQ(second->frames_without_gyro(), 1u);
  EXPECT_EQ(first->frames_without_gyro(), 1u);
  second->disconnect();
}
