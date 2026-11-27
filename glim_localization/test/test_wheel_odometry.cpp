#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/util/config.hpp>

#include <glim_localization/map/map_manager.hpp>
#include <glim_localization/odometry/odometry_estimation_localization_cpu.hpp>
#include <glim_localization/odometry/wheel_odometry.hpp>

#include "synthetic/synthetic_map.hpp"
#include "synthetic/synthetic_scene.hpp"
#include "synthetic/synthetic_sensor.hpp"

namespace fs = std::filesystem;
using namespace glim_localization;
using namespace glim_localization::test;
using gtsam::symbol_shorthand::X;

namespace {

using WheelSamples = std::vector<WheelOdometrySample, Eigen::aligned_allocator<WheelOdometrySample>>;

constexpr double kEndTime = 12.0;
constexpr double kWheelRate = 50.0;

Eigen::Isometry3d make_pose(const Eigen::Vector3d& t, double yaw, double pitch = 0.0) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())).toRotationMatrix();
  T.translation() = t;
  return T;
}

/// A wheel base behind and below the IMU, slightly turned, so that the extrinsic matters
Eigen::Isometry3d test_T_imu_base() {
  return make_pose(Eigen::Vector3d(-0.3, 0.1, -0.5), 0.2);
}

/// The exact twist of the base frame along the synthetic trajectory, in the base frame. `scale(t)` multiplies the
/// linear velocity (1: an ideal wheel; 0: stuck; above 1: spinning).
WheelSamples make_wheel_samples(
  double t_begin,
  double t_end,
  const Eigen::Isometry3d& T_imu_base,
  const std::function<double(double)>& scale = [](double) { return 1.0; }) {
  SyntheticWheelParams params;
  params.rate = kWheelRate;
  params.T_imu_base = T_imu_base;
  WheelSamples samples;
  for (const auto& s : make_synthetic_wheel(t_begin, t_end, params)) {
    WheelOdometrySample sample;
    sample.stamp = s.stamp;
    sample.angular = s.angular;
    sample.linear = s.linear * scale(s.stamp);
    samples.push_back(sample);
  }
  return samples;
}

// ---------------------------------------------------------------------------------------------------------------------
// WheelOdometryBuffer and create_wheel_factor

TEST(WheelOdometryBufferTest, IntegratesConstantTwistExactly) {
  WheelOdometryParams params;
  WheelOdometryBuffer buffer(params);
  const Eigen::Vector3d v(1.5, 0.0, 0.0);
  const Eigen::Vector3d w(0.0, 0.0, 0.4);
  for (int i = 0; i <= 50; i++) {
    ASSERT_TRUE(buffer.insert(0.02 * i, v, w));
  }
  const auto T = buffer.integrate(0.13, 0.87);
  ASSERT_TRUE(T);
  // A circular arc of radius v / w
  const double dt = 0.74;
  const double angle = w.z() * dt;
  const double radius = v.x() / w.z();
  EXPECT_NEAR(T->translation().x(), radius * std::sin(angle), 1e-9);
  EXPECT_NEAR(T->translation().y(), radius * (1.0 - std::cos(angle)), 1e-9);
  EXPECT_NEAR(T->translation().z(), 0.0, 1e-12);
  EXPECT_NEAR(Eigen::AngleAxisd(T->linear()).angle(), angle, 1e-9);
}

TEST(WheelOdometryBufferTest, InterpolatesLinearlyBetweenSamples) {
  WheelOdometryBuffer buffer;
  ASSERT_TRUE(buffer.insert(0.0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero()));
  ASSERT_TRUE(buffer.insert(0.1, Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::Zero()));
  // Speed ramps from 0 to 1 m/s: 0.05 m
  const auto T = buffer.integrate(0.0, 0.1);
  ASSERT_TRUE(T);
  EXPECT_NEAR(T->translation().x(), 0.05, 1e-12);
  // Held constant past the last sample (within max_gap)
  const auto T_ext = buffer.integrate(0.1, 0.2);
  ASSERT_TRUE(T_ext);
  EXPECT_NEAR(T_ext->translation().x(), 0.1, 1e-12);
}

TEST(WheelOdometryBufferTest, RequiresCoverage) {
  WheelOdometryParams params;
  params.max_gap = 0.1;
  WheelOdometryBuffer buffer(params);
  EXPECT_FALSE(buffer.integrate(0.0, 0.1));  // Empty
  ASSERT_TRUE(buffer.insert(1.0, Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero()));
  ASSERT_TRUE(buffer.insert(1.05, Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero()));
  ASSERT_TRUE(buffer.insert(1.4, Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero()));  // A 0.35 s hole
  ASSERT_TRUE(buffer.insert(1.45, Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero()));

  EXPECT_FALSE(buffer.integrate(0.5, 1.05));   // Starts long before the first sample
  EXPECT_TRUE(buffer.integrate(0.95, 1.05));   // Within max_gap of the first sample
  EXPECT_FALSE(buffer.integrate(1.0, 1.42));   // Spans the hole
  EXPECT_TRUE(buffer.integrate(1.4, 1.5));     // Within max_gap of the last sample
  EXPECT_FALSE(buffer.integrate(1.45, 1.7));   // Ends long after the last sample
  EXPECT_FALSE(buffer.integrate(1.05, 1.05));  // Empty interval
}

TEST(WheelOdometryBufferTest, RejectsBadSamplesAndTrims) {
  WheelOdometryBuffer buffer;
  EXPECT_TRUE(buffer.insert(1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  EXPECT_FALSE(buffer.insert(1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));  // Not newer
  EXPECT_FALSE(buffer.insert(0.5, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  EXPECT_FALSE(buffer.insert(std::nan(""), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  EXPECT_FALSE(buffer.insert(2.0, Eigen::Vector3d::Constant(std::nan("")), Eigen::Vector3d::Zero()));
  for (int i = 1; i <= 10; i++) {
    EXPECT_TRUE(buffer.insert(1.0 + 0.1 * i, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  }
  EXPECT_EQ(buffer.size(), 11u);
  buffer.trim(1.55);  // Keeps the sample at 1.5 and later ones
  EXPECT_EQ(buffer.size(), 6u);
  EXPECT_TRUE(buffer.integrate(1.55, 1.9));

  // Bounded without any trim
  WheelOdometryBuffer unbounded;
  for (size_t i = 0; i < WheelOdometryBuffer::kMaxSamples + 100; i++) {
    unbounded.insert(0.01 * i, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  }
  EXPECT_EQ(unbounded.size(), WheelOdometryBuffer::kMaxSamples);
}

// A bag loop or a simulation clock reset: a few stray samples are dropped, a run of them restarts the buffer
TEST(WheelOdometryBufferTest, RestartsWhenTheClockGoesBack) {
  WheelOdometryBuffer buffer;
  for (int i = 0; i <= 50; i++) {
    ASSERT_TRUE(buffer.insert(10.0 + 0.02 * i, Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero()));
  }
  // Fewer than kRestartRejects stray samples are dropped, and a newer sample clears the count
  for (int i = 0; i + 1 < WheelOdometryBuffer::kRestartRejects; i++) {
    EXPECT_FALSE(buffer.insert(1.0 + 0.02 * i, Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero()));
  }
  EXPECT_TRUE(buffer.insert(11.02, Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero()));
  EXPECT_EQ(buffer.size(), 52u);
  for (int i = 0; i + 1 < WheelOdometryBuffer::kRestartRejects; i++) {
    EXPECT_FALSE(buffer.insert(1.0 + 0.02 * i, Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero()));
  }
  EXPECT_EQ(buffer.size(), 52u);

  // The next one restarts the buffer on the new clock
  EXPECT_TRUE(buffer.insert(1.1, Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero()));
  EXPECT_EQ(buffer.size(), 1u);
  for (int i = 1; i <= 10; i++) {
    EXPECT_TRUE(buffer.insert(1.1 + 0.02 * i, Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero()));
  }
  const auto T = buffer.integrate(1.1, 1.3);
  ASSERT_TRUE(T);
  EXPECT_NEAR(T->translation().x(), 0.2, 1e-9);
  EXPECT_FALSE(buffer.integrate(10.5, 10.7));  // The old samples are gone

  buffer.clear();
  EXPECT_EQ(buffer.size(), 0u);
  EXPECT_TRUE(buffer.insert(0.5, Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero()));  // Any stamp after a clear
}

TEST(WheelOdometryBufferTest, NonholonomicDropsLateralAndVerticalVelocity) {
  WheelOdometryParams params;
  params.nonholonomic = true;
  WheelOdometryBuffer buffer(params);
  ASSERT_TRUE(buffer.insert(0.0, Eigen::Vector3d(1.0, 0.3, -0.2), Eigen::Vector3d::Zero()));
  ASSERT_TRUE(buffer.insert(0.1, Eigen::Vector3d(1.0, 0.3, -0.2), Eigen::Vector3d::Zero()));
  const auto T = buffer.integrate(0.0, 0.1);
  ASSERT_TRUE(T);
  EXPECT_NEAR(T->translation().x(), 0.1, 1e-12);
  EXPECT_NEAR(T->translation().y(), 0.0, 1e-12);
  EXPECT_NEAR(T->translation().z(), 0.0, 1e-12);
}

TEST(WheelOdometryBufferTest, TwistAtInterpolatesWithinCoverage) {
  WheelOdometryParams params;
  params.max_gap = 0.1;
  WheelOdometryBuffer buffer(params);
  EXPECT_FALSE(buffer.twist_at(1.0));  // Empty
  ASSERT_TRUE(buffer.insert(1.0, Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.2)));
  ASSERT_TRUE(buffer.insert(1.1, Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.4)));
  ASSERT_TRUE(buffer.insert(1.5, Eigen::Vector3d(3.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.6)));  // A 0.4 s hole

  const auto mid = buffer.twist_at(1.025);
  ASSERT_TRUE(mid);
  EXPECT_DOUBLE_EQ(mid->stamp, 1.025);
  EXPECT_NEAR(mid->linear.x(), 1.25, 1e-12);
  EXPECT_NEAR(mid->angular.z(), 0.25, 1e-12);
  const auto at_sample = buffer.twist_at(1.1);
  ASSERT_TRUE(at_sample);
  EXPECT_NEAR(at_sample->linear.x(), 2.0, 1e-12);

  // Held within max_gap past the ends, empty further out and inside the hole
  const auto before = buffer.twist_at(0.95);
  ASSERT_TRUE(before);
  EXPECT_NEAR(before->linear.x(), 1.0, 1e-12);
  const auto after = buffer.twist_at(1.58);
  ASSERT_TRUE(after);
  EXPECT_NEAR(after->linear.x(), 3.0, 1e-12);
  EXPECT_FALSE(buffer.twist_at(0.85));
  EXPECT_FALSE(buffer.twist_at(1.65));
  EXPECT_FALSE(buffer.twist_at(1.3));
  EXPECT_FALSE(buffer.twist_at(std::nan("")));
}

// The IMU velocity of a base twist, in closed form with a lever arm and a turned base, and against the differentiated
// wheel motion that the factor uses (integrate and T_imu_base * T_base * T_imu_base^-1)
TEST(WheelOdometryBufferTest, VelocityInImuAppliesTheLeverArm) {
  WheelOdometryParams params;
  // The base 1 m ahead of the IMU, turned 90 deg: its forward axis is the IMU's y axis, and the IMU is 1 m to the left
  // of the base origin (p = (0, 1, 0) in the base frame)
  params.T_imu_base = make_pose(Eigen::Vector3d(1.0, 0.0, 0.0), M_PI / 2.0);
  WheelOdometrySample twist;
  twist.linear = Eigen::Vector3d(2.0, 0.0, 0.0);
  twist.angular = Eigen::Vector3d(0.0, 0.0, 0.5);
  // Turning left, a point 1 m to the left moves 0.5 m/s slower, along the base forward axis
  EXPECT_TRUE(wheel_velocity_in_imu(params, twist).isApprox(Eigen::Vector3d(0.0, 1.5, 0.0), 1e-12));
  twist.angular.setZero();
  EXPECT_TRUE(wheel_velocity_in_imu(params, twist).isApprox(Eigen::Vector3d(0.0, 2.0, 0.0), 1e-12));
  // Turning in place, the IMU moves on a circle around the base origin: w x p = (-0.5, 0, 0) in the base frame
  twist.linear.setZero();
  twist.angular = Eigen::Vector3d(0.0, 0.0, 0.5);
  EXPECT_TRUE(wheel_velocity_in_imu(params, twist).isApprox(Eigen::Vector3d(0.0, -0.5, 0.0), 1e-12));

  // A rolled, pitched and turned base with a 3D twist: the velocity is the derivative of the IMU motion
  params.T_imu_base = test_T_imu_base();
  params.T_imu_base.linear() = (Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()) * params.T_imu_base.linear()).eval();
  twist.linear = Eigen::Vector3d(1.2, -0.1, 0.05);
  twist.angular = Eigen::Vector3d(0.1, -0.05, 0.7);
  WheelOdometryBuffer buffer(params);
  ASSERT_TRUE(buffer.insert(0.0, twist.linear, twist.angular));
  ASSERT_TRUE(buffer.insert(0.1, twist.linear, twist.angular));
  constexpr double kDt = 1e-6;
  const auto T_base = buffer.integrate(0.0, kDt);
  ASSERT_TRUE(T_base);
  const Eigen::Isometry3d T_imu = params.T_imu_base * *T_base * params.T_imu_base.inverse();
  EXPECT_TRUE(wheel_velocity_in_imu(params, twist).isApprox(T_imu.translation() / kDt, 1e-5));
}

// The measured IMU motion against closed forms, with a base that is rolled, pitched and turned on the IMU
TEST(WheelFactorTest, AppliesTheExtrinsic) {
  WheelOdometryParams params;
  const Eigen::Matrix3d R_imu_base =
    (Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-0.25, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
  const Eigen::Vector3d p(-0.3, 0.1, -0.5);  // The base origin in the IMU frame
  params.T_imu_base.linear() = R_imu_base;
  params.T_imu_base.translation() = p;
  params.gate_velocity = 0.0;
  params.gate_velocity_ratio = 0.0;
  params.gate_angular_velocity = 0.0;

  const auto measured = [&](const Eigen::Isometry3d& T_base) {
    WheelFactorResult result = WheelFactorResult::NO_DATA;
    const auto graph = create_wheel_factor(params, X(3), X(4), 0.1, T_base, Eigen::Isometry3d::Identity(), result);
    EXPECT_EQ(result, WheelFactorResult::ADDED);
    EXPECT_EQ(graph.size(), 1u);
    const auto factor = std::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(graph[0]);
    EXPECT_TRUE(factor);
    EXPECT_EQ(factor->key1(), X(3));
    EXPECT_EQ(factor->key2(), X(4));
    return factor->measured();
  };

  // Turning in place by theta about the base z axis: the IMU turns by theta about the same axis, a = R_imu_base * z,
  // and moves on a circle around the base origin. Its displacement in its own start frame is (I - R_a(theta)) p.
  const double theta = 0.3;
  const Eigen::Vector3d axis = R_imu_base.col(2);
  const Eigen::Matrix3d R_turn = Eigen::AngleAxisd(theta, axis).toRotationMatrix();
  Eigen::Isometry3d spin = Eigen::Isometry3d::Identity();
  spin.linear() = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const gtsam::Pose3 turn = measured(spin);
  EXPECT_TRUE(turn.rotation().matrix().isApprox(R_turn, 1e-9));
  EXPECT_TRUE(turn.translation().isApprox((Eigen::Matrix3d::Identity() - R_turn) * p, 1e-9));
  // The chord of that circle: 2 sin(theta / 2) times the distance of the IMU from the axis
  const Eigen::Vector3d lever = p - axis * axis.dot(p);
  EXPECT_NEAR(turn.translation().norm(), 2.0 * std::sin(theta / 2.0) * lever.norm(), 1e-9);
  EXPECT_GT(turn.translation().norm(), 0.05);

  // Driving straight by d along the base x axis: the IMU moves by d along R_imu_base * x, without turning
  const double d = 0.25;
  Eigen::Isometry3d straight = Eigen::Isometry3d::Identity();
  straight.translation() = Eigen::Vector3d(d, 0.0, 0.0);
  const gtsam::Pose3 drive = measured(straight);
  EXPECT_TRUE(drive.rotation().matrix().isApprox(Eigen::Matrix3d::Identity(), 1e-12));
  EXPECT_TRUE(drive.translation().isApprox(d * R_imu_base.col(0), 1e-12));
  EXPECT_GT(std::abs(drive.translation().z()), 0.05);  // The pitched base drives out of the IMU's horizontal plane
}

TEST(WheelFactorTest, GatesMotionThatDisagreesWithTheImu) {
  WheelOdometryParams params;
  const Eigen::Isometry3d imu = make_pose(Eigen::Vector3d(0.2, 0.0, 0.0), 0.01);  // 2 m/s for 0.1 s
  WheelFactorResult result = WheelFactorResult::NO_DATA;

  // Close to the IMU: added
  EXPECT_EQ(create_wheel_factor(params, X(0), X(1), 0.1, make_pose(Eigen::Vector3d(0.19, 0.0, 0.0), 0.012), imu, result).size(), 1u);
  EXPECT_EQ(result, WheelFactorResult::ADDED);
  // A stuck wheel (2 m/s off)
  EXPECT_TRUE(create_wheel_factor(params, X(0), X(1), 0.1, Eigen::Isometry3d::Identity(), imu, result).empty());
  EXPECT_EQ(result, WheelFactorResult::GATED);
  // A turn the gyro did not see (1 rad/s off)
  EXPECT_TRUE(create_wheel_factor(params, X(0), X(1), 0.1, make_pose(Eigen::Vector3d(0.2, 0.0, 0.0), 0.11), imu, result).empty());
  EXPECT_EQ(result, WheelFactorResult::GATED);
  // The right speed in a wrong direction (the gate takes the vector difference, not the speed difference)
  EXPECT_TRUE(create_wheel_factor(params, X(0), X(1), 0.1, make_pose(Eigen::Vector3d(0.0, 0.2, 0.0), 0.01), imu, result).empty());
  EXPECT_EQ(result, WheelFactorResult::GATED);
  // Re-admission adds it anyway
  EXPECT_EQ(create_wheel_factor(params, X(0), X(1), 0.1, Eigen::Isometry3d::Identity(), imu, result, false).size(), 1u);
  EXPECT_EQ(result, WheelFactorResult::ADDED);
  // Gate disabled
  params.gate_velocity = 0.0;
  params.gate_velocity_ratio = 0.0;
  params.gate_angular_velocity = 0.0;
  EXPECT_EQ(create_wheel_factor(params, X(0), X(1), 0.1, Eigen::Isometry3d::Identity(), imu, result).size(), 1u);
  EXPECT_EQ(result, WheelFactorResult::ADDED);
}

// The gate is max(gate_velocity, gate_velocity_ratio * speed): a stuck wheel is caught at a crawl, and at speed a
// small relative error passes while a gross one is still caught
TEST(WheelFactorTest, GateScalesWithSpeed) {
  WheelOdometryParams params;
  const auto check = [&](double imu_speed, double wheel_speed) {
    WheelFactorResult result = WheelFactorResult::NO_DATA;
    create_wheel_factor(
      params, X(0), X(1), 0.1, make_pose(Eigen::Vector3d(0.1 * wheel_speed, 0.0, 0.0), 0.0), make_pose(Eigen::Vector3d(0.1 * imu_speed, 0.0, 0.0), 0.0), result);
    return result;
  };
  // A slow robot (0.8 m/s) with a stuck wheel: zero motion would be the tightest factor of all
  EXPECT_EQ(check(0.8, 0.0), WheelFactorResult::GATED);
  EXPECT_EQ(check(0.4, 0.0), WheelFactorResult::GATED);
  EXPECT_EQ(check(0.0, 0.8), WheelFactorResult::GATED);  // Or spinning wheels on a robot that stands still
  EXPECT_EQ(check(0.8, 0.75), WheelFactorResult::ADDED);
  EXPECT_EQ(check(0.2, 0.0), WheelFactorResult::ADDED);  // Within the absolute floor
  // At 10 m/s the gate is 3 m/s
  EXPECT_EQ(check(10.0, 8.0), WheelFactorResult::ADDED);
  EXPECT_EQ(check(10.0, 6.5), WheelFactorResult::GATED);
  EXPECT_EQ(check(10.0, 0.0), WheelFactorResult::GATED);
  // Without the ratio term the small floor gates a modest relative error at speed
  params.gate_velocity_ratio = 0.0;
  EXPECT_EQ(check(10.0, 8.0), WheelFactorResult::GATED);
}

TEST(WheelFactorTest, ValidatesParams) {
  WheelOdometryParams params;
  EXPECT_NO_THROW(params.validate());
  auto bad = params;
  bad.max_gap = 0.0;
  EXPECT_THROW(bad.validate(), std::invalid_argument);
  bad = params;
  bad.translation_sigma_min = 0.0;
  EXPECT_THROW(bad.validate(), std::invalid_argument);
  bad = params;
  bad.rotation_sigma_ratio = -1.0;
  EXPECT_THROW(bad.validate(), std::invalid_argument);
  bad = params;
  bad.gate_velocity = std::nan("");
  EXPECT_THROW(bad.validate(), std::invalid_argument);
  bad = params;
  bad.gate_velocity_ratio = -0.1;
  EXPECT_THROW(bad.validate(), std::invalid_argument);
  bad = params;
  bad.gate_readmit_frames = -1;
  EXPECT_THROW(bad.validate(), std::invalid_argument);
  bad = params;
  bad.T_imu_base.linear() *= 2.0;
  EXPECT_THROW(bad.validate(), std::invalid_argument);
}

}  // namespace

// ---------------------------------------------------------------------------------------------------------------------
// Odometry with wheel input on synthetic data

class WheelOdometryTest : public testing::Test {
protected:
  static void SetUpTestSuite() {
    glim::GlobalConfig::instance(GLIM_LOCALIZATION_CONFIG_DIR, true);
    dir_ = new fs::path(fs::temp_directory_path() / ("glim_loc_wheel_" + std::to_string(::getpid())));
    fs::remove_all(*dir_);

    SyntheticMapParams map_params;
    map_params.num_submaps = 5;
    map_params.cross_walls = true;
    write_synthetic_dump((*dir_ / "map").string(), map_params);
    auto map = std::make_shared<MapManager>();
    ASSERT_TRUE(map->load((*dir_ / "map").string()));
    map_ = new std::shared_ptr<const MapManager>(map);

    // The same map 1 km away: no scan matches it, so the LiDAR gives no constraint (as with a dropped LiDAR or a scene
    // without usable structure) and only the IMU and the wheels observe the motion
    SyntheticMapParams far_params = map_params;
    far_params.world_offset = Eigen::Vector3d(1000.0, 0.0, 0.0);
    write_synthetic_dump((*dir_ / "far").string(), far_params);
    auto far_map = std::make_shared<MapManager>();
    ASSERT_TRUE(far_map->load((*dir_ / "far").string()));
    far_map_ = new std::shared_ptr<const MapManager>(far_map);

    glim::CloudPreprocessorParams preprocess_params;
    preprocess_params.global_shutter = true;
    glim::CloudPreprocessor preprocessor(preprocess_params);
    frames_ = new std::vector<glim::PreprocessedFrame::Ptr>();
    for (const auto& scan : make_synthetic_scans(make_scene_boxes(map_params), 0.0, kEndTime)) {
      frames_->push_back(preprocessor.preprocess(scan));
    }
    imu_ = new std::vector<SyntheticImuSample>(make_synthetic_imu(0.0, kEndTime + 0.05));
  }

  static void TearDownTestSuite() {
    delete imu_;
    delete frames_;
    delete far_map_;
    delete map_;
    fs::remove_all(*dir_);
    delete dir_;
  }

  static OdometryEstimationLocalizationCPU::Params make_params(int num_threads = 2) {
    OdometryEstimationLocalizationCPU::Params params;
    params.estimate_init_state = false;
    params.init_T_world_imu = synthetic_T_world_imu(0.0);
    params.init_v_world_imu.setZero();
    params.num_threads = num_threads;
    params.map_factors.num_threads = num_threads;
    return params;
  }

  struct RunResult {
    std::vector<glim::EstimationFrame::ConstPtr> estimates;
    LocalizationStatus status;
    double max_error = 0.0;      ///< Translation [m]
    double final_error = 0.0;    ///< Translation of the last frame [m]
    int rejected_wheel = 0;      ///< insert_wheel_odom() returned false
  };

  static RunResult run(
    const OdometryEstimationLocalizationCPU::Params& params,
    const std::shared_ptr<const MapManager>& map,
    size_t num_frames,
    const WheelSamples* wheel,
    size_t first = 0) {
    OdometryEstimationLocalizationCPU odometry(params, map);
    RunResult result;
    feed(odometry, first, num_frames, wheel, result);
    return result;
  }

  /// Feed frames [first, last) with the IMU and wheel samples from 0.1 s before frames[first] up to 0.02 s after each
  /// frame, and add the estimates and errors to result. Returns false if a frame got no estimate.
  static bool feed(OdometryEstimationLocalizationCPU& odometry, size_t first, size_t last, const WheelSamples* wheel, RunResult& result) {
    const double begin = (*frames_)[first]->stamp - 0.1;
    size_t imu_cursor = 0;
    while (imu_cursor < imu_->size() && (*imu_)[imu_cursor].stamp < begin) {
      imu_cursor++;
    }
    size_t wheel_cursor = 0;
    while (wheel && wheel_cursor < wheel->size() && (*wheel)[wheel_cursor].stamp < begin) {
      wheel_cursor++;
    }
    bool ok = true;
    std::vector<glim::EstimationFrame::ConstPtr> marginalized;
    for (size_t i = first; i < std::min(last, frames_->size()); i++) {
      const auto& frame = (*frames_)[i];
      while (imu_cursor < imu_->size() && (*imu_)[imu_cursor].stamp <= frame->stamp + 0.02) {
        const auto& sample = (*imu_)[imu_cursor++];
        odometry.insert_imu(sample.stamp, sample.linear_acc, sample.angular_vel);
      }
      while (wheel && wheel_cursor < wheel->size() && (*wheel)[wheel_cursor].stamp <= frame->stamp + 0.02) {
        const auto& sample = (*wheel)[wheel_cursor++];
        result.rejected_wheel += odometry.insert_wheel_odom(sample.stamp, sample.linear, sample.angular) ? 0 : 1;
      }
      const auto estimate = odometry.insert_frame(frame, marginalized);
      EXPECT_TRUE(estimate) << "t=" << frame->stamp;
      if (!estimate) {
        ok = false;
        break;
      }
      EXPECT_TRUE(estimate->T_world_imu.matrix().allFinite()) << "t=" << frame->stamp;
      result.estimates.push_back(estimate);
      const Eigen::Isometry3d error = synthetic_T_world_imu(estimate->stamp).inverse() * estimate->T_world_imu;
      result.max_error = std::max(result.max_error, error.translation().norm());
      result.final_error = error.translation().norm();
    }
    result.status = odometry.status();
    return ok;
  }

  static fs::path* dir_;
  static std::shared_ptr<const MapManager>* map_;
  static std::shared_ptr<const MapManager>* far_map_;
  static std::vector<glim::PreprocessedFrame::Ptr>* frames_;
  static std::vector<SyntheticImuSample>* imu_;
};

fs::path* WheelOdometryTest::dir_ = nullptr;
std::shared_ptr<const MapManager>* WheelOdometryTest::map_ = nullptr;
std::shared_ptr<const MapManager>* WheelOdometryTest::far_map_ = nullptr;
std::vector<glim::PreprocessedFrame::Ptr>* WheelOdometryTest::frames_ = nullptr;
std::vector<SyntheticImuSample>* WheelOdometryTest::imu_ = nullptr;

TEST_F(WheelOdometryTest, DefaultConfigKeepsWheelOdometryOff) {
  const auto params = make_params();
  EXPECT_FALSE(params.wheel_odom.enabled);
  EXPECT_TRUE(params.wheel_odom.T_imu_base.isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_DOUBLE_EQ(params.wheel_odom.max_gap, 0.2);
  EXPECT_DOUBLE_EQ(params.wheel_odom.gate_velocity, 0.3);
  EXPECT_DOUBLE_EQ(params.wheel_odom.gate_velocity_ratio, 0.3);
  EXPECT_EQ(params.wheel_odom.gate_readmit_frames, 20);
}

TEST_F(WheelOdometryTest, RejectsInvalidParamsOnlyWhenEnabled) {
  auto params = make_params();
  params.wheel_odom.max_gap = -1.0;
  EXPECT_NO_THROW(OdometryEstimationLocalizationCPU(params, *map_));  // Disabled: not used, not checked
  params.wheel_odom.enabled = true;
  EXPECT_THROW(OdometryEstimationLocalizationCPU(params, *map_), std::invalid_argument);
}

// Disabled (the default), the estimates are bit-identical to a run without any wheel input, also when samples are
// given; enabled without samples, too. One thread, so that the reference run is deterministic.
TEST_F(WheelOdometryTest, OffOrWithoutSamplesIsIdentical) {
  constexpr size_t kFrames = 50;
  const WheelSamples wheel = make_wheel_samples(0.0, kEndTime, test_T_imu_base());
  const auto params = make_params(1);
  auto enabled = params;
  enabled.wheel_odom.enabled = true;
  enabled.wheel_odom.T_imu_base = test_T_imu_base();

  const RunResult reference = run(params, *map_, kFrames, nullptr);
  const RunResult disabled_with_samples = run(params, *map_, kFrames, &wheel);
  const RunResult enabled_without_samples = run(enabled, *map_, kFrames, nullptr);

  ASSERT_EQ(reference.estimates.size(), kFrames);
  for (const auto* other : {&disabled_with_samples, &enabled_without_samples}) {
    ASSERT_EQ(other->estimates.size(), reference.estimates.size());
    for (size_t i = 0; i < reference.estimates.size(); i++) {
      EXPECT_EQ(other->estimates[i]->T_world_imu.matrix(), reference.estimates[i]->T_world_imu.matrix()) << "i=" << i;
      EXPECT_EQ(other->estimates[i]->v_world_imu, reference.estimates[i]->v_world_imu) << "i=" << i;
      EXPECT_EQ(other->estimates[i]->imu_bias, reference.estimates[i]->imu_bias) << "i=" << i;
    }
    EXPECT_EQ(other->status.wheel_factors, 0);
    EXPECT_EQ(other->status.wheel_gated, 0);
  }
  const auto fed = std::count_if(wheel.begin(), wheel.end(), [&](const auto& s) { return s.stamp <= reference.estimates.back()->stamp + 0.02; });
  EXPECT_EQ(disabled_with_samples.rejected_wheel, static_cast<int>(fed));
  EXPECT_EQ(reference.status.wheel_missing, 0);
  EXPECT_EQ(enabled_without_samples.status.wheel_missing, static_cast<int>(kFrames) - 1);
}

// With the map, wheel input keeps the accuracy of the map-only estimate.
TEST_F(WheelOdometryTest, KeepsAccuracyWithTheMap) {
  const WheelSamples wheel = make_wheel_samples(0.0, kEndTime + 0.05, test_T_imu_base());
  const auto params = make_params();
  auto with_wheel = params;
  with_wheel.wheel_odom.enabled = true;
  with_wheel.wheel_odom.T_imu_base = test_T_imu_base();

  const RunResult without = run(params, *map_, frames_->size(), nullptr);
  const RunResult with = run(with_wheel, *map_, frames_->size(), &wheel);
  std::cout << "with the map: without wheel max_error=" << without.max_error << " m; with wheel max_error=" << with.max_error
            << " m, wheel_factors=" << with.status.wheel_factors << ", gated=" << with.status.wheel_gated << std::endl;

  ASSERT_EQ(with.estimates.size(), frames_->size());
  EXPECT_EQ(with.status.wheel_factors, static_cast<int>(frames_->size()) - 1);
  EXPECT_EQ(with.status.wheel_gated, 0);
  EXPECT_EQ(with.status.wheel_missing, 0);
  EXPECT_LT(with.max_error, 0.1);
}

// Without any LiDAR constraint (no scan matches the map), the IMU alone drifts. Wheel odometry measures the motion and
// bounds the drift.
TEST_F(WheelOdometryTest, BoundsDriftWithoutLidarConstraints) {
  const WheelSamples wheel = make_wheel_samples(0.0, kEndTime + 0.05, test_T_imu_base());
  const auto params = make_params();
  auto with_wheel = params;
  with_wheel.wheel_odom.enabled = true;
  with_wheel.wheel_odom.T_imu_base = test_T_imu_base();

  const RunResult without = run(params, *far_map_, frames_->size(), nullptr);
  const RunResult with = run(with_wheel, *far_map_, frames_->size(), &wheel);
  std::cout << "no LiDAR constraint: IMU only max_error=" << without.max_error << " m, final=" << without.final_error
            << " m; with wheel max_error=" << with.max_error << " m, final=" << with.final_error << " m, wheel_factors=" << with.status.wheel_factors
            << ", gated=" << with.status.wheel_gated << ", missing=" << with.status.wheel_missing << std::endl;

  ASSERT_EQ(with.estimates.size(), frames_->size());
  EXPECT_EQ(with.status.num_map_factors, 0);
  EXPECT_EQ(with.status.wheel_factors, static_cast<int>(frames_->size()) - 1);
  EXPECT_EQ(with.status.wheel_gated, 0);
  EXPECT_EQ(with.status.wheel_missing, 0);
  EXPECT_LT(with.max_error, 0.4);
  EXPECT_LT(with.max_error, 0.2 * without.max_error);
}

// Without LiDAR constraints, a stuck wheel for 1.5 s while the robot drives at 2 m/s, then a spinning wheel (1.8x) for
// 1.5 s: the consistency gate rejects both, so the slip does not pull the estimate.
TEST_F(WheelOdometryTest, GateRejectsSlip) {
  const auto slip = [](double t) {
    if (t >= 4.0 && t < 5.5) {
      return 0.0;
    }
    if (t >= 7.0 && t < 8.5) {
      return 1.8;
    }
    return 1.0;
  };
  const WheelSamples ideal = make_wheel_samples(0.0, kEndTime + 0.05, test_T_imu_base());
  const WheelSamples slipping = make_wheel_samples(0.0, kEndTime + 0.05, test_T_imu_base(), slip);
  auto params = make_params();
  params.wheel_odom.enabled = true;
  params.wheel_odom.T_imu_base = test_T_imu_base();
  auto ungated = params;
  ungated.wheel_odom.gate_velocity = 0.0;
  ungated.wheel_odom.gate_velocity_ratio = 0.0;
  ungated.wheel_odom.gate_angular_velocity = 0.0;

  const RunResult reference = run(params, *far_map_, frames_->size(), &ideal);
  const RunResult gated = run(params, *far_map_, frames_->size(), &slipping);
  const RunResult trusting = run(ungated, *far_map_, frames_->size(), &slipping);
  std::cout << "slip: ideal wheel max_error=" << reference.max_error << " m; gated max_error=" << gated.max_error
            << " m, gated frames=" << gated.status.wheel_gated << "; without the gate max_error=" << trusting.max_error << " m" << std::endl;

  ASSERT_EQ(gated.estimates.size(), frames_->size());
  // About 15 frames per slip episode
  EXPECT_GE(gated.status.wheel_gated, 20);
  EXPECT_LE(gated.status.wheel_gated, 40);
  EXPECT_EQ(trusting.status.wheel_gated, 0);
  EXPECT_EQ(gated.status.wheel_readmitted, 0);  // Each episode is shorter than gate_readmit_frames
  EXPECT_LT(gated.max_error, reference.max_error + 0.2);
  EXPECT_LT(gated.max_error, 0.5 * trusting.max_error);
}

// A bag loop: the clock goes back and the odometry is reset there. The wheel samples of the old run are newer than the
// new ones; kept, they would make the buffer drop every new sample until the clock passed them again.
TEST_F(WheelOdometryTest, RecoversFromABagLoop) {
  const WheelSamples wheel = make_wheel_samples(0.0, kEndTime + 0.05, test_T_imu_base());
  auto params = make_params();
  params.wheel_odom.enabled = true;
  params.wheel_odom.T_imu_base = test_T_imu_base();

  OdometryEstimationLocalizationCPU odometry(params, *map_);
  RunResult first_run;
  ASSERT_TRUE(feed(odometry, 0, 80, &wheel, first_run));
  EXPECT_EQ(first_run.status.wheel_factors, 79);

  constexpr size_t kLoopFrom = 10;
  const double stamp = (*frames_)[kLoopFrom]->stamp;
  ASSERT_TRUE(odometry.reset(synthetic_T_world_imu(stamp), synthetic_v_world_imu(stamp), stamp));
  RunResult second_run;
  ASSERT_TRUE(feed(odometry, kLoopFrom, 60, &wheel, second_run));
  std::cout << "bag loop: wheel_factors=" << second_run.status.wheel_factors << ", missing=" << second_run.status.wheel_missing
            << ", gated=" << second_run.status.wheel_gated << ", dropped samples=" << second_run.rejected_wheel << ", max_error=" << second_run.max_error
            << " m" << std::endl;

  EXPECT_EQ(second_run.status.reset_count, 1);
  EXPECT_EQ(second_run.estimates.size(), 50u);
  EXPECT_GE(second_run.status.wheel_factors, 48);
  EXPECT_LE(second_run.status.wheel_missing, 1);
  EXPECT_EQ(second_run.status.wheel_gated, 0);
  EXPECT_LT(second_run.rejected_wheel, WheelOdometryBuffer::kRestartRejects);
  EXPECT_LT(second_run.max_error, 0.1);
}

// Without LiDAR constraints, a start (or a reset) at 2 m/s with a zero velocity: the IMU predicts no motion, the wheels
// say 0.2 m per frame, and the gate rejects every wheel factor. Re-admission lets a factor through after
// gate_readmit_frames gated frames, which corrects the velocity; without it the wheels stay locked out.
TEST_F(WheelOdometryTest, ReadmitsWheelFactorsAfterAWrongVelocity) {
  constexpr size_t kFirst = 40;
  const double stamp = (*frames_)[kFirst]->stamp;
  const WheelSamples wheel = make_wheel_samples(0.0, kEndTime + 0.05, test_T_imu_base());
  auto params = make_params();
  params.init_T_world_imu = synthetic_T_world_imu(stamp);
  params.init_v_world_imu.setZero();  // Truth: about 2 m/s
  params.wheel_odom.enabled = true;
  params.wheel_odom.T_imu_base = test_T_imu_base();
  auto locked = params;
  locked.wheel_odom.gate_readmit_frames = 0;

  const RunResult readmit = run(params, *far_map_, frames_->size(), &wheel, kFirst);
  const RunResult lockout = run(locked, *far_map_, frames_->size(), &wheel, kFirst);
  const int frames = static_cast<int>(frames_->size() - kFirst);
  std::cout << "wrong initial velocity: re-admission max_error=" << readmit.max_error << " m, final=" << readmit.final_error
            << " m, factors=" << readmit.status.wheel_factors << ", gated=" << readmit.status.wheel_gated << ", readmitted=" << readmit.status.wheel_readmitted
            << "; locked out max_error=" << lockout.max_error << " m, final=" << lockout.final_error << " m, factors=" << lockout.status.wheel_factors
            << ", gated=" << lockout.status.wheel_gated << std::endl;

  ASSERT_EQ(readmit.estimates.size(), static_cast<size_t>(frames));
  ASSERT_EQ(lockout.estimates.size(), static_cast<size_t>(frames));
  EXPECT_EQ(readmit.status.num_map_factors, 0);
  // Locked out: hardly any wheel factor, and the zero velocity stays
  EXPECT_LE(lockout.status.wheel_factors, 5);
  EXPECT_EQ(lockout.status.wheel_readmitted, 0);
  // Re-admitted: one factor through after gate_readmit_frames gated frames corrects the velocity, then every factor
  // passes the gate, and the error made while locked out shrinks again
  EXPECT_EQ(readmit.status.wheel_readmitted, 1);
  EXPECT_EQ(readmit.status.wheel_gated, params.wheel_odom.gate_readmit_frames);
  EXPECT_EQ(readmit.status.wheel_factors, frames - 1 - params.wheel_odom.gate_readmit_frames);
  EXPECT_LT(readmit.final_error, 1.0);
  EXPECT_LT(readmit.final_error, 0.5 * readmit.max_error);
  EXPECT_LT(readmit.final_error, 0.1 * lockout.final_error);
}

// ---------------------------------------------------------------------------------------------------------------------
// IMU gap self-reset extrapolated with wheel odometry (wheel_odom_reset_extrapolation)

namespace {

using Trajectory = std::function<Eigen::Isometry3d(double)>;

constexpr double kGapDiffStep = 1e-3;
constexpr double kGapStartTime = 4.0;  ///< Frames from here, so that the run is short
constexpr double kGapEndTime = 7.0;
constexpr double kGapBegin = 5.25;  ///< No IMU samples in [kGapBegin, kGapEnd): a 0.31 s gap
constexpr double kGapEnd = 5.55;
constexpr double kResetStamp = 5.7;  ///< The first clean frame; the last estimate before the gap is at 5.2 s

/// Straight at 3 m/s with a heading of 0.4 rad, braking smoothly to a stop over [4.5, 6.5] s (2.8 m/s^2 at 5.5 s)
Eigen::Isometry3d braking_T_world_imu(double t) {
  constexpr double kSpeed = 3.0;
  constexpr double kBrakeStart = 4.5;
  constexpr double kBrakeTime = 2.0;
  constexpr double kHeading = 0.4;
  double s = kSpeed * t;
  if (t > kBrakeStart) {
    const double u = std::min((t - kBrakeStart) / kBrakeTime, 1.0);
    s = kSpeed * (kBrakeStart + kBrakeTime * (u - u * u * u * u * (2.5 + u * (-3.0 + u))));
  }
  return make_pose(Eigen::Vector3d(5.0 + s * std::cos(kHeading), s * std::sin(kHeading), 1.0), kHeading);
}

/// A circle of radius 2.5 m at 2 m/s: 0.8 rad/s, so the yaw turns 0.32 rad over the 0.4 s from the last estimate
Eigen::Isometry3d turning_T_world_imu(double t) {
  constexpr double kSpeed = 2.0;
  constexpr double kYawRate = 0.8;
  constexpr double kRadius = kSpeed / kYawRate;
  const double yaw = kYawRate * t;
  return make_pose(Eigen::Vector3d(5.0 + kRadius * std::sin(yaw), -kRadius * std::cos(yaw), 1.0), yaw);
}

Eigen::Vector3d trajectory_velocity(const Trajectory& T_world_imu, double t) {
  return (T_world_imu(t + kGapDiffStep).translation() - T_world_imu(t - kGapDiffStep).translation()) / (2.0 * kGapDiffStep);
}

/// IMU samples at 100 Hz along the trajectory (zero bias, the default noise), as make_synthetic_imu
std::vector<SyntheticImuSample> make_trajectory_imu(const Trajectory& T_world_imu, double t_begin, double t_end) {
  const SyntheticImuParams params;
  std::mt19937 rng(params.seed);
  std::normal_distribution<double> unit(0.0, 1.0);
  const auto noise = [&](double sigma) { return Eigen::Vector3d(sigma * unit(rng), sigma * unit(rng), sigma * unit(rng)); };
  std::vector<SyntheticImuSample> samples;
  for (int i = 0; t_begin + i / params.rate <= t_end + 1e-9; i++) {
    const double t = t_begin + i / params.rate;
    const Eigen::Isometry3d T_prev = T_world_imu(t - kGapDiffStep);
    const Eigen::Isometry3d T_curr = T_world_imu(t);
    const Eigen::Isometry3d T_next = T_world_imu(t + kGapDiffStep);
    const Eigen::Vector3d a_world = (T_next.translation() - 2.0 * T_curr.translation() + T_prev.translation()) / (kGapDiffStep * kGapDiffStep);
    const Eigen::AngleAxisd delta(T_prev.linear().transpose() * T_next.linear());
    SyntheticImuSample sample;
    sample.stamp = t;
    sample.linear_acc = T_curr.linear().transpose() * (a_world + Eigen::Vector3d(0.0, 0.0, 9.81)) + noise(params.acc_noise);
    sample.angular_vel = delta.axis() * delta.angle() / (2.0 * kGapDiffStep) + noise(params.gyro_noise);
    samples.push_back(sample);
  }
  return samples;
}

/// The exact base twist along the trajectory at kWheelRate, as make_synthetic_wheel
WheelSamples make_trajectory_wheel(const Trajectory& T_world_imu, double t_begin, double t_end, const Eigen::Isometry3d& T_imu_base) {
  WheelSamples samples;
  for (int i = 0; t_begin + i / kWheelRate <= t_end + 1e-9; i++) {
    const double t = t_begin + i / kWheelRate;
    const Eigen::Isometry3d T0 = T_world_imu(t - kGapDiffStep) * T_imu_base;
    const Eigen::Isometry3d T1 = T_world_imu(t + kGapDiffStep) * T_imu_base;
    const gtsam::Vector6 xi = gtsam::Pose3::Logmap(gtsam::Pose3((T0.inverse() * T1).matrix())) / (2.0 * kGapDiffStep);
    WheelOdometrySample sample;
    sample.stamp = t;
    sample.angular = xi.head<3>();
    sample.linear = xi.tail<3>();
    samples.push_back(sample);
  }
  return samples;
}

double yaw_of(const Eigen::Matrix3d& R) {
  return std::atan2(R(1, 0), R(0, 0));
}

double wrap_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

/// The reset state of today's IMU gap path: the last pose moved by its velocity, the orientation held
Eigen::Isometry3d constant_velocity_reset(const glim::EstimationFrame& last, double stamp) {
  Eigen::Isometry3d T = last.T_world_imu;
  T.translation() += last.v_world_imu * (stamp - last.stamp);
  return T;
}

}  // namespace

class WheelGapResetTest : public WheelOdometryTest {
protected:
  struct GapRun {
    std::vector<glim::EstimationFrame::ConstPtr> estimates;
    glim::EstimationFrame::ConstPtr before;  ///< The last estimate before the gap
    glim::EstimationFrame::ConstPtr after;   ///< The first estimate after it (the reset frame)
    int dropped = 0;
    LocalizationStatus status;
    bool used_wheel = false;
    double translation_error = 0.0;  ///< Of the reset frame [m]
    double yaw_error = 0.0;          ///< Of the reset frame [deg]
    double velocity_error = 0.0;     ///< Of the reset frame [m/s]
    double max_error_after = 0.0;    ///< Translation, over the frames from the reset to the end [m]
  };

  /// The far map (no LiDAR constraint), wheel odometry on with test_T_imu_base, starting at the truth at kGapStartTime
  static OdometryEstimationLocalizationCPU::Params gap_params(const Trajectory& T_world_imu, bool reset_extrapolation, int num_threads = 2) {
    auto params = make_params(num_threads);
    params.init_T_world_imu = T_world_imu(kGapStartTime);
    params.init_v_world_imu = trajectory_velocity(T_world_imu, kGapStartTime);
    params.imu_bias.setZero();
    params.wheel_odom.enabled = true;
    params.wheel_odom.T_imu_base = test_T_imu_base();
    params.wheel_odom.reset_extrapolation = reset_extrapolation;
    return params;
  }

  /// Feed the frames in [kGapStartTime, kGapEndTime] with the trajectory's IMU (none in [kGapBegin, kGapEnd)) and the
  /// given wheel samples, as feed() does
  static GapRun run_gap(const OdometryEstimationLocalizationCPU::Params& params, const Trajectory& T_world_imu, const WheelSamples* wheel) {
    const auto imu = make_trajectory_imu(T_world_imu, kGapStartTime - 0.2, kGapEndTime + 0.05);
    OdometryEstimationLocalizationCPU odometry(params, *far_map_);
    GapRun run;
    size_t imu_cursor = 0;
    size_t wheel_cursor = 0;
    const double begin = kGapStartTime - 0.1;
    while (imu_cursor < imu.size() && imu[imu_cursor].stamp < begin) {
      imu_cursor++;
    }
    while (wheel && wheel_cursor < wheel->size() && (*wheel)[wheel_cursor].stamp < begin) {
      wheel_cursor++;
    }
    std::vector<glim::EstimationFrame::ConstPtr> marginalized;
    for (const auto& frame : *frames_) {
      if (frame->stamp < kGapStartTime - 1e-6 || frame->stamp > kGapEndTime + 1e-6) {
        continue;
      }
      for (; imu_cursor < imu.size() && imu[imu_cursor].stamp <= frame->stamp + 0.02; imu_cursor++) {
        const auto& sample = imu[imu_cursor];
        if (sample.stamp < kGapBegin - 1e-9 || sample.stamp >= kGapEnd - 1e-9) {
          odometry.insert_imu(sample.stamp, sample.linear_acc, sample.angular_vel);
        }
      }
      for (; wheel && wheel_cursor < wheel->size() && (*wheel)[wheel_cursor].stamp <= frame->stamp + 0.02; wheel_cursor++) {
        const auto& sample = (*wheel)[wheel_cursor];
        odometry.insert_wheel_odom(sample.stamp, sample.linear, sample.angular);
      }
      const auto estimate = odometry.insert_frame(frame, marginalized);
      if (!estimate) {
        run.dropped++;
        continue;
      }
      if (run.dropped > 0 && !run.after) {
        run.before = run.estimates.empty() ? nullptr : run.estimates.back();
        run.after = estimate;
        run.used_wheel = odometry.last_reset_used_wheel();
      }
      run.estimates.push_back(estimate);
      if (run.after) {
        const Eigen::Isometry3d error = T_world_imu(estimate->stamp).inverse() * estimate->T_world_imu;
        run.max_error_after = std::max(run.max_error_after, error.translation().norm());
      }
    }
    run.status = odometry.status();
    if (run.after) {
      const Eigen::Isometry3d truth = T_world_imu(run.after->stamp);
      run.translation_error = (truth.translation() - run.after->T_world_imu.translation()).norm();
      run.yaw_error = std::abs(wrap_angle(yaw_of(run.after->T_world_imu.linear()) - yaw_of(truth.linear()))) * 180.0 / M_PI;
      run.velocity_error = (trajectory_velocity(T_world_imu, run.after->stamp) - run.after->v_world_imu).norm();
    }
    return run;
  }

  static void print(const std::string& name, const GapRun& run) {
    std::cout << name << ": reset error " << run.translation_error << " m, yaw " << run.yaw_error << " deg, velocity " << run.velocity_error
              << " m/s; max error after the reset " << run.max_error_after << " m; wheel " << run.used_wheel << ", dropped " << run.dropped
              << ", wheel_gated " << run.status.wheel_gated << std::endl;
  }

  /// The gap was handled by one self-reset at kResetStamp, and estimation went on to the end
  static void expect_one_gap_reset(const GapRun& run) {
    ASSERT_TRUE(run.before);
    ASSERT_TRUE(run.after);
    EXPECT_NEAR(run.before->stamp, kResetStamp - 0.5, 1e-6);
    EXPECT_NEAR(run.after->stamp, kResetStamp, 1e-6);
    EXPECT_EQ(run.dropped, 4);
    EXPECT_EQ(run.status.reset_count, 1);
    EXPECT_EQ(run.status.last_reset_reason, ResetReason::IMU_GAP);
    EXPECT_NE(run.status.state, LocalizationState::LOST);
    EXPECT_NEAR(run.estimates.back()->stamp, kGapEndTime, 1e-6);
  }

  /// The reset frame is today's constant velocity extrapolation of the last estimate. The first frame is the
  /// optimized prior with the IMU factors of its own interval, so it sits a little off the prior: up to 0.3 mrad and
  /// 5 mm/s here. The wheel extrapolation is centimeters and degrees away in every case below.
  static void expect_constant_velocity_reset(const GapRun& run) {
    ASSERT_TRUE(run.before && run.after);
    const Eigen::Isometry3d expected = constant_velocity_reset(*run.before, run.after->stamp);
    EXPECT_LT((run.after->T_world_imu.translation() - expected.translation()).norm(), 1e-3);
    EXPECT_LT(Eigen::AngleAxisd(expected.linear().transpose() * run.after->T_world_imu.linear()).angle(), 2e-3);
    EXPECT_LT((run.after->v_world_imu - run.before->v_world_imu).norm(), 2e-2);
  }

  /// With the key on the reset frame is far closer to the truth, and so are the frames after it
  static void expect_wheel_reset_better(const GapRun& off, const GapRun& on) {
    expect_one_gap_reset(off);
    expect_one_gap_reset(on);
    EXPECT_FALSE(off.used_wheel);
    EXPECT_TRUE(on.used_wheel);
    expect_constant_velocity_reset(off);
    EXPECT_LT(on.translation_error, 0.05);
    EXPECT_LT(on.translation_error, 0.25 * off.translation_error);
    EXPECT_LT(on.velocity_error, 0.1);
    EXPECT_LT(on.velocity_error, 0.25 * off.velocity_error);
    EXPECT_LT(on.yaw_error, 0.5);
    EXPECT_LT(on.max_error_after, 0.5 * off.max_error_after);
  }
};

// Braking through the gap: the constant velocity overshoots, and the velocity it restarts with is too high
TEST_F(WheelGapResetTest, ExtrapolatesWithTheWheelsWhileBraking) {
  const Trajectory trajectory = braking_T_world_imu;
  const WheelSamples wheel = make_trajectory_wheel(trajectory, kGapStartTime - 0.2, kGapEndTime + 0.05, test_T_imu_base());
  const GapRun off = run_gap(gap_params(trajectory, false), trajectory, &wheel);
  const GapRun on = run_gap(gap_params(trajectory, true), trajectory, &wheel);
  print("braking, key off", off);
  print("braking, key on", on);
  expect_wheel_reset_better(off, on);
}

// Turning through the gap: the held orientation misses the turn, and the position and velocity miss the arc
TEST_F(WheelGapResetTest, ExtrapolatesWithTheWheelsWhileTurning) {
  const Trajectory trajectory = turning_T_world_imu;
  const WheelSamples wheel = make_trajectory_wheel(trajectory, kGapStartTime - 0.2, kGapEndTime + 0.05, test_T_imu_base());
  const GapRun off = run_gap(gap_params(trajectory, false), trajectory, &wheel);
  const GapRun on = run_gap(gap_params(trajectory, true), trajectory, &wheel);
  print("turning, key off", off);
  print("turning, key on", on);
  expect_wheel_reset_better(off, on);
  EXPECT_GT(off.yaw_error, 10.0);  // 0.32 rad of turn held
  EXPECT_LT(on.yaw_error, 0.1 * off.yaw_error);
}

// Without wheel_odom_enabled the key does nothing: bit-identical to the key off, with the samples fed (and dropped)
TEST_F(WheelGapResetTest, KeyWithoutWheelOdometryIsIdentical) {
  const Trajectory trajectory = turning_T_world_imu;
  const WheelSamples wheel = make_trajectory_wheel(trajectory, kGapStartTime - 0.2, kGapEndTime + 0.05, test_T_imu_base());
  auto off = gap_params(trajectory, false, 1);
  off.wheel_odom.enabled = false;
  auto on = off;
  on.wheel_odom.reset_extrapolation = true;
  const GapRun reference = run_gap(off, trajectory, &wheel);
  const GapRun keyed = run_gap(on, trajectory, &wheel);
  expect_one_gap_reset(reference);
  expect_constant_velocity_reset(reference);
  EXPECT_FALSE(keyed.used_wheel);
  ASSERT_EQ(keyed.estimates.size(), reference.estimates.size());
  for (size_t i = 0; i < reference.estimates.size(); i++) {
    EXPECT_EQ(keyed.estimates[i]->T_world_imu.matrix(), reference.estimates[i]->T_world_imu.matrix()) << "i=" << i;
    EXPECT_EQ(keyed.estimates[i]->v_world_imu, reference.estimates[i]->v_world_imu) << "i=" << i;
  }
}

// Without samples, or with a hole above wheel_odom_max_gap across the gap, the reset falls back to today's path
TEST_F(WheelGapResetTest, FallsBackWithoutWheelCoverage) {
  const Trajectory trajectory = braking_T_world_imu;
  const WheelSamples wheel = make_trajectory_wheel(trajectory, kGapStartTime - 0.2, kGapEndTime + 0.05, test_T_imu_base());
  WheelSamples holed;
  std::copy_if(wheel.begin(), wheel.end(), std::back_inserter(holed), [](const auto& s) { return s.stamp < 5.25 || s.stamp > 5.5; });
  const auto params = gap_params(trajectory, true);
  ASSERT_LT(params.wheel_odom.max_gap, 0.25);

  const GapRun none = run_gap(params, trajectory, nullptr);
  const GapRun hole = run_gap(params, trajectory, &holed);
  for (const auto* run : {&none, &hole}) {
    expect_one_gap_reset(*run);
    EXPECT_FALSE(run->used_wheel);
    expect_constant_velocity_reset(*run);
  }
}

// A wheel twist above kMaxResetSpeed at the reset frame: the self-reset falls back instead of going LOST
TEST_F(WheelGapResetTest, FallsBackOnAnInvalidWheelState) {
  const Trajectory trajectory = turning_T_world_imu;
  WheelSamples wheel = make_trajectory_wheel(trajectory, kGapStartTime - 0.2, kGapEndTime + 0.05, test_T_imu_base());
  for (auto& sample : wheel) {
    if (sample.stamp > kResetStamp - 0.05 && sample.stamp < kResetStamp + 0.05) {
      sample.linear *= 80.0;  // About 160 m/s
    }
  }
  const GapRun run = run_gap(gap_params(trajectory, true), trajectory, &wheel);
  print("excessive wheel speed", run);
  expect_one_gap_reset(run);
  EXPECT_FALSE(run.used_wheel);
  expect_constant_velocity_reset(run);
}
