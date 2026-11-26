#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

#include <gtsam_points/types/gaussian_voxelmap.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/common/imu_integration.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/util/config.hpp>

#include <glim_localization/map/map_manager.hpp>

#include "synthetic/synthetic_map.hpp"
#include "synthetic/synthetic_scene.hpp"
#include "synthetic/synthetic_sensor.hpp"

namespace fs = std::filesystem;
using namespace glim_localization;
using namespace glim_localization::test;

class SyntheticSensorTest : public testing::Test {
protected:
  static void SetUpTestSuite() { glim::GlobalConfig::instance(GLIM_LOCALIZATION_CONFIG_DIR, true); }
};

TEST_F(SyntheticSensorTest, TrajectoryStartsAtRest) {
  EXPECT_LT(synthetic_v_world_imu(0.5).norm(), 1e-9);
  EXPECT_NEAR(synthetic_v_world_imu(6.0).x(), 2.0, 1e-6);
  EXPECT_NEAR(synthetic_T_world_imu(12.0).translation().x() - synthetic_T_world_imu(0.0).translation().x(), 20.0, 1e-9);
}

TEST_F(SyntheticSensorTest, NoiselessWheelTwistIntegratesToTruth) {
  // Integrating the twist of the base frame (constant over each 1 ms step) must reproduce its motion
  SyntheticWheelParams params;
  params.rate = 1000.0;
  params.T_imu_base.translation() = Eigen::Vector3d(-0.3, 0.1, -0.5);
  params.T_imu_base.linear() = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto samples = make_synthetic_wheel(4.0, 5.0, params);
  ASSERT_EQ(samples.size(), 1001u);
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  for (size_t i = 0; i + 1 < samples.size(); i++) {
    const double dt = samples[i + 1].stamp - samples[i].stamp;
    const Eigen::Vector3d w = 0.5 * (samples[i].angular + samples[i + 1].angular);
    const Eigen::Vector3d v = 0.5 * (samples[i].linear + samples[i + 1].linear);
    Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
    step.linear() = Eigen::AngleAxisd(w.norm() * dt, w.norm() > 0.0 ? Eigen::Vector3d(w.normalized()) : Eigen::Vector3d::UnitZ()).toRotationMatrix();
    step.translation() = v * dt;
    T = T * step;
  }
  const Eigen::Isometry3d truth = (synthetic_T_world_imu(4.0) * params.T_imu_base).inverse() * (synthetic_T_world_imu(5.0) * params.T_imu_base);
  EXPECT_LT((T.translation() - truth.translation()).norm(), 1e-3);
  EXPECT_LT(Eigen::AngleAxisd(T.linear().transpose() * truth.linear()).angle(), 1e-4);

  // Noise is added only when configured, and is repeatable
  params.linear_noise = 0.01;
  params.angular_noise = 0.002;
  const auto noisy = make_synthetic_wheel(4.0, 5.0, params);
  const auto again = make_synthetic_wheel(4.0, 5.0, params);
  ASSERT_EQ(noisy.size(), samples.size());
  EXPECT_NE(noisy[10].linear, samples[10].linear);
  EXPECT_EQ(noisy[10].linear, again[10].linear);
  EXPECT_LT((noisy[10].linear - samples[10].linear).norm(), 0.06);
}

TEST_F(SyntheticSensorTest, NoiselessImuIntegratesToTruth) {
  // IMUIntegration holds each sample over the preceding interval, which costs about
  // 0.2 deg per second at 100 Hz on this trajectory. Use 1 kHz to check the generator itself.
  SyntheticImuParams params;
  params.rate = 1000.0;
  params.acc_noise = 0.0;
  params.gyro_noise = 0.0;
  params.acc_bias.setZero();
  params.gyro_bias.setZero();

  glim::IMUIntegration integration;
  for (const auto& sample : make_synthetic_imu(3.9, 5.1, params)) {
    integration.insert_imu(sample.stamp, sample.linear_acc, sample.angular_vel);
  }

  const double t0 = 4.0;
  const double t1 = 5.0;
  const gtsam::NavState state(gtsam::Pose3(synthetic_T_world_imu(t0).matrix()), synthetic_v_world_imu(t0));
  std::vector<double> times;
  std::vector<Eigen::Isometry3d> poses;
  integration.integrate_imu(t0, t1, state, gtsam::imuBias::ConstantBias(), times, poses);
  ASSERT_FALSE(poses.empty());

  const Eigen::Isometry3d error = synthetic_T_world_imu(t1).inverse() * poses.back();
  EXPECT_LT(error.translation().norm(), 0.01);
  EXPECT_LT(Eigen::AngleAxisd(error.linear()).angle(), 0.1 * M_PI / 180.0);
}

TEST_F(SyntheticSensorTest, ScanOverlapsMapAtTruePose) {
  const fs::path dir = fs::temp_directory_path() / ("glim_loc_sensor_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  SyntheticMapParams map_params;
  map_params.num_submaps = 5;
  write_synthetic_dump(dir.string(), map_params);
  MapManager manager;
  ASSERT_TRUE(manager.load(dir.string()));

  const double t = 6.0;
  const auto scans = make_synthetic_scans(make_scene_boxes(map_params), t, t);
  ASSERT_EQ(scans.size(), 1u);
  EXPECT_GT(scans[0]->size(), 2000);

  glim::CloudPreprocessorParams preprocess_params;
  preprocess_params.global_shutter = true;
  glim::CloudPreprocessor preprocessor(preprocess_params);
  const auto frame = preprocessor.preprocess(scans[0]);
  ASSERT_TRUE(frame);
  EXPECT_GT(frame->size(), 2000);

  const auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(frame->points);
  const Eigen::Isometry3d T_world_imu = synthetic_T_world_imu(t);
  std::vector<gtsam_points::GaussianVoxelMap::ConstPtr> voxelmaps;
  std::vector<Eigen::Isometry3d> transforms;
  for (const auto& tile : manager.query(T_world_imu, 60.0, 5)) {
    voxelmaps.push_back(tile->voxelmaps.front());
    transforms.push_back(tile->T_world_origin.inverse() * T_world_imu);
  }
  ASSERT_FALSE(voxelmaps.empty());
  EXPECT_GT(gtsam_points::overlap(voxelmaps, cloud, transforms), 0.9);

  fs::remove_all(dir);
}
