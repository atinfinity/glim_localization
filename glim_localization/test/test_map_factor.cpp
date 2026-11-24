#include <filesystem>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <unistd.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim_localization/map/map_manager.hpp>
#include <glim_localization/odometry/map_factor_builder.hpp>
#include <glim_localization/odometry/weighted_factor.hpp>

#include "synthetic/synthetic_map.hpp"
#include "synthetic/synthetic_scene.hpp"
#include "synthetic/synthetic_sensor.hpp"

namespace fs = std::filesystem;
using namespace glim_localization;
using namespace glim_localization::test;
using gtsam::symbol_shorthand::X;

namespace {

constexpr double kScanTime = 6.0;

Eigen::Isometry3d perturbation(double tx, double ty, double tz, double yaw_deg, double pitch_deg) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(tx, ty, tz);
  T.linear() = (Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(pitch_deg * M_PI / 180.0, Eigen::Vector3d::UnitY()))
                 .toRotationMatrix();
  return T;
}

}  // namespace

class MapFactorTest : public testing::Test {
protected:
  static void SetUpTestSuite() {
    dir_ = new fs::path(fs::temp_directory_path() / ("glim_loc_map_factor_" + std::to_string(::getpid())));
    fs::remove_all(*dir_);
    SyntheticMapParams params;
    params.num_submaps = 5;
    write_synthetic_dump(dir_->string(), params);
    manager_ = new MapManager();
    ASSERT_TRUE(manager_->load(dir_->string()));

    const auto scans = make_synthetic_scans(make_scene_boxes(params), kScanTime, kScanTime);
    auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(scans.front()->points);
    cloud->add_covs(gtsam_points::estimate_covariances(cloud->points, cloud->size()));
    frame_ = new gtsam_points::PointCloud::ConstPtr(cloud);
  }

  static void TearDownTestSuite() {
    delete frame_;
    delete manager_;
    fs::remove_all(*dir_);
    delete dir_;
  }

  static fs::path* dir_;
  static MapManager* manager_;
  static gtsam_points::PointCloud::ConstPtr* frame_;
};

fs::path* MapFactorTest::dir_ = nullptr;
MapManager* MapFactorTest::manager_ = nullptr;
gtsam_points::PointCloud::ConstPtr* MapFactorTest::frame_ = nullptr;

TEST_F(MapFactorTest, ConvergesFromPerturbedPose) {
  const Eigen::Isometry3d T_truth = synthetic_T_world_imu(kScanTime);
  const Eigen::Isometry3d T_init = T_truth * perturbation(0.3, -0.2, 0.1, 3.0, 0.5);

  const auto map_factors = create_map_factors(*manager_, X(0), *frame_, T_init, MapFactorParams());
  ASSERT_FALSE(map_factors.tiles.empty());
  EXPECT_EQ(map_factors.factors.size(), map_factors.tiles.size() * 3);

  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3(T_init.matrix()));
  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(30);
  values = gtsam_points::LevenbergMarquardtOptimizerExt(map_factors.factors, values, lm_params).optimize();

  const Eigen::Isometry3d T_estimate(values.at<gtsam::Pose3>(X(0)).matrix());
  const Eigen::Isometry3d error = T_truth.inverse() * T_estimate;
  EXPECT_LT(error.translation().norm(), 0.05);
  EXPECT_LT(Eigen::AngleAxisd(error.linear()).angle(), 0.5 * M_PI / 180.0);
  EXPECT_GT(map_inlier_ratio(map_factors.tiles, *frame_, T_estimate), 0.9);
}

TEST_F(MapFactorTest, NoFactorsAwayFromMap) {
  Eigen::Isometry3d T_far = Eigen::Isometry3d::Identity();
  T_far.translation() = Eigen::Vector3d(1000.0, 1000.0, 0.0);
  const auto outside_radius = create_map_factors(*manager_, X(0), *frame_, T_far, MapFactorParams());
  EXPECT_TRUE(outside_radius.tiles.empty());
  EXPECT_TRUE(outside_radius.factors.empty());
  EXPECT_DOUBLE_EQ(map_inlier_ratio(outside_radius.tiles, *frame_, T_far), 0.0);

  // Within the search radius, but the scan does not touch the tiles.
  Eigen::Isometry3d T_side = Eigen::Isometry3d::Identity();
  T_side.translation() = Eigen::Vector3d(20.0, 45.0, 1.0);
  MapFactorParams params;
  params.search_radius = 100.0;
  ASSERT_FALSE(manager_->query(T_side, params.search_radius, params.max_submaps).empty());
  EXPECT_TRUE(create_map_factors(*manager_, X(0), *frame_, T_side, params).tiles.empty());
}

TEST_F(MapFactorTest, NonPositiveWeightCreatesNoFactors) {
  const Eigen::Isometry3d T = synthetic_T_world_imu(kScanTime);
  EXPECT_TRUE(create_map_factors(*manager_, X(0), *frame_, T, MapFactorParams(), 0.0).factors.empty());
  EXPECT_TRUE(create_map_factors(*manager_, X(0), *frame_, T, MapFactorParams(), -1.0).factors.empty());
}

TEST_F(MapFactorTest, WeightScalesInformation) {
  const Eigen::Isometry3d T = synthetic_T_world_imu(kScanTime) * perturbation(0.05, 0.02, 0.01, 0.5, 0.1);
  const auto plain = create_map_factors(*manager_, X(0), *frame_, T, MapFactorParams(), 1.0);
  const auto weighted = create_map_factors(*manager_, X(0), *frame_, T, MapFactorParams(), 0.5);
  ASSERT_FALSE(plain.factors.empty());
  ASSERT_EQ(plain.factors.size(), weighted.factors.size());

  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3(T.matrix()));
  const auto H_plain = std::dynamic_pointer_cast<gtsam::HessianFactor>(plain.factors[0]->linearize(values));
  const auto H_weighted = std::dynamic_pointer_cast<gtsam::HessianFactor>(weighted.factors[0]->linearize(values));
  ASSERT_TRUE(H_plain);
  ASSERT_TRUE(H_weighted);

  const gtsam::Matrix G_plain = H_plain->information();
  EXPECT_TRUE(H_weighted->information().isApprox(0.5 * G_plain, 1e-9));
  const gtsam::Vector g_plain = H_plain->linearTerm();
  const gtsam::Vector g_weighted = H_weighted->linearTerm();
  EXPECT_TRUE(g_weighted.isApprox(0.5 * g_plain, 1e-9));
  EXPECT_NEAR(weighted.factors[0]->error(values), 0.5 * plain.factors[0]->error(values), 1e-6);
}

TEST(WeightedFactorTest, RejectsInvalidArguments) {
  EXPECT_THROW(WeightedFactor(nullptr, 1.0), std::invalid_argument);
}
