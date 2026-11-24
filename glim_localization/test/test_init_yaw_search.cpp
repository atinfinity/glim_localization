// The yaw search of the initial pose in the odometry: a large yaw error in the given pose, with and without the search.
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/util/config.hpp>

#include <glim_localization/map/map_manager.hpp>
#include <glim_localization/odometry/odometry_estimation_localization_cpu.hpp>

#include "synthetic/synthetic_map.hpp"
#include "synthetic/synthetic_replay.hpp"
#include "synthetic/synthetic_scene.hpp"
#include "synthetic/synthetic_sensor.hpp"

namespace fs = std::filesystem;
using namespace glim_localization;
using namespace glim_localization::test;

namespace {

constexpr double kDeg = M_PI / 180.0;
constexpr double kEndTime = 12.0;
constexpr double kSettleTime = 3.0;
constexpr double kMaxTranslationError = 0.1;
constexpr double kMaxRotationErrorDeg = 1.0;

/// The ground truth with a yaw error about world z and a small translation error
Eigen::Isometry3d perturbed_pose(double t, double yaw_error) {
  Eigen::Isometry3d T = synthetic_T_world_imu(t);
  T.linear() = Eigen::AngleAxisd(yaw_error, Eigen::Vector3d::UnitZ()).toRotationMatrix() * T.linear();
  T.translation() += Eigen::Vector3d(0.3, -0.2, 0.1);
  return T;
}

struct RunStats {
  int estimated = 0;
  double max_translation = 0.0;  ///< After kSettleTime
  double max_rotation_deg = 0.0;
  double last_translation = 0.0;
  double last_rotation_deg = 0.0;
  double first_frame_time = 0.0;  ///< processing_time of the first estimated frame (includes the search)
  double mean_frame_time = 0.0;   ///< Of the other estimated frames
};

}  // namespace

class InitYawSearchTest : public testing::Test {
protected:
  static void SetUpTestSuite() {
    glim::GlobalConfig::instance(GLIM_LOCALIZATION_CONFIG_DIR, true);
    dir_ = new fs::path(fs::temp_directory_path() / ("glim_loc_yaw_search_" + std::to_string(::getpid())));
    fs::remove_all(*dir_);
    const SyntheticMapParams map_params = replay_map_params();
    write_synthetic_dump(dir_->string(), map_params);
    auto map = std::make_shared<MapManager>();
    ASSERT_TRUE(map->load(dir_->string()));
    map_ = new std::shared_ptr<const MapManager>(map);

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
    delete map_;
    fs::remove_all(*dir_);
    delete dir_;
  }

  static OdometryEstimationLocalizationCPU::Params params(bool search) {
    OdometryEstimationLocalizationCPU::Params p;
    p.num_threads = 2;
    p.map_factors.num_threads = 2;
    p.init_yaw_search.enabled = search;
    p.init_yaw_search.range = M_PI;
    p.init_yaw_search.step = 15.0 * kDeg;
    p.init_yaw_search.min_inlier_ratio = 0.5;
    p.init_yaw_search.time_budget = 30.0;  // Not the subject here; a loaded CI machine must not cut the search short
    return p;
  }

  /// Feed frames [begin, end) with their IMU data; on_frame gets each frame and its estimate (may be null)
  static RunStats run(
    OdometryEstimationLocalizationCPU& odometry,
    size_t& imu_cursor,
    size_t begin,
    size_t end,
    const std::function<void(size_t, const glim::EstimationFrame::ConstPtr&)>& on_frame = nullptr) {
    RunStats stats;
    double sum_time = 0.0;
    for (size_t i = begin; i < end; i++) {
      const auto& frame = (*frames_)[i];
      while (imu_cursor < imu_->size() && (*imu_)[imu_cursor].stamp <= frame->stamp + 0.02) {
        const auto& sample = (*imu_)[imu_cursor++];
        odometry.insert_imu(sample.stamp, sample.linear_acc, sample.angular_vel);
      }
      std::vector<glim::EstimationFrame::ConstPtr> marginalized;
      const auto estimate = odometry.insert_frame(frame, marginalized);
      if (on_frame) {
        on_frame(i, estimate);
      }
      if (!estimate) {
        continue;
      }
      EXPECT_TRUE(estimate->T_world_imu.matrix().allFinite());
      const Eigen::Isometry3d error = synthetic_T_world_imu(estimate->stamp).inverse() * estimate->T_world_imu;
      stats.last_translation = error.translation().norm();
      stats.last_rotation_deg = Eigen::AngleAxisd(error.linear()).angle() / kDeg;
      if (estimate->stamp >= kSettleTime) {
        stats.max_translation = std::max(stats.max_translation, stats.last_translation);
        stats.max_rotation_deg = std::max(stats.max_rotation_deg, stats.last_rotation_deg);
      }
      if (stats.estimated == 0) {
        stats.first_frame_time = odometry.status().processing_time;
      } else {
        sum_time += odometry.status().processing_time;
      }
      stats.estimated++;
    }
    stats.mean_frame_time = stats.estimated > 1 ? sum_time / (stats.estimated - 1) : 0.0;
    return stats;
  }

  static void print(const std::string& name, const RunStats& stats, const LocalizationStatus& status) {
    std::cout << name << ": state=" << to_string(status.state) << " search=" << to_string(status.init_search)
              << " candidates=" << status.init_search_candidates << " score=" << status.init_search_score
              << " yaw_offset=" << status.init_search_yaw_offset / kDeg << " deg search_time=" << status.init_search_time * 1e3
              << " ms first_frame=" << stats.first_frame_time * 1e3 << " ms mean_frame=" << stats.mean_frame_time * 1e3
              << " ms max_translation=" << stats.max_translation << " m max_rotation=" << stats.max_rotation_deg
              << " deg last_translation=" << stats.last_translation << " m last_rotation=" << stats.last_rotation_deg << " deg" << std::endl;
  }

  static fs::path* dir_;
  static std::shared_ptr<const MapManager>* map_;
  static std::vector<glim::PreprocessedFrame::Ptr>* frames_;
  static std::vector<SyntheticImuSample>* imu_;
};

fs::path* InitYawSearchTest::dir_ = nullptr;
std::shared_ptr<const MapManager>* InitYawSearchTest::map_ = nullptr;
std::vector<glim::PreprocessedFrame::Ptr>* InitYawSearchTest::frames_ = nullptr;
std::vector<SyntheticImuSample>* InitYawSearchTest::imu_ = nullptr;

TEST_F(InitYawSearchTest, WithoutSearchLargeYawErrorDoesNotConverge) {
  auto p = params(false);
  p.estimate_init_state = false;
  p.init_T_world_imu = perturbed_pose(0.0, 75.0 * kDeg);
  p.init_v_world_imu.setZero();
  OdometryEstimationLocalizationCPU odometry(p, *map_);

  size_t imu_cursor = 0;
  const auto stats = run(odometry, imu_cursor, 0, frames_->size());
  const auto status = odometry.status();
  print("no search, yaw error 75 deg", stats, status);
  EXPECT_EQ(status.init_search, InitSearchResult::NONE);
  EXPECT_EQ(status.init_search_candidates, 0);
  // The registration stays in the wrong minimum: the pose is far off, or the quality monitor never trusts it
  EXPECT_TRUE(stats.last_rotation_deg > 10.0 || stats.last_translation > 1.0 || status.state != LocalizationState::TRACKING)
    << "rotation " << stats.last_rotation_deg << " deg, translation " << stats.last_translation << " m, " << to_string(status.state);
}

TEST_F(InitYawSearchTest, ConfiguredPoseWithLargeYawErrorConverges) {
  for (const double yaw_error_deg : {75.0, -90.0}) {
    auto p = params(true);
    p.estimate_init_state = false;
    p.init_T_world_imu = perturbed_pose(0.0, yaw_error_deg * kDeg);
    p.init_v_world_imu.setZero();
    OdometryEstimationLocalizationCPU odometry(p, *map_);

    size_t imu_cursor = 0;
    const auto stats = run(odometry, imu_cursor, 0, frames_->size());
    const auto status = odometry.status();
    print("search, configured pose, yaw error " + std::to_string(yaw_error_deg) + " deg", stats, status);
    EXPECT_EQ(stats.estimated, static_cast<int>(frames_->size()));
    EXPECT_EQ(status.state, LocalizationState::TRACKING) << to_string(status.state);
    EXPECT_EQ(status.init_search, InitSearchResult::ACCEPTED);
    EXPECT_EQ(status.init_search_candidates, 24);
    EXPECT_GT(status.init_search_score, 0.5);
    EXPECT_NEAR(status.init_search_yaw_offset / kDeg, -yaw_error_deg, 1.0);
    EXPECT_LT(stats.max_translation, kMaxTranslationError);
    EXPECT_LT(stats.max_rotation_deg, kMaxRotationErrorDeg);
  }
}

TEST_F(InitYawSearchTest, ResetWithLargeYawErrorConvergesAndRelocalizeDoesNotSearch) {
  auto p = params(true);
  p.estimate_init_state = true;  // Wait for reset()
  OdometryEstimationLocalizationCPU odometry(p, *map_);
  size_t imu_cursor = 0;

  constexpr size_t kResetIndex = 5;  // Still at rest
  EXPECT_EQ(run(odometry, imu_cursor, 0, kResetIndex).estimated, 0);
  ASSERT_TRUE(odometry.reset(perturbed_pose((*frames_)[kResetIndex]->stamp, -80.0 * kDeg), Eigen::Vector3d::Zero(), (*frames_)[kResetIndex]->stamp));

  constexpr size_t kRelocalizeIndex = 60;
  const auto stats = run(odometry, imu_cursor, kResetIndex, kRelocalizeIndex);
  auto status = odometry.status();
  print("search, reset(), yaw error -80 deg", stats, status);
  EXPECT_EQ(status.reset_count, 1);
  EXPECT_EQ(status.init_search, InitSearchResult::ACCEPTED);
  EXPECT_NEAR(status.init_search_yaw_offset / kDeg, 80.0, 1.0);
  EXPECT_EQ(status.state, LocalizationState::TRACKING) << to_string(status.state);
  EXPECT_LT(stats.max_translation, kMaxTranslationError);
  EXPECT_LT(stats.max_rotation_deg, kMaxRotationErrorDeg);

  // relocalize() restarts from the estimate: no search, and the result of the old run is cleared
  ASSERT_EQ(odometry.relocalize(0.5), "");
  const auto after = run(odometry, imu_cursor, kRelocalizeIndex, kRelocalizeIndex + 20);
  status = odometry.status();
  EXPECT_EQ(status.reset_count, 2);
  EXPECT_EQ(status.init_search, InitSearchResult::NONE);
  EXPECT_EQ(status.init_search_candidates, 0);
  EXPECT_LT(after.max_translation, kMaxTranslationError);
}

TEST_F(InitYawSearchTest, PoseOffTheMapIsRejectedUntilANewPose) {
  auto p = params(true);
  p.estimate_init_state = true;
  OdometryEstimationLocalizationCPU odometry(p, *map_);
  size_t imu_cursor = 0;

  constexpr size_t kResetIndex = 2;
  run(odometry, imu_cursor, 0, kResetIndex);
  Eigen::Isometry3d off = synthetic_T_world_imu((*frames_)[kResetIndex]->stamp);
  off.translation() += Eigen::Vector3d(500.0, 0.0, 0.0);
  ASSERT_TRUE(odometry.reset(off, Eigen::Vector3d::Zero(), (*frames_)[kResetIndex]->stamp));

  std::vector<LocalizationState> states;
  const auto rejected = run(odometry, imu_cursor, kResetIndex, kResetIndex + 3, [&](size_t, const glim::EstimationFrame::ConstPtr& estimate) {
    EXPECT_FALSE(estimate) << "a rejected pose must not produce estimates";
    states.push_back(odometry.status().state);
  });
  auto status = odometry.status();
  print("search, pose 500 m off the map", rejected, status);
  EXPECT_EQ(rejected.estimated, 0);
  EXPECT_EQ(status.state, LocalizationState::LOST);
  EXPECT_EQ(status.init_search, InitSearchResult::REJECTED);
  EXPECT_EQ(status.init_search_candidates, 24);
  EXPECT_EQ(status.init_search_score, 0.0);
  for (const auto state : states) {
    EXPECT_EQ(state, LocalizationState::LOST);
  }

  // A new pose clears the rejection
  const size_t index = kResetIndex + 3;
  ASSERT_TRUE(odometry.reset(perturbed_pose((*frames_)[index]->stamp, 60.0 * kDeg), Eigen::Vector3d::Zero(), (*frames_)[index]->stamp));
  const auto recovered = run(odometry, imu_cursor, index, index + 40);
  status = odometry.status();
  print("search, new pose after a rejection, yaw error 60 deg", recovered, status);
  EXPECT_EQ(recovered.estimated, 40);
  EXPECT_EQ(status.init_search, InitSearchResult::ACCEPTED);
  EXPECT_EQ(status.state, LocalizationState::TRACKING) << to_string(status.state);
  EXPECT_LT(recovered.last_translation, kMaxTranslationError);
  EXPECT_LT(recovered.last_rotation_deg, kMaxRotationErrorDeg);
}

TEST_F(InitYawSearchTest, InvalidParametersAreRejected) {
  auto p = params(true);
  p.init_yaw_search.step = 0.0;
  EXPECT_THROW(OdometryEstimationLocalizationCPU(p, *map_), std::invalid_argument);
  p = params(true);
  p.init_yaw_search.min_inlier_ratio = 1.5;
  EXPECT_THROW(OdometryEstimationLocalizationCPU(p, *map_), std::invalid_argument);
  p = params(true);
  p.init_yaw_search.time_budget = -1.0;
  EXPECT_THROW(OdometryEstimationLocalizationCPU(p, *map_), std::invalid_argument);
  p = params(true);
  p.init_yaw_search.max_iterations = 0;
  EXPECT_THROW(OdometryEstimationLocalizationCPU(p, *map_), std::invalid_argument);
  // Validated also when disabled: a typo should not wait until someone turns the search on
  p = params(false);
  p.init_yaw_search.range = -1.0;
  EXPECT_THROW(OdometryEstimationLocalizationCPU(p, *map_), std::invalid_argument);
}
