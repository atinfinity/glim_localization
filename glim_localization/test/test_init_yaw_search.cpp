// The yaw search of the initial pose in the odometry: a large yaw error in the given pose, with and without the search.
#include <algorithm>
#include <cmath>
#include <limits>
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
    // Not the subject here: a loaded CI machine must not cut the search short. 30 s was not enough under a parallel
    // ctest run on amd64 (14 of 24 candidates scored), so leave the bound to the ctest TIMEOUT instead
    p.init_yaw_search.time_budget = 1e6;
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
              << " candidates=" << status.init_search_candidates << "/" << status.init_search_candidates_total << " score=" << status.init_search_score
              << " yaw_offset=" << status.init_search_yaw_offset / kDeg
              << " deg runner_up=" << status.init_search_runner_up_score << " at " << status.init_search_runner_up_yaw_offset / kDeg << " deg search_time=" << status.init_search_time * 1e3
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
  EXPECT_EQ(status.init_search_reason, InitSearchReason::NONE);
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
    EXPECT_EQ(status.init_search_reason, InitSearchReason::NONE);
    EXPECT_EQ(status.init_search_candidates, 24);
    EXPECT_EQ(status.init_search_candidates_total, 24);
    EXPECT_GT(status.init_search_score, 0.9);
    EXPECT_NEAR(status.init_search_yaw_offset / kDeg, -yaw_error_deg, 1.0);
    // The corridor flipped by a half turn is the runner-up
    EXPECT_LT(status.init_search_runner_up_score, status.init_search_score - p.init_yaw_search.min_score_gap);
    EXPECT_GT(std::abs(std::remainder(status.init_search_runner_up_yaw_offset - status.init_search_yaw_offset, 2.0 * M_PI)), 2.0 * p.init_yaw_search.step);
    EXPECT_LT(stats.max_translation, kMaxTranslationError);
    EXPECT_LT(stats.max_rotation_deg, kMaxRotationErrorDeg);
  }
}

TEST_F(InitYawSearchTest, OffsetPoseWithinTheBoundConverges) {
  // 1.5 m and 90 deg off, with a 2.5 m bound: the match moves 1.5 m, within it
  auto p = params(true);
  p.estimate_init_state = false;
  p.init_T_world_imu = perturbed_pose(0.0, 90.0 * kDeg);
  p.init_T_world_imu.translation() = synthetic_T_world_imu(0.0).translation() + Eigen::Vector3d(1.5, 0.0, 0.0);
  p.init_v_world_imu.setZero();
  p.init_yaw_search.max_translation = 2.5;
  OdometryEstimationLocalizationCPU odometry(p, *map_);

  size_t imu_cursor = 0;
  const auto stats = run(odometry, imu_cursor, 0, 30);
  const auto status = odometry.status();
  print("search, 1.5 m and 90 deg off, bound 2.5 m", stats, status);
  EXPECT_EQ(status.init_search, InitSearchResult::ACCEPTED);
  EXPECT_EQ(status.init_search_reason, InitSearchReason::NONE);
  EXPECT_EQ(status.state, LocalizationState::TRACKING) << to_string(status.state);
  EXPECT_LT(stats.last_translation, kMaxTranslationError);
  EXPECT_LT(stats.last_rotation_deg, kMaxRotationErrorDeg);
}

TEST_F(InitYawSearchTest, CoarseStepStillReportsARunnerUp) {
  // With a 90 deg step, two steps (180 deg) would leave no candidate far enough away for the ambiguity check;
  // distinct_yaw is capped at 90 deg, so the half-turn flip of the corridor is still the runner-up
  auto p = params(true);
  p.estimate_init_state = false;
  p.init_T_world_imu = perturbed_pose(0.0, 75.0 * kDeg);
  p.init_v_world_imu.setZero();
  p.init_yaw_search.step = 90.0 * kDeg;
  OdometryEstimationLocalizationCPU odometry(p, *map_);

  size_t imu_cursor = 0;
  const auto stats = run(odometry, imu_cursor, 0, 30);
  const auto status = odometry.status();
  print("search, step 90 deg, yaw error 75 deg", stats, status);
  EXPECT_EQ(status.init_search, InitSearchResult::ACCEPTED);
  EXPECT_EQ(status.init_search_candidates_total, 4);
  EXPECT_NEAR(status.init_search_yaw_offset / kDeg, -75.0, 1.0);
  EXPECT_GT(status.init_search_runner_up_score, 0.0) << "no runner-up: the ambiguity check could not run";
  EXPECT_GT(std::abs(std::remainder(status.init_search_runner_up_yaw_offset - status.init_search_yaw_offset, 2.0 * M_PI)), 90.0 * kDeg);
  EXPECT_LT(status.init_search_runner_up_score, status.init_search_score - p.init_yaw_search.min_score_gap);
  EXPECT_EQ(status.state, LocalizationState::TRACKING) << to_string(status.state);
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
  EXPECT_EQ(status.init_search_reason, InitSearchReason::NONE);
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
  EXPECT_EQ(status.init_search_reason, InitSearchReason::LOW_SCORE);
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
  EXPECT_EQ(status.init_search_reason, InitSearchReason::NONE);
  EXPECT_EQ(recovered.estimated, 40);
  EXPECT_EQ(status.init_search, InitSearchResult::ACCEPTED);
  EXPECT_EQ(status.state, LocalizationState::TRACKING) << to_string(status.state);
  EXPECT_LT(recovered.last_translation, kMaxTranslationError);
  EXPECT_LT(recovered.last_rotation_deg, kMaxRotationErrorDeg);
}

TEST_F(InitYawSearchTest, TruncatedSearchIsRejected) {
  // A budget that ends the search after the hint: the hint registers into a wrong minimum, which must not be accepted
  auto p = params(true);
  p.estimate_init_state = false;
  p.init_T_world_imu = perturbed_pose(0.0, 75.0 * kDeg);
  p.init_v_world_imu.setZero();
  p.init_yaw_search.time_budget = 0.0;
  OdometryEstimationLocalizationCPU odometry(p, *map_);

  size_t imu_cursor = 0;
  const auto stats = run(odometry, imu_cursor, 0, 5);
  const auto status = odometry.status();
  print("search, budget 0, yaw error 75 deg", stats, status);
  EXPECT_EQ(stats.estimated, 0);
  EXPECT_EQ(status.state, LocalizationState::LOST);
  EXPECT_EQ(status.init_search, InitSearchResult::REJECTED);
  EXPECT_EQ(status.init_search_reason, InitSearchReason::TRUNCATED);
  EXPECT_EQ(status.init_search_candidates, 1);
  EXPECT_EQ(status.init_search_candidates_total, 24);
}

TEST_F(InitYawSearchTest, WrongYawOnTheMapIsRejectedWithoutASearchRange) {
  // range 0 scores the given pose alone: on the map, but at a wrong yaw that registers into a wrong minimum
  auto p = params(true);
  p.estimate_init_state = false;
  p.init_T_world_imu = perturbed_pose(0.0, 75.0 * kDeg);
  p.init_v_world_imu.setZero();
  p.init_yaw_search.range = 0.0;
  OdometryEstimationLocalizationCPU odometry(p, *map_);

  size_t imu_cursor = 0;
  const auto stats = run(odometry, imu_cursor, 0, 5);
  const auto status = odometry.status();
  print("search range 0, yaw error 75 deg", stats, status);
  EXPECT_EQ(stats.estimated, 0);
  EXPECT_EQ(status.state, LocalizationState::LOST);
  EXPECT_EQ(status.init_search, InitSearchResult::REJECTED);
  EXPECT_EQ(status.init_search_reason, InitSearchReason::LOW_SCORE);
  EXPECT_EQ(status.init_search_candidates, 1);
  EXPECT_EQ(status.init_search_candidates_total, 1);
  EXPECT_GT(status.init_search_score, 0.0) << "the pose is on the map";
  EXPECT_LT(status.init_search_score, p.init_yaw_search.min_inlier_ratio);
}

TEST_F(InitYawSearchTest, EmptyAndSparseFramesKeepTheSearchPending) {
  auto p = params(true);
  p.estimate_init_state = false;
  p.init_T_world_imu = perturbed_pose(0.0, 75.0 * kDeg);
  p.init_v_world_imu.setZero();
  OdometryEstimationLocalizationCPU odometry(p, *map_);

  // Frame 0 without points, frame 1 with 50 (fewer than min_points); their neighbor lists stay consistent
  auto empty = std::make_shared<glim::PreprocessedFrame>(*(*frames_)[0]);
  empty->points.clear();
  empty->times.clear();
  empty->intensities.clear();
  empty->neighbors.clear();
  auto sparse = std::make_shared<glim::PreprocessedFrame>(*(*frames_)[1]);
  constexpr int kSparse = 50;
  ASSERT_LT(kSparse, p.init_yaw_search.min_points);
  sparse->points.resize(kSparse);
  sparse->times.resize(std::min<size_t>(sparse->times.size(), kSparse));
  sparse->intensities.resize(std::min<size_t>(sparse->intensities.size(), kSparse));
  sparse->neighbors.resize(kSparse * sparse->k_neighbors);
  for (auto& n : sparse->neighbors) {
    n %= kSparse;
  }

  size_t imu_cursor = 0;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;
  for (const auto& frame : {empty, sparse}) {
    while (imu_cursor < imu_->size() && (*imu_)[imu_cursor].stamp <= frame->stamp + 0.02) {
      odometry.insert_imu((*imu_)[imu_cursor].stamp, (*imu_)[imu_cursor].linear_acc, (*imu_)[imu_cursor].angular_vel);
      imu_cursor++;
    }
    EXPECT_FALSE(odometry.insert_frame(frame, marginalized)) << frame->size() << " points";
    EXPECT_EQ(odometry.status().init_search, InitSearchResult::NONE) << "still pending";
  }

  // The first full frame runs the search
  const auto stats = run(odometry, imu_cursor, 2, frames_->size());
  const auto status = odometry.status();
  print("search after an empty and a sparse frame, yaw error 75 deg", stats, status);
  EXPECT_EQ(stats.estimated, static_cast<int>(frames_->size()) - 2);
  EXPECT_EQ(status.init_search, InitSearchResult::ACCEPTED);
  EXPECT_NEAR(status.init_search_yaw_offset / kDeg, -75.0, 1.0);
  EXPECT_EQ(status.state, LocalizationState::TRACKING) << to_string(status.state);
  EXPECT_LT(stats.max_translation, kMaxTranslationError);
}

TEST_F(InitYawSearchTest, VelocityTurnsWithTheYaw) {
  // A reset while moving: the given velocity is in the frame of the given (wrong) heading, so it turns with the yaw
  auto p = params(true);
  p.estimate_init_state = true;
  OdometryEstimationLocalizationCPU odometry(p, *map_);
  size_t imu_cursor = 0;

  constexpr size_t kResetIndex = 30;
  const double stamp = (*frames_)[kResetIndex]->stamp;
  const Eigen::Vector3d v_true = synthetic_v_world_imu(stamp);
  ASSERT_GT(v_true.norm(), 1.0) << "moving";
  constexpr double kYawError = 60.0 * kDeg;
  const Eigen::Matrix3d R_error = Eigen::AngleAxisd(kYawError, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  run(odometry, imu_cursor, 0, kResetIndex);
  ASSERT_TRUE(odometry.reset(perturbed_pose(stamp, kYawError), R_error * v_true, stamp));

  glim::EstimationFrame::ConstPtr first;
  const auto stats = run(odometry, imu_cursor, kResetIndex, kResetIndex + 40, [&](size_t, const glim::EstimationFrame::ConstPtr& estimate) {
    if (!first && estimate) {
      first = estimate;
    }
  });
  const auto status = odometry.status();
  print("search, reset() while moving, yaw error 60 deg", stats, status);
  ASSERT_TRUE(first);
  EXPECT_EQ(status.init_search, InitSearchResult::ACCEPTED);
  const Eigen::Vector3d v_error = first->v_world_imu - synthetic_v_world_imu(first->stamp);
  std::cout << "first velocity error " << v_error.norm() << " m/s (unturned it would be about "
            << ((R_error - Eigen::Matrix3d::Identity()) * v_true).norm() << " m/s)" << std::endl;
  EXPECT_LT(v_error.norm(), 0.3);
  EXPECT_EQ(status.state, LocalizationState::TRACKING) << to_string(status.state);
  EXPECT_LT(stats.last_translation, kMaxTranslationError);
}

TEST_F(InitYawSearchTest, ImuGapSelfResetDoesNotSearch) {
  auto p = params(true);
  p.estimate_init_state = false;
  p.init_T_world_imu = perturbed_pose(0.0, 75.0 * kDeg);
  p.init_v_world_imu.setZero();
  OdometryEstimationLocalizationCPU odometry(p, *map_);
  size_t imu_cursor = 0;

  constexpr size_t kGapIndex = 40;
  run(odometry, imu_cursor, 0, kGapIndex);
  ASSERT_EQ(odometry.status().init_search, InitSearchResult::ACCEPTED);
  // Drop 0.2 s of IMU data (below imu_gap_reset_max): the odometry resets itself from its estimate
  while (imu_cursor < imu_->size() && (*imu_)[imu_cursor].stamp <= (*frames_)[kGapIndex + 1]->stamp + 0.02) {
    imu_cursor++;
  }
  const auto stats = run(odometry, imu_cursor, kGapIndex, kGapIndex + 30);
  const auto status = odometry.status();
  print("search, IMU gap self-reset", stats, status);
  EXPECT_EQ(status.reset_count, 1);
  EXPECT_EQ(status.last_reset_reason, ResetReason::IMU_GAP);
  EXPECT_EQ(status.init_search, InitSearchResult::NONE) << "a self-reset starts from the estimate";
  EXPECT_EQ(status.init_search_candidates, 0);
  EXPECT_EQ(status.state, LocalizationState::TRACKING) << to_string(status.state);
  EXPECT_LT(stats.last_translation, kMaxTranslationError);
}

TEST_F(InitYawSearchTest, NonIdentityLidarImuExtrinsic) {
  // The LiDAR turned by 90 deg and shifted on the IMU: the search scores the points in the IMU frame
  auto p = params(true);
  p.estimate_init_state = false;
  p.init_T_world_imu = perturbed_pose(0.0, 75.0 * kDeg);
  p.init_v_world_imu.setZero();
  Eigen::Isometry3d T_lidar_imu = Eigen::Isometry3d::Identity();
  T_lidar_imu.linear() = Eigen::AngleAxisd(90.0 * kDeg, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T_lidar_imu.translation() = Eigen::Vector3d(0.2, -0.1, 0.3);
  p.T_lidar_imu = T_lidar_imu;
  OdometryEstimationLocalizationCPU odometry(p, *map_);

  std::vector<glim::PreprocessedFrame::Ptr> lidar_frames;
  for (const auto& frame : *frames_) {
    auto copy = std::make_shared<glim::PreprocessedFrame>(*frame);
    for (auto& point : copy->points) {
      point = T_lidar_imu * point;  // The synthetic points are in the IMU frame
    }
    lidar_frames.push_back(copy);
  }
  // The fixture's frames are shared by the suite: swap them back even if run() throws
  struct SwapBack {
    std::vector<glim::PreprocessedFrame::Ptr>& a;
    std::vector<glim::PreprocessedFrame::Ptr>& b;
    ~SwapBack() { std::swap(a, b); }
  };
  std::swap(*frames_, lidar_frames);
  size_t imu_cursor = 0;
  const auto stats = [&] {
    SwapBack swap_back{*frames_, lidar_frames};
    return run(odometry, imu_cursor, 0, 60);
  }();
  const auto status = odometry.status();
  print("search, non-identity T_lidar_imu, yaw error 75 deg", stats, status);
  EXPECT_EQ(status.init_search, InitSearchResult::ACCEPTED);
  EXPECT_NEAR(status.init_search_yaw_offset / kDeg, -75.0, 1.0);
  EXPECT_EQ(status.state, LocalizationState::TRACKING) << to_string(status.state);
  EXPECT_LT(stats.max_translation, kMaxTranslationError);
  EXPECT_LT(stats.max_rotation_deg, kMaxRotationErrorDeg);
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
  p = params(true);
  p.init_yaw_search.min_points = 0;
  EXPECT_THROW(OdometryEstimationLocalizationCPU(p, *map_), std::invalid_argument);
  p = params(true);
  p.init_yaw_search.min_score_gap = -0.1;
  EXPECT_THROW(OdometryEstimationLocalizationCPU(p, *map_), std::invalid_argument);
  p = params(true);
  p.init_yaw_search.step = 0.4 * kDeg;  // Below the 0.5 deg floor
  EXPECT_THROW(OdometryEstimationLocalizationCPU(p, *map_), std::invalid_argument);
  for (const double max_translation : {-1.0, std::nan(""), std::numeric_limits<double>::infinity()}) {
    p = params(true);
    p.init_yaw_search.max_translation = max_translation;
    EXPECT_THROW(OdometryEstimationLocalizationCPU(p, *map_), std::invalid_argument) << max_translation;
  }
  // Validated also when disabled: a typo should not wait until someone turns the search on
  p = params(false);
  p.init_yaw_search.range = -1.0;
  EXPECT_THROW(OdometryEstimationLocalizationCPU(p, *map_), std::invalid_argument);
}
