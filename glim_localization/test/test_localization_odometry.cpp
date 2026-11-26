#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

#include <glim/common/imu_integration.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/util/config.hpp>

#include <glim_localization/map/map_manager.hpp>
#include <glim_localization/odometry/odometry_estimation_localization_cpu.hpp>

#include "synthetic/synthetic_map.hpp"
#include "synthetic/synthetic_scene.hpp"
#include "synthetic/synthetic_sensor.hpp"

namespace fs = std::filesystem;
using namespace glim_localization;
using namespace glim_localization::test;

namespace {

constexpr double kEndTime = 12.0;
constexpr double kSettleTime = 3.0;
constexpr double kMaxTranslationError = 0.1;
constexpr double kMaxRotationErrorDeg = 1.0;

Eigen::Isometry3d initial_error() {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(0.3, -0.2, 0.1);
  T.linear() = (Eigen::AngleAxisd(3.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0.5 * M_PI / 180.0, Eigen::Vector3d::UnitY()))
                 .toRotationMatrix();
  return T;
}

}  // namespace

class LocalizationOdometryTest : public testing::Test {
protected:
  static void SetUpTestSuite() {
    glim::GlobalConfig::instance(GLIM_LOCALIZATION_CONFIG_DIR, true);
    dir_ = new fs::path(fs::temp_directory_path() / ("glim_loc_odometry_" + std::to_string(::getpid())));
    fs::remove_all(*dir_);
    SyntheticMapParams map_params;
    map_params.num_submaps = 5;
    map_params.cross_walls = true;  // The plain corridor is nearly degenerate along x
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

  static fs::path* dir_;
  static std::shared_ptr<const MapManager>* map_;
  static std::vector<glim::PreprocessedFrame::Ptr>* frames_;
  static std::vector<SyntheticImuSample>* imu_;
};

fs::path* LocalizationOdometryTest::dir_ = nullptr;
std::shared_ptr<const MapManager>* LocalizationOdometryTest::map_ = nullptr;
std::vector<glim::PreprocessedFrame::Ptr>* LocalizationOdometryTest::frames_ = nullptr;
std::vector<SyntheticImuSample>* LocalizationOdometryTest::imu_ = nullptr;

TEST_F(LocalizationOdometryTest, ConvergesFromPerturbedInitialPose) {
  OdometryEstimationLocalizationCPU::Params params;
  params.estimate_init_state = false;
  params.init_T_world_imu = synthetic_T_world_imu(0.0) * initial_error();
  params.init_v_world_imu.setZero();
  params.num_threads = 2;
  params.map_factors.num_threads = 2;

  OdometryEstimationLocalizationCPU odometry(params, *map_);

  const auto start = std::chrono::steady_clock::now();
  size_t imu_cursor = 0;
  double max_translation = 0.0;
  double max_rotation_deg = 0.0;
  double sum_sq_translation = 0.0;
  int num_tail = 0;
  std::vector<double> latencies;

  for (const auto& frame : *frames_) {
    while (imu_cursor < imu_->size() && (*imu_)[imu_cursor].stamp <= frame->stamp + 0.02) {
      const auto& sample = (*imu_)[imu_cursor++];
      odometry.insert_imu(sample.stamp, sample.linear_acc, sample.angular_vel);
    }

    std::vector<glim::EstimationFrame::ConstPtr> marginalized;
    const auto estimate = odometry.insert_frame(frame, marginalized);
    ASSERT_TRUE(estimate) << "t=" << frame->stamp;
    ASSERT_TRUE(estimate->T_world_imu.matrix().allFinite()) << "t=" << frame->stamp;
    latencies.push_back(odometry.status().processing_time);

    const Eigen::Isometry3d error = synthetic_T_world_imu(frame->stamp).inverse() * estimate->T_world_imu;
    const double translation = error.translation().norm();
    const double rotation_deg = Eigen::AngleAxisd(error.linear()).angle() * 180.0 / M_PI;
    if (frame->stamp >= kSettleTime) {
      max_translation = std::max(max_translation, translation);
      max_rotation_deg = std::max(max_rotation_deg, rotation_deg);
      EXPECT_LT(translation, kMaxTranslationError) << "t=" << frame->stamp;
      EXPECT_LT(rotation_deg, kMaxRotationErrorDeg) << "t=" << frame->stamp;
    }
    if (frame->stamp >= kEndTime - 2.0) {
      sum_sq_translation += translation * translation;
      num_tail++;
    }
  }

  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  const auto status = odometry.status();
  std::cout << "max_translation=" << max_translation << " m, max_rotation=" << max_rotation_deg
            << " deg, tail_rms=" << std::sqrt(sum_sq_translation / std::max(num_tail, 1)) << " m, inlier_ratio=" << status.inlier_ratio
            << ", residual=" << status.residual << ", elapsed=" << elapsed << " s" << std::endl;
  // Per-frame latency of insert_frame, to compare with the GPU module (test_localization_odometry_gpu)
  double mean_latency = 0.0;
  for (const double t : latencies) {
    mean_latency += t;
  }
  mean_latency /= std::max<size_t>(latencies.size(), 1);
  std::sort(latencies.begin(), latencies.end());
  const double p95_latency = latencies.empty() ? 0.0 : latencies[std::min(latencies.size() - 1, latencies.size() * 95 / 100)];
  std::cout << "cpu: latency_mean=" << mean_latency * 1e3 << " ms, latency_p95=" << p95_latency * 1e3 << " ms, frames=" << latencies.size()
            << std::endl;

  EXPECT_EQ(status.state, LocalizationState::TRACKING) << to_string(status.state);
  EXPECT_GT(status.inlier_ratio, 0.5);
  EXPECT_GT(status.num_map_factors, 0);
  EXPECT_LT(elapsed, 60.0);  // Only flags gross slowness (it runs after the loop; ctest's TIMEOUT catches a hang). Parallel ctest and other containers slow this test.
}

namespace {

OdometryEstimationLocalizationCPU::Params reset_test_params() {
  OdometryEstimationLocalizationCPU::Params params;
  params.estimate_init_state = true;  // No configured pose: wait for reset()
  params.num_threads = 2;
  params.map_factors.num_threads = 2;
  return params;
}

void feed_imu(OdometryEstimationLocalizationCPU& odometry, const std::vector<SyntheticImuSample>& imu, size_t& cursor, double stamp) {
  while (cursor < imu.size() && imu[cursor].stamp <= stamp + 0.02) {
    odometry.insert_imu(imu[cursor].stamp, imu[cursor].linear_acc, imu[cursor].angular_vel);
    cursor++;
  }
}

double translation_error(const glim::EstimationFrame::ConstPtr& estimate) {
  return (synthetic_T_world_imu(estimate->stamp).inverse() * estimate->T_world_imu).translation().norm();
}

double rotation_error_deg(const glim::EstimationFrame::ConstPtr& estimate) {
  const Eigen::Isometry3d error = synthetic_T_world_imu(estimate->stamp).inverse() * estimate->T_world_imu;
  return Eigen::AngleAxisd(error.linear()).angle() * 180.0 / M_PI;
}

}  // namespace

TEST_F(LocalizationOdometryTest, WaitsForInitialPoseThenConverges) {
  OdometryEstimationLocalizationCPU odometry(reset_test_params(), *map_);
  size_t imu_cursor = 0;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;

  constexpr size_t kResetIndex = 5;  // The sensor is still at rest
  EXPECT_FALSE(odometry.last_input_frame_stamp());
  for (size_t i = 0; i < kResetIndex; i++) {
    feed_imu(odometry, *imu_, imu_cursor, (*frames_)[i]->stamp);
    EXPECT_FALSE(odometry.insert_frame((*frames_)[i], marginalized));
    // Dropped frames count too: the late-pose check of R2 relies on it before the first pose
    ASSERT_TRUE(odometry.last_input_frame_stamp());
    EXPECT_EQ(*odometry.last_input_frame_stamp(), (*frames_)[i]->stamp);
  }
  EXPECT_EQ(odometry.status().state, LocalizationState::INITIALIZING);
  EXPECT_EQ(odometry.status().reset_count, 0);

  const double reset_stamp = (*frames_)[kResetIndex]->stamp;
  EXPECT_FALSE(odometry.reset_pending());
  odometry.reset(synthetic_T_world_imu(reset_stamp) * initial_error(), Eigen::Vector3d::Zero(), reset_stamp);
  EXPECT_TRUE(odometry.reset_pending());
  for (size_t i = kResetIndex; i < frames_->size(); i++) {
    feed_imu(odometry, *imu_, imu_cursor, (*frames_)[i]->stamp);
    const auto estimate = odometry.insert_frame((*frames_)[i], marginalized);
    ASSERT_TRUE(estimate) << "t=" << (*frames_)[i]->stamp;
    EXPECT_FALSE(odometry.reset_pending());
    if (estimate->stamp >= reset_stamp + kSettleTime) {
      EXPECT_LT(translation_error(estimate), kMaxTranslationError) << "t=" << estimate->stamp;
      EXPECT_LT(rotation_error_deg(estimate), kMaxRotationErrorDeg) << "t=" << estimate->stamp;
    }
  }
  EXPECT_EQ(odometry.status().state, LocalizationState::TRACKING);
  EXPECT_EQ(odometry.status().reset_count, 1);
}

TEST_F(LocalizationOdometryTest, ResetsWhileRunning) {
  auto params = reset_test_params();
  params.estimate_init_state = false;
  params.init_T_world_imu = synthetic_T_world_imu(0.0);
  params.init_v_world_imu.setZero();
  OdometryEstimationLocalizationCPU odometry(params, *map_);
  size_t imu_cursor = 0;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;

  const size_t reset_index = frames_->size() / 2;
  const double reset_stamp = (*frames_)[reset_index]->stamp;
  for (size_t i = 0; i < frames_->size(); i++) {
    if (i == reset_index) {
      // A pose request carries no velocity, so restart from zero while moving.
      odometry.reset(synthetic_T_world_imu(reset_stamp) * initial_error(), Eigen::Vector3d::Zero(), reset_stamp);
    }
    feed_imu(odometry, *imu_, imu_cursor, (*frames_)[i]->stamp);
    const auto estimate = odometry.insert_frame((*frames_)[i], marginalized);
    ASSERT_TRUE(estimate) << "t=" << (*frames_)[i]->stamp;
    ASSERT_TRUE(estimate->T_world_imu.matrix().allFinite()) << "t=" << estimate->stamp;
    if (i == reset_index) {
      EXPECT_EQ(estimate->id, 0);
    }
    if (estimate->stamp >= reset_stamp + kSettleTime) {
      EXPECT_LT(translation_error(estimate), kMaxTranslationError) << "t=" << estimate->stamp;
      EXPECT_LT(rotation_error_deg(estimate), kMaxRotationErrorDeg) << "t=" << estimate->stamp;
    }
  }
  EXPECT_EQ(odometry.status().state, LocalizationState::TRACKING);
  EXPECT_EQ(odometry.status().reset_count, 1);
}

TEST_F(LocalizationOdometryTest, RelocalizesFromTheNewestEstimate) {
  auto params = reset_test_params();
  params.estimate_init_state = false;
  params.init_T_world_imu = synthetic_T_world_imu(0.0);
  params.init_v_world_imu.setZero();
  OdometryEstimationLocalizationCPU odometry(params, *map_);
  size_t imu_cursor = 0;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;

  EXPECT_NE(odometry.relocalize(0.2), "");  // No estimate yet
  const size_t relocalize_index = frames_->size() / 2;
  for (size_t i = 0; i < frames_->size(); i++) {
    if (i == relocalize_index) {
      ASSERT_EQ(odometry.status().state, LocalizationState::TRACKING);
      EXPECT_EQ(odometry.relocalize(0.2), "");
      EXPECT_TRUE(odometry.reset_pending());
      EXPECT_EQ(odometry.relocalize(0.2), "a reset is pending");
    }
    feed_imu(odometry, *imu_, imu_cursor, (*frames_)[i]->stamp);
    const auto estimate = odometry.insert_frame((*frames_)[i], marginalized);
    ASSERT_TRUE(estimate) << "t=" << (*frames_)[i]->stamp;
    if (i == relocalize_index) {
      EXPECT_EQ(estimate->id, 0);
      // Moved and turned to this frame: as good as tracking, unlike a restart from the previous frame's pose
      EXPECT_LT(translation_error(estimate), kMaxTranslationError) << "t=" << estimate->stamp;
      EXPECT_LT(rotation_error_deg(estimate), kMaxRotationErrorDeg) << "t=" << estimate->stamp;
    }
    if (i >= relocalize_index && estimate->stamp >= (*frames_)[relocalize_index]->stamp + kSettleTime) {
      EXPECT_LT(translation_error(estimate), kMaxTranslationError) << "t=" << estimate->stamp;
      EXPECT_LT(rotation_error_deg(estimate), kMaxRotationErrorDeg) << "t=" << estimate->stamp;
    }
  }
  EXPECT_EQ(odometry.status().reset_count, 1);
  EXPECT_EQ(odometry.status().last_reset_reason, ResetReason::EXTERNAL);
  // An estimate older than max_age before the last input frame is refused
  EXPECT_NE(odometry.relocalize(-1.0), "");
}

TEST_F(LocalizationOdometryTest, RelocalizeIsDroppedAfterAStall) {
  auto params = reset_test_params();
  params.estimate_init_state = false;
  params.init_T_world_imu = synthetic_T_world_imu(0.0);
  params.init_v_world_imu.setZero();
  OdometryEstimationLocalizationCPU odometry(params, *map_);
  size_t imu_cursor = 0;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;

  const size_t request_index = frames_->size() / 2;
  constexpr size_t kSkipped = 5;  // 0.6 s from the newest estimate to the next frame, max_age 0.2 s
  for (size_t i = 0; i < frames_->size(); i++) {
    if (i == request_index) {
      ASSERT_EQ(odometry.status().state, LocalizationState::TRACKING);
      ASSERT_EQ(odometry.relocalize(0.2), "");
    }
    if (i >= request_index && i < request_index + kSkipped) {
      continue;  // The LiDAR stalls; the IMU does not, so there is no IMU gap
    }
    feed_imu(odometry, *imu_, imu_cursor, (*frames_)[i]->stamp);
    const auto estimate = odometry.insert_frame((*frames_)[i], marginalized);
    ASSERT_TRUE(estimate) << "t=" << (*frames_)[i]->stamp;
    if (i == 0) {
      // R2-F4: the first frames are INITIALIZING, and relocalize is refused there
      ASSERT_EQ(odometry.status().state, LocalizationState::INITIALIZING);
      EXPECT_EQ(odometry.relocalize(0.2), "the state is INITIALIZING, not TRACKING or DEGRADED");
    }
    if (i == request_index + kSkipped) {
      EXPECT_FALSE(odometry.reset_pending());
      EXPECT_NE(estimate->id, 0) << "the relocalization was applied after the stall";
    }
  }
  EXPECT_EQ(odometry.status().reset_count, 0);
}

TEST_F(LocalizationOdometryTest, RelocalizeIsDroppedWhenLostBeforeItApplies) {
  auto params = reset_test_params();
  params.estimate_init_state = false;
  params.init_T_world_imu = synthetic_T_world_imu(0.0);
  params.init_v_world_imu.setZero();
  OdometryEstimationLocalizationCPU odometry(params, *map_);
  size_t imu_cursor = 0;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;

  const size_t request_index = frames_->size() / 2;
  for (size_t i = 0; i < request_index; i++) {
    feed_imu(odometry, *imu_, imu_cursor, (*frames_)[i]->stamp);
    ASSERT_TRUE(odometry.insert_frame((*frames_)[i], marginalized));
  }
  ASSERT_EQ(odometry.status().state, LocalizationState::TRACKING);
  ASSERT_EQ(odometry.relocalize(0.2), "");

  // An IMU fault before the next frame: repeated stamps make the odometry LOST
  ASSERT_GT(imu_cursor, 0u);
  const auto& repeated = (*imu_)[imu_cursor - 1];
  for (int k = 0; k < 5; k++) {
    odometry.insert_imu(repeated.stamp, repeated.linear_acc, repeated.angular_vel);
  }
  ASSERT_EQ(odometry.status().state, LocalizationState::LOST);

  for (size_t i = request_index; i < request_index + 5; i++) {
    feed_imu(odometry, *imu_, imu_cursor, (*frames_)[i]->stamp);
    EXPECT_FALSE(odometry.insert_frame((*frames_)[i], marginalized)) << "the relocalization cleared LOST at t=" << (*frames_)[i]->stamp;
    EXPECT_FALSE(odometry.reset_pending());
  }
  EXPECT_EQ(odometry.status().state, LocalizationState::LOST);
  EXPECT_EQ(odometry.status().reset_count, 0);
  // R2-F4: refused while LOST
  EXPECT_EQ(odometry.relocalize(0.2), "the state is LOST, not TRACKING or DEGRADED");
}

TEST_F(LocalizationOdometryTest, DropsFramesBeforeMinStamp) {
  OdometryEstimationLocalizationCPU odometry(reset_test_params(), *map_);
  size_t imu_cursor = 0;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;

  constexpr size_t kFirstAccepted = 3;
  odometry.reset(synthetic_T_world_imu(0.0), Eigen::Vector3d::Zero(), (*frames_)[kFirstAccepted]->stamp);
  for (size_t i = 0; i <= kFirstAccepted; i++) {
    feed_imu(odometry, *imu_, imu_cursor, (*frames_)[i]->stamp);
    const auto estimate = odometry.insert_frame((*frames_)[i], marginalized);
    EXPECT_EQ(static_cast<bool>(estimate), i == kFirstAccepted) << "i=" << i;
  }
}

TEST_F(LocalizationOdometryTest, ResetCallbackAndStatusNotification) {
  OdometryEstimationLocalizationCPU odometry(reset_test_params(), *map_);
  std::vector<LocalizationStatus> statuses;
  const int callback_id = LocalizationCallbacks::on_status.add([&](const LocalizationStatus& status) { statuses.push_back(status); });

  size_t imu_cursor = 0;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;
  LocalizationCallbacks::request_reset(synthetic_T_world_imu(0.0), Eigen::Vector3d::Zero(), 0.0);
  for (size_t i = 0; i < 3; i++) {
    feed_imu(odometry, *imu_, imu_cursor, (*frames_)[i]->stamp);
    EXPECT_TRUE(odometry.insert_frame((*frames_)[i], marginalized));
  }
  LocalizationCallbacks::on_status.remove(callback_id);

  ASSERT_EQ(statuses.size(), 4u);  // The reset, then one per frame
  EXPECT_EQ(statuses.front().state, LocalizationState::INITIALIZING);
  EXPECT_EQ(statuses.front().frame_id, -1);
  EXPECT_EQ(statuses.back().frame_id, 2);
  EXPECT_EQ(statuses.back().reset_count, 1);
}

TEST_F(LocalizationOdometryTest, LoadedMapIsTheMapOfTheOldestLiveInstance) {
  EXPECT_EQ(LocalizationCallbacks::loaded_map(), nullptr);
  auto other_map = std::make_shared<MapManager>();
  {
    auto first = std::make_unique<OdometryEstimationLocalizationCPU>(reset_test_params(), *map_);
    EXPECT_EQ(first->map(), *map_);
    EXPECT_EQ(LocalizationCallbacks::loaded_map(), *map_);
    OdometryEstimationLocalizationCPU second(reset_test_params(), other_map);
    EXPECT_EQ(LocalizationCallbacks::loaded_map(), *map_);
    first.reset();
    EXPECT_EQ(LocalizationCallbacks::loaded_map(), other_map);
  }
  EXPECT_EQ(LocalizationCallbacks::loaded_map(), nullptr);
}

TEST_F(LocalizationOdometryTest, BecomesLostWhenMapIsFarAway) {
  const fs::path far_dir = fs::temp_directory_path() / ("glim_loc_odometry_far_" + std::to_string(::getpid()));
  fs::remove_all(far_dir);
  SyntheticMapParams map_params;
  map_params.num_submaps = 5;
  map_params.cross_walls = true;
  map_params.world_offset = Eigen::Vector3d(1000.0, 0.0, 0.0);
  write_synthetic_dump(far_dir.string(), map_params);
  auto far_map = std::make_shared<MapManager>();
  ASSERT_TRUE(far_map->load(far_dir.string()));

  auto params = reset_test_params();
  params.quality.lost_frames = 10;
  OdometryEstimationLocalizationCPU odometry(params, far_map);
  odometry.reset(synthetic_T_world_imu(0.0));

  size_t imu_cursor = 0;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;
  for (size_t i = 0; i < 15; i++) {
    feed_imu(odometry, *imu_, imu_cursor, (*frames_)[i]->stamp);
    const auto estimate = odometry.insert_frame((*frames_)[i], marginalized);
    ASSERT_TRUE(estimate);
    ASSERT_TRUE(estimate->T_world_imu.matrix().allFinite());
    EXPECT_EQ(odometry.status().state == LocalizationState::LOST, i >= 9) << "i=" << i;
  }
  EXPECT_EQ(odometry.status().num_map_factors, 0);
  EXPECT_DOUBLE_EQ(odometry.status().inlier_ratio, 0.0);
  fs::remove_all(far_dir);
}

TEST_F(LocalizationOdometryTest, ConvergesWithLocalModel) {
  auto params = reset_test_params();
  params.use_keyframes = true;
  params.estimate_init_state = false;
  params.init_T_world_imu = synthetic_T_world_imu(0.0) * initial_error();
  params.init_v_world_imu.setZero();
  OdometryEstimationLocalizationCPU odometry(params, *map_);

  size_t imu_cursor = 0;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;
  for (const auto& frame : *frames_) {
    feed_imu(odometry, *imu_, imu_cursor, frame->stamp);
    const auto estimate = odometry.insert_frame(frame, marginalized);
    ASSERT_TRUE(estimate) << "t=" << frame->stamp;
    ASSERT_TRUE(estimate->T_world_imu.matrix().allFinite()) << "t=" << frame->stamp;
    if (estimate->stamp >= kSettleTime) {
      EXPECT_LT(translation_error(estimate), kMaxTranslationError) << "t=" << estimate->stamp;
      EXPECT_LT(rotation_error_deg(estimate), kMaxRotationErrorDeg) << "t=" << estimate->stamp;
    }
  }
  EXPECT_EQ(odometry.status().state, LocalizationState::TRACKING);
}

TEST_F(LocalizationOdometryTest, ResetsWhileRunningWithLocalModel) {
  auto params = reset_test_params();
  params.use_keyframes = true;
  params.estimate_init_state = false;
  params.init_T_world_imu = synthetic_T_world_imu(0.0);
  params.init_v_world_imu.setZero();
  OdometryEstimationLocalizationCPU odometry(params, *map_);

  size_t imu_cursor = 0;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;
  const size_t reset_index = frames_->size() / 2;
  const double reset_stamp = (*frames_)[reset_index]->stamp;
  for (size_t i = 0; i < frames_->size(); i++) {
    if (i == reset_index) {
      odometry.reset(synthetic_T_world_imu(reset_stamp) * initial_error(), Eigen::Vector3d::Zero(), reset_stamp);
    }
    feed_imu(odometry, *imu_, imu_cursor, (*frames_)[i]->stamp);
    const auto estimate = odometry.insert_frame((*frames_)[i], marginalized);
    ASSERT_TRUE(estimate) << "t=" << (*frames_)[i]->stamp;
    ASSERT_TRUE(estimate->T_world_imu.matrix().allFinite()) << "t=" << estimate->stamp;
    if (estimate->stamp >= reset_stamp + kSettleTime) {
      EXPECT_LT(translation_error(estimate), kMaxTranslationError) << "t=" << estimate->stamp;
      EXPECT_LT(rotation_error_deg(estimate), kMaxRotationErrorDeg) << "t=" << estimate->stamp;
    }
  }
  EXPECT_EQ(odometry.status().state, LocalizationState::TRACKING);
  EXPECT_EQ(odometry.status().reset_count, 1);
}

TEST_F(LocalizationOdometryTest, RejectsInvalidLocalModelParams) {
  auto params = reset_test_params();
  params.use_keyframes = true;
  params.vgicp_voxelmap_levels = 0;
  EXPECT_THROW(OdometryEstimationLocalizationCPU(params, *map_), std::invalid_argument);
}

TEST_F(LocalizationOdometryTest, RejectsInvalidResetRequests) {
  auto params = reset_test_params();
  params.estimate_init_state = false;
  params.init_T_world_imu = synthetic_T_world_imu(0.0);
  params.init_v_world_imu.setZero();
  OdometryEstimationLocalizationCPU odometry(params, *map_);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  Eigen::Isometry3d non_finite = Eigen::Isometry3d::Identity();
  non_finite.translation().x() = nan;
  Eigen::Isometry3d scaled = Eigen::Isometry3d::Identity();
  scaled.linear() *= 2.0;
  Eigen::Isometry3d mirrored = Eigen::Isometry3d::Identity();
  mirrored.linear()(2, 2) = -1.0;

  EXPECT_FALSE(odometry.reset(non_finite));
  EXPECT_FALSE(odometry.reset(scaled));
  EXPECT_FALSE(odometry.reset(mirrored));
  EXPECT_FALSE(odometry.reset(Eigen::Isometry3d::Identity(), Eigen::Vector3d::Constant(nan)));
  EXPECT_FALSE(odometry.reset(Eigen::Isometry3d::Identity(), Eigen::Vector3d::Zero(), nan));
  // Finite but huge velocities made the smoother throw IndeterminantLinearSystemException and abort the process
  EXPECT_FALSE(odometry.reset(Eigen::Isometry3d::Identity(), Eigen::Vector3d(1e50, 0.0, 0.0)));
  EXPECT_FALSE(odometry.reset(Eigen::Isometry3d::Identity(), Eigen::Vector3d(0.0, 0.0, OdometryEstimationLocalization::kMaxResetSpeed + 1.0)));
  EXPECT_FALSE(odometry.reset_pending());

  size_t imu_cursor = 0;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;
  feed_imu(odometry, *imu_, imu_cursor, (*frames_)[0]->stamp);
  EXPECT_TRUE(odometry.insert_frame((*frames_)[0], marginalized));
  EXPECT_EQ(odometry.status().reset_count, 0);
}

TEST_F(LocalizationOdometryTest, ImuQueueStaysBoundedWhileWaiting) {
  struct Probe : public OdometryEstimationLocalizationCPU {
    using OdometryEstimationLocalizationCPU::OdometryEstimationLocalizationCPU;
    double queue_span() const {
      const auto& queue = imu_integration->imu_data_in_queue();
      return queue.empty() ? 0.0 : queue.back()[0] - queue.front()[0];
    }
  };
  Probe odometry(reset_test_params(), *map_);

  size_t imu_cursor = 0;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;
  for (const auto& frame : *frames_) {
    feed_imu(odometry, *imu_, imu_cursor, frame->stamp);
    EXPECT_FALSE(odometry.insert_frame(frame, marginalized));
    EXPECT_LT(odometry.queue_span(), 1.5) << "t=" << frame->stamp;
  }
}

TEST_F(LocalizationOdometryTest, RequestResetIsSafeDuringInstanceLifetime) {
  std::atomic<bool> done{false};
  std::thread requester([&] {
    while (!done) {
      LocalizationCallbacks::request_reset(synthetic_T_world_imu(0.0), Eigen::Vector3d::Zero(), 0.0);
    }
  });
  const auto params = reset_test_params();
  for (int i = 0; i < 200; i++) {
    OdometryEstimationLocalizationCPU odometry(params, *map_);
  }
  done = true;
  requester.join();
}

TEST_F(LocalizationOdometryTest, ResidualDoesNotScaleWithTileCount) {
  const auto final_residual = [&](int max_submaps) {
    auto params = reset_test_params();
    params.estimate_init_state = false;
    params.init_T_world_imu = synthetic_T_world_imu(0.0);
    params.init_v_world_imu.setZero();
    params.map_factors.max_submaps = max_submaps;
    OdometryEstimationLocalizationCPU odometry(params, *map_);
    size_t imu_cursor = 0;
    std::vector<glim::EstimationFrame::ConstPtr> marginalized;
    for (size_t i = 0; i < 40; i++) {
      feed_imu(odometry, *imu_, imu_cursor, (*frames_)[i]->stamp);
      odometry.insert_frame((*frames_)[i], marginalized);
    }
    EXPECT_EQ(odometry.status().num_tiles, max_submaps);
    return odometry.status().residual;
  };
  const double one = final_residual(1);
  const double three = final_residual(3);
  std::cout << "residual one_tile=" << one << " three_tiles=" << three << std::endl;
  EXPECT_GT(one, 0.0);
  EXPECT_LT(std::abs(three - one), 0.3 * one);
}

TEST_F(LocalizationOdometryTest, RejectsNonPositiveDegradedWeight) {
  auto params = reset_test_params();
  params.degraded_weight_scale = 0.0;
  EXPECT_THROW(OdometryEstimationLocalizationCPU(params, *map_), std::invalid_argument);
}

TEST_F(LocalizationOdometryTest, AcceptsAResetAtTheSpeedLimit) {
  OdometryEstimationLocalizationCPU odometry(reset_test_params(), *map_);
  const double stamp = (*frames_)[0]->stamp;
  EXPECT_TRUE(odometry.reset(synthetic_T_world_imu(stamp), Eigen::Vector3d(OdometryEstimationLocalization::kMaxResetSpeed, 0.0, 0.0), stamp));
  EXPECT_TRUE(odometry.reset_pending());

  // The limit itself must be safe to apply: the smoother solves the first frames without throwing
  size_t imu_cursor = 0;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;
  for (size_t i = 0; i < 3; i++) {
    feed_imu(odometry, *imu_, imu_cursor, (*frames_)[i]->stamp);
    const auto estimate = odometry.insert_frame((*frames_)[i], marginalized);
    ASSERT_TRUE(estimate) << "i=" << i;
    EXPECT_TRUE(estimate->T_world_imu.matrix().allFinite()) << "i=" << i;
  }
  EXPECT_EQ(odometry.status().reset_count, 1);
}

TEST_F(LocalizationOdometryTest, RejectsInvalidConfiguredInitialState) {
  {
    auto params = reset_test_params();
    params.init_v_without_pose = true;  // Upstream would start at the identity pose in the map
    EXPECT_THROW(OdometryEstimationLocalizationCPU(params, *map_), std::invalid_argument);
  }
  {
    auto params = reset_test_params();
    params.estimate_init_state = false;
    params.init_T_world_imu = synthetic_T_world_imu(0.0);
    params.init_v_world_imu = Eigen::Vector3d(1e50, 0.0, 0.0);
    EXPECT_THROW(OdometryEstimationLocalizationCPU(params, *map_), std::invalid_argument);
  }
  {
    auto params = reset_test_params();
    params.estimate_init_state = false;
    params.init_T_world_imu = synthetic_T_world_imu(0.0);
    params.init_T_world_imu.linear() *= 2.0;
    params.init_v_world_imu.setZero();
    EXPECT_THROW(OdometryEstimationLocalizationCPU(params, *map_), std::invalid_argument);
  }
}

TEST_F(LocalizationOdometryTest, SecondResetBeforeAnyFrameKeepsTheEstimatedBias) {
  auto params = reset_test_params();
  params.estimate_init_state = false;
  params.init_T_world_imu = synthetic_T_world_imu(0.0);
  params.init_v_world_imu.setZero();
  params.imu_bias.setZero();
  params.imu_bias[0] = 0.02;  // A small wrong accelerometer bias, so the estimate moves away from it
  const Eigen::Matrix<double, 6, 1> configured = params.imu_bias;
  OdometryEstimationLocalizationCPU odometry(params, *map_);
  EXPECT_EQ(odometry.start_imu_bias(), configured);
  size_t imu_cursor = 0;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;

  constexpr size_t kFirstReset = 30;
  for (size_t i = 0; i < kFirstReset; i++) {
    feed_imu(odometry, *imu_, imu_cursor, (*frames_)[i]->stamp);
    ASSERT_TRUE(odometry.insert_frame((*frames_)[i], marginalized)) << "i=" << i;
  }

  // The first reset applies at the next frame, which it drops (min_stamp is later): no frame of the new run is estimated
  const double second_stamp = (*frames_)[kFirstReset + 1]->stamp;
  ASSERT_TRUE(odometry.reset(synthetic_T_world_imu(second_stamp), Eigen::Vector3d::Zero(), second_stamp + 1e-3));
  feed_imu(odometry, *imu_, imu_cursor, (*frames_)[kFirstReset]->stamp);
  EXPECT_FALSE(odometry.insert_frame((*frames_)[kFirstReset], marginalized));
  EXPECT_EQ(odometry.status().reset_count, 1);
  const Eigen::Matrix<double, 6, 1> estimated = odometry.start_imu_bias();
  ASSERT_TRUE(estimated.allFinite());
  EXPECT_GT((estimated - configured).norm(), 1e-9) << "the first run did not move the bias, so this test checks nothing";

  // The second reset comes before any frame of the first one's run: it must not fall back to the configured bias
  ASSERT_TRUE(odometry.reset(synthetic_T_world_imu(second_stamp), Eigen::Vector3d::Zero(), second_stamp));
  feed_imu(odometry, *imu_, imu_cursor, second_stamp);
  const auto estimate = odometry.insert_frame((*frames_)[kFirstReset + 1], marginalized);
  ASSERT_TRUE(estimate);
  EXPECT_EQ(odometry.status().reset_count, 2);
  EXPECT_EQ(odometry.start_imu_bias(), estimated);
  // The smoother starts from it too: the first estimate of the new run is near the carried bias, not the configured one
  EXPECT_LT((estimate->imu_bias - estimated).norm(), (estimate->imu_bias - configured).norm());
}

TEST_F(LocalizationOdometryTest, RejectsNullMap) {
  OdometryEstimationLocalizationCPU::Params params;
  EXPECT_THROW(OdometryEstimationLocalizationCPU(params, nullptr), std::invalid_argument);
}

TEST_F(LocalizationOdometryTest, RejectsMissingMapPath) {
  OdometryEstimationLocalizationCPU::Params params;
  params.map_path = (*dir_ / "does_not_exist").string();
  EXPECT_THROW(OdometryEstimationLocalizationCPU{params}, std::runtime_error);
}
