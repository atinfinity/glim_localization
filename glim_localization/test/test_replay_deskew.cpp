// Rolling-shutter scans: every point is observed at its own time along the true trajectory, so the scan is only
// consistent with the map if the odometry deskews it with the IMU. The same points without per-point times are the
// negative control: they show that the accuracy comes from deskewing.
#include <algorithm>
#include <cmath>
#include <filesystem>
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
#include <glim_localization/replay/replay_runner.hpp>

#include "synthetic/synthetic_map.hpp"
#include "synthetic/synthetic_replay.hpp"
#include "synthetic/synthetic_sensor.hpp"

namespace fs = std::filesystem;
using namespace glim_localization;
using namespace glim_localization::test;

namespace {

constexpr double kEndTime = 12.0;
constexpr double kSettleTime = 3.0;
constexpr double kMaxTranslationError = 0.1;  // The localization acceptance limits
constexpr double kMaxRotationErrorDeg = 1.0;
constexpr double kScanDuration = 0.1;  // A full sweep per frame, like a spinning lidar at 10 Hz

Eigen::Isometry3d initial_error() {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(0.3, -0.2, 0.1);
  T.linear() = (Eigen::AngleAxisd(3.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.5 * M_PI / 180.0, Eigen::Vector3d::UnitY()))
                 .toRotationMatrix();
  return T;
}

using Records = std::vector<ReplayFrameRecord, Eigen::aligned_allocator<ReplayFrameRecord>>;

struct Errors {
  double translation = 0.0;
  double rotation_deg = 0.0;
  int num_estimated = 0;
};

Errors measure(const Records& records, double from) {
  Errors errors;
  for (const auto& record : records) {
    if (record.result != ReplayResult::ESTIMATED) {
      continue;
    }
    errors.num_estimated++;
    const double t = record.stamp_ns * 1e-9;
    if (t < from) {
      continue;
    }
    const Eigen::Isometry3d error = synthetic_T_world_imu(t).inverse() * *record.T_world_imu;
    errors.translation = std::max(errors.translation, error.translation().norm());
    errors.rotation_deg = std::max(errors.rotation_deg, Eigen::AngleAxisd(error.linear()).angle() * 180.0 / M_PI);
  }
  return errors;
}

}  // namespace

class ReplayDeskewTest : public testing::Test {
protected:
  static void SetUpTestSuite() {
    glim::GlobalConfig::instance(GLIM_LOCALIZATION_CONFIG_DIR, true);
    dir_ = new fs::path(fs::temp_directory_path() / ("glim_loc_replay_deskew_" + std::to_string(::getpid())));
    fs::remove_all(*dir_);
    const auto map_params = replay_map_params();
    write_synthetic_dump((*dir_ / "map").string(), map_params);
    map_ = new std::shared_ptr<const MapManager>();
    auto map = std::make_shared<MapManager>();
    ASSERT_TRUE(map->load((*dir_ / "map").string()));
    *map_ = map;
    for (const bool with_times : {true, false}) {
      SyntheticReplayOptions options;
      options.lidar.scan_duration = kScanDuration;
      options.write_point_times = with_times;
      const fs::path dir = *dir_ / (with_times ? "skewed" : "skewed_no_times");
      write_synthetic_replay(dir.string(), kEndTime, map_params, options);
      (with_times ? skewed_ : skewed_no_times_) = new ReplayDataset(ReplayDataset::open(dir.string()));
    }
  }

  static void TearDownTestSuite() {
    delete skewed_;
    delete skewed_no_times_;
    delete map_;
    fs::remove_all(*dir_);
    delete dir_;
  }

  static Records run(const ReplayDataset& dataset, const OdometryEstimationLocalizationCPU::Params& params, const ReplayOptions& options = ReplayOptions()) {
    glim::CloudPreprocessor preprocessor;
    OdometryEstimationLocalizationCPU odometry(params, *map_);
    ReplayRunner runner(dataset, odometry, preprocessor, options);
    Records records;
    runner.run([&](const ReplayFrameRecord& record) { records.push_back(record); });
    return records;
  }

  static OdometryEstimationLocalizationCPU::Params perturbed_start_params() {
    OdometryEstimationLocalizationCPU::Params params;
    params.num_threads = 2;
    params.map_factors.num_threads = 2;
    params.estimate_init_state = false;
    params.init_T_world_imu = synthetic_T_world_imu(0.0) * initial_error();
    params.init_v_world_imu.setZero();
    return params;
  }

  static ReplayOptions init_stamp_options(double stamp) {
    ReplayOptions options;
    options.initial_reset = ReplayOptions::InitialReset();
    options.initial_reset->T_world_imu = synthetic_T_world_imu(stamp) * initial_error();
    options.initial_reset->v_world_imu = synthetic_v_world_imu(stamp);
    options.initial_reset->min_stamp = stamp;
    return options;
  }

  static fs::path* dir_;
  static std::shared_ptr<const MapManager>* map_;
  static ReplayDataset* skewed_;
  static ReplayDataset* skewed_no_times_;
};

fs::path* ReplayDeskewTest::dir_ = nullptr;
std::shared_ptr<const MapManager>* ReplayDeskewTest::map_ = nullptr;
ReplayDataset* ReplayDeskewTest::skewed_ = nullptr;
ReplayDataset* ReplayDeskewTest::skewed_no_times_ = nullptr;

TEST_F(ReplayDeskewTest, DatasetCarriesPerPointTimes) {
  EXPECT_TRUE(skewed_->meta().per_point_time);
  EXPECT_FALSE(skewed_no_times_->meta().per_point_time);
  const auto cloud = skewed_->load_cloud(20);
  const auto [t_min, t_max] = std::minmax_element(cloud->times.begin(), cloud->times.end());
  EXPECT_DOUBLE_EQ(*t_min, 0.0);
  EXPECT_GT(*t_max, 0.9 * kScanDuration);
  EXPECT_LT(*t_max, kScanDuration);
  // The skew is real: at 3 s the motion over one sweep is large compared to the 0.1 m accuracy limit
  const double stamp = skewed_->frames()[30].stamp_ns * 1e-9;
  const double shift = (synthetic_T_world_imu(stamp + kScanDuration).translation() - synthetic_T_world_imu(stamp).translation()).norm();
  const double turn_deg = Eigen::AngleAxisd(synthetic_T_world_imu(stamp).linear().transpose() * synthetic_T_world_imu(stamp + kScanDuration).linear()).angle() * 180.0 / M_PI;
  EXPECT_GT(shift, 0.1);
  EXPECT_GT(turn_deg, 1.0);
}

TEST_F(ReplayDeskewTest, DeskewedScansMeetAccuracy) {
  const auto records = run(*skewed_, perturbed_start_params());
  ASSERT_EQ(records.size(), skewed_->frames().size());
  const auto errors = measure(records, kSettleTime);
  EXPECT_EQ(errors.num_estimated, static_cast<int>(records.size()));
  std::cout << "[deskew] with times:    translation " << errors.translation << " m, rotation " << errors.rotation_deg << " deg" << std::endl;
  EXPECT_LT(errors.translation, kMaxTranslationError);
  EXPECT_LT(errors.rotation_deg, kMaxRotationErrorDeg);
}

TEST_F(ReplayDeskewTest, WithoutPointTimesTheSameScansAreWorse) {
  const auto with_times = measure(run(*skewed_, perturbed_start_params()), kSettleTime);
  const auto without_times = measure(run(*skewed_no_times_, perturbed_start_params()), kSettleTime);
  std::cout << "[deskew] without times: translation " << without_times.translation << " m, rotation " << without_times.rotation_deg << " deg"
            << std::endl;
  EXPECT_GT(without_times.translation, kMaxTranslationError);
  EXPECT_GT(without_times.rotation_deg, kMaxRotationErrorDeg);
  EXPECT_GT(without_times.translation, 2.0 * with_times.translation);
}

TEST_F(ReplayDeskewTest, DeskewsAfterImuTrimmingWhileWaitingForAPose) {
  // Frames before the stamp are dropped while the odometry trims its IMU queue. Upstream inserts the first
  // frame after the stamp without deskewing, so deskewing after the trim starts at the second frame.
  OdometryEstimationLocalizationCPU::Params params;
  params.num_threads = 2;
  params.map_factors.num_threads = 2;
  params.estimate_init_state = true;
  const double stamp = 5.0;
  const auto records = run(*skewed_, params, init_stamp_options(stamp));
  for (const auto& record : records) {
    EXPECT_EQ(record.result, record.stamp_ns * 1e-9 < stamp - 1e-6 ? ReplayResult::DROPPED : ReplayResult::ESTIMATED) << record.stamp_ns;
  }
  const auto errors = measure(records, stamp + kSettleTime);
  std::cout << "[deskew] late init-stamp: translation " << errors.translation << " m, rotation " << errors.rotation_deg << " deg" << std::endl;
  EXPECT_LT(errors.translation, kMaxTranslationError);
  EXPECT_LT(errors.rotation_deg, kMaxRotationErrorDeg);
  // Trimming can only affect the frames right after the stamp, so check them too, skipping only the first frame
  const auto after_trim = measure(records, stamp + 0.05);
  std::cout << "[deskew] right after trim: translation " << after_trim.translation << " m, rotation " << after_trim.rotation_deg << " deg" << std::endl;
  EXPECT_LT(after_trim.translation, kMaxTranslationError);
  EXPECT_LT(after_trim.rotation_deg, kMaxRotationErrorDeg);
  // The same late start without the per-point times is clearly worse, so this test does depend on deskewing
  const auto no_times = measure(run(*skewed_no_times_, params, init_stamp_options(stamp)), stamp + kSettleTime);
  std::cout << "[deskew] late init-stamp, no times: translation " << no_times.translation << " m, rotation " << no_times.rotation_deg << " deg"
            << std::endl;
  EXPECT_GT(no_times.translation, 2.0 * errors.translation);
  EXPECT_GT(no_times.rotation_deg, 2.0 * errors.rotation_deg);
}
