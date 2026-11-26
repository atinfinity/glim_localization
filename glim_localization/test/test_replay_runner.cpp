#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

#include <glim/common/imu_integration.hpp>
#include <glim/odometry/async_odometry_estimation.hpp>
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
constexpr double kMaxTranslationError = 0.1;
constexpr double kMaxRotationErrorDeg = 1.0;
constexpr double kFirstPoseAfterResetMaxError = 0.1;

Eigen::Isometry3d initial_error() {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(0.3, -0.2, 0.1);
  T.linear() = (Eigen::AngleAxisd(3.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0.5 * M_PI / 180.0, Eigen::Vector3d::UnitY()))
                 .toRotationMatrix();
  return T;
}

using Records = std::vector<ReplayFrameRecord, Eigen::aligned_allocator<ReplayFrameRecord>>;

struct RunOutput {
  Records records;
  ReplaySummary summary;
};

OdometryEstimationLocalizationCPU::Params base_params() {
  OdometryEstimationLocalizationCPU::Params params;
  params.num_threads = 2;
  params.map_factors.num_threads = 2;
  return params;
}

/// Starts from a known pose that is off by initial_error().
OdometryEstimationLocalizationCPU::Params perturbed_start_params() {
  auto params = base_params();
  params.estimate_init_state = false;
  params.init_T_world_imu = synthetic_T_world_imu(0.0) * initial_error();
  params.init_v_world_imu.setZero();
  return params;
}

/// The real module with a way to alter the IMU and frame stamps it receives, and a view of upstream's IMU queue
class ProbeOdometry : public OdometryEstimationLocalizationCPU {
public:
  using OdometryEstimationLocalizationCPU::OdometryEstimationLocalizationCPU;
  /// Called instead of forwarding a sample; calls `forward(stamp)` for every sample that reaches the module
  std::function<void(double stamp, const std::function<void(double)>& forward)> imu_hook;
  std::function<void(const glim::PreprocessedFrame::Ptr&)> frame_hook;

  void insert_imu(double stamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) override {
    if (!imu_hook) {
      OdometryEstimationLocalizationCPU::insert_imu(stamp, acc, gyro);
      return;
    }
    imu_hook(stamp, [&](double s) { OdometryEstimationLocalizationCPU::insert_imu(s, acc, gyro); });
  }

  glim::EstimationFrame::ConstPtr insert_frame(const glim::PreprocessedFrame::Ptr& original, std::vector<glim::EstimationFrame::ConstPtr>& marginalized) override {
    // The hook alters a copy: the runner keeps its own view of the frame (its stamp anchors the runner's gap check)
    auto frame = original;
    if (frame_hook) {
      frame = std::make_shared<glim::PreprocessedFrame>(*original);
      frame_hook(frame);
    }
    // Oracle, read from upstream's real queue: the largest interval upstream integrates across for (last pose, frame].
    // Written out on its own here; it does not call the production helper.
    std::optional<double> hold;
    bool covered = false;
    const int resets_before = status().reset_count;
    if (!frames.empty()) {
      std::vector<double> points{frames.back()->stamp};
      for (const auto& imu : imu_integration->imu_data_in_queue()) {
        if (imu[0] > points.front() && imu[0] <= frame->stamp) {
          points.push_back(imu[0]);
        }
      }
      points.push_back(frame->stamp);
      double largest = 0.0;
      for (size_t k = 1; k < points.size(); k++) {
        largest = std::max(largest, points[k] - points[k - 1]);
      }
      hold = largest;
      // Covered: the queue holds a sample at or after the scan end, which is when the callers release a frame
      const auto& queue = imu_integration->imu_data_in_queue();
      covered = !queue.empty() && queue.back()[0] >= frame->scan_end_time - 1e-6;
    }
    const auto result = OdometryEstimationLocalizationCPU::insert_frame(frame, marginalized);
    if (result && hold && status().reset_count == resets_before) {  // A pose was integrated from the last pose
      if (covered) {
        max_hold = std::max(max_hold, *hold);
        covered_integrations++;
        if (*hold > 0.05 - 1e-6) {
          boundary_holds.push_back(*hold);
        }
      } else {
        // The queue ends before the scan end (the INCOMPLETE tail, a known limit): counted, not judged
        uncovered_integrations++;
        max_uncovered_hold = std::max(max_uncovered_hold, *hold);
        if (*hold > 0.05 - 1e-6 && *hold <= 0.05 + 1e-6) {
          uncovered_boundary_holds.push_back(*hold);
        }
      }
    }
    return result;
  }

  double max_hold = 0.0;  ///< The longest hold upstream integrated across, over the poses published so far (covered integrations only)
  int covered_integrations = 0;
  int uncovered_integrations = 0;  ///< Poses integrated while the queue ended before the scan end: not part of max_hold
  double max_uncovered_hold = 0.0;
  std::vector<double> uncovered_boundary_holds;  ///< Uncovered holds within 1e-6 of max_imu_gap (the tail ends exactly one interval before the frame)
  std::vector<double> boundary_holds;  ///< Covered holds within 1e-6 of max_imu_gap or above

  std::vector<double> queue_stamps() const {
    std::vector<double> stamps;
    for (const auto& imu : imu_integration->imu_data_in_queue()) {
      stamps.push_back(imu[0]);
    }
    return stamps;
  }
};

/// The queue is finite and strictly ascending, and the gap history is a copy of it wherever both have data
::testing::AssertionResult queue_and_history_agree(const ProbeOdometry& odometry) {
  const auto queue = odometry.queue_stamps();
  for (size_t k = 0; k < queue.size(); k++) {
    if (!std::isfinite(queue[k]) || (k > 0 && !(queue[k] > queue[k - 1]))) {
      return ::testing::AssertionFailure() << "the IMU queue is not strictly ascending at index " << k << " (" << queue[k] << ")";
    }
  }
  const auto& history = odometry.imu_stamp_history();
  if (queue.empty() || history.empty()) {
    return ::testing::AssertionSuccess();
  }
  const double lo = std::max(queue.front(), history.front());
  std::vector<double> q, h;
  std::copy_if(queue.begin(), queue.end(), std::back_inserter(q), [&](double t) { return t >= lo; });
  std::copy_if(history.begin(), history.end(), std::back_inserter(h), [&](double t) { return t >= lo; });
  if (q != h) {
    return ::testing::AssertionFailure() << "the gap history and the IMU queue disagree from " << lo << " s: " << h.size() << " vs " << q.size() << " stamps";
  }
  return ::testing::AssertionSuccess();
}

struct Variant {
  size_t num_frames = static_cast<size_t>(-1);  ///< Number of leading frames to keep
  std::function<bool(const ReplayImu&)> keep_imu = [](const ReplayImu&) { return true; };
  double scan_span = 0.0;  ///< Per-point times are spread over [0, scan_span]
  int imu_stride = 1;      ///< Keep every n-th IMU sample
  std::function<void(ReplayImu&)> modify_imu;
  std::function<bool(size_t)> empty_frame = [](size_t) { return false; };  ///< Frames whose cloud has no points
  std::function<bool(size_t)> keep_frame = [](size_t) { return true; };    ///< Frames that are written (a recording stall drops both topics)
};

}  // namespace

class ReplayRunnerTest : public testing::Test {
protected:
  static void SetUpTestSuite() {
    glim::GlobalConfig::instance(GLIM_LOCALIZATION_CONFIG_DIR, true);
    dir_ = new fs::path(fs::temp_directory_path() / ("glim_loc_replay_runner_" + std::to_string(::getpid())));
    fs::remove_all(*dir_);
    const auto map_params = replay_map_params();
    write_synthetic_dump((*dir_ / "map").string(), map_params);
    write_synthetic_replay((*dir_ / "data").string(), kEndTime, map_params);
    map_ = new std::shared_ptr<const MapManager>();
    auto map = std::make_shared<MapManager>();
    ASSERT_TRUE(map->load((*dir_ / "map").string()));
    *map_ = map;
    dataset_ = new ReplayDataset(ReplayDataset::open((*dir_ / "data").string()));
  }

  static void TearDownTestSuite() {
    delete dataset_;
    delete map_;
    fs::remove_all(*dir_);
    delete dir_;
  }

  static RunOutput run(
    const ReplayDataset& dataset,
    const OdometryEstimationLocalizationCPU::Params& params,
    const ReplayOptions& options = ReplayOptions()) {
    glim::CloudPreprocessor preprocessor;
    OdometryEstimationLocalizationCPU odometry(params, *map_);
    ReplayRunner runner(dataset, odometry, preprocessor, options);
    RunOutput output;
    output.summary = runner.run([&](const ReplayFrameRecord& record) { output.records.push_back(record); });
    return output;
  }

  struct ProbeRun {
    Records records;
    ReplaySummary summary;
    std::unique_ptr<ProbeOdometry> odometry;
  };

  /// A run in `reset` mode through a ProbeOdometry. The runner lets every gap through, so the odometry's own policy decides.
  static ProbeRun run_probe(
    const ReplayDataset& dataset,
    const std::function<void(ProbeOdometry&)>& setup,
    const std::function<void(ProbeOdometry&, const ReplayFrameRecord&)>& on_frame = nullptr,
    const ReplayOptions& base_options = ReplayOptions()) {
    ReplayOptions options = base_options;
    options.on_imu_gap = ReplayOptions::OnImuGap::RESET;
    options.imu_gap_reset_max = 1000.0;
    ProbeRun out;
    glim::CloudPreprocessor preprocessor;
    out.odometry = std::make_unique<ProbeOdometry>(perturbed_start_params(), *map_);
    setup(*out.odometry);
    ReplayRunner runner(dataset, *out.odometry, preprocessor, options);
    out.summary = runner.run([&](const ReplayFrameRecord& record) {
      out.records.push_back(record);
      if (on_frame) {
        on_frame(*out.odometry, record);
      }
    });
    return out;
  }

  /// The first pose is at `first`, the IMU restarts at `restart` (see fault_restarting_at): either no pose exists before
  /// (`external_reset` false) or an external reset() with `min_stamp` starts it while running
  static void check_gap_case(double scan, int64_t from_ns, int64_t to_ns);

  /// Two backward steps of the frame clock with reset() between them (see the tests below)
  static void check_two_backward_steps(bool through_min_stamp, double t);

  static ProbeRun run_first_frame_before_restart(const ReplayDataset& dataset, double first, double restart, bool external_reset, double min_stamp);

  /// Writes a modified copy of the synthetic dataset and opens it
  static ReplayDataset make_variant(const std::string& name, const Variant& variant);

  static fs::path* dir_;
  static std::shared_ptr<const MapManager>* map_;
  static ReplayDataset* dataset_;
};

fs::path* ReplayRunnerTest::dir_ = nullptr;
std::shared_ptr<const MapManager>* ReplayRunnerTest::map_ = nullptr;

ReplayDataset ReplayRunnerTest::make_variant(const std::string& name, const Variant& variant) {
  const fs::path dir = *dir_ / name;
  fs::remove_all(dir);
  {
    ReplayDatasetWriter writer(dir.string());
    const size_t count = std::min(variant.num_frames, dataset_->frames().size());
    for (size_t i = 0; i < count; i++) {
      if (!variant.keep_frame(i)) {
        continue;
      }
      const auto cloud = dataset_->load_cloud(i);
      std::vector<ReplayPoint> points;
      const size_t n = cloud->points.size();
      if (variant.empty_frame(i)) {
        // The format needs a point; this one is inside the preprocessor's near-distance filter, so nothing is left
        writer.write_cloud(dataset_->frames()[i].stamp_ns, {{0.01f, 0.0f, 0.0f, 0.0f, 0.0f}}, 0);
        continue;
      }
      for (size_t k = 0; k < n; k++) {
        const auto& p = cloud->points[k];
        const float t = variant.scan_span > 0.0 ? static_cast<float>(variant.scan_span) * static_cast<float>(k) / static_cast<float>(n - 1) : 0.0f;
        points.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()), t, 0.0f});
      }
      writer.write_cloud(dataset_->frames()[i].stamp_ns, points, variant.scan_span > 0.0 ? kReplayFlagPointTime : 0);
    }
    std::vector<ReplayImu> imu;
    size_t n = 0;
    for (const auto& sample : dataset_->imu()) {
      if (n++ % variant.imu_stride == 0 && variant.keep_imu(sample)) {
        imu.push_back(sample);
        if (variant.modify_imu) {
          variant.modify_imu(imu.back());
        }
      }
    }
    writer.write_imu(imu);
    writer.finish(variant.scan_span > 0.0);
  }
  return ReplayDataset::open(dir.string());
}
ReplayDataset* ReplayRunnerTest::dataset_ = nullptr;

TEST_F(ReplayRunnerTest, ConvergesFromPerturbedStartAndRepeats) {
  const auto first = run(*dataset_, perturbed_start_params());
  const auto second = run(*dataset_, perturbed_start_params());

  ASSERT_EQ(first.records.size(), dataset_->frames().size());
  EXPECT_EQ(first.summary.num_frames, static_cast<int>(dataset_->frames().size()));
  EXPECT_EQ(first.summary.frames_by_result.at("estimated"), static_cast<int>(dataset_->frames().size()));
  EXPECT_EQ(first.summary.frames_by_result.count("dropped"), 0u);

  double max_translation = 0.0;
  double max_rotation_deg = 0.0;
  for (const auto& record : first.records) {
    ASSERT_EQ(record.result, ReplayResult::ESTIMATED) << "stamp_ns=" << record.stamp_ns;
    ASSERT_TRUE(record.T_world_imu->matrix().allFinite());
    const double t = record.stamp_ns * 1e-9;
    const Eigen::Isometry3d error = synthetic_T_world_imu(t).inverse() * *record.T_world_imu;
    if (t >= kSettleTime) {
      max_translation = std::max(max_translation, error.translation().norm());
      max_rotation_deg = std::max(max_rotation_deg, Eigen::AngleAxisd(error.linear()).angle() * 180.0 / M_PI);
    }
  }
  std::cout << "replay accuracy after " << kSettleTime << " s: max_translation=" << max_translation << " m, max_rotation=" << max_rotation_deg
            << " deg, wall=" << first.summary.wall_seconds << " s" << std::endl;
  EXPECT_LT(max_translation, kMaxTranslationError);
  EXPECT_LT(max_rotation_deg, kMaxRotationErrorDeg);
  EXPECT_EQ(first.records.back().state, LocalizationState::TRACKING);
  EXPECT_GT(first.records.back().num_map_factors, 0);
  EXPECT_GT(first.records.back().inlier_ratio, 0.5);

  // The same input gives the same frames and, within the parallel-sum tolerance, the same poses.
  ASSERT_EQ(second.records.size(), first.records.size());
  for (size_t i = 0; i < first.records.size(); i++) {
    EXPECT_EQ(second.records[i].stamp_ns, first.records[i].stamp_ns);
    EXPECT_EQ(second.records[i].result, first.records[i].result);
    EXPECT_EQ(second.records[i].num_imu_fed, first.records[i].num_imu_fed);
    ASSERT_TRUE(first.records[i].T_world_imu && second.records[i].T_world_imu);
    EXPECT_LT((first.records[i].T_world_imu->translation() - second.records[i].T_world_imu->translation()).norm(), 1e-3) << "i=" << i;
  }
}

TEST_F(ReplayRunnerTest, FeedsImuUpToScanEndPlusOneSample) {
  ReplayOptions options;
  options.max_frames = 3;
  const auto output = run(*dataset_, perturbed_start_params(), options);
  ASSERT_EQ(output.records.size(), 3u);
  EXPECT_EQ(output.records[0].num_imu_fed, 2);  // t = 0.00 and the next sample, 0.01
  EXPECT_EQ(output.records[1].num_imu_fed, 10);
  EXPECT_EQ(output.records[2].num_imu_fed, 10);
  EXPECT_GE(output.records[0].num_points_raw, output.records[0].num_points_preprocessed);
  EXPECT_GT(output.records[0].num_points_preprocessed, 100);
  EXPECT_EQ(output.records[0].frame_id, 0);
  EXPECT_EQ(output.records[2].frame_id, 2);
}

TEST_F(ReplayRunnerTest, SelectsFramesByStartEndAndCount) {
  ReplayOptions options;
  options.start = 0.5;
  options.end = 0.9;
  options.max_frames = 3;
  const auto output = run(*dataset_, perturbed_start_params(), options);
  ASSERT_EQ(output.records.size(), 3u);
  EXPECT_EQ(output.records[0].stamp_ns, 500000000);
  EXPECT_EQ(output.records[2].stamp_ns, 700000000);
}

TEST_F(ReplayRunnerTest, DropsEveryFrameWithoutInitialPose) {
  auto params = base_params();
  params.estimate_init_state = true;
  ReplayOptions options;
  options.max_frames = 5;
  const auto output = run(*dataset_, params, options);
  ASSERT_EQ(output.records.size(), 5u);
  for (const auto& record : output.records) {
    EXPECT_EQ(record.result, ReplayResult::DROPPED);
    EXPECT_FALSE(record.T_world_imu);
  }
  EXPECT_EQ(output.summary.frames_by_result.at("dropped"), 5);
}

TEST_F(ReplayRunnerTest, DropsFramesBeforeInitialResetStamp) {
  auto params = base_params();
  params.estimate_init_state = true;
  ReplayOptions options;
  options.max_frames = 15;
  options.initial_reset = ReplayOptions::InitialReset();
  options.initial_reset->T_world_imu = synthetic_T_world_imu(1.0);
  options.initial_reset->min_stamp = 1.0;
  const auto output = run(*dataset_, params, options);
  ASSERT_EQ(output.records.size(), 15u);
  for (size_t i = 0; i < output.records.size(); i++) {
    EXPECT_EQ(output.records[i].result, i < 10 ? ReplayResult::DROPPED : ReplayResult::ESTIMATED) << "i=" << i;
  }
  EXPECT_EQ(output.records.back().reset_count, 1);
}

TEST_F(ReplayRunnerTest, RejectsInvalidInitialReset) {
  ReplayOptions options;
  options.initial_reset = ReplayOptions::InitialReset();
  options.initial_reset->T_world_imu.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(run(*dataset_, perturbed_start_params(), options), std::invalid_argument);
}

TEST_F(ReplayRunnerTest, SkipsFramesTheImuDoesNotCover) {
  // Keep the first 2 s of frames, but the IMU only from 0.35 s to 1.0 s.
  const fs::path short_dir = *dir_ / "short";
  {
    ReplayDatasetWriter writer(short_dir.string());
    for (size_t i = 0; i < 21; i++) {
      const auto cloud = dataset_->load_cloud(i);
      std::vector<ReplayPoint> points;
      for (const auto& p : cloud->points) {
        points.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()), 0.0f, 0.0f});
      }
      writer.write_cloud(dataset_->frames()[i].stamp_ns, points, 0);
    }
    std::vector<ReplayImu> imu;
    for (const auto& sample : dataset_->imu()) {
      if (sample.stamp_ns >= 350000000 && sample.stamp_ns <= 1000000000) {
        imu.push_back(sample);
      }
    }
    writer.write_imu(imu);
    writer.finish(false);
  }
  const auto short_dataset = ReplayDataset::open(short_dir.string());
  const auto output = run(short_dataset, perturbed_start_params());
  ASSERT_EQ(output.records.size(), 21u);
  for (size_t i = 0; i < output.records.size(); i++) {
    const bool covered = i >= 4 && i <= 10;  // stamps 0.4 s to 1.0 s
    EXPECT_EQ(output.records[i].result, covered ? ReplayResult::ESTIMATED : ReplayResult::SKIPPED_NO_IMU) << "i=" << i;
    if (!covered) {
      EXPECT_EQ(output.records[i].num_imu_fed, 0);
    }
  }
  EXPECT_EQ(output.summary.frames_by_result.at("skipped_no_imu"), 14);
}

namespace {

/// Max translation and rotation errors against the analytic trajectory after kSettleTime
struct Errors {
  double translation = 0.0;
  double rotation_deg = 0.0;
  size_t poses = 0;
};

Errors measure(const Records& records) {
  Errors errors;
  for (const auto& record : records) {
    if (!record.T_world_imu) {
      continue;
    }
    errors.poses++;
    const double t = record.stamp_ns * 1e-9;
    if (t < kSettleTime) {
      continue;
    }
    const Eigen::Isometry3d error = synthetic_T_world_imu(t).inverse() * *record.T_world_imu;
    errors.translation = std::max(errors.translation, error.translation().norm());
    errors.rotation_deg = std::max(errors.rotation_deg, Eigen::AngleAxisd(error.linear()).angle() * 180.0 / M_PI);
  }
  return errors;
}

}  // namespace

TEST_F(ReplayRunnerTest, AbortsOnImuDropoutAfterInsertedFrame) {
  // No IMU strictly between 4 s and 6 s: the first and last samples still cover the whole run.
  Variant variant;
  variant.keep_imu = [](const ReplayImu& s) { return s.stamp_ns <= 4000000000 || s.stamp_ns >= 6000000000; };
  const auto dataset = make_variant("dropout", variant);

  for (const double max_gap : {0.05, 3.0}) {  // The 2-sample rule holds even if the gap limit is raised
    ReplayOptions options;
    options.max_imu_gap = max_gap;
    Records records;
    glim::CloudPreprocessor preprocessor;
    OdometryEstimationLocalizationCPU odometry(perturbed_start_params(), *map_);
    ReplayRunner runner(dataset, odometry, preprocessor, options);
    try {
      runner.run([&](const ReplayFrameRecord& record) { records.push_back(record); });
      FAIL() << "run succeeded with max_imu_gap=" << max_gap;
    } catch (const InvalidDatasetError& e) {
      EXPECT_NE(std::string(e.what()).find("frame 4.1"), std::string::npos) << e.what();
    }
    ASSERT_FALSE(records.empty());
    EXPECT_EQ(records.back().stamp_ns, 4000000000);
    for (const auto& record : records) {
      EXPECT_EQ(record.result, ReplayResult::ESTIMATED);  // Nothing is skipped or dropped
    }
    const auto errors = measure(records);
    EXPECT_LT(errors.translation, kMaxTranslationError);
    EXPECT_LT(errors.rotation_deg, kMaxRotationErrorDeg);
  }
}

TEST_F(ReplayRunnerTest, StartAfterDropoutRuns) {
  Variant variant;
  variant.keep_imu = [](const ReplayImu& s) { return s.stamp_ns <= 4000000000 || s.stamp_ns >= 6000000000; };
  const auto dataset = make_variant("dropout_start", variant);

  ReplayOptions options;
  options.start = 6.0;
  options.initial_reset = ReplayOptions::InitialReset();
  options.initial_reset->T_world_imu = synthetic_T_world_imu(6.0) * initial_error();
  options.initial_reset->v_world_imu = synthetic_v_world_imu(6.0);
  auto params = base_params();
  params.estimate_init_state = true;
  RunOutput output;
  ASSERT_NO_THROW(output = run(dataset, params, options));
  ASSERT_FALSE(output.records.empty());
  EXPECT_EQ(output.records.front().stamp_ns, 6000000000);
  EXPECT_EQ(output.summary.frames_by_result.at("estimated"), static_cast<int>(output.records.size()));
  // Measured from 3 s after the restart
  Errors errors;
  for (const auto& record : output.records) {
    if (record.stamp_ns * 1e-9 < 6.0 + kSettleTime) {
      continue;
    }
    const Eigen::Isometry3d error = synthetic_T_world_imu(record.stamp_ns * 1e-9).inverse() * *record.T_world_imu;
    errors.translation = std::max(errors.translation, error.translation().norm());
    errors.rotation_deg = std::max(errors.rotation_deg, Eigen::AngleAxisd(error.linear()).angle() * 180.0 / M_PI);
  }
  EXPECT_LT(errors.translation, kMaxTranslationError);
  EXPECT_LT(errors.rotation_deg, kMaxRotationErrorDeg);
}

// --init-stamp (initial_reset.min_stamp) with an IMU that does not cover the frames before it
static ReplayOptions init_stamp_options(double stamp) {
  ReplayOptions options;
  options.initial_reset = ReplayOptions::InitialReset();
  options.initial_reset->T_world_imu = synthetic_T_world_imu(stamp) * initial_error();
  options.initial_reset->v_world_imu = synthetic_v_world_imu(stamp);
  options.initial_reset->min_stamp = stamp;
  return options;
}

TEST_F(ReplayRunnerTest, InitStampDropsFramesInAnImuHole) {
  auto params = base_params();
  params.estimate_init_state = true;
  for (const double stamp : {5.0, 6.0}) {
    SCOPED_TRACE(stamp);
    Variant variant;
    if (stamp == 5.0) {
      variant.keep_imu = [](const ReplayImu& s) { return s.stamp_ns >= 5000000000; };  // IMU starts at 5 s
    } else {
      variant.keep_imu = [](const ReplayImu& s) { return s.stamp_ns <= 4000000000 || s.stamp_ns >= 6000000000; };  // 4-6 s hole
    }
    const auto dataset = make_variant("init_stamp_hole", variant);
    RunOutput output;
    ASSERT_NO_THROW(output = run(dataset, params, init_stamp_options(stamp)));
    ASSERT_EQ(output.records.size(), dataset.frames().size());
    Errors errors;
    for (const auto& record : output.records) {
      const double t = record.stamp_ns * 1e-9;
      if (t < stamp - 1e-6) {
        EXPECT_EQ(record.result, ReplayResult::DROPPED) << "t=" << t;  // Not skipped_no_imu
      } else {
        EXPECT_EQ(record.result, ReplayResult::ESTIMATED) << "t=" << t;
      }
      if (record.result == ReplayResult::ESTIMATED && t >= stamp + kSettleTime) {
        const Eigen::Isometry3d error = synthetic_T_world_imu(t).inverse() * *record.T_world_imu;
        errors.translation = std::max(errors.translation, error.translation().norm());
        errors.rotation_deg = std::max(errors.rotation_deg, Eigen::AngleAxisd(error.linear()).angle() * 180.0 / M_PI);
      }
    }
    EXPECT_EQ(output.summary.frames_by_result.count("skipped_no_imu"), 0u);
    EXPECT_LT(errors.translation, kMaxTranslationError);
    EXPECT_LT(errors.rotation_deg, kMaxRotationErrorDeg);
  }
}

TEST_F(ReplayRunnerTest, GapCheckStillAppliesAfterTheInitStamp) {
  // The hole (4-6 s) is after the first inserted frame (2 s): the frames before 2 s do not anchor or hide it
  Variant variant;
  variant.keep_imu = [](const ReplayImu& s) { return s.stamp_ns <= 4000000000 || s.stamp_ns >= 6000000000; };
  const auto dataset = make_variant("init_stamp_gap", variant);
  auto params = base_params();
  params.estimate_init_state = true;
  Records records;
  glim::CloudPreprocessor preprocessor;
  OdometryEstimationLocalizationCPU odometry(params, *map_);
  ReplayRunner runner(dataset, odometry, preprocessor, init_stamp_options(2.0));
  EXPECT_THROW(runner.run([&](const ReplayFrameRecord& record) { records.push_back(record); }), InvalidDatasetError);
  ASSERT_FALSE(records.empty());
  EXPECT_EQ(records.front().result, ReplayResult::DROPPED);
  EXPECT_EQ(records.back().stamp_ns, 4000000000);
  for (const auto& record : records) {
    EXPECT_NE(record.result, ReplayResult::SKIPPED_NO_IMU);
  }
}

TEST_F(ReplayRunnerTest, LeadingImuHoleIsSkipped) {
  Variant variant;
  variant.keep_imu = [](const ReplayImu& s) { return s.stamp_ns >= 2000000000; };
  variant.num_frames = 41;  // 0 s to 4 s
  const auto dataset = make_variant("leading", variant);
  RunOutput output;
  ASSERT_NO_THROW(output = run(dataset, perturbed_start_params()));
  ASSERT_EQ(output.records.size(), 41u);
  for (size_t i = 0; i < output.records.size(); i++) {
    EXPECT_EQ(output.records[i].result == ReplayResult::SKIPPED_NO_IMU, i < 20) << "i=" << i;
  }
  EXPECT_EQ(output.records[20].result, ReplayResult::ESTIMATED);
}

TEST_F(ReplayRunnerTest, LastFrameEndingOnLastImuSampleIsEstimated) {
  // The last frame spans 1.00 s to 1.05 s as float32 times, and the last IMU sample is at 1.05 s
  Variant variant;
  variant.num_frames = 11;
  variant.keep_imu = [](const ReplayImu& s) { return s.stamp_ns <= 1050000000; };
  variant.scan_span = 0.05;
  const auto dataset = make_variant("last_sample", variant);
  const auto output = run(dataset, perturbed_start_params());
  ASSERT_EQ(output.records.size(), 11u);
  for (const auto& record : output.records) {
    EXPECT_EQ(record.result, ReplayResult::ESTIMATED) << "stamp_ns=" << record.stamp_ns;
  }
}

TEST_F(ReplayRunnerTest, LowRateImuMeetsAccuracy) {
  Variant variant;
  variant.imu_stride = 4;  // 25 Hz
  const auto dataset = make_variant("imu25", variant);
  RunOutput output;
  ASSERT_NO_THROW(output = run(dataset, perturbed_start_params()));
  const auto errors = measure(output.records);
  std::cout << "25 Hz IMU: max_translation=" << errors.translation << " m, max_rotation=" << errors.rotation_deg << " deg" << std::endl;
  EXPECT_EQ(output.summary.frames_by_result.at("estimated"), static_cast<int>(dataset.frames().size()));
  EXPECT_LT(errors.translation, kMaxTranslationError);
  EXPECT_LT(errors.rotation_deg, kMaxRotationErrorDeg);
}

TEST(ReplayEstimateTest, SanityCheck) {
  glim::EstimationFrame frame;
  frame.T_world_imu = Eigen::Isometry3d::Identity();
  frame.v_world_imu = Eigen::Vector3d(5.0, 0.0, 0.0);
  frame.imu_bias.setZero();
  EXPECT_TRUE(estimate_is_sane(frame));
  auto bad = frame;
  bad.T_world_imu.translation().y() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(estimate_is_sane(bad));
  bad = frame;
  bad.v_world_imu.z() = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(estimate_is_sane(bad));
  bad = frame;
  bad.imu_bias(3) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(estimate_is_sane(bad));
  bad = frame;
  bad.v_world_imu = Eigen::Vector3d(1001.0, 0.0, 0.0);
  EXPECT_FALSE(estimate_is_sane(bad));
}

TEST_F(ReplayRunnerTest, StopsOnDivergedEstimate) {
  // From 3 s the accelerometer reports a huge but finite force, which blows up the integrated velocity
  Variant variant;
  variant.num_frames = 61;
  variant.modify_imu = [](ReplayImu& s) {
    if (s.stamp_ns >= 3000000000) {
      s.acc.x() += 1e9;
    }
  };
  const auto dataset = make_variant("diverge", variant);
  Records records;
  glim::CloudPreprocessor preprocessor;
  OdometryEstimationLocalizationCPU odometry(perturbed_start_params(), *map_);
  ReplayRunner runner(dataset, odometry, preprocessor);
  EXPECT_THROW(runner.run([&](const ReplayFrameRecord& record) { records.push_back(record); }), std::runtime_error);
  ASSERT_FALSE(records.empty());
  EXPECT_EQ(records.back().result, ReplayResult::DIVERGED);
  EXPECT_FALSE(records.back().T_world_imu);
  for (const auto& record : records) {
    if (record.T_world_imu) {
      EXPECT_TRUE(record.T_world_imu->matrix().allFinite());
    }
  }
}

TEST_F(ReplayRunnerTest, WritesOutputsIncrementally) {
  const fs::path out = *dir_ / "out";
  ReplayOptions options;
  options.max_frames = 4;
  glim::CloudPreprocessor preprocessor;
  OdometryEstimationLocalizationCPU odometry(perturbed_start_params(), *map_);
  ReplayRunner runner(*dataset_, odometry, preprocessor, options);
  ReplayOutputWriter writer(out.string());
  const auto summary = runner.run([&](const ReplayFrameRecord& record) {
    writer.write(record);
    // Each frame is on disk before the next one is processed
    std::ifstream frames(out / "frames.csv");
    int lines = 0;
    std::string line;
    while (std::getline(frames, line)) {
      lines++;
    }
    EXPECT_EQ(lines, 1 + writer.num_poses());
  });
  EXPECT_FALSE(fs::exists(out / "summary.json"));
  writer.write_summary(summary, 0.5);
  EXPECT_TRUE(fs::exists(out / "summary.json"));
  EXPECT_EQ(writer.num_poses(), 4);

  std::ifstream traj(out / "traj_localization.txt");
  std::string first_line;
  std::getline(traj, first_line);
  EXPECT_EQ(first_line.substr(0, 12), "0.000000000 ");
}

TEST_F(ReplayRunnerTest, WriterReportsTheInitSearch) {
  const fs::path out = *dir_ / "init_search";
  ReplaySummary summary;
  ReplaySummary::InitSearch search;
  search.result = InitSearchResult::REJECTED;
  search.reason = InitSearchReason::TRUNCATED;
  search.candidates = 3;
  search.candidates_total = 24;
  search.score = 0.8;
  search.yaw_offset = M_PI / 2.0;
  search.runner_up_score = 0.4;
  search.runner_up_yaw_offset = -M_PI / 2.0;
  search.seconds = 10.0;
  summary.init_search = search;
  ReplayOutputWriter writer(out.string());
  writer.write_summary(summary, 0.5);
  const auto json = nlohmann::json::parse(std::ifstream(out / "summary.json"));
  const auto& written = json.at("init_search");
  EXPECT_EQ(written.at("result"), "REJECTED");
  EXPECT_EQ(written.at("reason"), "TRUNCATED");
  EXPECT_EQ(written.at("candidates"), 3);
  EXPECT_EQ(written.at("candidates_total"), 24);
  EXPECT_EQ(written.at("truncated"), true);
  EXPECT_NEAR(written.at("yaw_offset_deg").get<double>(), 90.0, 1e-9);
  EXPECT_NEAR(written.at("runner_up_yaw_offset_deg").get<double>(), -90.0, 1e-9);
}

TEST_F(ReplayRunnerTest, WriterRemovesStaleSummary) {
  const fs::path out = *dir_ / "stale";
  fs::create_directories(out);
  { std::ofstream(out / "summary.json") << "{}\n"; }
  ReplayOutputWriter writer(out.string());
  EXPECT_FALSE(fs::exists(out / "summary.json"));
}

// ---- IMU gap policy ----

namespace {

/// The IMU samples at 100 Hz, with the open interval (from_ns, to_ns) removed: a gap of (to - from)
Variant imu_hole(int64_t from_ns, int64_t to_ns) {
  Variant variant;
  variant.keep_imu = [=](const ReplayImu& s) { return s.stamp_ns <= from_ns || s.stamp_ns >= to_ns; };
  return variant;
}

}  // namespace

TEST(ImuCoverageTest, GapDefinitionBoundaries) {
  // 100 Hz samples with a missing stretch from 1.00 s to 1.06 s (a 0.06 s gap)
  std::vector<double> stamps;
  for (int i = 0; i <= 100; i++) {
    stamps.push_back(i * 0.01);
  }
  for (int i = 106; i <= 200; i++) {
    stamps.push_back(i * 0.01);
  }
  const auto stamp_at = [&](size_t k) { return stamps[k]; };
  const auto check = [&](std::optional<double> last, double frame, double end, double max_gap) {
    return check_imu_coverage(stamps.size(), stamp_at, last, frame, end, max_gap);
  };
  EXPECT_EQ(check(0.5, 0.6, 0.65, 0.05).kind, ImuCoverage::OK);
  const auto gap = check(0.5, 1.1, 1.15, 0.05);
  EXPECT_EQ(gap.kind, ImuCoverage::GAP);
  EXPECT_NEAR(gap.gap_seconds, 0.06, 1e-9);
  EXPECT_EQ(check(0.5, 1.1, 1.15, 0.07).kind, ImuCoverage::OK);   // Gap just under the limit
  EXPECT_EQ(check(0.5, 1.1, 1.15, 0.059).kind, ImuCoverage::GAP);  // Gap just over the limit
  EXPECT_EQ(check(0.5, 1.1, 5.0, 0.05).kind, ImuCoverage::INCOMPLETE);
  EXPECT_EQ(check(1.1, 1.1, 1.15, 0.05).kind, ImuCoverage::TOO_FEW_SAMPLES);
  EXPECT_EQ(check(std::nullopt, 0.5, 0.55, 0.05).kind, ImuCoverage::OK);
  EXPECT_EQ(check(std::nullopt, -1.0, -0.9, 0.05).kind, ImuCoverage::GAP);  // Leading hole
}

TEST(TrackingRatioTest, GateBoundary) {
  const auto record = [](LocalizationState state, ReplayResult result = ReplayResult::ESTIMATED) {
    ReplayFrameRecord r;
    r.state = state;
    r.result = result;
    return r;
  };
  TrackingRatio at_threshold;  // 9 of 10 from the first TRACKING frame on; the start-up and non-estimates do not count
  at_threshold.add(record(LocalizationState::INITIALIZING));
  at_threshold.add(record(LocalizationState::INITIALIZING));
  at_threshold.add(record(LocalizationState::LOST, ReplayResult::DROPPED));
  for (int i = 0; i < 9; i++) {
    at_threshold.add(record(LocalizationState::TRACKING));
  }
  at_threshold.add(record(LocalizationState::DEGRADED));
  EXPECT_DOUBLE_EQ(at_threshold.ratio(), 0.9);
  EXPECT_TRUE(at_threshold.passes(0.9));

  TrackingRatio below = at_threshold;
  below.add(record(LocalizationState::INITIALIZING));  // 9 of 11
  EXPECT_LT(below.ratio(), 0.9);
  EXPECT_FALSE(below.passes(0.9));
  EXPECT_TRUE(below.passes(0.0));  // 0 disables the gate

  TrackingRatio never;
  never.add(record(LocalizationState::DEGRADED));
  EXPECT_DOUBLE_EQ(never.ratio(), 0.0);
  EXPECT_FALSE(never.passes(0.9));
  EXPECT_TRUE(never.passes(0.0));
}

TEST_F(ReplayRunnerTest, ImuGapAbortModeStopsOnAnyGapAboveTheLimit) {
  // 0.49 s and 0.51 s: the default mode aborts on both. The online self-reset limit does not apply to replay's abort.
  for (const int64_t hole_end : {4490000000, 4510000000}) {
    SCOPED_TRACE(hole_end);
    const auto dataset = make_variant("gap_abort", imu_hole(4000000000, hole_end));
    ReplayOptions options;  // OnImuGap::ABORT
    Records records;
    glim::CloudPreprocessor preprocessor;
    OdometryEstimationLocalizationCPU odometry(perturbed_start_params(), *map_);
    ReplayRunner runner(dataset, odometry, preprocessor, options);
    EXPECT_THROW(runner.run([&](const ReplayFrameRecord& record) { records.push_back(record); }), InvalidDatasetError);
    ASSERT_FALSE(records.empty());
    EXPECT_EQ(records.back().stamp_ns, 4000000000);  // The module never saw a frame in the gap
  }
}

TEST_F(ReplayRunnerTest, ImuGapResetModeRecoversFromAGapJustUnderTheLimit) {
  const auto dataset = make_variant("gap_reset_under", imu_hole(4000000000, 4490000000));
  ReplayOptions options;
  options.on_imu_gap = ReplayOptions::OnImuGap::RESET;
  RunOutput output;
  ASSERT_NO_THROW(output = run(dataset, perturbed_start_params(), options));
  EXPECT_EQ(output.summary.imu_gap_resets, 1);
  EXPECT_GE(output.summary.frames_by_result.at("skipped_no_imu"), 3);
  int skipped = 0;
  for (const auto& record : output.records) {
    const double t = record.stamp_ns * 1e-9;
    if (record.result == ReplayResult::SKIPPED_NO_IMU) {
      EXPECT_GT(t, 4.0 - 1e-6);
      EXPECT_LT(t, 4.5);
      skipped++;
    }
    EXPECT_NE(record.state, LocalizationState::LOST) << "t=" << t;
  }
  EXPECT_GE(skipped, 3);
  // The odometry sees the first clean frame as still spanning the gap (relative to its last pose): it drops it and resets at the next one
  size_t last_skipped = 0;
  for (size_t i = 0; i < output.records.size(); i++) {
    if (output.records[i].result == ReplayResult::SKIPPED_NO_IMU) {
      last_skipped = i;
    }
  }
  ASSERT_LT(last_skipped + 2, output.records.size());
  EXPECT_EQ(output.records[last_skipped + 1].result, ReplayResult::DROPPED);
  EXPECT_EQ(output.records[last_skipped + 2].result, ReplayResult::ESTIMATED);
  EXPECT_EQ(output.records[last_skipped + 2].reset_count, 1);
  // The pose restarts from the last pose moved along the last velocity, not from where it stood before the gap
  {
    const auto& first = output.records[last_skipped + 2];
    const Eigen::Isometry3d error = synthetic_T_world_imu(first.stamp_ns * 1e-9).inverse() * *first.T_world_imu;
    std::cout << "first pose after the reset: translation error " << error.translation().norm() << " m" << std::endl;
    EXPECT_LT(error.translation().norm(), kFirstPoseAfterResetMaxError);
  }
  // The reset counts, restarts the state machine, and the pose converges again
  const auto last = output.records.back();
  EXPECT_EQ(last.result, ReplayResult::ESTIMATED);
  EXPECT_EQ(last.reset_count, 1);
  EXPECT_EQ(last.state, LocalizationState::TRACKING);
  Errors errors;
  for (const auto& record : output.records) {
    if (record.T_world_imu && record.stamp_ns * 1e-9 >= 4.5 + kSettleTime) {
      const Eigen::Isometry3d error = synthetic_T_world_imu(record.stamp_ns * 1e-9).inverse() * *record.T_world_imu;
      errors.translation = std::max(errors.translation, error.translation().norm());
      errors.rotation_deg = std::max(errors.rotation_deg, Eigen::AngleAxisd(error.linear()).angle() * 180.0 / M_PI);
    }
  }
  EXPECT_LT(errors.translation, kMaxTranslationError);
  EXPECT_LT(errors.rotation_deg, kMaxRotationErrorDeg);
}

TEST_F(ReplayRunnerTest, ImuGapResetModeStillAbortsOnAGapJustOverTheLimit) {
  const auto dataset = make_variant("gap_reset_over", imu_hole(4000000000, 4510000000));
  ReplayOptions options;
  options.on_imu_gap = ReplayOptions::OnImuGap::RESET;
  Records records;
  glim::CloudPreprocessor preprocessor;
  OdometryEstimationLocalizationCPU odometry(perturbed_start_params(), *map_);
  ReplayRunner runner(dataset, odometry, preprocessor, options);
  EXPECT_THROW(runner.run([&](const ReplayFrameRecord& record) { records.push_back(record); }), InvalidDatasetError);
  ASSERT_FALSE(records.empty());
  EXPECT_EQ(records.back().stamp_ns, 4000000000);
}

TEST_F(ReplayRunnerTest, OdometryGapPolicyResetsUnderAndGoesLostOverTheLimit) {
  // The runner lets any gap through (limit 10 s) so that the odometry's own policy decides
  for (const int64_t hole_end : {4490000000, 4510000000}) {
    SCOPED_TRACE(hole_end);
    const bool over = hole_end > 4500000000;
    const auto dataset = make_variant("gap_module", imu_hole(4000000000, hole_end));
    ReplayOptions options;
    options.on_imu_gap = ReplayOptions::OnImuGap::RESET;
    options.imu_gap_reset_max = 10.0;
    const auto output = run(dataset, perturbed_start_params(), options);
    const auto last = output.records.back();
    if (over) {
      EXPECT_EQ(output.summary.imu_gap_resets, 0);
      EXPECT_EQ(last.state, LocalizationState::LOST);
      EXPECT_NE(last.result, ReplayResult::ESTIMATED);  // It stays lost: no pose until an external initialization
      EXPECT_EQ(last.reset_count, 0);
    } else {
      EXPECT_EQ(output.summary.imu_gap_resets, 1);
      EXPECT_EQ(last.result, ReplayResult::ESTIMATED);
      EXPECT_NE(last.state, LocalizationState::LOST);
    }
  }
}

TEST_F(ReplayRunnerTest, InitialPoseJustUnderTheDelayLimitIsApplied) {
  // IMU starts at 1.2 s: with min_stamp 1.01 s the first frame it covers is 0.19 s late
  const auto dataset = make_variant("late_under", [] {
    Variant v;
    v.keep_imu = [](const ReplayImu& s) { return s.stamp_ns >= 1200000000; };
    return v;
  }());
  auto params = base_params();
  params.estimate_init_state = true;
  auto options = init_stamp_options(1.01);
  RunOutput output;
  ASSERT_NO_THROW(output = run(dataset, params, options));
  for (const auto& record : output.records) {
    const double t = record.stamp_ns * 1e-9;
    if (t < 1.01) {
      EXPECT_EQ(record.result, ReplayResult::DROPPED) << t;
    } else if (t < 1.2 - 1e-6) {
      EXPECT_EQ(record.result, ReplayResult::SKIPPED_NO_IMU) << t;
    } else {
      EXPECT_EQ(record.result, ReplayResult::ESTIMATED) << t;
    }
  }
}

TEST_F(ReplayRunnerTest, InitialPoseJustOverTheDelayLimitIsRejected) {
  const auto dataset = make_variant("late_over", [] {
    Variant v;
    v.keep_imu = [](const ReplayImu& s) { return s.stamp_ns >= 1200000000; };
    return v;
  }());
  auto params = base_params();
  params.estimate_init_state = true;
  try {
    run(dataset, params, init_stamp_options(0.99));  // 0.21 s late
    FAIL() << "the late initial pose was accepted";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("initial pose"), std::string::npos) << e.what();
  }
  // The limit is an option
  auto options = init_stamp_options(0.99);
  options.max_init_pose_delay = 0.25;
  EXPECT_NO_THROW(run(dataset, params, options));
}

TEST_F(ReplayRunnerTest, UnstampedInitialPoseIsNeverLate) {
  const auto dataset = make_variant("late_unstamped", [] {
    Variant v;
    v.keep_imu = [](const ReplayImu& s) { return s.stamp_ns >= 2000000000; };
    return v;
  }());
  auto params = base_params();
  params.estimate_init_state = true;
  const auto options = init_stamp_options(0.0);
  RunOutput output;
  ASSERT_NO_THROW(output = run(dataset, params, options));
  EXPECT_GT(output.summary.frames_by_result.at("skipped_no_imu"), 0);
  EXPECT_EQ(output.records.back().result, ReplayResult::ESTIMATED);
}

// ---- IMU gap edge cases: gap windows, the IMU stamp gate, resets and holds across holes ----

TEST(ImuCoverageTest, LongestGapInTheWindowDecides) {
  // 100 Hz with a 0.1 s gap (1.0 to 1.1 s) in front of a 0.6 s gap (1.11 to 1.71 s)
  std::vector<double> stamps;
  for (int i = 0; i <= 100; i++) {
    stamps.push_back(i * 0.01);
  }
  for (int i = 110; i <= 111; i++) {
    stamps.push_back(i * 0.01);
  }
  for (int i = 171; i <= 300; i++) {
    stamps.push_back(i * 0.01);
  }
  const auto result = check_imu_coverage(stamps.size(), [&](size_t k) { return stamps[k]; }, std::optional<double>(0.9), 1.2, 1.8, 0.05);
  ASSERT_EQ(result.kind, ImuCoverage::GAP);
  EXPECT_NEAR(result.gap_seconds, 0.6, 1e-9);
  EXPECT_NEAR(result.gap_begin, 1.11, 1e-9);
}

TEST_F(ReplayRunnerTest, ResetModeStillAbortsOnALongGapRightAfterARecovery) {
  // 0.2 s gap (recoverable), a clean stretch that covers the recovery frame (4.2 s) only, then 2.0 s without IMU
  Variant variant;
  variant.keep_imu = [](const ReplayImu& s) { return s.stamp_ns <= 4000000000 || (s.stamp_ns >= 4200000000 && s.stamp_ns <= 4250000000) || s.stamp_ns >= 6310000000; };
  const auto dataset = make_variant("gap_after_recovery", variant);
  ReplayOptions options;
  options.on_imu_gap = ReplayOptions::OnImuGap::RESET;
  Records records;
  glim::CloudPreprocessor preprocessor;
  OdometryEstimationLocalizationCPU odometry(perturbed_start_params(), *map_);
  ReplayRunner runner(dataset, odometry, preprocessor, options);
  EXPECT_THROW(runner.run([&](const ReplayFrameRecord& record) { records.push_back(record); }), InvalidDatasetError);
  int skipped = 0;
  for (const auto& record : records) {
    skipped += record.result == ReplayResult::SKIPPED_NO_IMU ? 1 : 0;
  }
  EXPECT_GT(skipped, 0);  // The first gap was handled; the second one was not
}

namespace {

/// IMU 0 to 4.0 s, a 0.2 s gap, 4.2 to 4.25 s, then an outage until 6.31 s. A recording stall also drops the LiDAR frames inside it.
Variant imu_outage_without_frames(const bool with_recovery_frame) {
  Variant variant;
  variant.keep_imu = [](const ReplayImu& s) { return s.stamp_ns <= 4000000000 || (s.stamp_ns >= 4200000000 && s.stamp_ns <= 4250000000) || s.stamp_ns >= 6310000000; };
  variant.keep_frame = [with_recovery_frame](size_t i) {
    const double t = static_cast<double>(i) * 0.1;
    return t <= (with_recovery_frame ? 4.2 : 4.1) + 1e-6 || t >= 6.4 - 1e-6;
  };
  return variant;
}

}  // namespace

TEST_F(ReplayRunnerTest, ResetModeAbortsOnAnOutageWithoutLidarFramesAfterARecovery) {
  const auto dataset = make_variant("stall_after_recovery", imu_outage_without_frames(true));
  ReplayOptions options;
  options.on_imu_gap = ReplayOptions::OnImuGap::RESET;
  glim::CloudPreprocessor preprocessor;
  OdometryEstimationLocalizationCPU odometry(perturbed_start_params(), *map_);
  ReplayRunner runner(dataset, odometry, preprocessor, options);
  EXPECT_THROW(runner.run(nullptr), InvalidDatasetError);
}

TEST_F(ReplayRunnerTest, ResetModeAbortsOnAnOutageWithoutLidarFramesBeforeTheRecoveryFrame) {
  // The 0.2 s gap starts an episode at 4.1 s; the next frame is 2.3 s later and clean around itself, but the outage in between is 2.06 s
  const auto dataset = make_variant("stall_in_episode", imu_outage_without_frames(false));
  ReplayOptions options;
  options.on_imu_gap = ReplayOptions::OnImuGap::RESET;
  glim::CloudPreprocessor preprocessor;
  OdometryEstimationLocalizationCPU odometry(perturbed_start_params(), *map_);
  ReplayRunner runner(dataset, odometry, preprocessor, options);
  EXPECT_THROW(runner.run(nullptr), InvalidDatasetError);
}

TEST_F(ReplayRunnerTest, OdometryGoesLostOnAnOutageWithoutLidarFrames) {
  // The runner lets the outage through (limit 10 s): the odometry's own limit (0.5 s) must see the whole interval between two frames
  for (const bool with_recovery_frame : {true, false}) {
    SCOPED_TRACE(with_recovery_frame);
    const auto dataset = make_variant("stall_module", imu_outage_without_frames(with_recovery_frame));
    ReplayOptions options;
    options.on_imu_gap = ReplayOptions::OnImuGap::RESET;
    options.imu_gap_reset_max = 10.0;
    const auto output = run(dataset, perturbed_start_params(), options);
    EXPECT_EQ(output.summary.imu_gap_resets, 0);  // It must not extrapolate the pose across 2 s
    EXPECT_EQ(output.records.back().state, LocalizationState::LOST);
    EXPECT_NE(output.records.back().result, ReplayResult::ESTIMATED);
  }
}

TEST_F(ReplayRunnerTest, ResetModeAbortsWhenAShortGapPrecedesALongOneInTheSameWindow) {
  Variant variant;
  variant.keep_imu = [](const ReplayImu& s) { return s.stamp_ns <= 4000000000 || (s.stamp_ns >= 4100000000 && s.stamp_ns <= 4110000000) || s.stamp_ns >= 4710000000; };
  const auto dataset = make_variant("short_then_long", variant);
  ReplayOptions options;
  options.on_imu_gap = ReplayOptions::OnImuGap::RESET;
  glim::CloudPreprocessor preprocessor;
  OdometryEstimationLocalizationCPU odometry(perturbed_start_params(), *map_);
  ReplayRunner runner(dataset, odometry, preprocessor, options);
  EXPECT_THROW(runner.run(nullptr), InvalidDatasetError);
}

TEST_F(ReplayRunnerTest, EmptyCloudsDoNotSatisfyTheInitialPoseDelayCheck) {
  // Frames 20 to 29 (2.0 to 2.9 s) have no points: the pose first applies to frame 30, 1.0 s after --init-stamp
  Variant variant;
  variant.empty_frame = [](size_t i) { return i >= 20 && i < 30; };
  const auto dataset = make_variant("empty_frames", variant);
  auto params = base_params();
  params.estimate_init_state = true;
  EXPECT_THROW(run(dataset, params, init_stamp_options(2.0)), std::invalid_argument);
  // Control: the same dataset with a delay that is allowed
  auto options = init_stamp_options(2.0);
  options.max_init_pose_delay = 1.5;
  EXPECT_NO_THROW(run(dataset, params, options));
}

namespace {

/// Delivers one IMU sample 0.08 s late (out of order) in an otherwise complete stream
class LateSampleOdometry : public OdometryEstimationLocalizationCPU {
public:
  using OdometryEstimationLocalizationCPU::OdometryEstimationLocalizationCPU;
  void insert_imu(double stamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) override {
    OdometryEstimationLocalizationCPU::insert_imu(stamp, acc, gyro);
    if (inject && !injected_ && stamp >= 4.0) {
      injected_ = true;
      OdometryEstimationLocalizationCPU::insert_imu(stamp - 0.08, acc, gyro);
    }
  }
  bool inject = true;

private:
  bool injected_ = false;
};

/// Calls a function when a log message contains a text
class TriggerSink : public spdlog::sinks::base_sink<std::mutex> {
public:
  std::string text;
  std::function<void()> action;

protected:
  void sink_it_(const spdlog::details::log_msg& msg) override {
    if (action && std::string(msg.payload.begin(), msg.payload.end()).find(text) != std::string::npos) {
      auto fn = std::move(action);
      action = nullptr;
      fn();
    }
  }
  void flush_() override {}
};

}  // namespace

TEST_F(ReplayRunnerTest, AnOutOfOrderImuSampleIsNotAGap) {
  for (const bool inject : {false, true}) {
    SCOPED_TRACE(inject);
    ReplayOptions options;
    options.on_imu_gap = ReplayOptions::OnImuGap::RESET;
    Records records;
    glim::CloudPreprocessor preprocessor;
    LateSampleOdometry odometry(perturbed_start_params(), *map_);
    odometry.inject = inject;
    ReplayRunner runner(*dataset_, odometry, preprocessor, options);
    const auto summary = runner.run([&](const ReplayFrameRecord& record) { records.push_back(record); });
    EXPECT_EQ(summary.imu_gap_resets, 0);
    EXPECT_EQ(records.back().reset_count, 0);
    for (const auto& record : records) {
      EXPECT_EQ(record.result, ReplayResult::ESTIMATED) << record.stamp_ns;  // No frame is dropped for a gap that is not there
    }
  }
}

namespace {

/// The odometry went LOST, stayed LOST, and no pose came out after that; it never reset itself
void expect_fails_closed(const Records& records, const ReplaySummary& summary, const ProbeOdometry& odometry) {
  bool lost = false;
  for (const auto& record : records) {
    lost = lost || record.state == LocalizationState::LOST;
    if (lost) {
      EXPECT_NE(record.result, ReplayResult::ESTIMATED) << "a pose was published while LOST at " << record.stamp_ns;
    }
  }
  EXPECT_TRUE(lost);
  EXPECT_EQ(records.back().state, LocalizationState::LOST);
  EXPECT_EQ(records.back().reset_count, 0);
  EXPECT_EQ(summary.imu_gap_resets, 0);
  EXPECT_EQ(odometry.status().reset_count, 0);
}

/// Every frame is estimated: the stream is fine as far as the odometry is concerned
void expect_untouched(const Records& records, const ProbeOdometry& odometry) {
  for (const auto& record : records) {
    EXPECT_EQ(record.result, ReplayResult::ESTIMATED) << record.stamp_ns;
    EXPECT_NE(record.state, LocalizationState::LOST) << record.stamp_ns;
  }
  EXPECT_EQ(records.back().reset_count, 0);
  EXPECT_EQ(odometry.status().reset_count, 0);
}

using Forward = std::function<void(double)>;

/// Calls `act` instead of forwarding the first sample at or after 4.0 s, once
std::function<void(double, const Forward&)> at_four_seconds(std::function<void(double, const Forward&)> act) {
  auto done = std::make_shared<bool>(false);
  return [act, done](double stamp, const Forward& forward) {
    if (!*done && stamp >= 4.0) {
      *done = true;
      act(stamp, forward);
    } else {
      forward(stamp);
    }
  };
}

}  // namespace

TEST_F(ReplayRunnerTest, IMUStampGateForwardJumpGoesLostAndTheStreamRestarts) {
  const auto run = run_probe(*dataset_, [](ProbeOdometry& o) {
    o.imu_hook = at_four_seconds([](double stamp, const Forward& forward) { forward(stamp + 1000.0); });
  });
  expect_fails_closed(run.records, run.summary, *run.odometry);
  EXPECT_TRUE(queue_and_history_agree(*run.odometry));
  // The 5 real samples after the jump were rejected: that is a fault, the queue restarted on the real clock
  const auto queue = run.odometry->queue_stamps();
  ASSERT_FALSE(queue.empty());
  EXPECT_LT(queue.back(), 100.0);
  EXPECT_NEAR(queue.back(), kEndTime, 0.1);
}

TEST_F(ReplayRunnerTest, IMUStampGateBackwardStepGoesLostWithoutMovingThePose) {
  for (const double shift : {100.0, 5.0}) {
    SCOPED_TRACE(shift);
    auto stepped = std::make_shared<bool>(false);
    const auto run = run_probe(*dataset_, [&](ProbeOdometry& o) {
      o.imu_hook = [stepped, shift](double stamp, const Forward& forward) {
        *stepped = *stepped || stamp >= 4.0;
        forward(*stepped ? stamp - shift : stamp);
      };
    });
    expect_fails_closed(run.records, run.summary, *run.odometry);
    EXPECT_TRUE(queue_and_history_agree(*run.odometry));
    for (const auto& record : run.records) {
      if (record.T_world_imu) {  // The last pose is where the truth was, not moved by v * dt across the step
        EXPECT_LT((synthetic_T_world_imu(record.stamp_ns * 1e-9).inverse() * *record.T_world_imu).translation().norm(), 0.5) << record.stamp_ns;
      }
    }
  }
}

TEST_F(ReplayRunnerTest, IMUStampGateLateBatch) {
  for (const int late : {3, 90}) {
    SCOPED_TRACE(late);
    auto held = std::make_shared<std::vector<double>>();
    auto state = std::make_shared<int>(0);  // 0 before, 1 holding, 2 done
    const auto run = run_probe(*dataset_, [&](ProbeOdometry& o) {
      o.imu_hook = [held, state, late](double stamp, const Forward& forward) {
        if (*state == 0 && stamp >= 4.0) {
          *state = 1;
        }
        if (*state == 1) {
          if (static_cast<int>(held->size()) < late) {
            held->push_back(stamp);
            return;
          }
          *state = 2;
          forward(stamp);
          for (const double t : *held) {  // The held samples arrive after newer ones
            forward(t);
          }
          return;
        }
        forward(stamp);
      };
    });
    EXPECT_TRUE(queue_and_history_agree(*run.odometry));
    if (late < 5) {
      expect_untouched(run.records, *run.odometry);  // The 0.04 s hole is within the limit; the stale samples are dropped
    } else {
      expect_fails_closed(run.records, run.summary, *run.odometry);  // 0.9 s of IMU is missing
    }
  }
}

TEST_F(ReplayRunnerTest, IMUStampGateStaleStampsAreDroppedUntilTheFifthInARow) {
  for (const int count : {1, 4, 5}) {
    SCOPED_TRACE(count);
    const auto run = run_probe(*dataset_, [&](ProbeOdometry& o) {
      o.imu_hook = at_four_seconds([count](double stamp, const Forward& forward) {
        forward(stamp);
        for (int k = 0; k < count; k++) {
          forward(1.0 + k);  // Old stamps arriving right after a good sample
        }
      });
    });
    EXPECT_TRUE(queue_and_history_agree(*run.odometry));
    if (count < 5) {
      expect_untouched(run.records, *run.odometry);
    } else {
      expect_fails_closed(run.records, run.summary, *run.odometry);
    }
  }
}

TEST_F(ReplayRunnerTest, IMUStampGateRejectsNonFiniteStamps) {
  for (const double bad : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
    SCOPED_TRACE(bad);
    const auto run = run_probe(*dataset_, [&](ProbeOdometry& o) {
      o.imu_hook = at_four_seconds([bad](double stamp, const Forward& forward) {
        forward(stamp);
        forward(bad);
      });
    });
    expect_untouched(run.records, *run.odometry);
    EXPECT_TRUE(queue_and_history_agree(*run.odometry));
  }
}

TEST_F(ReplayRunnerTest, IMUStampGateAnExternalResetRecoversAfterAFault) {
  const auto dataset = make_variant("imu_fault_then_reset", Variant());
  auto reset_done = std::make_shared<bool>(false);
  const auto run = run_probe(
    dataset,
    [](ProbeOdometry& o) { o.imu_hook = at_four_seconds([](double stamp, const Forward& forward) { forward(stamp + 1000.0); }); },
    [&](ProbeOdometry& odometry, const ReplayFrameRecord& record) {
      if (record.state == LocalizationState::LOST && !*reset_done) {
        const double t = record.stamp_ns * 1e-9 + 0.05;
        *reset_done = odometry.reset(synthetic_T_world_imu(t + 0.05) * initial_error(), synthetic_v_world_imu(t + 0.05), t);
      }
    });
  ASSERT_TRUE(*reset_done);
  EXPECT_EQ(run.records.back().result, ReplayResult::ESTIMATED);
  EXPECT_EQ(run.records.back().reset_count, 1);
  EXPECT_NE(run.records.back().state, LocalizationState::LOST);
  EXPECT_EQ(run.odometry->status().last_reset_reason, ResetReason::EXTERNAL);
  EXPECT_TRUE(queue_and_history_agree(*run.odometry));
  const auto& last = run.records.back();
  EXPECT_LT((synthetic_T_world_imu(last.stamp_ns * 1e-9).inverse() * *last.T_world_imu).translation().norm(), kMaxTranslationError);
}

TEST_F(ReplayRunnerTest, IMUStampGateAnEqualStampIsRejected) {
  // `stamp <= last` at the gate: a repeated stamp must not reach the queue (`<` would let it through)
  // Frames consume the queue as the run goes on, so it is checked just before the frame that follows the repeated stamp
  const auto run = run_probe(*dataset_, [](ProbeOdometry& o) {
    o.imu_hook = at_four_seconds([](double stamp, const Forward& forward) {
      forward(stamp);
      forward(stamp);
    });
    o.frame_hook = [&o](const glim::PreprocessedFrame::Ptr& frame) {
      if (frame->stamp > 4.0 - 1e-6 && frame->stamp < 4.05) {
        EXPECT_TRUE(queue_and_history_agree(o));
        EXPECT_FALSE(o.queue_stamps().empty());
      }
    };
  });
  expect_untouched(run.records, *run.odometry);
}

namespace {

/// Before the first frame at `trigger`, a +1000 s sample and then 5 stale ones make a fault; the stream restarts at `restart`.
/// The real samples up to `restart` are swallowed, as if the IMU had stalled until then.
std::function<void(double, const Forward&)> fault_restarting_at(double trigger, double restart) {
  auto started = std::make_shared<bool>(false);
  return [=](double stamp, const Forward& forward) {
    if (!*started) {
      if (stamp < trigger) {
        forward(stamp);
        return;
      }
      *started = true;
      forward(stamp + 1000.0);
      for (int k = 4; k >= 0; k--) {
        forward(restart - 0.01 * k);  // The 5th stale sample restarts the queue here
      }
      return;
    }
    if (stamp > restart + 1e-6) {
      forward(stamp);
    }
  };
}

/// No frame in (first, restart + 0.02] is estimated: its interval would hold the first sample across the hole
void expect_no_pose_across_the_hole(const Records& records, double first, double restart) {
  bool saw_first = false;
  for (const auto& record : records) {
    const double t = record.stamp_ns * 1e-9;
    if (std::abs(t - first) < 1e-6) {
      saw_first = true;
      EXPECT_EQ(record.result, ReplayResult::ESTIMATED);  // The first pose needs no IMU interval
    } else if (t > first && t <= restart + 0.02) {
      EXPECT_NE(record.result, ReplayResult::ESTIMATED) << "a pose was integrated across the IMU hole at " << t;
    }
  }
  EXPECT_TRUE(saw_first);
  // The stream is clean again: the odometry restarted itself from its last pose and tracks
  EXPECT_EQ(records.back().result, ReplayResult::ESTIMATED);
  EXPECT_NE(records.back().state, LocalizationState::LOST);
}

}  // namespace

/// The first pose is at `first`, the IMU restarts at `restart`: either no pose exists before (`external_reset` false) or
/// an external reset() with min_stamp `min_stamp` starts it while running
ReplayRunnerTest::ProbeRun ReplayRunnerTest::run_first_frame_before_restart(
  const ReplayDataset& dataset, double first, double restart, bool external_reset, double min_stamp) {
  auto reset_done = std::make_shared<bool>(false);
  return run_probe(
    dataset,
    [&](ProbeOdometry& o) { o.imu_hook = fault_restarting_at(first - 0.05, restart); },
    [&](ProbeOdometry& odometry, const ReplayFrameRecord& record) {
      if (external_reset && !*reset_done && record.stamp_ns * 1e-9 > first - 0.15) {
        *reset_done = odometry.reset(synthetic_T_world_imu(min_stamp) * initial_error(), synthetic_v_world_imu(min_stamp), min_stamp);
      }
    });
}

TEST_F(ReplayRunnerTest, AFirstFrameOlderThanTheRestartedImuIsNotIntegratedAcrossTheHole) {
  // Only frames from 3.0 s on are written, so the first pose is at 3.0 s and none exists when the fault happens
  Variant variant;
  variant.keep_frame = [this](size_t i) { return dataset_->frames()[i].stamp_ns >= 3000000000; };
  const auto dataset = make_variant("first_frame_before_restart", variant);
  for (const double restart : {3.09, 3.44}) {
    SCOPED_TRACE(restart);
    const auto run = run_first_frame_before_restart(dataset, 3.0, restart, false, 0.0);
    expect_no_pose_across_the_hole(run.records, 3.0, restart);
    EXPECT_TRUE(queue_and_history_agree(*run.odometry));
    EXPECT_EQ(run.odometry->status().reset_count, 1);
    EXPECT_EQ(run.odometry->status().last_reset_reason, ResetReason::IMU_GAP);
  }
}

TEST_F(ReplayRunnerTest, AnExternalResetBeforeTheRestartedImuIsNotIntegratedAcrossTheHole) {
  // Running; the fault comes right before the frame at 3.0 s, and reset() makes that frame the first pose
  const auto dataset = make_variant("reset_before_restart", Variant());
  for (const double restart : {3.09, 3.44}) {
    SCOPED_TRACE(restart);
    const auto run = run_first_frame_before_restart(dataset, 3.0, restart, true, 2.95);
    expect_no_pose_across_the_hole(run.records, 3.0, restart);
    EXPECT_TRUE(queue_and_history_agree(*run.odometry));
    EXPECT_EQ(run.odometry->status().reset_count, 2);  // The external one, then the self-reset over the hole
    EXPECT_EQ(run.odometry->status().last_reset_reason, ResetReason::IMU_GAP);
  }
}

TEST_F(ReplayRunnerTest, ARestartedImuThatStartsJustAfterTheFirstFrameIsNotAGap) {
  // Control 1: the IMU restarts 0.04 s after the first frame, within max_imu_gap (0.05): nothing is dropped
  Variant variant;
  variant.keep_frame = [this](size_t i) { return dataset_->frames()[i].stamp_ns >= 3000000000; };
  const auto dataset = make_variant("first_frame_before_close_restart", variant);
  const auto run = run_first_frame_before_restart(dataset, 3.0, 3.04, false, 0.0);
  expect_untouched(run.records, *run.odometry);
  EXPECT_TRUE(queue_and_history_agree(*run.odometry));
}

TEST_F(ReplayRunnerTest, AnExternalResetAfterTheRestartedImuNeedsNoRecovery) {
  // Control 2: min_stamp is after the restart sample, so the first frame has IMU data at or before it
  const auto dataset = make_variant("reset_after_restart", Variant());
  for (const double restart : {3.09, 3.44}) {
    SCOPED_TRACE(restart);
    const double first = restart < 3.1 ? 3.1 : 3.5;  // The first frame at or after the restart
    const auto run = run_first_frame_before_restart(dataset, 3.0, restart, true, first - 0.05);
    for (const auto& record : run.records) {
      if (record.stamp_ns * 1e-9 > first - 1e-6) {
        EXPECT_EQ(record.result, ReplayResult::ESTIMATED) << record.stamp_ns;
        EXPECT_NE(record.state, LocalizationState::LOST) << record.stamp_ns;
      }
    }
    EXPECT_EQ(run.odometry->status().reset_count, 1);
    EXPECT_EQ(run.odometry->status().last_reset_reason, ResetReason::EXTERNAL);
  }
}

namespace {

constexpr unsigned kRandomStreams = 100;

/// Whether a frame's interval (the previous pose, this pose) overlaps the open hole (from, to) while no reset separates them
void expect_no_pose_across(const Records& records, double from, double to) {
  std::optional<double> previous;
  int previous_resets = -1;
  for (const auto& record : records) {
    if (record.result != ReplayResult::ESTIMATED) {
      continue;
    }
    const double t = record.stamp_ns * 1e-9;
    if (previous && record.reset_count == previous_resets) {
      EXPECT_FALSE(*previous < to - 1e-6 && t > from + 1e-6) << "a pose at " << t << " integrates from " << *previous << " across the hole " << from << " to " << to;
    }
    previous = t;
    previous_resets = record.reset_count;
  }
}

}  // namespace

void ReplayRunnerTest::check_gap_case(double scan, int64_t from, int64_t to) {
  Variant variant;
  variant.scan_span = scan;
  variant.num_frames = 51;  // Up to 5.0 s: the longest hole (from 4.09 s) is over by 4.6 s, the recovery two frames later
  variant.keep_frame = [](size_t i) { return dataset_->frames()[i].stamp_ns >= 3800000000; };
  variant.keep_imu = imu_hole(from, to).keep_imu;
  const auto dataset = make_variant("gap_phase", variant);
  const bool over = to - from > 500000000;

  ReplayOptions runner_options = init_stamp_options(3.8);  // The first pose at 3.8 s, the last pose before the hole at 3.9 s
  runner_options.on_imu_gap = ReplayOptions::OnImuGap::RESET;
  glim::CloudPreprocessor preprocessor;
  OdometryEstimationLocalizationCPU odometry(perturbed_start_params(), *map_);
  ReplayRunner runner(dataset, odometry, preprocessor, runner_options);
  Records runner_records;
  bool exits_3 = false;
  try {
    runner.run([&](const ReplayFrameRecord& record) { runner_records.push_back(record); });
  } catch (const InvalidDatasetError&) {
    exits_3 = true;
  }

  const auto module = run_probe(dataset, [](ProbeOdometry&) {}, nullptr, init_stamp_options(3.8));  // The runner lets the gap through: the module decides
  const bool lost = module.records.back().state == LocalizationState::LOST;
  EXPECT_EQ(exits_3, over);
  EXPECT_EQ(lost, over);
  expect_no_pose_across(runner_records, from * 1e-9, to * 1e-9);
  expect_no_pose_across(module.records, from * 1e-9, to * 1e-9);
  EXPECT_LE(module.odometry->max_hold, 0.05 + 1e-6);  // What upstream integrated across, from its real queue
  EXPECT_EQ(module.odometry->hold_check_trips(), 0u);
  EXPECT_EQ(module.odometry->uncovered_integrations, 0);
  if (over) {
    EXPECT_EQ(module.records.back().reset_count, 1);  // Only the initial pose: it never resets itself
    EXPECT_NE(module.records.back().result, ReplayResult::ESTIMATED);
  } else {
    EXPECT_EQ(module.records.back().result, ReplayResult::ESTIMATED);
    EXPECT_EQ(module.records.back().reset_count, 2);  // The initial pose, then one self-reset
    EXPECT_EQ(module.odometry->status().last_reset_reason, ResetReason::IMU_GAP);
    EXPECT_EQ(runner_records.back().reset_count, 2);
  }
}

/// The gap phase sweep, one case per (scan span in s, gap in ms): ctest runs each case as its own process, so the sweep spreads over the jobs
class ReplayRunnerGapPhaseTest : public ReplayRunnerTest, public testing::WithParamInterface<std::tuple<double, int>> {};

TEST_P(ReplayRunnerGapPhaseTest, AGapAtAnyPhaseAgainstTheFramesGetsTheSameVerdictFromTheRunnerAndTheModule) {
  // Single gaps at 5 of the 10 phases against the 10 Hz frames (even ones: the failing band of the trimmed-history regression is 3 phases wide), scan 0 and
  // 0.09 s: up to the limit the odometry resets itself and tracks, above it the runner exits 3 and the module goes LOST
  const auto [scan, gap_ms] = GetParam();
  const auto begin = std::chrono::steady_clock::now();
  for (int phase = 0; phase < 10; phase += 2) {
    SCOPED_TRACE("scan " + std::to_string(scan) + " gap " + std::to_string(gap_ms) + " ms phase " + std::to_string(phase));
    const int64_t from = 4000000000 + phase * 10000000LL;
    check_gap_case(scan, from, from + gap_ms * 1000000LL);
  }
  const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
  std::cout << "gap phase sweep, scan " << scan << " s, gap " << gap_ms << " ms: " << seconds << " s" << std::endl;
}

INSTANTIATE_TEST_SUITE_P(
  ScanAndGap,
  ReplayRunnerGapPhaseTest,
  testing::Combine(testing::Values(0.0, 0.09), testing::Values(300, 350, 400, 450, 490, 510)),
  [](const testing::TestParamInfo<std::tuple<double, int>>& info) {
    return "scan" + std::to_string(static_cast<int>(std::lround(std::get<0>(info.param) * 1000.0))) + "ms_gap" + std::to_string(std::get<1>(info.param)) + "ms";
  });

TEST_F(ReplayRunnerTest, AHoleFrom395To444SecondsWithScanZeroSelfResetsOnceAndExitsZero) {
  // Trimmed-history regression: the last pose is at 3.9 s, the hole is 0.49 s and does not start on a frame
  check_gap_case(0.0, 3950000000, 4440000000);
}

TEST_F(ReplayRunnerTest, AnExternalResetThenAnOlderFrameIsNotIntegratedAcrossTheConsumedData) {
  // Frames up to 4.9 s are processed. reset(min_stamp = t) then comes with a frame at t < 4.9 s (the frame clock stepped back), and the
  // frame at 5.1 s follows. Upstream has consumed the IMU up to 4.9 s, so a pose at t would be followed by a hold from t.
  Variant variant;
  variant.num_frames = 56;
  variant.keep_frame = [](size_t i) { return dataset_->frames()[i].stamp_ns >= 3800000000; };
  const auto dataset = make_variant("reset_then_older_frame", variant);
  for (const double age : {0.0, 0.02, 0.10, 0.50, 2.00}) {  // 0: the frame repeats the stamp of the newest one
    SCOPED_TRACE(age);
    const double t = 4.9 - age;
    auto reset_done = std::make_shared<bool>(false);
    auto previous = std::make_shared<double>(0.0);
    const auto run = run_probe(
      dataset,
      [&](ProbeOdometry& o) {
        o.frame_hook = [t, age, previous](const glim::PreprocessedFrame::Ptr& frame) {
          if (frame->stamp > 4.95 && frame->stamp < 5.05) {  // The frame at 5.0 s arrives as an older one
            frame->stamp = age == 0.0 ? *previous : t;
            frame->scan_end_time = frame->stamp;
          } else {
            *previous = frame->stamp;
          }
        };
      },
      [&](ProbeOdometry& odometry, const ReplayFrameRecord& record) {
        if (!*reset_done && record.stamp_ns == 4900000000) {
          *reset_done = odometry.reset(synthetic_T_world_imu(t) * initial_error(), synthetic_v_world_imu(t), t);
        }
      },
      init_stamp_options(3.8));
    ASSERT_TRUE(*reset_done);
    EXPECT_LE(run.odometry->max_hold, 0.05 + 1e-6);
    EXPECT_EQ(run.odometry->hold_check_trips(), 0u);
    EXPECT_EQ(run.odometry->uncovered_integrations, 0);
    for (const auto& record : run.records) {
      if (record.stamp_ns == 5000000000) {
        EXPECT_NE(record.result, ReplayResult::ESTIMATED) << "a pose at the older frame";
      }
      if (record.stamp_ns == 5100000000) {
        EXPECT_EQ(record.result, ReplayResult::ESTIMATED);  // The first pose after the reset
        EXPECT_EQ(record.reset_count, 2);  // The initial pose, then the reset
      }
      if (record.stamp_ns >= 5100000000) {
        EXPECT_NE(record.state, LocalizationState::LOST) << record.stamp_ns;
      }
    }
    EXPECT_EQ(run.records.back().result, ReplayResult::ESTIMATED);
    EXPECT_EQ(run.odometry->status().reset_count, 2);
  }
}

void ReplayRunnerTest::check_two_backward_steps(bool through_min_stamp, double t) {
  // Frames up to 4.9 s; the frame at 5.0 s arrives as an old one (through LOST: 2.0 s, through min_stamp: 2.5 s, below the initial
  // min_stamp 3.0 s), reset(min_stamp = t) follows, the frame at 5.1 s arrives as t, and the frame at 5.3 s is a normal one
  const double first_stamp = through_min_stamp ? 3.0 : 1.0;
  const double old_stamp = through_min_stamp ? 2.5 : 2.0;
  Variant variant;
  variant.num_frames = 56;
  variant.keep_frame = [first_stamp](size_t i) {
    const int64_t ns = dataset_->frames()[i].stamp_ns;
    return ns >= static_cast<int64_t>(first_stamp * 1e9) && ns != 5200000000;
  };
  const auto dataset = make_variant("two_backward_steps", variant);
  auto reset_done = std::make_shared<bool>(false);
  const auto run = run_probe(
    dataset,
    [&](ProbeOdometry& o) {
      o.frame_hook = [=](const glim::PreprocessedFrame::Ptr& frame) {
        if (std::abs(frame->stamp - 5.0) < 1e-6 || std::abs(frame->stamp - 5.1) < 1e-6) {
          frame->stamp = std::abs(frame->stamp - 5.0) < 1e-6 ? old_stamp : t;
          frame->scan_end_time = frame->stamp;
        }
      };
    },
    [&](ProbeOdometry& odometry, const ReplayFrameRecord& record) {
      if (!*reset_done && record.stamp_ns == 5000000000) {
        *reset_done = odometry.reset(synthetic_T_world_imu(t) * initial_error(), synthetic_v_world_imu(t), t);
      }
    },
    init_stamp_options(first_stamp));
  ASSERT_TRUE(*reset_done);
  EXPECT_LE(run.odometry->max_hold, 0.05 + 1e-6);
  EXPECT_EQ(run.odometry->hold_check_trips(), 0u);
  EXPECT_EQ(run.odometry->uncovered_integrations, 0);
  for (const auto& record : run.records) {
    if (record.stamp_ns == 5100000000) {
      EXPECT_NE(record.result, ReplayResult::ESTIMATED) << "a pose at the older frame";
    }
    if (record.stamp_ns == 5300000000) {
      EXPECT_EQ(record.result, ReplayResult::ESTIMATED);  // The first pose after the reset
      EXPECT_EQ(record.reset_count, 2);                   // The initial pose, then the reset
    }
    if (record.stamp_ns >= 5300000000) {
      EXPECT_NE(record.state, LocalizationState::LOST) << record.stamp_ns;
    }
  }
}

TEST_F(ReplayRunnerTest, TwoBackwardFrameStepsThroughLostWithAResetBetweenAreNotIntegratedAcrossTheHole) {
  for (const double t : {4.8, 4.4, 3.0}) {
    SCOPED_TRACE(t);
    check_two_backward_steps(false, t);
  }
}

TEST_F(ReplayRunnerTest, TwoBackwardFrameStepsThroughMinStampWithAResetBetweenAreNotIntegratedAcrossTheHole) {
  for (const double t : {4.8, 4.4, 3.6}) {
    SCOPED_TRACE(t);
    check_two_backward_steps(true, t);
  }
}

TEST_F(ReplayRunnerTest, AnExternalResetIntoThePastNeedsNoOlderFrameAndStillGivesAPose) {
  // Control: reset(min_stamp) in the past, and only newer frames follow: the next frame is the first pose
  const auto dataset = make_variant("reset_into_the_past", Variant());
  auto reset_done = std::make_shared<bool>(false);
  const auto run = run_probe(
    dataset,
    [](ProbeOdometry&) {},
    [&](ProbeOdometry& odometry, const ReplayFrameRecord& record) {
      if (!*reset_done && record.stamp_ns == 4900000000) {
        *reset_done = odometry.reset(synthetic_T_world_imu(5.0) * initial_error(), synthetic_v_world_imu(5.0), 4.0);
      }
    });
  ASSERT_TRUE(*reset_done);
  for (const auto& record : run.records) {
    if (record.stamp_ns >= 5000000000) {
      EXPECT_EQ(record.result, ReplayResult::ESTIMATED) << record.stamp_ns;
      EXPECT_EQ(record.reset_count, 1);
    }
  }
  EXPECT_LE(run.odometry->max_hold, 0.05 + 1e-6);
  EXPECT_EQ(run.odometry->hold_check_trips(), 0u);
  EXPECT_EQ(run.odometry->uncovered_integrations, 0);
}

TEST_F(ReplayRunnerTest, BothClocksStepBackThenAnExternalResetRecovers) {
  // Control: the IMU and the frame clock both go back 100 s while reset() is already queued. The frames after the step are older than the
  // newest one seen, until the IMU fault restarts the epoch
  const auto dataset = make_variant("both_clocks_back", Variant());
  auto reset_done = std::make_shared<bool>(false);
  const auto run = run_probe(
    dataset,
    [](ProbeOdometry& o) {
      o.imu_hook = [](double stamp, const Forward& forward) { forward(stamp >= 4.05 ? stamp - 100.0 : stamp); };
      o.frame_hook = [](const glim::PreprocessedFrame::Ptr& frame) {
        if (frame->stamp > 4.05) {
          frame->stamp -= 100.0;
          frame->scan_end_time -= 100.0;
        }
      };
    },
    [&](ProbeOdometry& odometry, const ReplayFrameRecord& record) {
      if (!*reset_done && record.stamp_ns == 4000000000) {
        *reset_done = odometry.reset(synthetic_T_world_imu(4.1) * initial_error(), synthetic_v_world_imu(4.1), -96.0);
      }
    });
  ASSERT_TRUE(*reset_done);
  EXPECT_EQ(run.records.back().result, ReplayResult::ESTIMATED);
  EXPECT_NE(run.records.back().state, LocalizationState::LOST);
  EXPECT_EQ(run.records.back().reset_count, 1);
  EXPECT_LE(run.odometry->max_hold, 0.05 + 1e-6);
  EXPECT_EQ(run.odometry->hold_check_trips(), 0u);
  EXPECT_EQ(run.odometry->uncovered_integrations, 0);
}

/// The random streams, split into kRandomStreamChunks cases so that ctest spreads them over its jobs. Case k runs the seeds s with
/// s % kRandomStreamChunks == k, so every seed from 1 to kRandomStreams runs exactly once.
constexpr unsigned kRandomStreamChunks = 10;
static_assert(kRandomStreams % kRandomStreamChunks == 0, "every case runs the same number of seeds");
/// Seeds above this always replay twice, so every case has streams that can take the two-old-frames path (they take it when
/// the reset and the second replay also happen, which the seed decides). Each case must have at least
/// kMinTwoOldFramesPerCase of them, or its two-old-frames check could fail for reasons unrelated to the code under test.
constexpr unsigned kForcedReplaySeedsAbove = 60;
constexpr unsigned kMinTwoOldFramesPerCase = 2;
static_assert((kRandomStreams - kForcedReplaySeedsAbove) / kRandomStreamChunks >= kMinTwoOldFramesPerCase,
              "every case has enough seeds that force two replays");

class ReplayRunnerRandomStreamsTest : public ReplayRunnerTest, public testing::WithParamInterface<unsigned> {};

TEST_P(ReplayRunnerRandomStreamsTest, RandomGapsResetsAndReplayedFramesNeverMakeUpstreamHoldAcrossAGap) {
  // Seeded streams: legal IMU holes, reset() with min_stamp in the past, frames replayed from the past (one, or two with a reset between them). Whatever happens (LOST, resets,
  // dropped frames), no published pose may have integrated a hold above max_imu_gap, measured on upstream's real queue
  const unsigned chunk = GetParam();
  Variant variant;
  variant.num_frames = 51;
  variant.keep_frame = [](size_t i) { return dataset_->frames()[i].stamp_ns >= 3800000000; };
  const auto dataset = make_variant("random_streams", variant);
  const auto begin = std::chrono::steady_clock::now();
  int streams = 0;
  int two_old_frames = 0;
  int covered = 0;
  int uncovered = 0;
  double max_uncovered = 0.0;
  std::vector<double> boundary;
  std::vector<double> uncovered_boundary;
  for (unsigned seed = chunk == 0 ? kRandomStreamChunks : chunk; seed <= kRandomStreams; seed += kRandomStreamChunks) {
    std::mt19937 rng(seed);
    const auto uniform = [&](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); };
    const int holes = static_cast<int>(rng() % 3);
    std::vector<std::pair<double, double>> hole_list;
    for (int h = 0; h < holes; h++) {
      const double from = uniform(4.0, 4.6);
      hole_list.emplace_back(from, from + uniform(0.02, 0.45));
    }
    const bool do_reset = rng() % 2 == 0;
    const double reset_at = std::floor(uniform(4.0, 4.7) * 10.0) / 10.0;
    const double min_stamp = reset_at - uniform(0.0, 1.0);
    const bool replay = rng() % 2 == 0 || seed > kForcedReplaySeedsAbove;
    const double replay_at = std::floor(uniform(4.1, 4.9) * 10.0) / 10.0;
    const double age = uniform(0.02, 0.8);
    // Drawn last, so the streams above keep their values: a second old frame after the first, with a reset between the two
    const bool replay_twice = replay && (rng() % 2 == 0 || seed > kForcedReplaySeedsAbove);
    const double replay2_at = replay_at + 0.1 * static_cast<double>(1 + rng() % 3);
    const double age2 = uniform(0.02, 2.0);
    const double mid_min_stamp = replay_at + 0.1 - uniform(0.0, 2.0);
    auto mid_reset_done = std::make_shared<bool>(false);
    auto second_replayed = std::make_shared<bool>(false);
    auto reset_done = std::make_shared<bool>(false);
    const auto run = run_probe(
      dataset,
      [&](ProbeOdometry& o) {
        o.imu_hook = [hole_list](double stamp, const Forward& forward) {
          for (const auto& hole : hole_list) {
            if (stamp > hole.first + 1e-9 && stamp < hole.second - 1e-9) {
              return;
            }
          }
          forward(stamp);
        };
        o.frame_hook = [=](const glim::PreprocessedFrame::Ptr& frame) {
          if (replay && std::abs(frame->stamp - replay_at) < 1e-6) {
            frame->stamp -= age;
            frame->scan_end_time = frame->stamp;
          } else if (replay_twice && std::abs(frame->stamp - replay2_at) < 1e-6) {
            frame->stamp -= age2;
            frame->scan_end_time = frame->stamp;
            *second_replayed = true;
          }
        };
      },
      [&](ProbeOdometry& odometry, const ReplayFrameRecord& record) {
        if (replay_twice && !*mid_reset_done && record.stamp_ns * 1e-9 > replay_at - 1e-6) {
          *mid_reset_done = odometry.reset(synthetic_T_world_imu(replay_at + 0.1) * initial_error(), synthetic_v_world_imu(replay_at + 0.1), mid_min_stamp);
        }
        if (do_reset && !*reset_done && record.stamp_ns * 1e-9 > reset_at - 1e-6) {
          *reset_done = odometry.reset(synthetic_T_world_imu(reset_at + 0.1) * initial_error(), synthetic_v_world_imu(reset_at + 0.1), min_stamp);
        }
      },
      init_stamp_options(3.8));
    EXPECT_LE(run.odometry->max_hold, 0.05 + 1e-6) << "seed " << seed;
    EXPECT_EQ(run.odometry->hold_check_trips(), 0u) << "seed " << seed;
    EXPECT_TRUE(queue_and_history_agree(*run.odometry)) << "seed " << seed;
    streams++;
    covered += run.odometry->covered_integrations;
    uncovered += run.odometry->uncovered_integrations;
    max_uncovered = std::max(max_uncovered, run.odometry->max_uncovered_hold);
    boundary.insert(boundary.end(), run.odometry->boundary_holds.begin(), run.odometry->boundary_holds.end());
    uncovered_boundary.insert(uncovered_boundary.end(), run.odometry->uncovered_boundary_holds.begin(), run.odometry->uncovered_boundary_holds.end());
    two_old_frames += replay_twice && *mid_reset_done && *second_replayed ? 1 : 0;
  }
  const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
  // Per case, so that the whole sweep still has at least 20 streams on the two-old-frames path and 3 covered integrations per stream
  EXPECT_EQ(streams, static_cast<int>(kRandomStreams / kRandomStreamChunks));
  EXPECT_GE(two_old_frames, static_cast<int>(kMinTwoOldFramesPerCase)) << "streams that took the two-old-frames path";
  EXPECT_GE(covered, 3 * streams) << "covered integrations: the oracle must not be vacuous";
  std::cout << "covered integrations: " << covered << ", uncovered (INCOMPLETE tail, not judged): " << uncovered << ", longest uncovered hold " << max_uncovered << " s" << std::endl;
  std::cout << std::setprecision(17);
  for (const double hold : boundary) {
    std::cout << "boundary hold (covered) " << hold << std::endl;
  }
  for (const double hold : uncovered_boundary) {
    std::cout << "boundary hold (uncovered) " << hold << std::endl;
  }
  std::cout << std::setprecision(6) << "two old frames: " << two_old_frames << ", random streams: " << streams << " in " << seconds << " s" << std::endl;
}

INSTANTIATE_TEST_SUITE_P(Seeds, ReplayRunnerRandomStreamsTest, testing::Range(0u, kRandomStreamChunks), [](const testing::TestParamInfo<unsigned>& info) {
  return "seed_mod" + std::to_string(kRandomStreamChunks) + "_" + std::to_string(info.param);
});

TEST(LongestImuHold, MeasuresTheLargestIntervalUpstreamIntegratesAcross) {
  struct Case {
    const char* name;
    std::vector<double> samples;
    double last;
    double frame;
    double expected;
  };
  const std::vector<Case> cases = {
    {"clean stream", {0.02, 0.04, 0.06, 0.08, 0.10}, 0.0, 0.10, 0.02},
    {"leading hold: first sample late", {0.30, 0.32, 0.34}, 0.0, 0.34, 0.30},
    {"hole behind the front", {0.02, 0.04, 0.54, 0.56}, 0.0, 0.56, 0.50},
    {"tail: last sample far before the frame", {0.02, 0.04, 0.06}, 0.0, 0.40, 0.34},
    {"samples at or before the last pose are skipped", {-1.0, -0.5, 0.0, 0.02, 0.04}, 0.0, 0.04, 0.02},
    {"a hole before the last pose does not count", {0.10, 0.60, 0.62, 0.64}, 0.60, 0.64, 0.02},
    {"samples after the frame are not used", {0.02, 0.04, 0.90}, 0.0, 0.05, 0.02},
    {"no sample in range", {-1.0, 0.0}, 0.0, 0.30, 0.30},
    {"empty queue", {}, 1.0, 1.25, 0.25},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(c.name);
    const double hold = longest_imu_hold(c.samples.size(), [&](size_t k) { return c.samples[k]; }, c.last, c.frame);
    EXPECT_NEAR(hold, c.expected, 1e-9);
  }
}

TEST_F(ReplayRunnerTest, ANonIncreasingFrameStampGoesLostInsteadOfSelfResetting) {
  // One frame repeats the stamp of the frame before it (the frame clock stalled or stepped back). In the second case an IMU hole is open
  // at that moment, so a self-reset would extrapolate the pose by v * dt with dt = 0 (or < 0 for a step back)
  for (const bool with_hole : {false, true}) {
    SCOPED_TRACE(with_hole);
    const auto dataset = make_variant("frame_clock_step", with_hole ? imu_hole(4000000000, 4200000000) : Variant());
    const double from = with_hole ? 4.15 : 4.5;
    auto previous = std::make_shared<double>(0.0);
    auto done = std::make_shared<bool>(false);
    const auto run = run_probe(dataset, [&](ProbeOdometry& o) {
      o.frame_hook = [from, previous, done](const glim::PreprocessedFrame::Ptr& frame) {
        if (!*done && frame->stamp > from) {
          *done = true;
          frame->stamp = *previous;
          frame->scan_end_time = *previous;
        }
        *previous = frame->stamp;
      };
    });
    expect_fails_closed(run.records, run.summary, *run.odometry);
  }
}

TEST_F(ReplayRunnerTest, AForwardJumpThroughTheAsyncWrapperFailsClosed) {
  // AsyncOdometryEstimation releases a frame as soon as the last inserted stamp passes its end, so a jumped stamp releases every queued frame
  auto odometry = std::make_shared<ProbeOdometry>(perturbed_start_params(), *map_);
  glim::AsyncOdometryEstimation async(odometry, true);
  glim::CloudPreprocessor preprocessor;
  const auto& imu = dataset_->imu();
  size_t fed = 0;
  bool jumped = false;
  for (size_t index = 0; index < dataset_->frames().size(); index++) {
    const auto frame = preprocessor.preprocess(dataset_->load_cloud(index));
    size_t target = 0;
    while (target < imu.size() && imu[target].stamp_ns * 1e-9 <= frame->scan_end_time) {
      target++;
    }
    target = std::min(target + 1, imu.size());
    for (; fed < target; fed++) {
      double stamp = imu[fed].stamp_ns * 1e-9;
      if (!jumped && stamp >= 4.0) {
        jumped = true;
        stamp += 1000.0;
      }
      async.insert_imu(stamp, imu[fed].acc, imu[fed].gyro);
    }
    async.insert_frame(frame);
  }
  async.join();
  std::vector<glim::EstimationFrame::ConstPtr> results, marginalized;
  async.get_results(results, marginalized);

  EXPECT_EQ(odometry->status().state, LocalizationState::LOST);
  EXPECT_EQ(odometry->status().reset_count, 0);
  for (const auto& result : results) {
    if (result) {
      EXPECT_LE(result->stamp, 4.0 + 1e-6) << "a pose was published after the jump";
    }
  }
  EXPECT_TRUE(queue_and_history_agree(*odometry));
}

TEST_F(ReplayRunnerTest, AnExternalResetWinsOverASelfReset) {
  // 0.2 s gap. At the moment the odometry decides to reset itself, an external pose arrives
  const auto dataset = make_variant("external_vs_self", imu_hole(4000000000, 4200000000));
  ReplayOptions options;
  options.on_imu_gap = ReplayOptions::OnImuGap::RESET;
  glim::CloudPreprocessor preprocessor;
  OdometryEstimationLocalizationCPU odometry(perturbed_start_params(), *map_);
  auto sink = std::make_shared<TriggerSink>();
  sink->text = "resetting the pose after an IMU gap";
  bool accepted = false;
  sink->action = [&] { accepted = odometry.reset(synthetic_T_world_imu(4.3) * initial_error(), synthetic_v_world_imu(4.3), 4.3); };
  spdlog::default_logger()->sinks().push_back(sink);
  ReplayRunner runner(dataset, odometry, preprocessor, options);
  Records records;
  ReplaySummary summary;
  try {
    summary = runner.run([&](const ReplayFrameRecord& record) { records.push_back(record); });
  } catch (...) {
    spdlog::default_logger()->sinks().pop_back();
    throw;
  }
  spdlog::default_logger()->sinks().pop_back();
  ASSERT_TRUE(accepted) << "the self-reset never happened";
  const auto status = odometry.status();
  EXPECT_EQ(status.reset_count, 1);  // Only the external reset: it was not lost, and the self-reset did not follow it
  EXPECT_EQ(status.last_reset_reason, ResetReason::EXTERNAL);
  EXPECT_EQ(summary.imu_gap_resets, 0);
  EXPECT_EQ(records.back().result, ReplayResult::ESTIMATED);
}

TEST_F(ReplayRunnerTest, ProcessingTimeIsSetPerFrame) {
  glim::CloudPreprocessor preprocessor;
  OdometryEstimationLocalizationCPU odometry(perturbed_start_params(), *map_);
  ReplayOptions options;
  options.max_frames = 12;
  ReplayRunner runner(*dataset_, odometry, preprocessor, options);
  runner.run(nullptr);
  const double time = odometry.status().processing_time;
  EXPECT_GT(time, 0.0);
  EXPECT_LT(time, 10.0);
}

TEST_F(ReplayRunnerTest, ALostOdometryStaysLostUntilAnExternalResetClearsIt) {
  const auto dataset = make_variant("lost_then_reset", imu_hole(4000000000, 4510000000));
  for (const bool reset : {false, true}) {
    SCOPED_TRACE(reset);
    ReplayOptions options;
    options.on_imu_gap = ReplayOptions::OnImuGap::RESET;
    options.imu_gap_reset_max = 10.0;  // The odometry's own limit (0.5 s) decides
    glim::CloudPreprocessor preprocessor;
    OdometryEstimationLocalizationCPU odometry(perturbed_start_params(), *map_);
    ReplayRunner runner(dataset, odometry, preprocessor, options);
    Records records;
    bool lost_seen = false, reset_done = false;
    runner.run([&](const ReplayFrameRecord& record) {
      records.push_back(record);
      if (record.state == LocalizationState::LOST && !lost_seen) {
        lost_seen = true;
        if (reset) {
          const double t = record.stamp_ns * 1e-9 + 0.05;
          reset_done = odometry.reset(synthetic_T_world_imu(t + 0.05) * initial_error(), synthetic_v_world_imu(t + 0.05), t);
        }
      }
    });
    ASSERT_TRUE(lost_seen);
    bool after_lost = false;
    int estimated_after = 0;
    for (const auto& record : records) {
      if (record.state == LocalizationState::LOST) {
        after_lost = true;
      }
      if (after_lost && record.result == ReplayResult::ESTIMATED) {
        estimated_after++;
        EXPECT_NE(record.state, LocalizationState::LOST);
      }
      if (after_lost && !reset) {
        EXPECT_EQ(record.state, LocalizationState::LOST) << record.stamp_ns;  // Sticky: the IMU is clean again, but nothing recovers it
        EXPECT_NE(record.result, ReplayResult::ESTIMATED) << record.stamp_ns;
      }
    }
    if (reset) {
      ASSERT_TRUE(reset_done);
      EXPECT_GT(estimated_after, 10);
      EXPECT_EQ(records.back().result, ReplayResult::ESTIMATED);
      EXPECT_EQ(records.back().reset_count, 1);
      EXPECT_EQ(odometry.status().last_reset_reason, ResetReason::EXTERNAL);
    } else {
      EXPECT_EQ(estimated_after, 0);
      // While lost the IMU history must not grow with the rest of the run (about 750 samples at 100 Hz)
      EXPECT_LT(odometry.imu_stamp_history_size(), 100u);
    }
  }
}

namespace {

OdometryEstimationLocalizationCPU::Params wheel_params() {
  auto params = perturbed_start_params();
  params.wheel_odom.enabled = true;  // The synthetic wheel base is the IMU frame (identity extrinsic)
  return params;
}

/// Number of wheel samples with stamp in [from_ns, to_ns], plus the one after
int expected_wheel_fed(const ReplayDataset& dataset, int64_t from_ns, int64_t to_ns) {
  const auto& wheel = dataset.wheel_odom();
  int count = 0;
  size_t i = 0;
  for (; i < wheel.size() && wheel[i].stamp_ns <= to_ns; i++) {
    count += wheel[i].stamp_ns >= from_ns ? 1 : 0;
  }
  return count + (i < wheel.size() ? 1 : 0);
}

}  // namespace

TEST_F(ReplayRunnerTest, FeedsWheelOdometryUpToFrameStampPlusOneSample) {
  ASSERT_TRUE(dataset_->has_wheel_odom());
  ReplayOptions options;
  options.max_frames = 3;
  const auto output = run(*dataset_, wheel_params(), options);
  ASSERT_EQ(output.records.size(), 3u);
  const auto& frames = dataset_->frames();
  // The first frame gets the samples up to its stamp (none before 0 s) and the next one; later frames continue from there
  EXPECT_EQ(output.records[0].num_wheel_fed, expected_wheel_fed(*dataset_, 0, frames[0].stamp_ns));
  EXPECT_EQ(output.records[0].num_wheel_fed + output.records[1].num_wheel_fed, expected_wheel_fed(*dataset_, 0, frames[1].stamp_ns));
  EXPECT_EQ(output.records[1].num_wheel_fed, 5);  // 50 Hz over 0.1 s
  EXPECT_EQ(output.records[2].num_wheel_fed, 5);
  EXPECT_EQ(output.records[0].wheel, ReplayWheel::NONE);  // No previous frame
  EXPECT_EQ(output.records[1].wheel, ReplayWheel::ADDED);
  EXPECT_EQ(output.records[2].wheel, ReplayWheel::ADDED);
  ASSERT_TRUE(output.summary.wheel_odom);
  EXPECT_EQ(output.summary.wheel_odom->fed, output.records[0].num_wheel_fed + 10);
  EXPECT_EQ(output.summary.wheel_odom->added, 2);
  EXPECT_EQ(output.summary.wheel_odom->gated, 0);
  EXPECT_EQ(output.summary.wheel_odom->missing, 0);
  EXPECT_EQ(output.summary.wheel_odom->rejected, 0);
}

TEST_F(ReplayRunnerTest, SkipsWheelOdometryLongBeforeTheFirstFrame) {
  ReplayOptions options;
  options.start = 5.0;
  options.max_frames = 2;
  const auto output = run(*dataset_, wheel_params(), options);
  ASSERT_EQ(output.records.size(), 2u);
  const int64_t first = output.records[0].stamp_ns;
  EXPECT_EQ(output.records[0].num_wheel_fed, expected_wheel_fed(*dataset_, first - 1000000000, first));
  EXPECT_GT(output.records[0].num_wheel_fed, 40);  // 1 s at 50 Hz
  EXPECT_LT(output.records[0].num_wheel_fed, 60);
}

TEST_F(ReplayRunnerTest, WheelOdometryIsFedOnlyWhenEnabled) {
  ReplayOptions options;
  options.max_frames = 3;
  const auto disabled = run(*dataset_, perturbed_start_params(), options);
  options.feed_wheel_odom = false;
  const auto not_fed = run(*dataset_, wheel_params(), options);
  for (const auto* output : {&disabled, &not_fed}) {
    EXPECT_FALSE(output->summary.wheel_odom);
    for (const auto& record : output->records) {
      EXPECT_EQ(record.num_wheel_fed, 0);
      EXPECT_EQ(record.wheel, ReplayWheel::NONE);
    }
  }
  // A dataset without wheel_odom.csv runs as before
  Variant variant;
  variant.num_frames = 3;
  const auto without = run(make_variant("no_wheel", variant), wheel_params(), options);
  EXPECT_FALSE(without.summary.wheel_odom);
  EXPECT_EQ(without.summary.frames_by_result.at("estimated"), 3);
}

TEST_F(ReplayRunnerTest, WheelOdometryKeepsAccuracy) {
  const auto output = run(*dataset_, wheel_params());
  ASSERT_EQ(output.records.size(), dataset_->frames().size());
  ASSERT_TRUE(output.summary.wheel_odom);
  EXPECT_EQ(output.summary.wheel_odom->added, static_cast<int>(output.records.size()) - 1);
  EXPECT_EQ(output.summary.wheel_odom->gated, 0);
  EXPECT_EQ(output.summary.wheel_odom->missing, 0);
  for (const auto& record : output.records) {
    if (record.stamp_ns * 1e-9 < kSettleTime) {
      continue;
    }
    ASSERT_TRUE(record.T_world_imu);
    const Eigen::Isometry3d truth = synthetic_T_world_imu(record.stamp_ns * 1e-9);
    EXPECT_LT((record.T_world_imu->translation() - truth.translation()).norm(), kMaxTranslationError) << record.stamp_ns;
    const double angle = Eigen::AngleAxisd(record.T_world_imu->linear().transpose() * truth.linear()).angle();
    EXPECT_LT(angle * 180.0 / M_PI, kMaxRotationErrorDeg) << record.stamp_ns;
  }
}

TEST_F(ReplayRunnerTest, WriterReportsWheelOdometry) {
  const fs::path out = *dir_ / "wheel_out";
  ReplayOptions options;
  options.max_frames = 3;
  glim::CloudPreprocessor preprocessor;
  OdometryEstimationLocalizationCPU odometry(wheel_params(), *map_);
  ReplayRunner runner(*dataset_, odometry, preprocessor, options);
  ReplayOutputWriter writer(out.string());
  const auto summary = runner.run([&](const ReplayFrameRecord& record) { writer.write(record); });
  writer.write_summary(summary, 0.0);

  std::ifstream frames(out / "frames.csv");
  std::string line;
  std::getline(frames, line);
  EXPECT_EQ(line.substr(line.size() - std::string(",num_wheel_fed,wheel").size()), ",num_wheel_fed,wheel");
  std::vector<std::string> last_fields;
  while (std::getline(frames, line)) {
    last_fields.push_back(line.substr(line.rfind(',') + 1));
  }
  EXPECT_EQ(last_fields, (std::vector<std::string>{"none", "added", "added"}));

  const auto json = nlohmann::json::parse(std::ifstream(out / "summary.json"));
  EXPECT_EQ(json.at("wheel_odom").at("added"), 2);
  EXPECT_EQ(json.at("wheel_odom").at("fed"), summary.wheel_odom->fed);
  EXPECT_EQ(json.at("wheel_odom").at("gated"), 0);
}
