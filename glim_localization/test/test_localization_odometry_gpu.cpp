// GPU localization odometry tests, built only with BUILD_WITH_CUDA=ON. They run the GPU module on the same synthetic
// data and with the same accuracy limits as test_localization_odometry (CPU). Without a GPU the tests skip
// (ctest reports them as skipped), unless GLIM_LOC_REQUIRE_GPU=1.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <unistd.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam_points/cuda/nonlinear_factor_set_gpu.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>

#include <glim/odometry/odometry_estimation_base.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/util/config.hpp>

#include <glim_localization/map/gpu_tile_cache.hpp>
#include <glim_localization/map/map_manager.hpp>
#include <glim_localization/odometry/localization_backend.hpp>
#include <glim_localization/odometry/odometry_estimation_localization_gpu.hpp>
#include <glim_localization/odometry/weighted_vgicp_factor_gpu.hpp>

#include "synthetic/synthetic_map.hpp"
#include "synthetic/synthetic_scene.hpp"
#include "synthetic/synthetic_sensor.hpp"

namespace fs = std::filesystem;
using namespace glim_localization;
using namespace glim_localization::test;
using gtsam::symbol_shorthand::X;

namespace {

constexpr double kEndTime = 12.0;
constexpr double kSettleTime = 3.0;
constexpr double kMaxTranslationError = 0.1;
constexpr double kMaxRotationErrorDeg = 1.0;

// GLIM_LOC_REQUIRE_GPU=1 (set by scripts/ci.sh when nvidia-smi lists a GPU) turns a skip into a failure.
bool require_gpu() {
  const char* value = std::getenv("GLIM_LOC_REQUIRE_GPU");
  return value != nullptr && std::string(value) == "1";
}

bool gpu_visible(std::string* reason) {
  int count = 0;
  const cudaError_t error = cudaGetDeviceCount(&count);
  if (error != cudaSuccess) {
    *reason = std::string("cudaGetDeviceCount: ") + cudaGetErrorString(error);
    return false;
  }
  if (count == 0) {
    *reason = "no CUDA device";
  }
  return count > 0;
}

#define SKIP_WITHOUT_GPU()                                                    \
  do {                                                                        \
    std::string reason;                                                       \
    if (!gpu_visible(&reason)) {                                              \
      if (require_gpu()) {                                                    \
        FAIL() << "GLIM_LOC_REQUIRE_GPU=1 but no GPU is usable: " << reason;  \
      }                                                                       \
      GTEST_SKIP() << "no GPU: " << reason;                                   \
    }                                                                         \
  } while (false)

Eigen::Isometry3d initial_error() {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(0.3, -0.2, 0.1);
  T.linear() = (Eigen::AngleAxisd(3.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0.5 * M_PI / 180.0, Eigen::Vector3d::UnitY()))
                 .toRotationMatrix();
  return T;
}

/// The same p95 as test_localization_odometry (CPU) prints
double p95(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  return values[std::min(values.size() - 1, values.size() * 95 / 100)];
}

std::string gpu_config_dir() {
  return (fs::path(GLIM_LOCALIZATION_CONFIG_DIR) / "gpu").string();
}

}  // namespace

class LocalizationOdometryGPUTest : public testing::Test {
protected:
  static void SetUpTestSuite() {
    glim::GlobalConfig::instance(gpu_config_dir(), true);
    dir_ = new fs::path(fs::temp_directory_path() / ("glim_loc_odometry_gpu_" + std::to_string(::getpid())));
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

  static OdometryEstimationLocalizationGPU::Params params_at(const Eigen::Isometry3d& T_world_imu) {
    OdometryEstimationLocalizationGPU::Params params;
    params.estimate_init_state = false;
    params.init_T_world_imu = T_world_imu;
    params.init_v_world_imu.setZero();
    params.num_threads = 2;
    return params;
  }

  struct RunResult {
    double max_translation = 0.0;
    double max_rotation_deg = 0.0;
    double tail_rms = 0.0;
    std::vector<double> latencies;
  };

  /// Feed all frames; errors after kSettleTime are checked against the CPU limits.
  static RunResult run(OdometryEstimationLocalization& odometry) {
    RunResult result;
    size_t imu_cursor = 0;
    double sum_sq = 0.0;
    int num_tail = 0;
    for (const auto& frame : *frames_) {
      while (imu_cursor < imu_->size() && (*imu_)[imu_cursor].stamp <= frame->stamp + 0.02) {
        const auto& sample = (*imu_)[imu_cursor++];
        odometry.insert_imu(sample.stamp, sample.linear_acc, sample.angular_vel);
      }
      std::vector<glim::EstimationFrame::ConstPtr> marginalized;
      const auto estimate = odometry.insert_frame(frame, marginalized);
      EXPECT_TRUE(estimate) << "t=" << frame->stamp;
      if (!estimate) {
        return result;
      }
      EXPECT_TRUE(estimate->T_world_imu.matrix().allFinite()) << "t=" << frame->stamp;
      result.latencies.push_back(odometry.status().processing_time);

      const Eigen::Isometry3d error = synthetic_T_world_imu(frame->stamp).inverse() * estimate->T_world_imu;
      const double translation = error.translation().norm();
      const double rotation_deg = Eigen::AngleAxisd(error.linear()).angle() * 180.0 / M_PI;
      if (frame->stamp >= kSettleTime) {
        result.max_translation = std::max(result.max_translation, translation);
        result.max_rotation_deg = std::max(result.max_rotation_deg, rotation_deg);
        EXPECT_LT(translation, kMaxTranslationError) << "t=" << frame->stamp;
        EXPECT_LT(rotation_deg, kMaxRotationErrorDeg) << "t=" << frame->stamp;
      }
      if (frame->stamp >= kEndTime - 2.0) {
        sum_sq += translation * translation;
        num_tail++;
      }
    }
    result.tail_rms = std::sqrt(sum_sq / std::max(num_tail, 1));
    return result;
  }

  static void print(const std::string& name, const RunResult& result, const LocalizationStatus& status) {
    double mean = 0.0;
    for (const double t : result.latencies) {
      mean += t;
    }
    mean /= std::max<size_t>(result.latencies.size(), 1);
    std::cout << name << ": max_translation=" << result.max_translation << " m, max_rotation=" << result.max_rotation_deg
              << " deg, tail_rms=" << result.tail_rms << " m, inlier_ratio=" << status.inlier_ratio << ", residual=" << status.residual
              << ", latency_mean=" << mean * 1e3 << " ms, latency_p95=" << p95(result.latencies) * 1e3
              << " ms, frames=" << result.latencies.size() << std::endl;
  }

  static fs::path* dir_;
  static std::shared_ptr<const MapManager>* map_;
  static std::vector<glim::PreprocessedFrame::Ptr>* frames_;
  static std::vector<SyntheticImuSample>* imu_;
};

fs::path* LocalizationOdometryGPUTest::dir_ = nullptr;
std::shared_ptr<const MapManager>* LocalizationOdometryGPUTest::map_ = nullptr;
std::vector<glim::PreprocessedFrame::Ptr>* LocalizationOdometryGPUTest::frames_ = nullptr;
std::vector<SyntheticImuSample>* LocalizationOdometryGPUTest::imu_ = nullptr;

TEST_F(LocalizationOdometryGPUTest, ConvergesFromPerturbedInitialPose) {
  SKIP_WITHOUT_GPU();
  auto params = params_at(synthetic_T_world_imu(0.0) * initial_error());
  params.use_keyframes = false;
  OdometryEstimationLocalizationGPU odometry(params, *map_);
  EXPECT_STREQ(odometry.backend_name(), "gpu");

  const auto result = run(odometry);
  const auto status = odometry.status();
  print("gpu", result, status);

  EXPECT_EQ(status.state, LocalizationState::TRACKING) << to_string(status.state);
  EXPECT_GT(status.inlier_ratio, 0.5);
  EXPECT_GT(status.num_map_factors, 0);
  EXPECT_GT(status.residual, 0.0);
  EXPECT_TRUE(odometry.keyframes().empty());
  EXPECT_GT(odometry.tile_cache().size(), 0u);
  EXPECT_LE(odometry.tile_cache().size(), static_cast<size_t>(params.map_gpu_max_tiles));
}

TEST_F(LocalizationOdometryGPUTest, ConvergesWithKeyframes) {
  SKIP_WITHOUT_GPU();
  auto params = params_at(synthetic_T_world_imu(0.0) * initial_error());
  params.use_keyframes = true;
  OdometryEstimationLocalizationGPU odometry(params, *map_);

  const auto result = run(odometry);
  const auto status = odometry.status();
  print("gpu+keyframes", result, status);

  EXPECT_EQ(status.state, LocalizationState::TRACKING) << to_string(status.state);
  EXPECT_GT(status.inlier_ratio, 0.5);
  EXPECT_FALSE(odometry.keyframes().empty());
  EXPECT_LE(odometry.keyframes().size(), static_cast<size_t>(params.max_num_keyframes));
}

TEST_F(LocalizationOdometryGPUTest, ConvergesWithReducedMapWeight) {
  SKIP_WITHOUT_GPU();
  // A weight other than 1 goes through WeightedVGICPFactorGPU
  auto params = params_at(synthetic_T_world_imu(0.0) * initial_error());
  params.map_factor_weight = 0.5;
  OdometryEstimationLocalizationGPU odometry(params, *map_);
  run(odometry);
  EXPECT_EQ(odometry.status().state, LocalizationState::TRACKING) << to_string(odometry.status().state);
}

TEST_F(LocalizationOdometryGPUTest, WaitsForInitialPoseThenConverges) {
  SKIP_WITHOUT_GPU();
  OdometryEstimationLocalizationGPU::Params params;
  params.estimate_init_state = true;  // No configured pose: wait for reset()
  params.num_threads = 2;
  params.use_keyframes = true;
  OdometryEstimationLocalizationGPU odometry(params, *map_);

  size_t imu_cursor = 0;
  const auto feed_imu = [&](double stamp) {
    while (imu_cursor < imu_->size() && (*imu_)[imu_cursor].stamp <= stamp + 0.02) {
      odometry.insert_imu((*imu_)[imu_cursor].stamp, (*imu_)[imu_cursor].linear_acc, (*imu_)[imu_cursor].angular_vel);
      imu_cursor++;
    }
  };

  std::vector<glim::EstimationFrame::ConstPtr> marginalized;
  constexpr size_t kResetIndex = 5;
  for (size_t i = 0; i < kResetIndex; i++) {
    feed_imu((*frames_)[i]->stamp);
    EXPECT_FALSE(odometry.insert_frame((*frames_)[i], marginalized));
  }
  EXPECT_EQ(odometry.status().state, LocalizationState::INITIALIZING);

  const double reset_stamp = (*frames_)[kResetIndex]->stamp;
  odometry.reset(synthetic_T_world_imu(reset_stamp) * initial_error(), Eigen::Vector3d::Zero(), reset_stamp);

  constexpr size_t kSecondReset = 60;
  ASSERT_LT(kSecondReset, frames_->size());
  glim::EstimationFrame::ConstPtr last;
  for (size_t i = kResetIndex; i < frames_->size(); i++) {
    if (i == kSecondReset) {
      // A reset while running drops the keyframes of the old run
      const double stamp = (*frames_)[i]->stamp;
      odometry.reset(synthetic_T_world_imu(stamp) * initial_error(), Eigen::Vector3d::Zero(), stamp);
    }
    feed_imu((*frames_)[i]->stamp);
    last = odometry.insert_frame((*frames_)[i], marginalized);
    ASSERT_TRUE(last) << "t=" << (*frames_)[i]->stamp;
  }

  const Eigen::Isometry3d error = synthetic_T_world_imu(last->stamp).inverse() * last->T_world_imu;
  EXPECT_LT(error.translation().norm(), kMaxTranslationError);
  EXPECT_LT(Eigen::AngleAxisd(error.linear()).angle() * 180.0 / M_PI, kMaxRotationErrorDeg);
  EXPECT_EQ(odometry.status().state, LocalizationState::TRACKING) << to_string(odometry.status().state);
  EXPECT_EQ(odometry.status().reset_count, 2);
}

TEST_F(LocalizationOdometryGPUTest, YawSearchCorrectsALargeYawError) {
  SKIP_WITHOUT_GPU();
  // The search scores with CPU factors in the GPU module too; then the GPU odometry starts from the registered pose
  Eigen::Isometry3d init = synthetic_T_world_imu(0.0) * initial_error();
  init.linear() = Eigen::AngleAxisd(75.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix() * init.linear();
  auto params = params_at(init);
  params.map_factors.num_threads = 2;
  params.init_yaw_search.enabled = true;
  params.init_yaw_search.range = M_PI;
  params.init_yaw_search.step = 15.0 * M_PI / 180.0;
  params.init_yaw_search.time_budget = 1e6;  // Not the subject here; a loaded CI machine must not cut the search short
  OdometryEstimationLocalizationGPU odometry(params, *map_);

  const auto result = run(odometry);
  const auto status = odometry.status();
  print("gpu, yaw search", result, status);
  std::cout << "gpu, yaw search: " << status.init_search_candidates << " of " << status.init_search_candidates_total << " candidates in "
            << status.init_search_time * 1e3 << " ms, score " << status.init_search_score << ", runner-up " << status.init_search_runner_up_score << std::endl;
  EXPECT_EQ(status.init_search, InitSearchResult::ACCEPTED);
  EXPECT_EQ(status.init_search_candidates, 24);
  EXPECT_EQ(status.init_search_candidates_total, 24);
  EXPECT_NEAR(status.init_search_yaw_offset * 180.0 / M_PI, -78.0, 2.0);  // 75 deg plus the 3 deg of initial_error()
  EXPECT_EQ(status.state, LocalizationState::TRACKING) << to_string(status.state);
}

TEST_F(LocalizationOdometryGPUTest, RejectsTooFewGpuTiles) {
  auto params = params_at(Eigen::Isometry3d::Identity());
  params.map_gpu_max_tiles = params.map_factors.max_submaps - 1;
  // Rejected before anything touches the GPU
  EXPECT_THROW(OdometryEstimationLocalizationGPU(params, *map_), std::invalid_argument);
}

TEST_F(LocalizationOdometryGPUTest, FailsClosedWithoutAVisibleDevice) {
  // Hide every device before the first CUDA call of this process (ctest runs each test in its own process). A CUDA
  // build started without a GPU must not run on invalid device memory: gtsam_points only warns on CUDA errors.
  setenv("CUDA_VISIBLE_DEVICES", "", 1);
  std::string reason;
  if (gpu_visible(&reason)) {
    GTEST_SKIP() << "the CUDA runtime was initialized before the devices could be hidden";
  }
  const auto params = params_at(Eigen::Isometry3d::Identity());
  EXPECT_THROW(OdometryEstimationLocalizationGPU(params, *map_), std::runtime_error);
}

TEST_F(LocalizationOdometryGPUTest, MapFactorMatchesTheCpuFactor) {
  SKIP_WITHOUT_GPU();
  // The GPU tile against the CPU tile it was uploaded from, with the same source and pose: the same correspondences
  // and the same cost up to float precision, on every level.
  const auto tile = (*map_)->tiles().front();
  GpuTileCache cache(1, nullptr);
  const auto gpu_tile = cache.get(tile);
  ASSERT_EQ(gpu_tile->voxelmaps.size(), tile->voxelmaps.size());

  const auto source_gpu = gtsam_points::PointCloudGPU::clone(*tile->points);
  const gtsam::Pose3 T_world_origin(tile->T_world_origin.matrix());
  const gtsam::Pose3 delta(gtsam::Rot3::Rz(0.01), gtsam::Point3(0.05, -0.03, 0.02));
  gtsam::Values values;
  values.insert(X(0), T_world_origin * delta);

  for (size_t level = 0; level < tile->voxelmaps.size(); level++) {
    const auto cpu = std::make_shared<gtsam_points::IntegratedVGICPFactor>(T_world_origin, X(0), tile->voxelmaps[level], tile->points);
    const auto gpu = std::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(T_world_origin, X(0), gpu_tile->voxelmaps[level], source_gpu);
    gtsam::NonlinearFactorGraph graph;
    graph.add(gpu);
    gtsam_points::NonlinearFactorSetGPU factor_set;
    factor_set.add(graph);
    factor_set.linearize(values);
    factor_set.error(values);

    const auto cpu_linear = std::dynamic_pointer_cast<gtsam::HessianFactor>(cpu->linearize(values));
    const auto gpu_linear = std::dynamic_pointer_cast<gtsam::HessianFactor>(gpu->linearize(values));
    ASSERT_TRUE(cpu_linear);
    ASSERT_TRUE(gpu_linear);
    const double cpu_error = cpu->error(values);
    ASSERT_GT(cpu_error, 0.0);
    EXPECT_NEAR(gpu->error(values), cpu_error, 1e-3 * cpu_error) << "level " << level;
    const gtsam::Matrix H_cpu = cpu_linear->information();
    const gtsam::Matrix H_gpu = gpu_linear->information();
    EXPECT_LT((H_gpu - H_cpu).norm(), 1e-3 * H_cpu.norm()) << "level " << level << "\n" << H_gpu << "\n" << H_cpu;
    const gtsam::Vector b_cpu = cpu_linear->linearTerm();
    EXPECT_LT((gpu_linear->linearTerm() - b_cpu).norm(), 1e-3 * b_cpu.norm() + 1e-9) << "level " << level;
  }
}

TEST_F(LocalizationOdometryGPUTest, TileCacheEvictsLeastRecentlyUsed) {
  SKIP_WITHOUT_GPU();
  const auto tiles = (*map_)->tiles();
  ASSERT_GE(tiles.size(), 4u);

  GpuTileCache cache(2, nullptr);
  const auto first = cache.get(tiles[0]);
  ASSERT_EQ(first->voxelmaps.size(), tiles[0]->voxelmaps.size());
  for (size_t i = 0; i < first->voxelmaps.size(); i++) {
    EXPECT_DOUBLE_EQ(first->voxelmaps[i]->voxel_resolution(), tiles[0]->voxelmaps[i]->voxel_resolution());
  }
  EXPECT_GT(first->memory_usage_gpu, 0u);

  cache.get(tiles[1]);
  EXPECT_EQ(cache.get(tiles[0]), first);  // Resident: no upload, and now the most recently used
  EXPECT_EQ(cache.num_uploads(), 2u);

  cache.get(tiles[2]);  // Evicts tiles[1]
  EXPECT_EQ(cache.size(), 2u);
  EXPECT_TRUE(cache.contains(tiles[0]->id));
  EXPECT_FALSE(cache.contains(tiles[1]->id));
  EXPECT_TRUE(cache.contains(tiles[2]->id));
  EXPECT_EQ(cache.num_evictions(), 1u);

  cache.get(tiles[3]);
  EXPECT_EQ(cache.size(), 2u);
  EXPECT_EQ(cache.num_uploads(), 4u);
  EXPECT_EQ(cache.num_evictions(), 2u);
  EXPECT_GT(cache.memory_usage_gpu(), 0u);
  // An evicted tile stays valid while someone holds it
  EXPECT_FALSE(cache.contains(tiles[0]->id));
  EXPECT_GT(first->points->size(), 0u);

  cache.clear();
  EXPECT_EQ(cache.size(), 0u);
  EXPECT_THROW(GpuTileCache(0, nullptr), std::invalid_argument);
}

TEST_F(LocalizationOdometryGPUTest, WeightedFactorScalesTheFactor) {
  SKIP_WITHOUT_GPU();
  const auto tile = (*map_)->tiles().front();
  GpuTileCache cache(1, nullptr);
  const auto gpu_tile = cache.get(tile);

  // The tile's own points, slightly displaced, as the source
  const auto source = gtsam_points::PointCloudGPU::clone(*tile->points);
  const gtsam::Pose3 T_world_origin(tile->T_world_origin.matrix());
  const gtsam::Pose3 delta(gtsam::Rot3::Rz(0.01), gtsam::Point3(0.05, -0.03, 0.02));
  gtsam::Values values;
  values.insert(X(0), T_world_origin * delta);
  values.insert(X(1), T_world_origin * delta);

  constexpr double kWeight = 0.25;
  const auto plain = std::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(T_world_origin, X(0), gpu_tile->voxelmaps.front(), source);
  const auto weighted = std::make_shared<WeightedVGICPFactorGPU>(T_world_origin, X(1), gpu_tile->voxelmaps.front(), source, kWeight);
  EXPECT_THROW(WeightedVGICPFactorGPU(T_world_origin, X(1), gpu_tile->voxelmaps.front(), source, 0.0), std::invalid_argument);

  gtsam::NonlinearFactorGraph graph;
  graph.add(plain);
  graph.add(weighted);
  gtsam_points::NonlinearFactorSetGPU factor_set;
  factor_set.add(graph);
  factor_set.linearize(values);
  factor_set.error(values);

  const double plain_error = plain->error(values);
  ASSERT_GT(plain_error, 0.0);
  EXPECT_NEAR(weighted->error(values), kWeight * plain_error, 1e-6 * plain_error);

  const auto plain_linear = std::dynamic_pointer_cast<gtsam::HessianFactor>(plain->linearize(values));
  const auto weighted_linear = std::dynamic_pointer_cast<gtsam::HessianFactor>(weighted->linearize(values));
  ASSERT_TRUE(plain_linear);
  ASSERT_TRUE(weighted_linear);
  const gtsam::Matrix H_plain = plain_linear->information();
  const gtsam::Matrix H_weighted = weighted_linear->information();
  EXPECT_TRUE(H_weighted.isApprox(kWeight * H_plain, 1e-6)) << H_weighted << "\n" << H_plain;
  EXPECT_TRUE(weighted_linear->linearTerm().isApprox(kWeight * plain_linear->linearTerm(), 1e-6));
  EXPECT_NEAR(weighted_linear->constantTerm(), kWeight * plain_linear->constantTerm(), 1e-6 * std::abs(plain_linear->constantTerm()) + 1e-12);

  const auto clone = std::dynamic_pointer_cast<WeightedVGICPFactorGPU>(weighted->clone());
  ASSERT_TRUE(clone);
  EXPECT_DOUBLE_EQ(clone->weight(), kWeight);
}

TEST_F(LocalizationOdometryGPUTest, FactoryFollowsTheConfig) {
  SKIP_WITHOUT_GPU();
  EXPECT_TRUE(gpu_backend_available());
  ASSERT_EQ(localization_backend_from_config(), LocalizationBackend::GPU);
  const auto params = make_localization_params(LocalizationBackend::GPU);
  params->estimate_init_state = false;
  const auto odometry = make_localization_odometry(*params, *map_);
  ASSERT_TRUE(dynamic_cast<OdometryEstimationLocalizationGPU*>(odometry.get()));
  EXPECT_STREQ(odometry->backend_name(), "gpu");
}

TEST(LocalizationPluginGPUTest, LoadsModuleFromConfiguredMap) {
  SKIP_WITHOUT_GPU();
  const fs::path dir = fs::temp_directory_path() / ("glim_loc_plugin_gpu_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  // Restore the config and remove the directory even when an assertion returns early
  struct Cleanup {
    fs::path dir;
    ~Cleanup() {
      glim::GlobalConfig::instance(gpu_config_dir(), true);
      std::error_code ec;
      fs::remove_all(dir, ec);
    }
  } cleanup{dir};
  const fs::path config_dir = dir / "config";
  const fs::path map_dir = dir / "map";
  fs::create_directories(dir);
  fs::copy(GLIM_LOCALIZATION_CONFIG_DIR, config_dir, fs::copy_options::recursive);

  SyntheticMapParams map_params;
  map_params.num_submaps = 2;
  write_synthetic_dump(map_dir.string(), map_params);

  const fs::path odometry_config = config_dir / "config_odometry_localization_gpu.json";
  nlohmann::json config = nlohmann::json::parse(std::ifstream(odometry_config), nullptr, true, true);
  config["odometry_estimation"]["map_path"] = map_dir.string();
  std::ofstream(odometry_config) << config.dump(2);

  glim::GlobalConfig::instance((config_dir / "gpu").string(), true);
  {
    const auto module = glim::OdometryEstimationBase::load_module(GLIM_LOCALIZATION_GPU_PLUGIN_PATH);
    ASSERT_TRUE(module);
    const auto odometry = std::dynamic_pointer_cast<OdometryEstimationLocalizationGPU>(module);
    ASSERT_TRUE(odometry);
    EXPECT_EQ(odometry->status().state, LocalizationState::INITIALIZING);
  }
}
