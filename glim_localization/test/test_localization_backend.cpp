#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <unistd.h>

#include <glim/util/config.hpp>

#include <glim_localization/map/map_manager.hpp>
#include <glim_localization/odometry/localization_backend.hpp>
#include <glim_localization/odometry/odometry_estimation_localization_cpu.hpp>

#include "synthetic/synthetic_map.hpp"

namespace fs = std::filesystem;
using namespace glim_localization;
using namespace glim_localization::test;

TEST(LocalizationBackendTest, ShippedConfigsSelectTheirBackend) {
  glim::GlobalConfig::instance(GLIM_LOCALIZATION_CONFIG_DIR, true);
  EXPECT_EQ(localization_backend_from_config(), LocalizationBackend::CPU);
  glim::GlobalConfig::instance((fs::path(GLIM_LOCALIZATION_CONFIG_DIR) / "gpu").string(), true);
  EXPECT_EQ(localization_backend_from_config(), LocalizationBackend::GPU);
  EXPECT_EQ(to_string(LocalizationBackend::CPU), "cpu");
  EXPECT_EQ(to_string(LocalizationBackend::GPU), "gpu");
}

TEST(LocalizationBackendTest, RejectsOtherModules) {
  const fs::path dir = fs::temp_directory_path() / ("glim_loc_backend_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  fs::create_directories(dir);
  std::ofstream(dir / "config.json") << R"({"global": {"config_odometry": "config_odometry.json"}})";
  std::ofstream(dir / "config_odometry.json") << R"({"odometry_estimation": {"so_name": "libodometry_estimation_cpu.so"}})";
  glim::GlobalConfig::instance(dir.string(), true);
  EXPECT_THROW(localization_backend_from_config(), std::runtime_error);
  fs::remove_all(dir);
}

TEST(LocalizationBackendTest, CreatesTheCpuModule) {
  glim::GlobalConfig::instance(GLIM_LOCALIZATION_CONFIG_DIR, true);
  const fs::path dir = fs::temp_directory_path() / ("glim_loc_backend_map_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  SyntheticMapParams map_params;
  map_params.num_submaps = 2;
  write_synthetic_dump(dir.string(), map_params);
  auto map = std::make_shared<MapManager>();
  ASSERT_TRUE(map->load(dir.string()));

  const auto params = make_localization_params(LocalizationBackend::CPU);
  ASSERT_TRUE(dynamic_cast<OdometryEstimationLocalizationCPU::Params*>(params.get()));
  const auto odometry = make_localization_odometry(*params, map);
  ASSERT_TRUE(dynamic_cast<OdometryEstimationLocalizationCPU*>(odometry.get()));
  EXPECT_STREQ(odometry->backend_name(), "cpu");
  EXPECT_EQ(odometry->map(), map);
  fs::remove_all(dir);
}

TEST(LocalizationBackendTest, GpuNeedsACudaBuild) {
  glim::GlobalConfig::instance((fs::path(GLIM_LOCALIZATION_CONFIG_DIR) / "gpu").string(), true);
  if (gpu_backend_available()) {
    GTEST_SKIP() << "CUDA build: covered by test_localization_odometry_gpu";
  }
  EXPECT_THROW(make_localization_params(LocalizationBackend::GPU), std::runtime_error);
}
