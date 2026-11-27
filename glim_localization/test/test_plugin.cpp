#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

#include <glim/odometry/odometry_estimation_base.hpp>
#include <glim/util/config.hpp>

#include <glim_localization/odometry/odometry_estimation_localization_cpu.hpp>

#include "synthetic/synthetic_map.hpp"

namespace fs = std::filesystem;
using namespace glim_localization;
using namespace glim_localization::test;

TEST(PluginTest, LoadsModuleFromConfiguredMap) {
  const fs::path dir = fs::temp_directory_path() / ("glim_loc_plugin_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  const fs::path config_dir = dir / "config";
  const fs::path map_dir = dir / "map";
  fs::create_directories(dir);
  fs::copy(GLIM_LOCALIZATION_CONFIG_DIR, config_dir, fs::copy_options::recursive);

  SyntheticMapParams map_params;
  map_params.num_submaps = 2;
  write_synthetic_dump(map_dir.string(), map_params);

  const fs::path odometry_config = config_dir / "config_odometry_localization_cpu.json";
  // The shipped configs start with a // provenance comment, which GLIM's own loader accepts
  nlohmann::json config = nlohmann::json::parse(std::ifstream(odometry_config), nullptr, true, true);
  config["odometry_estimation"]["map_path"] = map_dir.string();
  std::ofstream(odometry_config) << config.dump(2);

  glim::GlobalConfig::instance(config_dir.string(), true);
  {
    const auto module = glim::OdometryEstimationBase::load_module(GLIM_LOCALIZATION_PLUGIN_PATH);
    ASSERT_TRUE(module);
    const auto odometry = std::dynamic_pointer_cast<OdometryEstimationLocalizationCPU>(module);
    ASSERT_TRUE(odometry);
    EXPECT_EQ(odometry->status().state, LocalizationState::INITIALIZING);
  }
  fs::remove_all(dir);
}

TEST(PluginDeathTest, AbortsWhenMapCannotBeLoaded) {
  testing::FLAGS_gtest_death_test_style = "threadsafe";
  glim::GlobalConfig::instance(GLIM_LOCALIZATION_CONFIG_DIR, true);  // map_path is empty
  // The module logs the reason through spdlog before it aborts. The default logger writes to stdout, and a death test
  // matches stderr only, so the child logs to stderr. The message must name the module and the map, so that a death
  // for another reason (a missing symbol, an unrelated exception) fails the test.
  EXPECT_DEATH(
    {
      spdlog::set_default_logger(std::make_shared<spdlog::logger>("plugin_death_test", std::make_shared<spdlog::sinks::stderr_sink_mt>()));
      glim::OdometryEstimationBase::load_module(GLIM_LOCALIZATION_PLUGIN_PATH);
    },
    "failed to create odometry_localization_cpu: failed to load the localization map");
}
