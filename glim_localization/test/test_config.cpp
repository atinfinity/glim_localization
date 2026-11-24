#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace {

namespace fs = std::filesystem;
using nlohmann::json;

json read_config(const std::string& name) {
  std::ifstream ifs(fs::path(GLIM_LOCALIZATION_CONFIG_DIR) / name);
  EXPECT_TRUE(ifs.good()) << name;
  return json::parse(ifs, nullptr, true, true);
}

TEST(ConfigTest, GlobalEntriesPointToExistingFiles) {
  const json global = read_config("config.json").at("global");
  for (const auto& [key, value] : global.items()) {
    if (key == "config_path") {
      continue;
    }
    const std::string name = value.get<std::string>();
    EXPECT_TRUE(fs::is_regular_file(fs::path(GLIM_LOCALIZATION_CONFIG_DIR) / name)) << key << ": " << name;
    EXPECT_EQ(name.find("gpu"), std::string::npos) << key << ": " << name;
  }
}

TEST(ConfigTest, ExtensionModulesExcludeViewers) {
  const auto modules = read_config("config_ros.json").at("glim_ros").at("extension_modules").get<std::vector<std::string>>();
  for (const std::string excluded : {"libmemory_monitor.so", "libstandard_viewer.so", "librviz_viewer.so"}) {
    EXPECT_EQ(std::count(modules.begin(), modules.end(), excluded), 0) << excluded;
  }
}

TEST(ConfigTest, MappingIsDisabled) {
  const json ros = read_config("config_ros.json").at("glim_ros");
  EXPECT_FALSE(ros.at("enable_local_mapping").get<bool>());
  EXPECT_FALSE(ros.at("enable_global_mapping").get<bool>());
  EXPECT_TRUE(read_config("config_sub_mapping_none.json").at("sub_mapping").at("so_name").get<std::string>().empty());
}

TEST(ConfigTest, OdometryUsesLocalizationModule) {
  const json global = read_config("config.json").at("global");
  const json odometry = read_config(global.at("config_odometry").get<std::string>()).at("odometry_estimation");
  EXPECT_EQ(odometry.at("so_name").get<std::string>(), "libodometry_localization_cpu.so");
  EXPECT_DOUBLE_EQ(odometry.at("init_pose_damping_scale").get<double>(), 1e2);
  // The yaw search of the given initial pose is opt-in
  EXPECT_FALSE(odometry.at("init_yaw_search").get<bool>());
  EXPECT_DOUBLE_EQ(odometry.at("init_yaw_search_range_deg").get<double>(), 180.0);
  EXPECT_DOUBLE_EQ(odometry.at("init_yaw_search_step_deg").get<double>(), 15.0);
}

TEST(ConfigTest, GpuConfigSelectsGpuModule) {
  const json global = read_config("gpu/config.json").at("global");
  for (const auto& [key, value] : global.items()) {
    if (key == "config_path") {
      continue;
    }
    const std::string name = value.get<std::string>();
    EXPECT_TRUE(fs::is_regular_file(fs::path(GLIM_LOCALIZATION_CONFIG_DIR) / "gpu" / name)) << key << ": " << name;
  }
  const json odometry = read_config("gpu/" + global.at("config_odometry").get<std::string>()).at("odometry_estimation");
  EXPECT_EQ(odometry.at("so_name").get<std::string>(), "libodometry_localization_gpu.so");
  EXPECT_GE(odometry.at("map_gpu_max_tiles").get<int>(), odometry.at("map_max_submaps").get<int>());
}

TEST(ConfigTest, GpuConfigSharesCommonKeysWithCpu) {
  // The keys of the shared module (map, status, IMU gap, smoother) must mean the same for both backends.
  const json cpu = read_config("config_odometry_localization_cpu.json").at("odometry_estimation");
  const json gpu = read_config("config_odometry_localization_gpu.json").at("odometry_estimation");
  const std::vector<std::string> cpu_only = {
    "so_name", "max_iterations", "lru_thresh", "target_downsampling_rate", "vgicp_resolution", "vgicp_voxelmap_levels", "vgicp_voxelmap_scaling_factor"};
  for (const auto& [key, value] : cpu.items()) {
    if (std::count(cpu_only.begin(), cpu_only.end(), key)) {
      continue;
    }
    ASSERT_TRUE(gpu.contains(key)) << key;
    EXPECT_EQ(gpu.at(key), value) << key;
  }
}

}  // namespace
