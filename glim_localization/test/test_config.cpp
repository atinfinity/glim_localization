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
}

}  // namespace
