#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <sys/wait.h>
#include <unistd.h>

#include <glim_localization/map/map_io.hpp>
#include <glim_localization/map/map_manager.hpp>

#include "synthetic/synthetic_map.hpp"

namespace fs = std::filesystem;
using namespace glim_localization;
using namespace glim_localization::test;

class MapCompilerTest : public testing::Test {
protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("glim_loc_map_compiler_" + std::to_string(::getpid()));
    fs::remove_all(dir_);
    input_ = dir_ / "dump";
    output_ = dir_ / "compiled";
    params_.num_submaps = 4;
    params_.point_spacing = 0.4;
    params_.world_offset = Eigen::Vector3d(12345.678901, -9876.54321, 321.123456);
    write_synthetic_dump(input_.string(), params_);
  }
  void TearDown() override { fs::remove_all(dir_); }

  int compile(const fs::path& input, const fs::path& output, const std::string& options = "") const {
    const std::string command =
      std::string("'") + MAP_COMPILER_PATH + "' '" + input.string() + "' '" + output.string() + "' " + options + " > '" + log_path().string() + "' 2>&1";
    const int status = std::system(command.c_str());
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  }

  /// stdout and stderr of the last compile()
  fs::path log_path() const { return dir_ / "map_compiler.log"; }
  std::string last_log() const {
    std::ifstream ifs(log_path());
    return std::string(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
  }

  void edit_index(const std::function<void(nlohmann::json&)>& edit) const {
    const auto file = output_ / "map_index.json";
    nlohmann::json index = nlohmann::json::parse(std::ifstream(file));
    edit(index);
    std::ofstream(file, std::ios::trunc) << index.dump();
  }

  void expect_compiled_load_fails() const {
    LoadedSubmaps submaps;
    EXPECT_FALSE(load_map(output_.string(), submaps));
    EXPECT_TRUE(submaps.empty());
  }

  fs::path dir_, input_, output_;
  SyntheticMapParams params_;
};

TEST_F(MapCompilerTest, RoundTripPreservesMap) {
  ASSERT_EQ(compile(input_, output_), 0);
  EXPECT_TRUE(fs::exists(output_ / "map_index.json"));
  EXPECT_FALSE(fs::exists(output_ / "values.bin"));
  EXPECT_FALSE(fs::exists(output_ / "graph.bin"));
  EXPECT_FALSE(fs::exists(output_ / "graph.txt"));

  MapManager original, compiled;
  ASSERT_TRUE(original.load(input_.string()));
  ASSERT_TRUE(compiled.load(output_.string()));
  ASSERT_EQ(compiled.size(), original.size());

  for (int id = 0; id < params_.num_submaps; id++) {
    const auto a = original.tile(id);
    const auto b = compiled.tile(id);
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_LT((a->T_world_origin.matrix() - b->T_world_origin.matrix()).cwiseAbs().maxCoeff(), 1e-9);
    ASSERT_EQ(a->points->size(), b->points->size());
    ASSERT_TRUE(b->points->has_covs());
    for (size_t i = 0; i < a->points->size(); i++) {
      ASSERT_LT((a->points->points[i] - b->points->points[i]).cwiseAbs().maxCoeff(), 1e-5);
    }
  }

  const Eigen::Isometry3d query = original.tile(1)->T_world_origin;
  const auto expected = original.query(query, 25.0, 3);
  const auto actual = compiled.query(query, 25.0, 3);
  ASSERT_EQ(actual.size(), expected.size());
  for (size_t i = 0; i < actual.size(); i++) {
    EXPECT_EQ(actual[i]->id, expected[i]->id);
  }
}

TEST_F(MapCompilerTest, CompiledMapCanBeCompiledAgain) {
  ASSERT_EQ(compile(input_, output_), 0);
  ASSERT_EQ(compile(output_, dir_ / "again"), 0);

  LoadedSubmaps first, second;
  ASSERT_TRUE(load_map(output_.string(), first));
  ASSERT_TRUE(load_map((dir_ / "again").string(), second));
  ASSERT_EQ(first.size(), second.size());
  EXPECT_LT((first[2].T_world_origin.matrix() - second[2].T_world_origin.matrix()).cwiseAbs().maxCoeff(), 1e-9);
}

TEST_F(MapCompilerTest, DirectoryNamedPcdIsAMap) {
  const fs::path named = dir_ / "compiled.pcd";
  ASSERT_EQ(compile(input_, named), 0);
  EXPECT_EQ(compile(named, output_), 0) << last_log();
  LoadedSubmaps submaps;
  EXPECT_TRUE(load_map(output_.string(), submaps));
}

TEST_F(MapCompilerTest, RefusesNonEmptyOutput) {
  fs::create_directories(output_);
  std::ofstream(output_ / "keep.txt") << "keep";
  EXPECT_EQ(compile(input_, output_), 1);
  EXPECT_TRUE(fs::exists(output_ / "keep.txt"));
  EXPECT_FALSE(fs::exists(output_ / "map_index.json"));
}

TEST_F(MapCompilerTest, AcceptsEmptyOutputDirectory) {
  fs::create_directories(output_);
  EXPECT_EQ(compile(input_, output_), 0);
  EXPECT_TRUE(fs::exists(output_ / "map_index.json"));
}

TEST_F(MapCompilerTest, InvalidInputLeavesNoOutput) {
  EXPECT_EQ(compile(dir_ / "missing", output_), 1);
  EXPECT_FALSE(fs::exists(output_));
}

TEST_F(MapCompilerTest, RejectsWrongFormat) {
  ASSERT_EQ(compile(input_, output_), 0);
  edit_index([](nlohmann::json& index) { index["format"] = "other"; });
  expect_compiled_load_fails();
}

TEST_F(MapCompilerTest, RejectsWrongVersion) {
  ASSERT_EQ(compile(input_, output_), 0);
  edit_index([](nlohmann::json& index) { index["version"] = 2; });
  expect_compiled_load_fails();
}

TEST_F(MapCompilerTest, RejectsCountMismatch) {
  ASSERT_EQ(compile(input_, output_), 0);
  edit_index([](nlohmann::json& index) { index["num_submaps"] = 5; });
  expect_compiled_load_fails();
}

TEST_F(MapCompilerTest, RejectsPointCountMismatch) {
  ASSERT_EQ(compile(input_, output_), 0);
  edit_index([](nlohmann::json& index) { index["submaps"][1]["num_points"] = 1; });
  expect_compiled_load_fails();
}

TEST_F(MapCompilerTest, RejectsPathOutsideMap) {
  ASSERT_EQ(compile(input_, output_), 0);
  edit_index([](nlohmann::json& index) { index["submaps"][0]["path"] = "../dump/000000"; });
  expect_compiled_load_fails();
}

TEST_F(MapCompilerTest, CompilesPcdIntoGridTiles) {
  const auto points = make_synthetic_world_points(params_);
  const fs::path pcd = dir_ / "cloud.PCD";
  write_synthetic_pcd(pcd.string(), points, PcdEncoding::Binary, true);
  ASSERT_EQ(compile(pcd, output_, "--grid-size 15 --k-neighbors 12"), 0);

  const auto index = nlohmann::json::parse(std::ifstream(output_ / "map_index.json"));
  EXPECT_EQ(index.at("source"), "pcd");
  LoadedSubmaps submaps;
  ASSERT_TRUE(load_map(output_.string(), submaps));
  ASSERT_EQ(submaps.size(), index.at("num_submaps").get<size_t>());
  ASSERT_GT(submaps.size(), 2u);

  size_t total = 0;
  for (size_t i = 0; i < submaps.size(); i++) {
    const auto& entry = index.at("submaps")[i];
    const auto& submap = submaps[i];
    ASSERT_TRUE(submap.points->has_covs());
    total += submap.points->size();
    const auto bbox_min = entry.at("bbox_min").get<std::vector<double>>();
    const auto bbox_max = entry.at("bbox_max").get<std::vector<double>>();
    // A tile spans at most one 15 m cell in x and y.
    EXPECT_LT(bbox_max[0] - bbox_min[0], 15.0);
    EXPECT_LT(bbox_max[1] - bbox_min[1], 15.0);
    const Eigen::Vector3d center = submap.T_world_origin.translation();
    for (int axis = 0; axis < 3; axis++) {
      EXPECT_GE(center[axis], bbox_min[axis]);
      EXPECT_LE(center[axis], bbox_max[axis]);
    }
  }
  EXPECT_EQ(total, points.size());

  // The tiles are usable by MapManager like any compiled map.
  MapManager manager;
  ASSERT_TRUE(manager.load(output_.string()));
  EXPECT_EQ(manager.size(), submaps.size());
}

TEST_F(MapCompilerTest, CompilesAsciiPcdWithDownsampling) {
  const auto points = make_synthetic_world_points(params_);
  const fs::path pcd = dir_ / "cloud.pcd";
  write_synthetic_pcd(pcd.string(), points, PcdEncoding::Ascii);
  ASSERT_EQ(compile(pcd, output_, "--voxel-size 1.0"), 0);
  LoadedSubmaps submaps;
  ASSERT_TRUE(load_map(output_.string(), submaps));
  size_t total = 0;
  for (const auto& submap : submaps) {
    total += submap.points->size();
  }
  EXPECT_LT(total, points.size() / 2);
}

TEST_F(MapCompilerTest, RejectsBadPcdOptions) {
  const fs::path pcd = dir_ / "cloud.pcd";
  write_synthetic_pcd(pcd.string(), make_synthetic_world_points(params_), PcdEncoding::Binary);
  for (const std::string options :
       {"--grid-size 0", "--grid-size -5", "--grid-size abc", "--voxel-size -1", "--k-neighbors 0", "--k-neighbors 2.5", "--grid-size", "--unknown 1"}) {
    EXPECT_EQ(compile(pcd, output_, options), 1) << options;
    EXPECT_FALSE(fs::exists(output_)) << options;
  }
  // Below the lower bounds, or not finite: rejected with a message that names the option.
  for (const auto& [options, message] : std::vector<std::pair<std::string, std::string>>{{"--k-neighbors 4", "k_neighbors 4 must be at least 5"},
                                                                                         {"--grid-size 0.5", "grid size"},
                                                                                         {"--grid-size nan", "grid size"},
                                                                                         {"--grid-size inf", "grid size"},
                                                                                         {"--voxel-size 0.001", "voxel size"},
                                                                                         {"--voxel-size inf", "voxel size"}}) {
    EXPECT_EQ(compile(pcd, output_, options), 1) << options;
    EXPECT_FALSE(fs::exists(output_)) << options;
    EXPECT_NE(last_log().find(message), std::string::npos) << options << ": " << last_log();
  }
  EXPECT_EQ(compile(pcd, output_, "--k-neighbors 5 --grid-size 1 --voxel-size 0.01"), 0) << last_log();
  fs::remove_all(output_);
  // Options only apply to PCD input.
  EXPECT_EQ(compile(input_, output_, "--grid-size 10"), 1);
  EXPECT_FALSE(fs::exists(output_));
}

TEST_F(MapCompilerTest, MissingPcdIsReportedAsNotFound) {
  // Not as a broken GLIM dump, and not as "options apply only to .pcd input"
  for (const std::string options : {"", "--grid-size 10"}) {
    EXPECT_EQ(compile(dir_ / "typo.pcd", output_, options), 1) << options;
    EXPECT_FALSE(fs::exists(output_)) << options;
    EXPECT_NE(last_log().find("file not found"), std::string::npos) << options << ": " << last_log();
  }
}

TEST_F(MapCompilerTest, BrokenPcdLeavesNoOutput) {
  const fs::path pcd = dir_ / "broken.pcd";
  std::ofstream(pcd) << "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nPOINTS 100\nDATA binary\n";
  EXPECT_EQ(compile(pcd, output_), 1);
  EXPECT_FALSE(fs::exists(output_));
}

TEST_F(MapCompilerTest, RejectsBrokenJson) {
  ASSERT_EQ(compile(input_, output_), 0);
  std::ofstream(output_ / "map_index.json", std::ios::trunc) << "{";
  expect_compiled_load_fails();
}
