#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <sys/wait.h>
#include <unistd.h>

#include <gtsam_points/features/covariance_estimation.hpp>

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

TEST_F(MapCompilerTest, LoadMapReportsTheSource) {
  std::string source = "untouched";
  LoadedSubmaps submaps;
  ASSERT_TRUE(load_map(input_.string(), submaps, &source));
  EXPECT_EQ(source, "glim_dump");

  ASSERT_TRUE(save_compiled_map(output_.string(), submaps, "ply"));
  ASSERT_TRUE(load_map(output_.string(), submaps, &source));
  EXPECT_EQ(source, "ply");

  // The source is informational: a missing or non-string value loads as "unknown".
  edit_index([](nlohmann::json& index) { index.erase("source"); });
  ASSERT_TRUE(load_map(output_.string(), submaps, &source));
  EXPECT_EQ(source, "unknown");
  edit_index([](nlohmann::json& index) { index["source"] = 3; });
  source = "untouched";
  ASSERT_TRUE(load_map(output_.string(), submaps, &source));
  EXPECT_EQ(source, "unknown");

  // A failed load leaves it unchanged.
  source = "untouched";
  EXPECT_FALSE(load_map((dir_ / "missing").string(), submaps, &source));
  EXPECT_EQ(source, "untouched");
}

TEST_F(MapCompilerTest, RecompilingKeepsTheSource) {
  const auto read_source = [](const fs::path& map) {
    return nlohmann::json::parse(std::ifstream(map / "map_index.json")).at("source").get<std::string>();
  };
  // A GLIM dump, compiled twice
  ASSERT_EQ(compile(input_, output_), 0) << last_log();
  ASSERT_EQ(compile(output_, dir_ / "dump_again"), 0) << last_log();
  EXPECT_EQ(read_source(output_), "glim_dump");
  EXPECT_EQ(read_source(dir_ / "dump_again"), "glim_dump");

  // A PCD map, compiled again as it is and with recomputed covariances. Both used to record "glim_dump".
  write_synthetic_pcd((dir_ / "cloud.pcd").string(), make_synthetic_world_points(params_), PcdEncoding::Binary);
  // One large tile, so no tile has fewer than k points.
  ASSERT_EQ(compile(dir_ / "cloud.pcd", dir_ / "from_pcd", "--grid-size 1000"), 0) << last_log();
  ASSERT_EQ(compile(dir_ / "from_pcd", dir_ / "pcd_again"), 0) << last_log();
  ASSERT_EQ(compile(dir_ / "from_pcd", dir_ / "pcd_recomputed", "--recompute-covariances --k-neighbors 5"), 0) << last_log();
  EXPECT_EQ(read_source(dir_ / "from_pcd"), "pcd");
  EXPECT_EQ(read_source(dir_ / "pcd_again"), "pcd");
  EXPECT_EQ(read_source(dir_ / "pcd_recomputed"), "pcd");
}

TEST_F(MapCompilerTest, KeepsDumpCovariancesByDefault) {
  ASSERT_EQ(compile(input_, output_), 0) << last_log();
  LoadedSubmaps dump, compiled;
  ASSERT_TRUE(load_map(input_.string(), dump));
  ASSERT_TRUE(load_map(output_.string(), compiled));
  ASSERT_EQ(compiled.size(), dump.size());
  for (size_t i = 0; i < dump.size(); i++) {
    ASSERT_EQ(compiled[i].points->size(), dump[i].points->size());
    ASSERT_TRUE(compiled[i].points->has_covs());
    for (size_t j = 0; j < dump[i].points->size(); j++) {
      // Both are float files, so the covariances are copied exactly.
      ASSERT_EQ(compiled[i].points->covs[j], dump[i].points->covs[j]) << i << " " << j;
    }
  }
}

TEST_F(MapCompilerTest, RecomputesCovariancesForDumpAndCompiledInput) {
  ASSERT_EQ(compile(input_, dir_ / "plain"), 0) << last_log();
  // The synthetic dump has covariances from k = 10, so k = 20 must change them.
  for (const fs::path& input : {input_, dir_ / "plain"}) {
    fs::remove_all(output_);
    ASSERT_EQ(compile(input, output_, "--recompute-covariances --k-neighbors 20"), 0) << input << ": " << last_log();
    EXPECT_NE(last_log().find("recomputed the covariances"), std::string::npos) << last_log();
    const auto index = nlohmann::json::parse(std::ifstream(output_ / "map_index.json"));
    EXPECT_EQ(index.at("source"), "glim_dump");

    LoadedSubmaps original, recomputed;
    ASSERT_TRUE(load_map(input.string(), original));
    ASSERT_TRUE(load_map(output_.string(), recomputed));
    ASSERT_EQ(recomputed.size(), original.size());
    size_t changed = 0;
    for (size_t i = 0; i < original.size(); i++) {
      EXPECT_LT((recomputed[i].T_world_origin.matrix() - original[i].T_world_origin.matrix()).cwiseAbs().maxCoeff(), 1e-9);
      const auto& points = *original[i].points;
      ASSERT_EQ(recomputed[i].points->size(), points.size());
      const auto expected = gtsam_points::estimate_covariances(points.points, points.size(), 20, 1);
      for (size_t j = 0; j < points.size(); j++) {
        ASSERT_EQ(recomputed[i].points->points[j], points.points[j]);
        // Stored as floats
        ASSERT_LT((recomputed[i].points->covs[j] - expected[j]).cwiseAbs().maxCoeff(), 1e-5) << i << " " << j;
        changed += (recomputed[i].points->covs[j] - points.covs[j]).cwiseAbs().maxCoeff() > 1e-3;
      }
    }
    EXPECT_GT(changed, 0u) << input;
  }

  // The default k is 10, as in the synthetic dump.
  fs::remove_all(output_);
  ASSERT_EQ(compile(input_, output_, "--recompute-covariances"), 0) << last_log();
  EXPECT_NE(last_log().find("k_neighbors 10"), std::string::npos) << last_log();
}

TEST_F(MapCompilerTest, RecomputeCovariancesFailsForSubmapsSmallerThanK) {
  SyntheticMapParams small = params_;
  small.submap_radius = 3.0;
  const fs::path dump = dir_ / "small_dump";
  const SyntheticMap truth = write_synthetic_dump(dump.string(), small);
  size_t smallest = std::numeric_limits<size_t>::max();
  for (const auto& submap : truth.submaps) {
    smallest = std::min(smallest, submap.num_points);
  }
  ASSERT_GT(smallest, 0u);
  ASSERT_LT(smallest, 1000u);
  const size_t k = std::max<size_t>(smallest + 1, 5);
  EXPECT_EQ(compile(dump, output_, "--recompute-covariances --k-neighbors " + std::to_string(k)), 1) << last_log();
  EXPECT_FALSE(fs::exists(output_));
  EXPECT_NE(last_log().find("fewer than k_neighbors " + std::to_string(k)), std::string::npos) << last_log();
}

TEST_F(MapCompilerTest, RejectsBadRecomputeCovariancesOptions) {
  const fs::path pcd = dir_ / "cloud.pcd";
  write_synthetic_pcd(pcd.string(), make_synthetic_world_points(params_), PcdEncoding::Binary);
  for (const auto& [input, options, message] : std::vector<std::tuple<fs::path, std::string, std::string>>{
         {pcd, "--recompute-covariances", "--recompute-covariances applies only to a GLIM dump or a compiled map"},
         {pcd, "--k-neighbors 12 --recompute-covariances", "--recompute-covariances applies only to a GLIM dump or a compiled map"},
         {input_, "--k-neighbors 12", "--k-neighbors applies to a GLIM dump or a compiled map only with --recompute-covariances"},
         {input_, "--recompute-covariances --k-neighbors 4", "k_neighbors 4 must be at least 5"},
         {input_, "--recompute-covariances --k-neighbors 0", "invalid value for --k-neighbors"},
         {input_, "--recompute-covariances --grid-size 10", "--grid-size and --voxel-size apply only to .pcd and .ply input"},
         {input_, "--recompute-covariances --k-neighbors", "missing value for --k-neighbors"}}) {
    EXPECT_EQ(compile(input, output_, options), 1) << options;
    EXPECT_FALSE(fs::exists(output_)) << options;
    EXPECT_NE(last_log().find(message), std::string::npos) << options << ": " << last_log();
  }
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
  // Not as a broken GLIM dump, and not as "options apply only to .pcd and .ply input"
  for (const std::string name : {"typo.pcd", "typo.ply"}) {
    for (const std::string options : {"", "--grid-size 10"}) {
      EXPECT_EQ(compile(dir_ / name, output_, options), 1) << name << " " << options;
      EXPECT_FALSE(fs::exists(output_)) << name << " " << options;
      EXPECT_NE(last_log().find("file not found"), std::string::npos) << name << " " << options << ": " << last_log();
    }
  }
}

TEST_F(MapCompilerTest, BrokenPcdLeavesNoOutput) {
  const fs::path pcd = dir_ / "broken.pcd";
  std::ofstream(pcd) << "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nPOINTS 100\nDATA binary\n";
  EXPECT_EQ(compile(pcd, output_), 1);
  EXPECT_FALSE(fs::exists(output_));
}

TEST_F(MapCompilerTest, PlyAndPcdGiveTheSameMap) {
  const auto points = make_synthetic_world_points(params_);
  for (const auto& [ply_encoding, pcd_encoding] : {std::make_pair(PlyEncoding::Ascii, PcdEncoding::Ascii),
                                                    std::make_pair(PlyEncoding::BinaryLittleEndian, PcdEncoding::Binary)}) {
    fs::remove_all(dir_ / "from_ply");
    fs::remove_all(dir_ / "from_pcd");
    // Double x, y, z in both (ascii is printed with 17 digits), so both readers see the same values.
    write_synthetic_ply((dir_ / "cloud.Ply").string(), points, ply_encoding, PlyOptions{true, true});
    write_synthetic_pcd((dir_ / "cloud.pcd").string(), points, pcd_encoding, true);
    ASSERT_EQ(compile(dir_ / "cloud.Ply", dir_ / "from_ply", "--grid-size 15 --k-neighbors 12"), 0) << last_log();
    ASSERT_EQ(compile(dir_ / "cloud.pcd", dir_ / "from_pcd", "--grid-size 15 --k-neighbors 12"), 0) << last_log();

    auto ply_index = nlohmann::json::parse(std::ifstream(dir_ / "from_ply" / "map_index.json"));
    auto pcd_index = nlohmann::json::parse(std::ifstream(dir_ / "from_pcd" / "map_index.json"));
    EXPECT_EQ(ply_index.at("source"), "ply");
    EXPECT_EQ(pcd_index.at("source"), "pcd");
    ply_index.erase("source");
    pcd_index.erase("source");
    EXPECT_EQ(ply_index, pcd_index);

    LoadedSubmaps from_ply, from_pcd;
    ASSERT_TRUE(load_map((dir_ / "from_ply").string(), from_ply));
    ASSERT_TRUE(load_map((dir_ / "from_pcd").string(), from_pcd));
    ASSERT_EQ(from_ply.size(), from_pcd.size());
    ASSERT_GT(from_ply.size(), 2u);
    for (size_t i = 0; i < from_ply.size(); i++) {
      EXPECT_EQ(from_ply[i].T_world_origin.matrix(), from_pcd[i].T_world_origin.matrix());
      ASSERT_EQ(from_ply[i].points->size(), from_pcd[i].points->size());
      for (size_t j = 0; j < from_ply[i].points->size(); j++) {
        ASSERT_EQ(from_ply[i].points->points[j], from_pcd[i].points->points[j]);
        ASSERT_EQ(from_ply[i].points->covs[j], from_pcd[i].points->covs[j]);
      }
    }
  }
}

TEST_F(MapCompilerTest, BinaryCompressedPcdGivesTheSameMapAsBinary) {
  const auto points = make_synthetic_world_points(params_);
  for (const bool double_xyz : {false, true}) {
    fs::remove_all(dir_ / "from_binary");
    fs::remove_all(dir_ / "from_compressed");
    write_synthetic_pcd((dir_ / "binary.pcd").string(), points, PcdEncoding::Binary, double_xyz);
    write_synthetic_pcd((dir_ / "compressed.pcd").string(), points, PcdEncoding::BinaryCompressed, double_xyz);
    ASSERT_EQ(compile(dir_ / "binary.pcd", dir_ / "from_binary", "--grid-size 15 --voxel-size 0.5"), 0) << last_log();
    ASSERT_EQ(compile(dir_ / "compressed.pcd", dir_ / "from_compressed", "--grid-size 15 --voxel-size 0.5"), 0) << last_log();

    const auto binary_index = nlohmann::json::parse(std::ifstream(dir_ / "from_binary" / "map_index.json"));
    const auto compressed_index = nlohmann::json::parse(std::ifstream(dir_ / "from_compressed" / "map_index.json"));
    EXPECT_EQ(compressed_index.at("source"), "pcd");
    EXPECT_EQ(binary_index, compressed_index);

    LoadedSubmaps from_binary, from_compressed;
    ASSERT_TRUE(load_map((dir_ / "from_binary").string(), from_binary));
    ASSERT_TRUE(load_map((dir_ / "from_compressed").string(), from_compressed));
    ASSERT_EQ(from_binary.size(), from_compressed.size());
    ASSERT_GT(from_binary.size(), 2u);
    for (size_t i = 0; i < from_binary.size(); i++) {
      ASSERT_EQ(from_binary[i].points->size(), from_compressed[i].points->size());
      for (size_t j = 0; j < from_binary[i].points->size(); j++) {
        ASSERT_EQ(from_binary[i].points->points[j], from_compressed[i].points->points[j]);
        ASSERT_EQ(from_binary[i].points->covs[j], from_compressed[i].points->covs[j]);
      }
    }
  }
}

TEST_F(MapCompilerTest, CompilesPlyWithDownsampling) {
  const auto points = make_synthetic_world_points(params_);
  write_synthetic_ply((dir_ / "cloud.ply").string(), points, PlyEncoding::BinaryLittleEndian);
  ASSERT_EQ(compile(dir_ / "cloud.ply", output_, "--voxel-size 1.0"), 0) << last_log();
  MapManager manager;
  ASSERT_TRUE(manager.load(output_.string()));
  EXPECT_GT(manager.size(), 0u);
}

TEST_F(MapCompilerTest, BrokenPlyLeavesNoOutput) {
  const std::string xyz = "property float x\nproperty float y\nproperty float z\n";
  for (const auto& [text, message] : std::vector<std::pair<std::string, std::string>>{
         {"ply\nformat binary_big_endian 1.0\nelement vertex 1\n" + xyz + "end_header\n0123456789ab", "big-endian PLY is not supported"},
         {"ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\nend_header\n1 2\n", "missing vertex property x, y or z"},
         {"ply\nformat binary_little_endian 1.0\nelement vertex 100\n" + xyz + "end_header\n0123456789ab", "truncated binary data"}}) {
    std::ofstream(dir_ / "broken.ply", std::ios::binary | std::ios::trunc) << text;
    EXPECT_EQ(compile(dir_ / "broken.ply", output_), 1) << message;
    EXPECT_FALSE(fs::exists(output_)) << message;
    EXPECT_NE(last_log().find(message), std::string::npos) << last_log();
  }
}

namespace {

struct Pgm {
  int width = 0;
  int height = 0;
  std::vector<int> pixels;  ///< Row-major from the top row
  int at(int column, int row) const { return pixels[static_cast<size_t>(row) * width + column]; }
};

Pgm read_pgm(const fs::path& path) {
  std::ifstream ifs(path, std::ios::binary);
  Pgm pgm;
  std::string magic, comment;
  int max_value = 0;
  std::getline(ifs, magic);
  std::getline(ifs, comment);
  ifs >> pgm.width >> pgm.height >> max_value;
  ifs.get();
  EXPECT_EQ(magic, "P5");
  EXPECT_EQ(max_value, 255);
  for (char c; ifs.get(c);) {
    pgm.pixels.push_back(static_cast<unsigned char>(c));
  }
  EXPECT_EQ(pgm.pixels.size(), static_cast<size_t>(pgm.width) * pgm.height);
  return pgm;
}

/// A 4 x 2 m floor at z = 10 with its lower-left corner at (100, -50), sampled every 0.1 m at cell-centred offsets,
/// a 1 x 0.5 m box at z = 10.5 in its top-left (highest y) corner, a hole in the floor at x 103 to 103.5,
/// y -50 to -49.5, and a ceiling at z = 15.
void write_floor_ply(const fs::path& path) {
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 40; i++) {
    for (int j = 0; j < 20; j++) {
      const double x = 100.05 + 0.1 * i;
      const double y = -49.95 + 0.1 * j;
      if (!(i >= 30 && i < 35 && j < 5)) {
        points.emplace_back(x, y, 10.0);
      }
      points.emplace_back(x, y, 15.0);
      if (i < 10 && j >= 15) {
        points.emplace_back(x, y, 10.5);
      }
    }
  }
  std::ofstream ofs(path, std::ios::trunc);
  ofs << "ply\nformat ascii 1.0\nelement vertex " << points.size() << "\nproperty double x\nproperty double y\nproperty double z\nend_header\n";
  ofs.precision(17);
  for (const auto& p : points) {
    ofs << p.x() << " " << p.y() << " " << p.z() << "\n";
  }
}

}  // namespace

TEST_F(MapCompilerTest, WritesOccupancyGrid) {
  const fs::path ply = dir_ / "floor.ply";
  write_floor_ply(ply);
  const fs::path prefix = dir_ / "grid" / "floor2d";
  ASSERT_EQ(compile(ply, output_, "--grid-size 2 --occupancy-grid '" + prefix.string() + "' --occupancy-resolution 0.5 --occupancy-z-min 10.2 --occupancy-z-max 12"), 0)
    << last_log();
  LoadedSubmaps submaps;
  ASSERT_TRUE(load_map(output_.string(), submaps));

  std::ifstream yaml(dir_ / "grid" / "floor2d.yaml");
  const std::string text((std::istreambuf_iterator<char>(yaml)), std::istreambuf_iterator<char>());
  EXPECT_EQ(text,
            "image: \"floor2d.pgm\"\n"
            "mode: trinary\n"
            "resolution: 0.500000000\n"
            "origin: [100.000000000, -50.000000000, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.1\n");

  const Pgm pgm = read_pgm(dir_ / "grid" / "floor2d.pgm");
  ASSERT_EQ(pgm.width, 8);
  ASSERT_EQ(pgm.height, 4);
  for (int row = 0; row < pgm.height; row++) {
    for (int column = 0; column < pgm.width; column++) {
      // Row 0 is the highest y: the box is in the top-left corner, the hole in the bottom row.
      int expected = 254;
      if (row == 0 && column < 2) {
        expected = 0;
      } else if (row == 3 && column == 6) {
        expected = 205;
      }
      EXPECT_EQ(pgm.at(column, row), expected) << column << " " << row;
    }
  }
}

TEST_F(MapCompilerTest, WritesOccupancyGridFromAGlimDump) {
  // The corridor: floor at z = 0 and walls up to 3 m along y = +-5, in a frame shifted by world_offset.
  const double z0 = params_.world_offset.z();
  const std::string band = " --occupancy-z-min " + std::to_string(z0 + 0.5) + " --occupancy-z-max " + std::to_string(z0 + 2.0);
  ASSERT_EQ(compile(input_, output_, "--occupancy-grid '" + (dir_ / "corridor").string() + "' --occupancy-resolution 0.4" + band), 0) << last_log();
  const Pgm pgm = read_pgm(dir_ / "corridor.pgm");
  ASSERT_GT(pgm.width, 10);
  ASSERT_GT(pgm.height, 10);
  int occupied = 0, num_free = 0;
  for (const int pixel : pgm.pixels) {
    occupied += pixel == 0;
    num_free += pixel == 254;
  }
  EXPECT_GT(occupied, pgm.width);  // At least the two walls along x
  EXPECT_GT(num_free, occupied);   // The floor between them
  // Dump options other than the occupancy grid are still refused.
  fs::remove_all(output_);
  EXPECT_EQ(compile(input_, output_, "--grid-size 10 --occupancy-grid '" + (dir_ / "again").string() + "'" + band), 1);
  EXPECT_FALSE(fs::exists(dir_ / "again.pgm"));
}

TEST_F(MapCompilerTest, RejectsBadOccupancyOptions) {
  const fs::path ply = dir_ / "floor.ply";
  write_floor_ply(ply);
  const std::string grid = "--occupancy-grid '" + (dir_ / "grid").string() + "'";
  for (const auto& [options, message] : std::vector<std::pair<std::string, std::string>>{
         {grid, "must both be set"},
         {grid + " --occupancy-z-min 10", "must both be set"},
         {grid + " --occupancy-z-min 12 --occupancy-z-max 10", "must be below"},
         {grid + " --occupancy-z-min 10 --occupancy-z-max 12 --occupancy-resolution 0.001", "resolution"},
         {"--occupancy-z-min 10 --occupancy-z-max 12", "need --occupancy-grid"},
         {"--occupancy-grid '" + (dir_ / "sub").string() + "/' --occupancy-z-min 10 --occupancy-z-max 12", "has no file name"},
         // Every point is at z >= 10, so nothing is below this band's top.
         {grid + " --occupancy-z-min 1 --occupancy-z-max 2", "no points below occupancy z_max"}}) {
    EXPECT_EQ(compile(ply, output_, options), 1) << options;
    EXPECT_FALSE(fs::exists(output_)) << options;
    EXPECT_FALSE(fs::exists(dir_ / "grid.pgm")) << options;
    EXPECT_NE(last_log().find(message), std::string::npos) << options << ": " << last_log();
  }

  // An existing grid file is refused before anything is written.
  std::ofstream(dir_ / "grid.yaml") << "keep";
  EXPECT_EQ(compile(ply, output_, grid + " --occupancy-z-min 10.2 --occupancy-z-max 12"), 1);
  EXPECT_FALSE(fs::exists(output_));
  EXPECT_FALSE(fs::exists(dir_ / "grid.pgm"));
  EXPECT_NE(last_log().find("already exists"), std::string::npos) << last_log();

  // A non-empty output directory is refused up front too, so no grid is written next to a map that was not.
  fs::remove(dir_ / "grid.yaml");
  fs::create_directories(output_);
  std::ofstream(output_ / "keep.txt") << "keep";
  EXPECT_EQ(compile(ply, output_, grid + " --occupancy-z-min 10.2 --occupancy-z-max 12"), 1);
  EXPECT_FALSE(fs::exists(dir_ / "grid.pgm"));
  EXPECT_FALSE(fs::exists(output_ / "map_index.json"));
  EXPECT_NE(last_log().find("not an empty directory"), std::string::npos) << last_log();
  EXPECT_FALSE(fs::exists(dir_ / "sub" / ".pgm"));
}

TEST_F(MapCompilerTest, RejectsBrokenJson) {
  ASSERT_EQ(compile(input_, output_), 0);
  std::ofstream(output_ / "map_index.json", std::ios::trunc) << "{";
  expect_compiled_load_fails();
}
