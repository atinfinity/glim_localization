#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>
#include <unistd.h>

#include <gtsam/base/serialization.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include <glim_localization/map/map_io.hpp>

#include "synthetic/synthetic_map.hpp"

namespace fs = std::filesystem;
using namespace glim_localization;
using namespace glim_localization::test;
using gtsam::symbol_shorthand::X;

class MapIoTest : public testing::Test {
protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("glim_loc_map_io_" + std::to_string(::getpid()));
    fs::remove_all(dir_);
    params_.num_submaps = 3;
    params_.point_spacing = 0.4;
  }
  void TearDown() override { fs::remove_all(dir_); }

  std::string dump() const { return dir_.string(); }

  /// A failed load must return false and keep the caller's data.
  void expect_load_fails() {
    LoadedSubmaps submaps(1);
    submaps[0].id = 77;
    EXPECT_FALSE(load_map(dump(), submaps));
    ASSERT_EQ(submaps.size(), 1u);
    EXPECT_EQ(submaps[0].id, 77);
  }

  fs::path dir_;
  SyntheticMapParams params_;
};

TEST_F(MapIoTest, LoadsPosesFromValuesAtFullPrecision) {
  params_.world_offset = Eigen::Vector3d(12345.678901, -9876.54321, 321.123456);
  const auto truth = write_synthetic_dump(dump(), params_);

  LoadedSubmaps submaps;
  ASSERT_TRUE(load_map(dump(), submaps));
  ASSERT_EQ(submaps.size(), truth.submaps.size());
  for (size_t i = 0; i < submaps.size(); i++) {
    EXPECT_EQ(submaps[i].id, truth.submaps[i].id);
    EXPECT_EQ(submaps[i].points->size(), truth.submaps[i].num_points);
    EXPECT_TRUE(submaps[i].points->has_covs());
    EXPECT_LT((submaps[i].T_world_origin.matrix() - truth.submaps[i].T_world_origin.matrix()).cwiseAbs().maxCoeff(), 1e-9);
  }
}

TEST_F(MapIoTest, MissingValuesFallsBackToDataTxt) {
  const auto truth = write_synthetic_dump(dump(), params_);
  fs::remove(dir_ / "values.bin");

  LoadedSubmaps submaps;
  ASSERT_TRUE(load_map(dump(), submaps));
  ASSERT_EQ(submaps.size(), truth.submaps.size());
  EXPECT_TRUE(submaps[1].T_world_origin.matrix().isApprox(truth.submaps[1].T_world_origin.matrix(), 1e-4));
}

TEST_F(MapIoTest, CountsDirectoriesWithoutGraphTxt) {
  write_synthetic_dump(dump(), params_);
  fs::remove(dir_ / "graph.txt");

  LoadedSubmaps submaps;
  ASSERT_TRUE(load_map(dump(), submaps));
  EXPECT_EQ(submaps.size(), 3u);
}

TEST_F(MapIoTest, CorruptedValuesFails) {
  write_synthetic_dump(dump(), params_);
  std::ofstream(dir_ / "values.bin", std::ios::binary | std::ios::trunc) << "not a gtsam archive";
  expect_load_fails();
}

TEST_F(MapIoTest, MissingPoseInValuesFails) {
  write_synthetic_dump(dump(), params_);
  gtsam::Values values;
  gtsam::deserializeFromBinaryFile((dir_ / "values.bin").string(), values);
  values.erase(X(1));
  gtsam::serializeToBinaryFile(values, (dir_ / "values.bin").string());
  expect_load_fails();
}

TEST_F(MapIoTest, MissingCovariancesFails) {
  write_synthetic_dump(dump(), params_);
  fs::remove(dir_ / "000001" / "covs_compact.bin");
  expect_load_fails();
}

TEST_F(MapIoTest, MissingDataTxtFails) {
  write_synthetic_dump(dump(), params_);
  fs::remove(dir_ / "000002" / "data.txt");
  expect_load_fails();
}

TEST_F(MapIoTest, IdMismatchFails) {
  write_synthetic_dump(dump(), params_);
  const auto file = dir_ / "000001" / "data.txt";
  std::stringstream content;
  content << std::ifstream(file).rdbuf();
  std::string text = content.str();
  ASSERT_EQ(text.rfind("id: 1\n", 0), 0u);
  text.replace(0, 6, "id: 9\n");
  std::ofstream(file, std::ios::trunc) << text;
  expect_load_fails();
}

TEST_F(MapIoTest, MissingDirectoryFails) {
  expect_load_fails();
}
