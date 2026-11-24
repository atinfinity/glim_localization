#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim_localization/map/map_io.hpp>
#include <glim_localization/map/pcd_map.hpp>

#include "synthetic/synthetic_map.hpp"

namespace fs = std::filesystem;
using namespace glim_localization;
using namespace glim_localization::test;

class PcdMapTest : public testing::Test {
protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("glim_loc_pcd_map_" + std::to_string(::getpid()));
    fs::remove_all(dir_);
    fs::create_directories(dir_);
    params_.num_submaps = 3;
    params_.point_spacing = 0.4;
    params_.world_offset = Eigen::Vector3d(12345.678901, -9876.54321, 321.123456);
    points_ = make_synthetic_world_points(params_);
  }
  void TearDown() override { fs::remove_all(dir_); }

  fs::path write_text(const std::string& name, const std::string& text) const {
    const fs::path path = dir_ / name;
    std::ofstream(path, std::ios::binary) << text;
    return path;
  }

  /// A failed read must return false and keep the caller's data.
  static void expect_read_fails(const fs::path& path) {
    PcdPoints points(1, Eigen::Vector4d(7.0, 7.0, 7.0, 1.0));
    EXPECT_FALSE(read_pcd(path.string(), points));
    ASSERT_EQ(points.size(), 1u);
    EXPECT_EQ(points[0].x(), 7.0);
  }

  /// Like expect_read_fails, and returns what read_pcd logged.
  static std::string read_fails_with_log(const fs::path& path) {
    std::ostringstream log;
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(log);
    spdlog::default_logger()->sinks().push_back(sink);
    expect_read_fails(path);
    spdlog::default_logger()->sinks().pop_back();
    return log.str();
  }

  fs::path dir_;
  SyntheticMapParams params_;
  SyntheticPoints points_;
};

TEST_F(PcdMapTest, ReadsAsciiAndBinaryWithExtraFields) {
  for (const bool double_xyz : {false, true}) {
    for (const auto encoding : {PcdEncoding::Ascii, PcdEncoding::Binary}) {
      const fs::path path = dir_ / "cloud.pcd";
      write_synthetic_pcd(path.string(), points_, encoding, double_xyz);
      PcdPoints read;
      ASSERT_TRUE(read_pcd(path.string(), read));
      ASSERT_EQ(read.size(), points_.size());
      // Binary F4 rounds to float, about 1e-3 m at 12345 m
      const double tolerance = encoding == PcdEncoding::Binary && !double_xyz ? 2e-3 : 1e-9;
      for (size_t i = 0; i < read.size(); i++) {
        ASSERT_LT((read[i] - points_[i]).cwiseAbs().maxCoeff(), tolerance) << i;
        ASSERT_EQ(read[i].w(), 1.0);
      }
    }
  }
}

TEST_F(PcdMapTest, SkipsFieldsWithCountAndDropsNonFinitePoints) {
  // rgb (U4), a two-element normal-like field, organized 2 x 2 with one NaN point, CRLF line ends
  const auto path = write_text(
    "fields.pcd",
    "VERSION .7\r\nFIELDS rgb x n y z\r\nSIZE 4 4 8 4 4\r\nTYPE U F F F F\r\nCOUNT 1 1 2 1 1\r\nWIDTH 2\r\nHEIGHT 2\r\n"
    "POINTS 4\r\nDATA ascii\r\n"
    "1 1.5 9 9 2.5 3.5\r\n2 nan 9 9 0 0\r\n\r\n3 -1 9 9 -2 -3\r\n4 10 9 9 20 30\r\n");
  PcdPoints read;
  ASSERT_TRUE(read_pcd(path.string(), read));
  ASSERT_EQ(read.size(), 3u);
  EXPECT_EQ(read[0], Eigen::Vector4d(1.5, 2.5, 3.5, 1.0));
  EXPECT_EQ(read[1], Eigen::Vector4d(-1.0, -2.0, -3.0, 1.0));
  EXPECT_EQ(read[2], Eigen::Vector4d(10.0, 20.0, 30.0, 1.0));
}

TEST_F(PcdMapTest, ReadsBinaryWithFieldsBeforeXyz) {
  std::string text = "FIELDS label x y z\nSIZE 2 4 4 4\nTYPE U F F F\nWIDTH 2\nHEIGHT 1\nDATA binary\n";
  for (int i = 0; i < 2; i++) {
    const uint16_t label = 5;
    const float xyz[3] = {1.0f + i, 2.0f + i, 3.0f + i};
    text.append(reinterpret_cast<const char*>(&label), sizeof(label));
    text.append(reinterpret_cast<const char*>(xyz), sizeof(xyz));
  }
  PcdPoints read;
  ASSERT_TRUE(read_pcd(write_text("binary.pcd", text).string(), read));
  ASSERT_EQ(read.size(), 2u);
  EXPECT_EQ(read[1], Eigen::Vector4d(2.0, 3.0, 4.0, 1.0));
}

TEST_F(PcdMapTest, RejectsUnsupportedOrBrokenFiles) {
  const std::string fields = "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n";
  expect_read_fails(dir_ / "missing.pcd");
  expect_read_fails(write_text("compressed.pcd", fields + "WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary_compressed\n"));
  expect_read_fails(write_text("no_data.pcd", fields + "WIDTH 1\nHEIGHT 1\nPOINTS 1\n"));
  expect_read_fails(write_text("no_z.pcd", "FIELDS x y\nSIZE 4 4\nTYPE F F\nPOINTS 1\nDATA ascii\n1 2\n"));
  expect_read_fails(write_text("int_x.pcd", "FIELDS x y z\nSIZE 4 4 4\nTYPE I F F\nPOINTS 1\nDATA ascii\n1 2 3\n"));
  expect_read_fails(write_text("size_mismatch.pcd", "FIELDS x y z\nSIZE 4 4\nTYPE F F F\nPOINTS 1\nDATA ascii\n1 2 3\n"));
  expect_read_fails(write_text("count_mismatch.pcd", fields + "WIDTH 2\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1 2 3\n"));
  expect_read_fails(write_text("no_points.pcd", fields + "DATA ascii\n1 2 3\n"));
  expect_read_fails(write_text("short_ascii.pcd", fields + "POINTS 2\nDATA ascii\n1 2 3\n"));
  expect_read_fails(write_text("short_row.pcd", fields + "POINTS 1\nDATA ascii\n1 2\n"));
  expect_read_fails(write_text("bad_value.pcd", fields + "POINTS 1\nDATA ascii\n1 2 x3\n"));
  expect_read_fails(write_text("short_binary.pcd", fields + "POINTS 2\nDATA binary\n" + std::string(12, '\0')));
  expect_read_fails(write_text("huge_binary.pcd", fields + "POINTS 9223372036854775807\nDATA binary\n" + std::string(12, '\0')));
}

TEST_F(PcdMapTest, RejectsOverflowingCounts) {
  // COUNT 2^32 - 3 wrapped to -3 in int and made a point 0 bytes long (division by zero, then a crash).
  expect_read_fails(write_text("count_zero_bytes.pcd", "FIELDS x y z w\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 4294967293\nPOINTS 2\n"
                                                       "DATA binary\n" +
                                                         std::string(32, '\0')));
  // COUNT 2^32 - 1 wrapped to -1 and made a point 8 bytes long, so z was read past the buffer.
  expect_read_fails(write_text("count_short_point.pcd", "FIELDS x y z w\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 4294967295\nPOINTS 2\n"
                                                        "DATA binary\n" +
                                                          std::string(16, '\0')));
  // A wrapped COUNT before x made the ascii value index of x 2^64 - 1 (a crash on the token lookup).
  expect_read_fails(
    write_text("count_ascii_index.pcd", "FIELDS pad x y z\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 4294967295 1 1 1\nPOINTS 1\nDATA ascii\n1 2\n"));
  // 8 x (2^61 - 1) wraps even in 64 bits: the point would be 4 bytes long.
  expect_read_fails(write_text("count_wraps_size_t.pcd", "FIELDS x y z w\nSIZE 4 4 4 8\nTYPE F F F F\nCOUNT 1 1 1 2305843009213693951\nPOINTS 3\n"
                                                         "DATA binary\n" +
                                                           std::string(12, '\0')));
  // 8 x (2^61 + 1) wraps to 8: x, y and z fit, and one 20-byte point matches the file. Only the COUNT cap catches it.
  expect_read_fails(write_text("count_wraps_to_fit.pcd", "FIELDS x y z w\nSIZE 4 4 4 8\nTYPE F F F F\nCOUNT 1 1 1 2305843009213693953\nPOINTS 1\n"
                                                         "DATA binary\n" +
                                                           std::string(20, '\0')));
}

TEST_F(PcdMapTest, RejectsRepeatedFieldNames) {
  // A repeated x would silently take the last one.
  expect_read_fails(write_text("repeated_x.pcd", "FIELDS x x y z\nSIZE 4 4 4 4\nTYPE F F F F\nPOINTS 1\nDATA ascii\n1 2 3 4\n"));
  expect_read_fails(
    write_text("repeated_other.pcd", "FIELDS x y z rgb rgb\nSIZE 4 4 4 4 4\nTYPE F F F U U\nPOINTS 1\nDATA ascii\n1 2 3 4 5\n"));
  // PCL pads with fields named "_", which may repeat.
  PcdPoints read;
  ASSERT_TRUE(read_pcd(
    write_text("padding.pcd", "FIELDS _ x _ y z\nSIZE 1 4 1 4 4\nTYPE U F U F F\nPOINTS 1\nDATA ascii\n0 1 0 2 3\n").string(), read));
  ASSERT_EQ(read.size(), 1u);
  EXPECT_EQ(read[0], Eigen::Vector4d(1.0, 2.0, 3.0, 1.0));
}

TEST_F(PcdMapTest, RejectsWidthTimesHeightOverflow) {
  // 2^32 x 2^32 wrapped to 0 points, which read as an empty cloud.
  expect_read_fails(write_text("width_height.pcd", "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nWIDTH 4294967296\nHEIGHT 4294967296\nDATA ascii\n"));
}

TEST_F(PcdMapTest, RejectsPointsAboveTheFileSizeBeforeAllocating) {
  // 10^15 points need 12 PB (binary) or 32 PB of points (ascii). The size check must fail first, not the allocation.
  const std::string fields = "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nPOINTS 1000000000000000\n";
  const std::string binary_log = read_fails_with_log(write_text("many_binary.pcd", fields + "DATA binary\n" + std::string(12, '\0')));
  EXPECT_NE(binary_log.find("truncated binary data"), std::string::npos) << binary_log;
  const std::string ascii_log = read_fails_with_log(write_text("many_ascii.pcd", fields + "DATA ascii\n1 2 3\n"));
  EXPECT_NE(ascii_log.find("truncated ascii data"), std::string::npos) << ascii_log;

  // The bound must still accept files that are exactly long enough, with or without a final newline.
  PcdPoints read;
  ASSERT_TRUE(read_pcd(write_text("exact_ascii.pcd", "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nPOINTS 2\nDATA ascii\n1 2 3\n4 5 6").string(), read));
  ASSERT_EQ(read.size(), 2u);
  EXPECT_EQ(read[1], Eigen::Vector4d(4.0, 5.0, 6.0, 1.0));
}

TEST_F(PcdMapTest, SplitsIntoGridTilesWithCovariances) {
  PcdMapParams params;
  params.grid_size = 20.0;
  LoadedSubmaps submaps;
  ASSERT_TRUE(build_pcd_submaps(points_, params, submaps));
  ASSERT_GT(submaps.size(), 2u);

  size_t total = 0;
  std::set<std::pair<long, long>> cells;
  for (size_t i = 0; i < submaps.size(); i++) {
    const auto& submap = submaps[i];
    EXPECT_EQ(submap.id, static_cast<int>(i));
    EXPECT_TRUE(submap.T_world_origin.linear().isIdentity(0.0));
    ASSERT_TRUE(submap.points->has_covs());
    ASSERT_GT(submap.points->size(), 0u);
    total += submap.points->size();

    // Every point of a tile is in one grid cell, and the origin is the tile centroid.
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    const Eigen::Vector3d first = submap.T_world_origin * submap.points->points[0].head<3>();
    const long cx = static_cast<long>(std::floor(first.x() / params.grid_size));
    const long cy = static_cast<long>(std::floor(first.y() / params.grid_size));
    EXPECT_TRUE(cells.insert({cx, cy}).second);
    for (size_t j = 0; j < submap.points->size(); j++) {
      const Eigen::Vector3d p = submap.T_world_origin * submap.points->points[j].head<3>();
      ASSERT_EQ(static_cast<long>(std::floor(p.x() / params.grid_size)), cx);
      ASSERT_EQ(static_cast<long>(std::floor(p.y() / params.grid_size)), cy);
      ASSERT_EQ(submap.points->points[j].w(), 1.0);
      const Eigen::Matrix4d& cov = submap.points->covs[j];
      ASSERT_TRUE(cov.allFinite());
      ASSERT_TRUE(cov.isApprox(cov.transpose()));
      const Eigen::Matrix3d cov3 = cov.topLeftCorner<3, 3>();
      ASSERT_GT(cov3.determinant(), 0.0);
      mean += submap.points->points[j].head<3>();
    }
    EXPECT_LT((mean / submap.points->size()).norm(), 1e-6);
  }
  EXPECT_EQ(total, points_.size());
}

TEST_F(PcdMapTest, CovariancesOnAFloorAreFlat) {
  // A flat 10 x 10 m floor: the smallest eigenvector of every covariance is the z axis.
  PcdPoints floor;
  for (double x = 0.0; x < 10.0; x += 0.2) {
    for (double y = 0.0; y < 10.0; y += 0.2) {
      floor.emplace_back(x, y, 0.0, 1.0);
    }
  }
  LoadedSubmaps submaps;
  ASSERT_TRUE(build_pcd_submaps(floor, PcdMapParams(), submaps));
  ASSERT_EQ(submaps.size(), 1u);
  for (size_t i = 0; i < submaps[0].points->size(); i++) {
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(submaps[0].points->covs[i].topLeftCorner<3, 3>());
    ASSERT_GT(std::abs(eig.eigenvectors().col(0).z()), 0.99) << i;
  }
}

TEST_F(PcdMapTest, VoxelSizeDownsamples) {
  PcdMapParams params;
  params.voxel_size = 1.0;
  LoadedSubmaps submaps;
  ASSERT_TRUE(build_pcd_submaps(points_, params, submaps));
  size_t total = 0;
  for (const auto& submap : submaps) {
    total += submap.points->size();
  }
  EXPECT_GT(total, 0u);
  EXPECT_LT(total, points_.size() / 2);
}

TEST_F(PcdMapTest, VoxelSizeWorksFarFromTheOrigin) {
  // A 5 x 5 m floor at UTM-like coordinates. In world coordinates the voxel grid would overflow its 21 bits per axis
  // and merge everything into a few made-up points.
  const Eigen::Vector3d origin(500000.0, 4000000.0, 50.0);
  PcdPoints floor;
  for (int i = 0; i <= 100; i++) {
    for (int j = 0; j <= 100; j++) {
      floor.emplace_back(origin.x() + 0.05 * i, origin.y() + 0.05 * j, origin.z(), 1.0);
    }
  }
  PcdMapParams params;
  params.voxel_size = 0.1;
  LoadedSubmaps submaps;
  ASSERT_TRUE(build_pcd_submaps(floor, params, submaps));

  size_t total = 0;
  for (const auto& submap : submaps) {
    for (size_t j = 0; j < submap.points->size(); j++) {
      const Eigen::Vector3d p = submap.T_world_origin * submap.points->points[j].head<3>();
      ASSERT_GE(p.x(), origin.x() - 1e-6);
      ASSERT_LE(p.x(), origin.x() + 5.0 + 1e-6);
      ASSERT_GE(p.y(), origin.y() - 1e-6);
      ASSERT_LE(p.y(), origin.y() + 5.0 + 1e-6);
      ASSERT_NEAR(p.z(), origin.z(), 1e-6);
    }
    total += submap.points->size();
  }
  // About 50 x 50 voxels of 0.1 m
  EXPECT_GT(total, 2000u);
  EXPECT_LT(total, 3000u);
}

TEST_F(PcdMapTest, VoxelSizeRejectsCloudsTooWideForTheGrid) {
  // 300 km at 0.1 m is about 3 x 10^6 voxels, more than the 2^21 the voxel grid can index.
  PcdPoints cloud;
  for (const double x : {0.0, 300000.0}) {
    for (int i = 0; i < 20; i++) {
      cloud.emplace_back(x + 0.3 * (i % 5), 0.3 * (i / 5), 0.0, 1.0);
    }
  }
  PcdMapParams params;
  params.voxel_size = 0.1;
  LoadedSubmaps submaps;
  // Without the check the voxel grid merges the far points into a few made-up ones, and the build may still fail
  // later for too few points. Require the extent error itself.
  std::ostringstream log;
  auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(log);
  spdlog::default_logger()->sinks().push_back(sink);
  const bool built = build_pcd_submaps(cloud, params, submaps);
  spdlog::default_logger()->sinks().pop_back();
  EXPECT_FALSE(built);
  EXPECT_NE(log.str().find("too wide for voxel size"), std::string::npos) << log.str();
  // A larger voxel fits.
  params.voxel_size = 0.2;
  EXPECT_TRUE(build_pcd_submaps(cloud, params, submaps));
}

TEST_F(PcdMapTest, SplitsCellsAcrossTheAxes) {
  // A floor from -30 to 30 m crosses x = 0 and y = 0. Truncating instead of flooring would merge cells -1 and 0.
  PcdPoints floor;
  for (int i = -60; i <= 60; i++) {
    for (int j = -60; j <= 60; j++) {
      floor.emplace_back(0.5 * i, 0.5 * j, 0.0, 1.0);
    }
  }
  PcdMapParams params;
  params.grid_size = 20.0;
  LoadedSubmaps submaps;
  ASSERT_TRUE(build_pcd_submaps(floor, params, submaps));
  ASSERT_EQ(submaps.size(), 16u);  // Cells -2 to 1 along each axis

  std::set<std::pair<long, long>> cells;
  for (const auto& submap : submaps) {
    const Eigen::Vector3d first = submap.T_world_origin * submap.points->points[0].head<3>();
    const long cx = static_cast<long>(std::floor(first.x() / params.grid_size));
    const long cy = static_cast<long>(std::floor(first.y() / params.grid_size));
    EXPECT_TRUE(cells.insert({cx, cy}).second);
    for (size_t j = 0; j < submap.points->size(); j++) {
      const Eigen::Vector3d p = submap.T_world_origin * submap.points->points[j].head<3>();
      ASSERT_EQ(static_cast<long>(std::floor(p.x() / params.grid_size)), cx) << p.transpose();
      ASSERT_EQ(static_cast<long>(std::floor(p.y() / params.grid_size)), cy) << p.transpose();
    }
  }
}

TEST_F(PcdMapTest, CovariancesUseNeighboursAcrossTileEdges) {
  // A curved surface over many 2 m tiles. Every covariance, including those of points on a tile edge, must equal
  // the estimate over the whole cloud, not one over the tile alone.
  PcdPoints surface;
  for (int i = -16; i <= 16; i++) {
    for (int j = -16; j <= 16; j++) {
      const double x = 0.25 * i;
      const double y = 0.25 * j;
      surface.emplace_back(x, y, 0.5 * std::sin(x) + 0.3 * std::cos(1.3 * y), 1.0);
    }
  }
  PcdMapParams params;
  params.grid_size = 2.0;
  LoadedSubmaps submaps;
  ASSERT_TRUE(build_pcd_submaps(surface, params, submaps));
  ASSERT_GT(submaps.size(), 9u);

  const auto expected = gtsam_points::estimate_covariances(surface.data(), surface.size(), params.k_neighbors, 1);
  std::map<std::pair<long, long>, size_t> index_of;
  for (size_t i = 0; i < surface.size(); i++) {
    index_of[{std::lround(surface[i].x() * 4.0), std::lround(surface[i].y() * 4.0)}] = i;
  }
  size_t checked = 0;
  for (const auto& submap : submaps) {
    for (size_t j = 0; j < submap.points->size(); j++) {
      const Eigen::Vector3d p = submap.T_world_origin * submap.points->points[j].head<3>();
      const auto found = index_of.find({std::lround(p.x() * 4.0), std::lround(p.y() * 4.0)});
      ASSERT_NE(found, index_of.end()) << p.transpose();
      ASSERT_LT((submap.points->covs[j] - expected[found->second]).cwiseAbs().maxCoeff(), 1e-9) << p.transpose();
      checked++;
    }
  }
  EXPECT_EQ(checked, surface.size());
}

TEST_F(PcdMapTest, RejectsBadParametersAndTooFewPoints) {
  const auto expect_build_fails = [](const PcdPoints& points, const PcdMapParams& params) {
    LoadedSubmaps submaps(1);
    submaps[0].id = 77;
    EXPECT_FALSE(build_pcd_submaps(points, params, submaps));
    ASSERT_EQ(submaps.size(), 1u);
    EXPECT_EQ(submaps[0].id, 77);
  };
  const PcdPoints points(points_.begin(), points_.end());

  PcdMapParams params;
  params.grid_size = 0.0;
  expect_build_fails(points, params);
  params = PcdMapParams();
  params.grid_size = std::numeric_limits<double>::infinity();
  expect_build_fails(points, params);
  params = PcdMapParams();
  params.voxel_size = -1.0;
  expect_build_fails(points, params);
  params = PcdMapParams();
  params.k_neighbors = 0;
  expect_build_fails(points, params);
  // Lower bounds: k >= 5, grid >= 1 m, voxel 0 or >= 0.01 m, all finite
  params = PcdMapParams();
  params.k_neighbors = 4;
  expect_build_fails(points, params);
  params = PcdMapParams();
  params.grid_size = 0.99;
  expect_build_fails(points, params);
  params = PcdMapParams();
  params.grid_size = std::numeric_limits<double>::quiet_NaN();
  expect_build_fails(points, params);
  params = PcdMapParams();
  params.voxel_size = 0.009;
  expect_build_fails(points, params);
  params = PcdMapParams();
  params.voxel_size = std::numeric_limits<double>::infinity();
  expect_build_fails(points, params);
  params = PcdMapParams();
  params.voxel_size = std::numeric_limits<double>::quiet_NaN();
  expect_build_fails(points, params);
  params = PcdMapParams();
  params.k_neighbors = kMinPcdKNeighbors;
  params.grid_size = kMinPcdGridSize;
  params.voxel_size = kMinPcdVoxelSize;
  EXPECT_EQ(check_pcd_map_params(params), "");

  expect_build_fails(PcdPoints(), PcdMapParams());
  expect_build_fails(PcdPoints(points.begin(), points.begin() + 9), PcdMapParams());
  PcdPoints huge = points;
  huge[3].x() = 1e300;
  expect_build_fails(huge, PcdMapParams());

  LoadedSubmaps submaps;
  EXPECT_TRUE(build_pcd_submaps(PcdPoints(points.begin(), points.begin() + 10), PcdMapParams(), submaps));
}

TEST_F(PcdMapTest, RoundTripsThroughCompiledMap) {
  LoadedSubmaps built, loaded;
  ASSERT_TRUE(build_pcd_submaps(points_, PcdMapParams(), built));
  ASSERT_TRUE(save_compiled_map((dir_ / "map").string(), built, "pcd"));
  ASSERT_TRUE(load_map((dir_ / "map").string(), loaded));
  ASSERT_EQ(loaded.size(), built.size());
  for (size_t i = 0; i < loaded.size(); i++) {
    EXPECT_EQ(loaded[i].id, built[i].id);
    EXPECT_LT((loaded[i].T_world_origin.matrix() - built[i].T_world_origin.matrix()).cwiseAbs().maxCoeff(), 1e-9);
    ASSERT_EQ(loaded[i].points->size(), built[i].points->size());
    ASSERT_TRUE(loaded[i].points->has_covs());
    for (size_t j = 0; j < loaded[i].points->size(); j++) {
      // Compact files store floats in the tile frame.
      ASSERT_LT((loaded[i].points->points[j] - built[i].points->points[j]).cwiseAbs().maxCoeff(), 1e-5);
      ASSERT_LT((loaded[i].points->covs[j] - built[i].points->covs[j]).cwiseAbs().maxCoeff(), 1e-5) << i << " " << j;
    }
  }

  std::ifstream index_file(dir_ / "map" / "map_index.json");
  const auto index = nlohmann::json::parse(index_file);
  EXPECT_EQ(index.at("source").get<std::string>(), "pcd");
}

namespace {

/// A curved surface on a jittered 0.25 m grid. The jitter breaks the distance ties of a regular grid, so the k nearest
/// neighbours do not depend on rounding in the frame they are searched in.
PcdPoints make_curved_surface(const Eigen::Vector3d& offset) {
  std::mt19937 rng(31);
  std::uniform_real_distribution<double> jitter(-0.08, 0.08);
  PcdPoints surface;
  for (int i = -16; i <= 16; i++) {
    for (int j = -16; j <= 16; j++) {
      const double x = 0.25 * i + jitter(rng);
      const double y = 0.25 * j + jitter(rng);
      surface.emplace_back(offset.x() + x, offset.y() + y, offset.z() + 0.5 * std::sin(x) + 0.3 * std::cos(1.3 * y), 1.0);
    }
  }
  return surface;
}

LoadedSubmap make_submap_with_identity_covs(int id, const PcdPoints& points) {
  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
  cloud->add_covs(std::vector<Eigen::Matrix4d>(points.size(), Eigen::Matrix4d::Identity()));
  LoadedSubmap submap;
  submap.id = id;
  submap.points = cloud;
  return submap;
}

}  // namespace

TEST_F(PcdMapTest, RecomputedCovariancesMatchThePcdPath) {
  const PcdPoints world = make_curved_surface(Eigen::Vector3d(500.0, -200.0, 30.0));
  for (const int k : {kMinPcdKNeighbors, 10, 25}) {
    // The PCD path with one tile: identity rotation, so its covariances are in the world frame.
    PcdMapParams params;
    params.grid_size = 1000.0;
    params.k_neighbors = k;
    LoadedSubmaps tiles;
    ASSERT_TRUE(build_pcd_submaps(world, params, tiles));
    ASSERT_EQ(tiles.size(), 1u);
    ASSERT_EQ(tiles[0].points->size(), world.size());

    // The same points as a GLIM submap with a rotated origin and stale (identity) covariances.
    Eigen::Isometry3d T_world_origin = Eigen::Isometry3d::Identity();
    T_world_origin.linear() = Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.3, -0.5, 1.0).normalized()).toRotationMatrix();
    T_world_origin.translation() = Eigen::Vector3d(498.0, -203.0, 31.0);
    PcdPoints local(world.size());
    for (size_t i = 0; i < world.size(); i++) {
      local[i] << T_world_origin.inverse() * world[i].head<3>(), 1.0;
    }
    LoadedSubmaps submaps;
    submaps.push_back(make_submap_with_identity_covs(3, local));
    submaps[0].T_world_origin = T_world_origin;
    const auto original = submaps[0].points;

    ASSERT_TRUE(recompute_covariances(submaps, k, 2));
    ASSERT_EQ(submaps.size(), 1u);
    EXPECT_EQ(submaps[0].id, 3);
    EXPECT_EQ(submaps[0].T_world_origin.matrix(), T_world_origin.matrix());
    ASSERT_NE(submaps[0].points, original);  // Copied, not changed in place
    EXPECT_EQ(original->covs[0], Eigen::Matrix4d::Identity());
    ASSERT_EQ(submaps[0].points->size(), world.size());
    ASSERT_TRUE(submaps[0].points->has_covs());
    const Eigen::Matrix3d R = T_world_origin.linear();
    for (size_t i = 0; i < world.size(); i++) {
      ASSERT_EQ(submaps[0].points->points[i], local[i]);
      const Eigen::Matrix4d& cov = submaps[0].points->covs[i];
      const Eigen::Matrix3d in_world = R * cov.topLeftCorner<3, 3>() * R.transpose();
      // About 5e-9 in the worst case (k = 5, points some 540 m from the origin); FMA can add to it, so keep a margin.
      ASSERT_LT((in_world - tiles[0].points->covs[i].topLeftCorner<3, 3>()).cwiseAbs().maxCoeff(), 1e-6) << "k " << k << " point " << i;
      ASSERT_EQ(cov.row(3).norm() + cov.col(3).norm(), 0.0) << "k " << k << " point " << i;
    }
  }
}

TEST_F(PcdMapTest, RecomputeCovariancesRejectsBadParametersAndSmallSubmaps) {
  const PcdPoints surface = make_curved_surface(Eigen::Vector3d::Zero());
  LoadedSubmaps submaps;
  submaps.push_back(make_submap_with_identity_covs(0, surface));
  submaps.push_back(make_submap_with_identity_covs(5, PcdPoints(surface.begin(), surface.begin() + 9)));
  const auto first = submaps[0].points;
  const auto second = submaps[1].points;
  const auto expect_unchanged = [&](const std::string& what) {
    ASSERT_EQ(submaps.size(), 2u) << what;
    EXPECT_EQ(submaps[0].points, first) << what;
    EXPECT_EQ(submaps[1].points, second) << what;
    EXPECT_EQ(first->covs[0], Eigen::Matrix4d::Identity()) << what;
  };

  EXPECT_FALSE(recompute_covariances(submaps, kMinPcdKNeighbors - 1, 1));
  expect_unchanged("k below the minimum");
  EXPECT_FALSE(recompute_covariances(submaps, 10, 0));
  expect_unchanged("no threads");
  // The second submap has 9 points, fewer than k = 10: nothing changes, not even the first submap.
  EXPECT_FALSE(recompute_covariances(submaps, 10, 1));
  expect_unchanged("small submap");
  EXPECT_TRUE(recompute_covariances(submaps, 9, 1));
  EXPECT_NE(submaps[0].points, first);
  EXPECT_NE(submaps[1].points, second);
}
