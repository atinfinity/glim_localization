#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <unistd.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim_localization/map/occupancy_grid.hpp>
#include <glim_localization/map/pcd_map.hpp>

namespace fs = std::filesystem;
using namespace glim_localization;

namespace {

/// One submap holding `world_points`, stored in a rotated and shifted origin frame so that the grid has to
/// transform them back.
LoadedSubmaps make_submaps(const PcdPoints& world_points, bool transform = true) {
  LoadedSubmap submap;
  submap.id = 0;
  if (transform) {
    submap.T_world_origin = Eigen::Translation3d(3.0, -2.0, 0.5) * Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ());
  }
  PcdPoints local(world_points.size());
  for (size_t i = 0; i < world_points.size(); i++) {
    local[i] << submap.T_world_origin.inverse() * world_points[i].head<3>(), 1.0;
  }
  submap.points = std::make_shared<gtsam_points::PointCloudCPU>(local);
  LoadedSubmaps submaps;
  submaps.push_back(submap);
  return submaps;
}

/// Cell centre of (ix, iy) at the given resolution, at height z
Eigen::Vector4d at_cell(int ix, int iy, double z, double resolution = 0.5) {
  return Eigen::Vector4d((ix + 0.5) * resolution, (iy + 0.5) * resolution, z, 1.0);
}

std::string read_file(const fs::path& path) {
  std::ifstream ifs(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
}

}  // namespace

class OccupancyGridTest : public testing::Test {
protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("glim_loc_occupancy_grid_" + std::to_string(::getpid()));
    fs::remove_all(dir_);
    params_.resolution = 0.5;
    params_.z_min = 0.2;
    params_.z_max = 2.0;

    // Cells x -2..1, y 3..5 (map frame, 0.5 m):
    //   y = 5: occupied (-2), free (-1), unknown (0), free (1)
    //   y = 4: free everywhere except (0, 4), which only has a ceiling point and stays unknown
    //   y = 3: free (-2), free (-1), free (0), occupied (1)
    for (int ix = -2; ix <= 1; ix++) {
      for (int iy = 3; iy <= 5; iy++) {
        if ((ix == 0 && iy == 5) || (ix == 0 && iy == 4)) {
          continue;
        }
        points_.push_back(at_cell(ix, iy, -0.1));  // ground
      }
    }
    points_.push_back(at_cell(-2, 5, 1.0));
    points_.push_back(at_cell(-2, 5, -0.5));
    points_.push_back(at_cell(1, 3, 0.25));
    points_.push_back(at_cell(0, 4, 2.5));   // above the band: ignored
    points_.push_back(at_cell(5, 9, 3.0));   // above the band, outside the extent: ignored
  }
  void TearDown() override { fs::remove_all(dir_); }

  fs::path dir_;
  OccupancyGridParams params_;
  PcdPoints points_;
};

TEST_F(OccupancyGridTest, ClassifiesCellsByTheHeightBand) {
  OccupancyGrid grid;
  ASSERT_TRUE(build_occupancy_grid(make_submaps(points_), params_, grid));
  EXPECT_EQ(grid.resolution, 0.5);
  EXPECT_EQ(grid.width, 4);
  EXPECT_EQ(grid.height, 3);
  EXPECT_NEAR(grid.origin.x(), -1.0, 1e-12);
  EXPECT_NEAR(grid.origin.y(), 1.5, 1e-12);
  ASSERT_EQ(grid.data.size(), 12u);

  const int8_t O = kOccupancyOccupied, F = kOccupancyFree, U = kOccupancyUnknown;
  // Rows from the lowest y (y = 3) up, as in nav_msgs/OccupancyGrid
  const std::vector<int8_t> expected = {F, F, F, O,  //
                                        F, F, U, F,  //
                                        O, F, U, F};
  EXPECT_EQ(grid.data, expected);
  EXPECT_EQ(grid.at(0, 2), O);
  EXPECT_EQ(grid.at(3, 0), O);
}

TEST_F(OccupancyGridTest, TheBandIncludesZMinAndExcludesZMax) {
  // No transform, so the heights stay exact.
  const PcdPoints points = {at_cell(0, 0, 0.2), at_cell(1, 0, 2.0), at_cell(1, 0, 0.0), at_cell(2, 0, 0.19)};
  OccupancyGrid grid;
  ASSERT_TRUE(build_occupancy_grid(make_submaps(points, false), params_, grid));
  ASSERT_EQ(grid.width, 3);
  ASSERT_EQ(grid.height, 1);
  EXPECT_EQ(grid.data, (std::vector<int8_t>{kOccupancyOccupied, kOccupancyFree, kOccupancyFree}));
}

TEST_F(OccupancyGridTest, CellEdgesAreMultiplesOfTheResolution) {
  // Far from the origin, as with UTM coordinates
  PcdPoints points;
  for (const auto& p : points_) {
    points.push_back(p + Eigen::Vector4d(500000.0, 4000000.0, 0.0, 0.0));
  }
  OccupancyGrid grid;
  ASSERT_TRUE(build_occupancy_grid(make_submaps(points), params_, grid));
  EXPECT_EQ(grid.width, 4);
  EXPECT_EQ(grid.height, 3);
  EXPECT_NEAR(grid.origin.x(), 500000.0 - 1.0, 1e-9);
  EXPECT_NEAR(grid.origin.y(), 4000000.0 + 1.5, 1e-9);
  EXPECT_EQ(std::fmod(grid.origin.x(), 0.5), 0.0);
}

TEST_F(OccupancyGridTest, RejectsBadParameters) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  OccupancyGridParams defaults;
  EXPECT_NE(check_occupancy_grid_params(defaults).find("must both be set"), std::string::npos);
  for (const auto& [resolution, z_min, z_max, message] : std::vector<std::tuple<double, double, double, std::string>>{
         {0.005, 0.0, 1.0, "resolution"},
         {nan, 0.0, 1.0, "resolution"},
         {std::numeric_limits<double>::infinity(), 0.0, 1.0, "resolution"},
         {0.05, nan, 1.0, "must both be set"},
         {0.05, 0.0, std::numeric_limits<double>::infinity(), "must both be set"},
         {0.05, 1.0, 1.0, "must be below"},
         {0.05, 2.0, 1.0, "must be below"}}) {
    OccupancyGridParams params;
    params.resolution = resolution;
    params.z_min = z_min;
    params.z_max = z_max;
    EXPECT_NE(check_occupancy_grid_params(params).find(message), std::string::npos) << resolution << " " << z_min << " " << z_max;
    OccupancyGrid grid;
    grid.width = 7;
    EXPECT_FALSE(build_occupancy_grid(make_submaps(points_), params, grid));
    EXPECT_EQ(grid.width, 7);
  }
  params_.resolution = 0.01;
  EXPECT_EQ(check_occupancy_grid_params(params_), "");
}

TEST_F(OccupancyGridTest, FailsWithoutPointsBelowTheTop) {
  PcdPoints ceiling = {at_cell(0, 0, 2.5), at_cell(1, 0, 3.0)};
  OccupancyGrid grid;
  EXPECT_FALSE(build_occupancy_grid(make_submaps(ceiling), params_, grid));
  EXPECT_FALSE(build_occupancy_grid(LoadedSubmaps(), params_, grid));
  EXPECT_TRUE(grid.data.empty());
}

TEST_F(OccupancyGridTest, RejectsGridsAboveTheCellLimit) {
  // 20 km x 20 km at 0.01 m is 4e12 cells; refused before allocating.
  PcdPoints wide = {Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), Eigen::Vector4d(20000.0, 20000.0, 0.0, 1.0)};
  params_.resolution = 0.01;
  OccupancyGrid grid;
  EXPECT_FALSE(build_occupancy_grid(make_submaps(wide), params_, grid));
  PcdPoints far = {Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), Eigen::Vector4d(1e17, 0.0, 0.0, 1.0)};
  EXPECT_FALSE(build_occupancy_grid(make_submaps(far), params_, grid));
}

TEST_F(OccupancyGridTest, SavesPgmWithTheTopRowAtTheHighestY) {
  OccupancyGrid grid;
  ASSERT_TRUE(build_occupancy_grid(make_submaps(points_), params_, grid));
  const fs::path prefix = dir_ / "nested" / "floor";
  ASSERT_TRUE(save_occupancy_grid(prefix.string(), grid));
  EXPECT_FALSE(fs::exists(dir_ / "nested" / "floor.pgm.tmp"));
  EXPECT_FALSE(fs::exists(dir_ / "nested" / "floor.yaml.tmp"));

  const std::string pgm = read_file(dir_ / "nested" / "floor.pgm");
  // Header: magic, a comment, width and height, max value
  std::istringstream header(pgm);
  std::string magic, comment;
  int width = 0, height = 0, max_value = 0;
  std::getline(header, magic);
  std::getline(header, comment);
  header >> width >> height >> max_value;
  header.get();  // The single whitespace before the pixels
  EXPECT_EQ(magic, "P5");
  EXPECT_EQ(comment[0], '#');
  EXPECT_EQ(width, 4);
  EXPECT_EQ(height, 3);
  EXPECT_EQ(max_value, 255);
  const auto offset = static_cast<size_t>(header.tellg());
  ASSERT_EQ(pgm.size(), offset + 12);
  std::vector<int> pixels;
  for (size_t i = offset; i < pgm.size(); i++) {
    pixels.push_back(static_cast<unsigned char>(pgm[i]));
  }
  // The first row is y = 5 (the highest y), the last y = 3.
  const std::vector<int> expected = {0,   254, 205, 254,  //
                                     254, 254, 205, 254,  //
                                     254, 254, 254, 0};
  EXPECT_EQ(pixels, expected);

  EXPECT_EQ(read_file(dir_ / "nested" / "floor.yaml"),
            "image: floor.pgm\n"
            "mode: trinary\n"
            "resolution: 0.500000000\n"
            "origin: [-1.000000000, 1.500000000, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.1\n");
}

TEST_F(OccupancyGridTest, SaveRefusesExistingFiles) {
  OccupancyGrid grid;
  ASSERT_TRUE(build_occupancy_grid(make_submaps(points_), params_, grid));
  for (const std::string existing : {"map.pgm", "map.yaml"}) {
    fs::remove_all(dir_);
    fs::create_directories(dir_);
    std::ofstream(dir_ / existing) << "keep";
    EXPECT_FALSE(save_occupancy_grid((dir_ / "map").string(), grid)) << existing;
    EXPECT_EQ(read_file(dir_ / existing), "keep");
    EXPECT_EQ(std::distance(fs::directory_iterator(dir_), fs::directory_iterator()), 1) << existing;
  }
  OccupancyGrid empty;
  EXPECT_FALSE(save_occupancy_grid((dir_ / "empty").string(), empty));
}
