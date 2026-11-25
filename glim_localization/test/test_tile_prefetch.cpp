#include <cmath>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

#include <glim_localization/map/map_manager.hpp>
#include <glim_localization/map/tile_prefetch.hpp>

#include "synthetic/synthetic_map.hpp"

namespace fs = std::filesystem;
using namespace glim_localization;
using namespace glim_localization::test;

namespace {

std::vector<int> ids(const std::vector<MapTile::ConstPtr>& tiles) {
  std::vector<int> result;
  for (const auto& tile : tiles) {
    result.push_back(tile->id);
  }
  return result;
}

Eigen::Isometry3d pose_at(double x) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(x, 0.0, 1.0);
  return T;
}

TilePrefetchParams params(double radius, int max_submaps, double lookahead, int max_tiles) {
  TilePrefetchParams p;
  p.search_radius = radius;
  p.max_submaps = max_submaps;
  p.lookahead = lookahead;
  p.max_tiles = max_tiles;
  return p;
}

}  // namespace

// Eight tiles with origins at x = 0, 10, ..., 70 (ids 0 to 7)
class TilePrefetchTest : public testing::Test {
protected:
  static void SetUpTestSuite() {
    dir_ = new fs::path(fs::temp_directory_path() / ("glim_loc_tile_prefetch_" + std::to_string(::getpid())));
    fs::remove_all(*dir_);
    SyntheticMapParams map_params;
    map_params.point_spacing = 0.5;
    write_synthetic_dump(dir_->string(), map_params);
    manager_ = new MapManager();
    ASSERT_TRUE(manager_->load(dir_->string()));
    ASSERT_EQ(manager_->size(), 8u);
  }
  static void TearDownTestSuite() {
    delete manager_;
    fs::remove_all(*dir_);
    delete dir_;
  }

  static fs::path* dir_;
  static MapManager* manager_;
};

fs::path* TilePrefetchTest::dir_ = nullptr;
MapManager* TilePrefetchTest::manager_ = nullptr;

TEST_F(TilePrefetchTest, StandingStillPrefetchesTheCurrentTiles) {
  const auto tiles = select_prefetch_tiles(*manager_, pose_at(21.0), Eigen::Vector3d::Zero(), params(15.0, 2, 1.0, 10));
  // What the map factors query at the pose
  EXPECT_EQ(ids(tiles), ids(manager_->query(pose_at(21.0), 15.0, 2)));
  EXPECT_EQ(ids(tiles), (std::vector<int>{2, 3}));
}

TEST_F(TilePrefetchTest, FollowsThePredictedPath) {
  // 10 m/s for 3 s: the path from x = 21 to x = 51, sampled at least every 7.5 m
  const auto tiles = select_prefetch_tiles(*manager_, pose_at(21.0), Eigen::Vector3d(10.0, 0.0, 0.0), params(15.0, 2, 3.0, 10));
  // Every tile the map factors would take along the path: those on it first (nearest to the current position
  // first), then those beside its ends. Tiles farther behind or ahead are not taken.
  EXPECT_EQ(ids(tiles), (std::vector<int>{3, 4, 5, 2, 6}));
}

TEST_F(TilePrefetchTest, KeepsTheMostUrgentWhenLimited) {
  const auto tiles = select_prefetch_tiles(*manager_, pose_at(21.0), Eigen::Vector3d(10.0, 0.0, 0.0), params(15.0, 2, 3.0, 2));
  EXPECT_EQ(ids(tiles), (std::vector<int>{3, 4}));
}

TEST_F(TilePrefetchTest, MovingBackwardPrefetchesBehind) {
  const auto tiles = select_prefetch_tiles(*manager_, pose_at(21.0), Eigen::Vector3d(-10.0, 0.0, 0.0), params(15.0, 2, 2.0, 10));
  EXPECT_EQ(ids(tiles), (std::vector<int>{2, 1, 0, 3}));
}

TEST_F(TilePrefetchTest, IgnoresAnInvalidVelocityOrLookahead) {
  const auto still = ids(select_prefetch_tiles(*manager_, pose_at(21.0), Eigen::Vector3d::Zero(), params(15.0, 2, 1.0, 10)));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(ids(select_prefetch_tiles(*manager_, pose_at(21.0), Eigen::Vector3d(nan, 0.0, 0.0), params(15.0, 2, 1.0, 10))), still);
  EXPECT_EQ(ids(select_prefetch_tiles(*manager_, pose_at(21.0), Eigen::Vector3d(10.0, 0.0, 0.0), params(15.0, 2, nan, 10))), still);
  EXPECT_EQ(ids(select_prefetch_tiles(*manager_, pose_at(21.0), Eigen::Vector3d(10.0, 0.0, 0.0), params(15.0, 2, -1.0, 10))), still);
  // A huge velocity is sampled at a bounded number of points and still returns at most max_tiles tiles.
  const auto fast = select_prefetch_tiles(*manager_, pose_at(21.0), Eigen::Vector3d(1e12, 0.0, 0.0), params(15.0, 2, 1.0, 3));
  EXPECT_FALSE(fast.empty());  // The tiles at the current position are still taken
  EXPECT_LE(fast.size(), 3u);
  // Far beyond the int range too: the sample count is capped before it is converted
  EXPECT_FALSE(select_prefetch_tiles(*manager_, pose_at(21.0), Eigen::Vector3d(1e300, 0.0, 0.0), params(15.0, 2, 1.0, 3)).empty());
}

TEST_F(TilePrefetchTest, ReturnsNothingForInvalidParameters) {
  EXPECT_TRUE(select_prefetch_tiles(*manager_, pose_at(21.0), Eigen::Vector3d::Zero(), params(15.0, 2, 1.0, 0)).empty());
  EXPECT_TRUE(select_prefetch_tiles(*manager_, pose_at(21.0), Eigen::Vector3d::Zero(), params(15.0, 0, 1.0, 10)).empty());
  EXPECT_TRUE(select_prefetch_tiles(*manager_, pose_at(21.0), Eigen::Vector3d::Zero(), params(0.0, 2, 1.0, 10)).empty());
  EXPECT_TRUE(select_prefetch_tiles(*manager_, pose_at(1000.0), Eigen::Vector3d::Zero(), params(15.0, 2, 1.0, 10)).empty());
  EXPECT_TRUE(select_prefetch_tiles(MapManager(), pose_at(21.0), Eigen::Vector3d::Zero(), params(15.0, 2, 1.0, 10)).empty());
}
