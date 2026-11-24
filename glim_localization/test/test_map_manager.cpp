#include <filesystem>
#include <memory>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

#include <glim_localization/map/map_manager.hpp>

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

}  // namespace

class MapManagerTest : public testing::Test {
protected:
  static void SetUpTestSuite() {
    dir_ = new fs::path(fs::temp_directory_path() / ("glim_loc_map_manager_" + std::to_string(::getpid())));
    fs::remove_all(*dir_);
    SyntheticMapParams params;
    params.point_spacing = 0.4;
    truth_ = new SyntheticMap(write_synthetic_dump(dir_->string(), params));
    manager_ = new MapManager();
    ASSERT_TRUE(manager_->load(dir_->string()));
  }
  static void TearDownTestSuite() {
    delete manager_;
    delete truth_;
    fs::remove_all(*dir_);
    delete dir_;
  }

  static fs::path* dir_;
  static SyntheticMap* truth_;
  static MapManager* manager_;
};

fs::path* MapManagerTest::dir_ = nullptr;
SyntheticMap* MapManagerTest::truth_ = nullptr;
MapManager* MapManagerTest::manager_ = nullptr;

TEST_F(MapManagerTest, BuildsTilesWithVoxelmaps) {
  ASSERT_EQ(manager_->size(), 8u);
  for (const auto& truth : truth_->submaps) {
    const auto tile = manager_->tile(truth.id);
    ASSERT_NE(tile, nullptr);
    EXPECT_EQ(tile->points->size(), truth.num_points);
    ASSERT_EQ(tile->voxelmaps.size(), 3u);
    EXPECT_DOUBLE_EQ(tile->voxelmaps[0]->voxel_resolution(), 0.5);
    EXPECT_DOUBLE_EQ(tile->voxelmaps[1]->voxel_resolution(), 1.0);
    EXPECT_DOUBLE_EQ(tile->voxelmaps[2]->voxel_resolution(), 2.0);
    EXPECT_TRUE((tile->bbox_min.array() <= tile->T_world_origin.translation().array()).all());
    EXPECT_TRUE((tile->bbox_max.array() >= tile->T_world_origin.translation().array()).all());
  }
  EXPECT_EQ(manager_->tile(99), nullptr);
}

TEST_F(MapManagerTest, TilesListsEveryTileById) {
  const auto tiles = manager_->tiles();
  ASSERT_EQ(tiles.size(), truth_->submaps.size());
  for (size_t i = 0; i < tiles.size(); i++) {
    EXPECT_EQ(tiles[i], manager_->tile(tiles[i]->id));
    if (i > 0) {
      EXPECT_LT(tiles[i - 1]->id, tiles[i]->id);
    }
  }
  EXPECT_TRUE(MapManager().tiles().empty());
}

TEST_F(MapManagerTest, QueryOrdersByDistanceThenId) {
  EXPECT_EQ(ids(manager_->query(pose_at(30.0), 25.0, 5)), (std::vector<int>{3, 2, 4, 1, 5}));
  EXPECT_EQ(ids(manager_->query(pose_at(30.0), 12.0, 5)), (std::vector<int>{3, 2, 4}));
  EXPECT_EQ(ids(manager_->query(pose_at(30.0), 25.0, 2)), (std::vector<int>{3, 2}));
  EXPECT_EQ(ids(manager_->query(pose_at(32.0), 25.0, 2)), (std::vector<int>{3, 4}));
}

TEST_F(MapManagerTest, QueryHandlesLimits) {
  EXPECT_TRUE(manager_->query(pose_at(1000.0), 25.0, 5).empty());
  EXPECT_EQ(manager_->query(pose_at(30.0), 1000.0, 100).size(), 8u);
  EXPECT_TRUE(manager_->query(pose_at(30.0), 0.0, 5).empty());
  EXPECT_TRUE(manager_->query(pose_at(30.0), -1.0, 5).empty());
  EXPECT_TRUE(manager_->query(pose_at(30.0), 25.0, 0).empty());
}

TEST_F(MapManagerTest, ConcurrentQueriesAgree) {
  const auto expected = ids(manager_->query(pose_at(30.0), 25.0, 5));
  std::vector<std::vector<int>> results(4);
  std::vector<std::thread> threads;
  for (int t = 0; t < 4; t++) {
    threads.emplace_back([&results, t] {
      for (int i = 0; i < 200; i++) {
        results[t] = ids(manager_->query(pose_at(30.0), 25.0, 5));
      }
    });
  }
  for (auto& thread : threads) {
    thread.join();
  }
  for (const auto& result : results) {
    EXPECT_EQ(result, expected);
  }
}

TEST_F(MapManagerTest, FailedLoadKeepsCurrentMap) {
  MapManager manager;
  ASSERT_TRUE(manager.load(dir_->string()));
  const auto held = manager.tile(3);
  EXPECT_FALSE(manager.load((*dir_ / "does_not_exist").string()));
  EXPECT_EQ(manager.size(), 8u);
  EXPECT_EQ(ids(manager.query(pose_at(30.0), 12.0, 1)), (std::vector<int>{3}));
  EXPECT_EQ(manager.tile(3), held);
}

TEST(MapManagerEmpty, QueriesReturnNothing) {
  MapManager manager;
  EXPECT_EQ(manager.size(), 0u);
  EXPECT_EQ(manager.tile(0), nullptr);
  EXPECT_TRUE(manager.query(Eigen::Isometry3d::Identity(), 10.0, 3).empty());
}

TEST(MapManagerParams, RejectsInvalidResolution) {
  MapManagerParams params;
  params.voxel_resolutions = {0.5, 0.0};
  MapManager manager(params);
  EXPECT_FALSE(manager.load("/nonexistent"));
}
