#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim_localization/map/tile_residency.hpp>

using namespace glim_localization;

namespace {

TileBudget budget(size_t max_bytes, int max_tiles) {
  TileBudget b;
  b.max_bytes = max_bytes;
  b.max_tiles = max_tiles;
  return b;
}

}  // namespace

TEST(TileResidency, RejectsAnEmptyBudget) {
  EXPECT_THROW(TileResidency(budget(0, 4)), std::invalid_argument);
  EXPECT_THROW(TileResidency(budget(100, 0)), std::invalid_argument);
  EXPECT_NO_THROW(TileResidency(budget(1, 1)));
}

TEST(TileResidency, CountsBytesAndOrdersByUse) {
  TileResidency residency(budget(100, 4));
  residency.commit(1, 10);
  residency.commit(2, 20);
  residency.add_pending(3, 30);
  EXPECT_EQ(residency.bytes(), 60u);
  EXPECT_EQ(residency.size(), 3u);
  EXPECT_EQ(residency.num_pending(), 1u);
  EXPECT_TRUE(residency.is_resident(1));
  EXPECT_TRUE(residency.is_pending(3));
  EXPECT_FALSE(residency.is_resident(3));
  EXPECT_EQ(residency.lru_order(), (std::vector<int>{1, 2}));

  residency.touch(1);
  EXPECT_EQ(residency.lru_order(), (std::vector<int>{2, 1}));
  residency.touch(3);  // Pending: no effect
  residency.touch(9);  // Unknown: no effect
  EXPECT_EQ(residency.lru_order(), (std::vector<int>{2, 1}));

  // The real size replaces the estimate, and the tile becomes the most recently used.
  residency.commit(3, 25);
  EXPECT_EQ(residency.bytes(), 55u);
  EXPECT_EQ(residency.num_pending(), 0u);
  EXPECT_EQ(residency.lru_order(), (std::vector<int>{2, 1, 3}));

  residency.erase(2);
  residency.erase(9);
  EXPECT_EQ(residency.bytes(), 35u);
  EXPECT_EQ(residency.lru_order(), (std::vector<int>{1, 3}));
  residency.clear();
  EXPECT_EQ(residency.bytes(), 0u);
  EXPECT_EQ(residency.size(), 0u);
}

TEST(TileResidency, PlansEvictionLeastRecentlyUsedFirst) {
  TileResidency residency(budget(100, 10));
  residency.commit(1, 30);
  residency.commit(2, 30);
  residency.commit(3, 30);
  residency.touch(1);  // LRU order: 2, 3, 1

  auto victims = residency.plan_eviction(10, {});
  ASSERT_TRUE(victims);
  EXPECT_TRUE(victims->empty());

  victims = residency.plan_eviction(40, {});
  ASSERT_TRUE(victims);
  EXPECT_EQ(*victims, (std::vector<int>{2}));

  victims = residency.plan_eviction(70, {});
  ASSERT_TRUE(victims);
  EXPECT_EQ(*victims, (std::vector<int>{2, 3}));

  // Kept tiles are skipped.
  victims = residency.plan_eviction(40, {2});
  ASSERT_TRUE(victims);
  EXPECT_EQ(*victims, (std::vector<int>{3}));

  // Too large for the budget, or only kept tiles left to evict
  EXPECT_FALSE(residency.plan_eviction(101, {}));
  EXPECT_FALSE(residency.plan_eviction(70, {1, 2}));
  // Planning does not change anything.
  EXPECT_EQ(residency.bytes(), 90u);
}

TEST(TileResidency, PlansEvictionForTheTileCount) {
  TileResidency residency(budget(1000, 2));
  residency.commit(1, 1);
  residency.commit(2, 1);
  auto victims = residency.plan_eviction(1, {});
  ASSERT_TRUE(victims);
  EXPECT_EQ(*victims, (std::vector<int>{1}));
  EXPECT_FALSE(residency.plan_eviction(1, {1, 2}));
}

TEST(TileResidency, NeverEvictsPendingTiles) {
  TileResidency residency(budget(100, 10));
  residency.add_pending(1, 60);
  residency.commit(2, 30);
  auto victims = residency.plan_eviction(30, {});
  ASSERT_TRUE(victims);
  EXPECT_EQ(*victims, (std::vector<int>{2}));
  EXPECT_FALSE(residency.plan_eviction(50, {}));
}

TEST(TileResidency, TrimsBackWithinTheBudget) {
  TileResidency residency(budget(50, 10));
  residency.commit(1, 20);
  residency.commit(2, 20);
  residency.commit(3, 20);
  EXPECT_TRUE(residency.over_budget());
  EXPECT_EQ(residency.plan_trim({}), (std::vector<int>{1}));
  EXPECT_EQ(residency.plan_trim({1}), (std::vector<int>{2}));
  // Only kept tiles: stays over budget
  EXPECT_TRUE(residency.plan_trim({1, 2, 3}).empty());

  residency.erase(1);
  EXPECT_FALSE(residency.over_budget());
  EXPECT_TRUE(residency.plan_trim({}).empty());
}

TEST(TileResidency, EstimatesTheDeviceSizeOfATile) {
  auto points = std::make_shared<gtsam_points::PointCloudCPU>();
  std::vector<Eigen::Vector4d> xyz;
  std::vector<Eigen::Matrix4d> covs;
  for (int i = 0; i < 1000; i++) {
    xyz.emplace_back(0.1 * (i % 10), 0.1 * ((i / 10) % 10), 0.1 * (i / 100), 1.0);
    covs.push_back(Eigen::Matrix4d::Identity() * 0.01);
  }
  points->add_points(xyz);
  points->add_covs(covs);

  MapTile tile;
  tile.points = points;
  // Points only: 12 bytes per point and 36 per covariance, as PointCloudGPU stores them
  EXPECT_EQ(estimate_tile_device_bytes(tile), 1000u * (12 + 36));

  auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(0.5);
  voxelmap->insert(*points);
  tile.voxelmaps.push_back(voxelmap);
  const size_t voxels = voxelmap->num_voxels();
  ASSERT_GT(voxels, 0u);
  // 52 bytes per voxel, and 16 bytes per bucket of the initial 16384-bucket table
  EXPECT_EQ(estimate_tile_device_bytes(tile), 1000u * (12 + 36) + voxels * 52 + 16384u * 16);

  // Without a CPU voxel map, one voxel per point is assumed.
  MapTile other;
  other.points = points;
  other.voxelmaps.push_back(nullptr);
  EXPECT_EQ(estimate_tile_device_bytes(other), 1000u * (12 + 36) + 1000u * 52 + 16384u * 16);
}
