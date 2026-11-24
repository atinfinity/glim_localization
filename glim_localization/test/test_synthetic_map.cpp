#include <filesystem>
#include <string>

#include <boost/format.hpp>
#include <gtest/gtest.h>
#include <unistd.h>

#include <glim/mapping/sub_map.hpp>

#include "synthetic/synthetic_map.hpp"

namespace fs = std::filesystem;
using namespace glim_localization::test;

class SyntheticMapTest : public testing::Test {
protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("glim_loc_synthetic_" + std::to_string(::getpid()));
    fs::remove_all(dir_);
  }
  void TearDown() override { fs::remove_all(dir_); }

  fs::path dir_;
};

TEST_F(SyntheticMapTest, UpstreamLoaderReadsDump) {
  SyntheticMapParams params;
  params.num_submaps = 3;
  params.point_spacing = 0.4;
  const auto map = write_synthetic_dump(dir_.string(), params);

  ASSERT_EQ(map.submaps.size(), 3u);
  EXPECT_TRUE(fs::exists(dir_ / "graph.txt"));
  EXPECT_TRUE(fs::exists(dir_ / "values.bin"));

  for (const auto& truth : map.submaps) {
    const auto path = (boost::format("%s/%06d") % dir_.string() % truth.id).str();
    const auto submap = glim::SubMap::load(path);
    ASSERT_NE(submap, nullptr);
    EXPECT_EQ(submap->id, truth.id);
    ASSERT_GT(truth.num_points, 0u);
    EXPECT_EQ(submap->frame->size(), truth.num_points);
    EXPECT_TRUE(submap->frame->has_covs());
    EXPECT_EQ(submap->frames.size(), 1u);
    EXPECT_TRUE(submap->T_world_origin.matrix().isApprox(truth.T_world_origin.matrix(), 1e-4));
  }
}

TEST_F(SyntheticMapTest, SameSeedGivesSamePointCounts) {
  SyntheticMapParams params;
  params.num_submaps = 2;
  params.point_spacing = 0.5;
  const auto first = write_synthetic_dump((dir_ / "a").string(), params);
  const auto second = write_synthetic_dump((dir_ / "b").string(), params);
  ASSERT_EQ(first.submaps.size(), second.submaps.size());
  for (size_t i = 0; i < first.submaps.size(); i++) {
    EXPECT_EQ(first.submaps[i].num_points, second.submaps[i].num_points);
  }
}
