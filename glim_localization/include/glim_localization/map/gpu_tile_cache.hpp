#pragma once

#include <cstddef>
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

#include <gtsam_points/types/gaussian_voxelmap.hpp>
#include <gtsam_points/types/point_cloud.hpp>

#include <glim_localization/map/map_tile.hpp>

struct CUstream_st;

namespace glim_localization {

/// A map tile uploaded to the GPU: points with covariances and one GaussianVoxelMapGPU per level of the CPU tile.
struct GpuTile {
  using ConstPtr = std::shared_ptr<const GpuTile>;

  MapTile::ConstPtr tile;                                           ///< The CPU tile it was made from
  gtsam_points::PointCloud::ConstPtr points;                        ///< PointCloudGPU, origin frame
  std::vector<gtsam_points::GaussianVoxelMap::ConstPtr> voxelmaps;  ///< GaussianVoxelMapGPU, fine to coarse, same resolutions as tile->voxelmaps
  size_t memory_usage_gpu = 0;                                      ///< Bytes on the GPU (points and voxelmaps)
};

/// Bounded set of map tiles resident on the GPU (CUDA builds only). Owned by one GPU odometry instance; not thread-safe.
///
/// get() uploads a tile on first use and evicts the least recently used tile once more than max_tiles are resident.
/// An evicted tile is only dropped from the cache: factors still in the smoother keep it alive until they are
/// marginalized, so GPU memory is bounded by max_tiles plus the tiles of the frames in the smoother window.
/// The tiles of one frame are never evicted by that frame's own lookups as long as max_tiles >= their number.
class GpuTileCache {
public:
  /// @throw std::invalid_argument if max_tiles < 1
  GpuTileCache(int max_tiles, CUstream_st* stream);
  ~GpuTileCache();

  GpuTileCache(const GpuTileCache&) = delete;
  GpuTileCache& operator=(const GpuTileCache&) = delete;

  /// The GPU copy of the tile, uploaded if it is not resident. Marks it most recently used.
  /// @throw std::invalid_argument if the tile has no points or covariances
  GpuTile::ConstPtr get(const MapTile::ConstPtr& tile);

  /// Drop all resident tiles.
  void clear();

  int max_tiles() const { return max_tiles_; }
  size_t size() const { return entries_.size(); }
  bool contains(int tile_id) const { return entries_.count(tile_id) > 0; }
  size_t num_uploads() const { return num_uploads_; }
  size_t num_evictions() const { return num_evictions_; }
  /// Bytes held by the resident tiles
  size_t memory_usage_gpu() const;

private:
  struct Entry {
    GpuTile::ConstPtr gpu;
    std::list<int>::iterator lru;
  };

  const int max_tiles_;
  CUstream_st* stream_;
  std::list<int> lru_;  ///< Tile ids, most recently used first
  std::unordered_map<int, Entry> entries_;
  size_t num_uploads_ = 0;
  size_t num_evictions_ = 0;
};

}  // namespace glim_localization
