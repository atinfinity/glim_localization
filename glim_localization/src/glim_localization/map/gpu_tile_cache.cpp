#include <glim_localization/map/gpu_tile_cache.hpp>

#include <stdexcept>
#include <string>

#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>

namespace glim_localization {

GpuTileCache::GpuTileCache(const int max_tiles, CUstream_st* stream) : max_tiles_(max_tiles), stream_(stream) {
  if (max_tiles < 1) {
    throw std::invalid_argument("the GPU tile cache needs room for at least one tile");
  }
}

GpuTileCache::~GpuTileCache() {}

GpuTile::ConstPtr GpuTileCache::get(const MapTile::ConstPtr& tile) {
  const auto found = entries_.find(tile->id);
  if (found != entries_.end() && found->second.gpu->tile == tile) {
    lru_.splice(lru_.begin(), lru_, found->second.lru);
    return found->second.gpu;
  }
  if (found != entries_.end()) {
    // Same id from another map: replace it
    lru_.erase(found->second.lru);
    entries_.erase(found);
  }

  if (!tile->points || !tile->points->points || !tile->points->covs) {
    throw std::invalid_argument("map tile " + std::to_string(tile->id) + " has no points or covariances to upload");
  }

  auto gpu = std::make_shared<GpuTile>();
  gpu->tile = tile;
  auto points = gtsam_points::PointCloudGPU::clone(*tile->points, stream_);
  gpu->memory_usage_gpu += points->memory_usage_gpu();
  for (const auto& cpu_voxelmap : tile->voxelmaps) {
    auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(cpu_voxelmap->voxel_resolution(), 8192 * 2, 10, 1e-3, stream_);
    voxelmap->insert(*points);
    gpu->memory_usage_gpu += voxelmap->memory_usage_gpu();
    gpu->voxelmaps.push_back(voxelmap);
  }
  gpu->points = points;
  num_uploads_++;

  lru_.push_front(tile->id);
  entries_[tile->id] = Entry{gpu, lru_.begin()};

  while (static_cast<int>(entries_.size()) > max_tiles_) {
    entries_.erase(lru_.back());
    lru_.pop_back();
    num_evictions_++;
  }
  return gpu;
}

void GpuTileCache::clear() {
  entries_.clear();
  lru_.clear();
}

size_t GpuTileCache::memory_usage_gpu() const {
  size_t bytes = 0;
  for (const auto& [id, entry] : entries_) {
    bytes += entry.gpu->memory_usage_gpu;
  }
  return bytes;
}

}  // namespace glim_localization
