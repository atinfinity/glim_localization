#include <glim_localization/map/tile_residency.hpp>

#include <stdexcept>

#include <Eigen/Core>

#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>

namespace glim_localization {

TileResidency::TileResidency(const TileBudget& budget) : budget_(budget) {
  if (budget.max_bytes == 0) {
    throw std::invalid_argument("the tile budget needs at least one byte");
  }
  if (budget.max_tiles < 1) {
    throw std::invalid_argument("the tile budget needs room for at least one tile");
  }
}

void TileResidency::add_pending(const int id, const size_t estimated_bytes) {
  erase(id);
  Entry entry;
  entry.bytes = estimated_bytes;
  entry.pending = true;
  entries_.emplace(id, entry);
  bytes_ += estimated_bytes;
}

void TileResidency::commit(const int id, const size_t bytes) {
  erase(id);
  lru_.push_front(id);
  Entry entry;
  entry.bytes = bytes;
  entry.pending = false;
  entry.lru = lru_.begin();
  entries_.emplace(id, entry);
  bytes_ += bytes;
}

void TileResidency::touch(const int id) {
  const auto found = entries_.find(id);
  if (found == entries_.end() || found->second.pending) {
    return;
  }
  lru_.splice(lru_.begin(), lru_, found->second.lru);
}

void TileResidency::erase(const int id) {
  const auto found = entries_.find(id);
  if (found == entries_.end()) {
    return;
  }
  if (!found->second.pending) {
    lru_.erase(found->second.lru);
  }
  bytes_ -= found->second.bytes;
  entries_.erase(found);
}

void TileResidency::clear() {
  entries_.clear();
  lru_.clear();
  bytes_ = 0;
}

bool TileResidency::is_pending(const int id) const {
  const auto found = entries_.find(id);
  return found != entries_.end() && found->second.pending;
}

bool TileResidency::is_resident(const int id) const {
  const auto found = entries_.find(id);
  return found != entries_.end() && !found->second.pending;
}

bool TileResidency::over_budget() const {
  return bytes_ > budget_.max_bytes || entries_.size() > static_cast<size_t>(budget_.max_tiles);
}

std::vector<int> TileResidency::lru_order() const {
  return std::vector<int>(lru_.rbegin(), lru_.rend());
}

template <typename Fits>
std::optional<std::vector<int>> TileResidency::plan(const std::unordered_set<int>& keep, Fits fits) const {
  std::vector<int> victims;
  size_t bytes = bytes_;
  size_t count = entries_.size();
  for (auto it = lru_.rbegin(); !fits(bytes, count); ++it) {
    if (it == lru_.rend()) {
      return std::nullopt;
    }
    if (keep.count(*it)) {
      continue;
    }
    victims.push_back(*it);
    bytes -= entries_.at(*it).bytes;
    count--;
  }
  return victims;
}

std::optional<std::vector<int>> TileResidency::plan_eviction(const size_t incoming_bytes, const std::unordered_set<int>& keep) const {
  if (incoming_bytes > budget_.max_bytes) {
    return std::nullopt;
  }
  const size_t max_bytes = budget_.max_bytes - incoming_bytes;
  const size_t max_count = static_cast<size_t>(budget_.max_tiles) - 1;
  return plan(keep, [&](size_t bytes, size_t count) { return bytes <= max_bytes && count <= max_count; });
}

std::vector<int> TileResidency::plan_trim(const std::unordered_set<int>& keep) const {
  std::vector<int> victims;
  size_t bytes = bytes_;
  size_t count = entries_.size();
  for (auto it = lru_.rbegin(); it != lru_.rend() && (bytes > budget_.max_bytes || count > static_cast<size_t>(budget_.max_tiles)); ++it) {
    if (keep.count(*it)) {
      continue;
    }
    victims.push_back(*it);
    bytes -= entries_.at(*it).bytes;
    count--;
  }
  return victims;
}

size_t estimate_tile_device_bytes(const MapTile& tile) {
  size_t bytes = 0;
  if (const auto& points = tile.points) {
    const size_t n = points->size();
    // The attributes PointCloudGPU::clone copies, at the sizes PointCloudGPU::memory_usage_gpu reports
    if (points->points) bytes += n * sizeof(Eigen::Vector3f);
    if (points->covs) bytes += n * sizeof(Eigen::Matrix3f);
    if (points->normals) bytes += n * sizeof(Eigen::Vector3f);
    if (points->intensities) bytes += n * sizeof(float);
    if (points->times) bytes += n * sizeof(float);
  }

  // GaussianVoxelMapGPU::memory_usage_gpu: per voxel a count, a mean and a covariance; per bucket a coordinate and an
  // index. The table starts at the 16384 buckets GpuTileCache asks for and doubles until the voxels fit; assume it
  // ends at most half full.
  constexpr size_t kVoxelBytes = sizeof(int) + sizeof(Eigen::Vector3f) + sizeof(Eigen::Matrix3f);
  constexpr size_t kBucketBytes = sizeof(Eigen::Vector3i) + sizeof(int);
  constexpr size_t kInitialBuckets = 8192 * 2;
  for (const auto& voxelmap : tile.voxelmaps) {
    const auto cpu = std::dynamic_pointer_cast<const gtsam_points::GaussianVoxelMapCPU>(voxelmap);
    // Without a CPU voxel map, at most one voxel per point
    const size_t voxels = cpu ? cpu->num_voxels() : (tile.points ? tile.points->size() : 0);
    size_t buckets = kInitialBuckets;
    while (buckets < 2 * voxels) {
      buckets *= 2;
    }
    bytes += voxels * kVoxelBytes + buckets * kBucketBytes;
  }
  return bytes;
}

}  // namespace glim_localization
