#pragma once

#include <cstddef>
#include <list>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <glim_localization/map/map_tile.hpp>

namespace glim_localization {

/// Limits for the map tiles kept on a device. Both apply.
struct TileBudget {
  size_t max_bytes = size_t(1) << 30;  ///< Bytes held by resident and in-flight tiles
  int max_tiles = 32;                  ///< Resident and in-flight tiles
};

/// Bookkeeping of the tiles held on a device: their bytes, their least recently used order, and the tiles still being
/// uploaded (pending). It holds no device data and no threads, so the eviction policy is testable without a GPU.
///
/// Pending tiles count against the budget with an estimate until commit() gives their real size. They are never chosen
/// for eviction. Not thread-safe.
class TileResidency {
public:
  /// @throw std::invalid_argument if budget.max_bytes is 0 or budget.max_tiles < 1
  explicit TileResidency(const TileBudget& budget);

  const TileBudget& budget() const { return budget_; }

  /// Count a tile whose upload has started, with its estimated size. Replaces an entry with the same id.
  void add_pending(int id, size_t estimated_bytes);
  /// A tile is resident with `bytes` on the device (a pending entry is replaced). It becomes the most recently used.
  void commit(int id, size_t bytes);
  /// Make a resident tile the most recently used. No effect for pending or unknown ids.
  void touch(int id);
  /// Forget a tile (resident or pending). No effect for unknown ids.
  void erase(int id);
  void clear();

  bool contains(int id) const { return entries_.count(id) > 0; }
  bool is_pending(int id) const;
  bool is_resident(int id) const;
  /// Bytes of all entries (pending ones with their estimate)
  size_t bytes() const { return bytes_; }
  /// Number of entries, pending ones included
  size_t size() const { return entries_.size(); }
  size_t num_pending() const { return entries_.size() - lru_.size(); }
  bool over_budget() const;

  /// Resident tiles, least recently used first
  std::vector<int> lru_order() const;

  /// The resident tiles to evict, least recently used first, so that one more tile of `incoming_bytes` fits in the
  /// budget. Tiles in `keep` and pending tiles are never chosen.
  /// @return The victims (possibly none), or nullopt if the tile cannot fit without evicting a kept or pending tile
  std::optional<std::vector<int>> plan_eviction(size_t incoming_bytes, const std::unordered_set<int>& keep) const;

  /// The resident tiles to evict, least recently used first, to bring the entries back within the budget, skipping
  /// `keep` and pending tiles. The result can leave the entries over budget if only those remain.
  std::vector<int> plan_trim(const std::unordered_set<int>& keep) const;

private:
  struct Entry {
    size_t bytes = 0;
    bool pending = false;
    std::list<int>::iterator lru;  ///< Valid only for resident entries
  };

  /// Victims among resident, non-kept tiles until `fits(bytes, count)` holds; nullopt if it never does.
  template <typename Fits>
  std::optional<std::vector<int>> plan(const std::unordered_set<int>& keep, Fits fits) const;

  TileBudget budget_;
  std::list<int> lru_;  ///< Resident tile ids, most recently used first
  std::unordered_map<int, Entry> entries_;
  size_t bytes_ = 0;
};

/// Estimated device bytes of a tile uploaded by GpuTileCache: its points with every attribute PointCloudGPU copies, and
/// one GaussianVoxelMapGPU per CPU voxel map (voxel statistics plus the hash table, sized as gtsam_points 1.2.2 sizes
/// it). Used to plan an upload before its real size is known. CPU only; needs no GPU.
size_t estimate_tile_device_bytes(const MapTile& tile);

}  // namespace glim_localization
