#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <gtsam_points/types/gaussian_voxelmap.hpp>
#include <gtsam_points/types/point_cloud.hpp>

#include <glim_localization/map/map_tile.hpp>
#include <glim_localization/map/tile_residency.hpp>
#include <glim_localization/map/tile_streamer.hpp>

struct CUstream_st;

namespace gtsam_points {
class CUDAStream;
}  // namespace gtsam_points

namespace glim_localization {

/// A map tile uploaded to the GPU: points with covariances and one GaussianVoxelMapGPU per level of the CPU tile.
struct GpuTile {
  using ConstPtr = std::shared_ptr<const GpuTile>;

  MapTile::ConstPtr tile;                                           ///< The CPU tile it was made from
  gtsam_points::PointCloud::ConstPtr points;                        ///< PointCloudGPU, origin frame
  std::vector<gtsam_points::GaussianVoxelMap::ConstPtr> voxelmaps;  ///< GaussianVoxelMapGPU, fine to coarse, same resolutions as tile->voxelmaps
  size_t memory_usage_gpu = 0;                                      ///< Bytes on the GPU (points and voxelmaps)
};

struct GpuTileCacheParams {
  TileBudget budget;     ///< Bytes and tiles held on the GPU
  bool prefetch = true;  ///< Upload prefetched tiles on a background thread with its own CUDA stream
};

/// Map tiles resident on the GPU (CUDA builds only), within a byte and tile budget: a TileStreamer with CUDA uploads.
/// Owned by one GPU odometry instance; not thread-safe.
///
/// acquire() returns the tiles one frame needs, uploading the missing ones on the caller's stream. prefetch() uploads
/// tiles likely needed soon on a background thread and a separate non-blocking stream, so the tracking path does not
/// wait for them. Least recently used tiles outside the frame's tiles and the prefetch list are evicted to stay within
/// the budget. An evicted tile stays alive while factors in the smoother still reference it, so GPU memory is bounded
/// by the budget plus the tiles of the frames in the smoother window.
///
/// Every upload synchronizes its stream and checks for CUDA errors. A failed acquire() upload fails closed with
/// std::runtime_error; a failed prefetch is logged and dropped, and the tile is uploaded when a frame needs it. A CUDA
/// error left over from earlier work on the same thread is reported as such and cleared first, so it is not blamed on
/// the upload. The background thread uses the device that was current when the cache was constructed.
class GpuTileCache {
public:
  /// @throw std::invalid_argument for an invalid budget
  /// @throw std::runtime_error if the current device cannot be read or the prefetch stream cannot be created
  GpuTileCache(const GpuTileCacheParams& params, CUstream_st* stream);
  /// At most max_tiles tiles, the default byte budget, no prefetch
  GpuTileCache(int max_tiles, CUstream_st* stream);
  ~GpuTileCache();

  GpuTileCache(const GpuTileCache&) = delete;
  GpuTileCache& operator=(const GpuTileCache&) = delete;

  /// The GPU copies of `tiles`, in order, uploading those that are not resident. They are kept until the next call.
  /// @throw std::invalid_argument if a tile has no points or covariances
  /// @throw std::runtime_error if an upload of one of `tiles` fails (a failed prefetch of one is retried here first)
  std::vector<GpuTile::ConstPtr> acquire(const std::vector<MapTile::ConstPtr>& tiles);

  /// acquire() for one tile
  GpuTile::ConstPtr get(const MapTile::ConstPtr& tile);

  /// Queue background uploads of `candidates`, most urgent first, as far as they fit. No effect without prefetch.
  /// @return The number of uploads queued
  size_t prefetch(const std::vector<MapTile::ConstPtr>& candidates);

  /// Wait for the queued prefetches (tests and benchmarks)
  void wait_idle();

  /// Drop all resident tiles (and cancel queued prefetches)
  void clear();

  const GpuTileCacheParams& params() const { return params_; }
  int max_tiles() const { return params_.budget.max_tiles; }
  size_t max_bytes() const { return params_.budget.max_bytes; }
  /// Resident and in-flight tiles
  size_t size() const { return streamer_->residency().size(); }
  bool contains(int tile_id) const { return streamer_->contains(tile_id); }
  const TileStreamerStats& stats() const { return streamer_->stats(); }
  size_t num_uploads() const { return stats().uploads; }
  size_t num_evictions() const { return stats().evictions; }
  /// Bytes held by the resident tiles (in-flight ones with their estimate)
  size_t memory_usage_gpu() const { return streamer_->residency().bytes(); }

private:
  GpuTileCacheParams params_;
  CUstream_st* stream_;
  int device_ = 0;  ///< CUDA device current at construction, also used by the background thread
  std::unique_ptr<gtsam_points::CUDAStream> prefetch_stream_;
  std::unique_ptr<TileStreamer<GpuTile>> streamer_;
};

/// Upload one tile on `stream`, synchronize it, and check for CUDA errors.
/// @throw std::invalid_argument if the tile has no points or covariances
/// @throw std::runtime_error on a CUDA error
GpuTile::ConstPtr upload_gpu_tile(const MapTile::ConstPtr& tile, CUstream_st* stream);

/// Clear the calling thread's last CUDA error and log it as left over from earlier work, so the next check does not
/// blame `before` for it. @return true if there was one
bool clear_earlier_cuda_error(const char* before);

}  // namespace glim_localization
