#include <glim_localization/map/gpu_tile_cache.hpp>

#include <exception>
#include <stdexcept>
#include <string>

#include <cuda_runtime.h>
#include <spdlog/spdlog.h>

#include <gtsam_points/cuda/cuda_stream.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>

namespace glim_localization {

bool clear_earlier_cuda_error(const char* before) {
  // cudaGetLastError returns and resets the calling thread's last error. A sticky error (a corrupted context) stays
  // and fails the next call again, where it is reported.
  const cudaError_t earlier = cudaGetLastError();
  if (earlier == cudaSuccess) {
    return false;
  }
  spdlog::warn("a CUDA error from earlier work was pending before {} (not caused by it): {}", before, cudaGetErrorString(earlier));
  return true;
}

GpuTile::ConstPtr upload_gpu_tile(const MapTile::ConstPtr& tile, CUstream_st* stream) {
  if (!tile->points || !tile->points->points || !tile->points->covs) {
    throw std::invalid_argument("map tile " + std::to_string(tile->id) + " has no points or covariances to upload");
  }

  const std::string what = "the upload of map tile " + std::to_string(tile->id);
  clear_earlier_cuda_error(what.c_str());

  auto gpu = std::make_shared<GpuTile>();
  gpu->tile = tile;
  auto points = gtsam_points::PointCloudGPU::clone(*tile->points, stream);
  gpu->memory_usage_gpu += points->memory_usage_gpu();
  for (const auto& cpu_voxelmap : tile->voxelmaps) {
    auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(cpu_voxelmap->voxel_resolution(), 8192 * 2, 10, 1e-3, stream);
    voxelmap->insert(*points);
    gpu->memory_usage_gpu += voxelmap->memory_usage_gpu();
    gpu->voxelmaps.push_back(voxelmap);
  }
  gpu->points = points;

  // gtsam_points only prints a warning when an allocation or a copy fails (for example out of device memory), so
  // check here and fail closed instead of building factors on invalid device memory. The earlier error was cleared
  // above, so an error now comes from this upload (or from a sticky error of the device).
  cudaError_t error = cudaStreamSynchronize(stream);
  const cudaError_t last = cudaGetLastError();
  if (error == cudaSuccess) {
    error = last;
  }
  if (error != cudaSuccess) {
    throw std::runtime_error("failed to upload map tile " + std::to_string(tile->id) + " to the GPU: " + cudaGetErrorString(error));
  }
  return gpu;
}

GpuTileCache::GpuTileCache(const GpuTileCacheParams& params, CUstream_st* stream) : params_(params), stream_(stream) {
  // The current device of the constructing (odometry) thread. A new thread starts on device 0, so the background
  // thread selects this one before each upload.
  clear_earlier_cuda_error("setting up the GPU tile cache");
  const cudaError_t device_error = cudaGetDevice(&device_);
  if (device_error != cudaSuccess) {
    cudaGetLastError();
    throw std::runtime_error(std::string("failed to get the current CUDA device: ") + cudaGetErrorString(device_error));
  }

  if (params_.prefetch) {
    // Non-blocking, so work on the legacy default stream does not wait for a background upload.
    clear_earlier_cuda_error("creating the GPU tile prefetch stream");
    prefetch_stream_ = std::make_unique<gtsam_points::CUDAStream>();
    const cudaError_t error = cudaGetLastError();
    if (error != cudaSuccess) {
      throw std::runtime_error(std::string("failed to create the GPU tile prefetch stream: ") + cudaGetErrorString(error));
    }
  }

  CUstream_st* const prefetch_stream = prefetch_stream_ ? static_cast<CUstream_st*>(*prefetch_stream_) : nullptr;
  const int device = device_;
  auto upload = [stream, prefetch_stream, device](const MapTile::ConstPtr& tile, TileStreamer<GpuTile>::Context context) {
    const bool background = context == TileStreamer<GpuTile>::Context::Background;
    if (background) {
      const cudaError_t error = cudaSetDevice(device);
      if (error != cudaSuccess) {
        cudaGetLastError();
        throw std::runtime_error("failed to select CUDA device " + std::to_string(device) + " for a tile prefetch: " + cudaGetErrorString(error));
      }
    }
    const auto gpu = upload_gpu_tile(tile, background ? prefetch_stream : stream);
    return TileStreamer<GpuTile>::Upload{gpu, gpu->memory_usage_gpu};
  };
  auto on_prefetch_failure = [](int id, std::exception_ptr error) {
    std::string what = "unknown error";
    try {
      std::rethrow_exception(error);
    } catch (const std::exception& e) {
      what = e.what();
    } catch (...) {
    }
    spdlog::warn("prefetch of map tile {} failed and was dropped (it is uploaded when a frame needs it): {}", id, what);
  };
  streamer_ = std::make_unique<TileStreamer<GpuTile>>(
    params_.budget,
    upload,
    [](const MapTile& tile) { return estimate_tile_device_bytes(tile); },
    on_prefetch_failure);
}

GpuTileCache::GpuTileCache(const int max_tiles, CUstream_st* stream)
: GpuTileCache(
    [max_tiles] {
      if (max_tiles < 1) {
        throw std::invalid_argument("the GPU tile cache needs room for at least one tile");
      }
      GpuTileCacheParams params;
      params.budget.max_tiles = max_tiles;
      params.prefetch = false;
      return params;
    }(),
    stream) {}

GpuTileCache::~GpuTileCache() {
  // Stop the background thread before its stream goes away.
  streamer_.reset();
  prefetch_stream_.reset();
}

std::vector<GpuTile::ConstPtr> GpuTileCache::acquire(const std::vector<MapTile::ConstPtr>& tiles) {
  return streamer_->acquire(tiles);
}

GpuTile::ConstPtr GpuTileCache::get(const MapTile::ConstPtr& tile) {
  return streamer_->acquire({tile}).front();
}

size_t GpuTileCache::prefetch(const std::vector<MapTile::ConstPtr>& candidates) {
  if (!prefetch_stream_) {
    return 0;
  }
  return streamer_->prefetch(candidates);
}

void GpuTileCache::wait_idle() {
  streamer_->wait_idle();
}

void GpuTileCache::clear() {
  streamer_->clear();
}

}  // namespace glim_localization
