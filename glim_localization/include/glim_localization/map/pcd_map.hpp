#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <glim_localization/map/map_io.hpp>

namespace glim_localization {

using PcdPoints = std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>;

/// Read x, y and z from a PCD file with `DATA ascii`, `binary` or `binary_compressed` (PCL's LZF-compressed,
/// field-major layout, without the `_` padding fields).
/// x, y and z must be single floats (F4 or F8). Other fields are skipped, and VIEWPOINT is ignored.
/// Points with a non-finite coordinate are dropped. The result has w = 1.
/// On failure, `points` is unchanged.
bool read_pcd(const std::string& path, PcdPoints& points);

/// Write the points of every submap in the world frame as one PCD file: `DATA binary`, fields x y z as F8 (double),
/// so UTM-sized coordinates keep their precision. Covariances and other attributes are not written. Points that
/// several submaps hold (the overlap of GLIM submaps) appear once per submap. Parent directories are created.
/// Fails without writing if `path` exists or the submaps hold no points. The file appears atomically.
bool write_pcd(const std::string& path, const LoadedSubmaps& submaps);

/// Decompress an LZF stream (the format of liblzf, which PCL uses for `binary_compressed`) into exactly `out_size` bytes.
/// Returns false if the stream is malformed (a back reference before the start of the output, a run past the end of
/// the input), or if it does not decompress to exactly `out_size` bytes. `out` is written up to the point of failure.
bool lzf_decompress(const char* in, size_t in_size, char* out, size_t out_size);

/// Lower bounds of PcdMapParams. Smaller tiles explode the submap count, smaller voxels flood the voxel grid,
/// and fewer than 5 neighbours give degenerate covariances (k = 1 is a zero matrix).
constexpr double kMinPcdGridSize = 1.0;
constexpr double kMinPcdVoxelSize = 0.01;
constexpr int kMinPcdKNeighbors = 5;

/// How a point cloud becomes submaps.
struct PcdMapParams {
  double grid_size = 20.0;  ///< Tile edge along world x and y [m], >= kMinPcdGridSize. Tiles span all z.
  double voxel_size = 0.0;  ///< Voxel grid downsampling before tiling [m]. 0 disables it, otherwise >= kMinPcdVoxelSize.
  int k_neighbors = 10;     ///< Neighbours for covariance estimation, searched in the whole cloud, >= kMinPcdKNeighbors
  int num_threads = 1;
};

/// Empty if `params` is valid, otherwise a message that names the bad parameter.
std::string check_pcd_map_params(const PcdMapParams& params);

/// Split world-frame points into grid tiles with per-point covariances.
/// Downsampling runs in a frame centred on the cloud's bounding box, so it works with large (for example UTM)
/// coordinates; it fails if the cloud is too wide for the voxel grid (about 2^21 voxels per axis).
/// Covariances are estimated over the whole cloud first, so points near a tile edge use neighbours across it.
/// Each non-empty tile becomes a submap with identity rotation, its origin at the centroid of its points,
/// and ids 0, 1, ... in (x, y) cell order. Fails if the cloud has fewer than k_neighbors points,
/// or more than INT_MAX after downsampling (`gtsam_points::estimate_covariances` takes an int count).
/// On failure, `submaps` is unchanged.
bool build_pcd_submaps(const PcdPoints& points, const PcdMapParams& params, LoadedSubmaps& submaps);

/// Replace the covariances of every submap with ones estimated from its points the way build_pcd_submaps does
/// (`gtsam_points::estimate_covariances` with k nearest neighbours). Neighbours are searched within each submap,
/// in its origin frame; the result equals the world-frame estimate rotated into that frame.
/// The clouds are copied, so clouds shared elsewhere are not changed. Points and the other attributes (normals,
/// intensities, times) are kept as they are; normals are not re-estimated, and nothing in this package reads them.
/// Fails if `k_neighbors` < kMinPcdKNeighbors, `num_threads` < 1, or a submap has fewer than `k_neighbors` points
/// or more than INT_MAX.
/// On failure, `submaps` is unchanged.
bool recompute_covariances(LoadedSubmaps& submaps, int k_neighbors, int num_threads);

}  // namespace glim_localization
