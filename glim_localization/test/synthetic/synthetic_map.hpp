#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim_localization::test {

struct SyntheticMapParams {
  int num_submaps = 8;
  double submap_spacing = 10.0;   // Distance between submap origins along x [m]
  double point_spacing = 0.2;     // Surface sampling interval [m]
  double submap_radius = 15.0;    // Points within this distance belong to a submap [m]
  double noise_sigma = 0.01;      // Gaussian noise on each point [m]
  unsigned int seed = 42;
  Eigen::Vector3d world_offset = Eigen::Vector3d::Zero();  // Added to every origin
  // Add walls across the corridor. Without them, translation along x is only weakly observable.
  bool cross_walls = false;
};

struct SyntheticSubmapTruth {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int id = -1;
  Eigen::Isometry3d T_world_origin = Eigen::Isometry3d::Identity();
  size_t num_points = 0;
};

struct SyntheticMap {
  std::vector<SyntheticSubmapTruth, Eigen::aligned_allocator<SyntheticSubmapTruth>> submaps;
};

/// Write a corridor scene as a GLIM dump using upstream SubMap::save.
/// The dump holds data.txt and compact clouds per submap, graph.txt, and values.bin.
SyntheticMap write_synthetic_dump(const std::string& path, const SyntheticMapParams& params = SyntheticMapParams());

using SyntheticPoints = std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>;

/// The same noisy corridor points as write_synthetic_dump, once each, in the world frame (w = 1).
SyntheticPoints make_synthetic_world_points(const SyntheticMapParams& params = SyntheticMapParams());

enum class PcdEncoding { Ascii, Binary, BinaryCompressed };

/// LZF-encode `data` with literal runs only (a valid stream that any LZF decoder accepts; no compression).
std::string lzf_encode_literals(const std::string& data);

/// Write points as a PCD v0.7 file with fields x y z (F4, or F8 if `double_xyz`) and a float intensity.
/// BinaryCompressed writes PCL's layout (field-major, LZF) with lzf_encode_literals.
void write_synthetic_pcd(const std::string& path, const SyntheticPoints& points, PcdEncoding encoding, bool double_xyz = false);

enum class PlyEncoding { Ascii, BinaryLittleEndian };

struct PlyOptions {
  bool double_xyz = false;  ///< x, y, z as double instead of float
  /// Add a `list uchar int` property to the vertex element (variable-size binary records), an element with a list
  /// before the vertex element, and a face element after it.
  bool lists = false;
};

/// Write points as a PLY 1.0 file. Vertex properties: float intensity, x y z, float nx ny nz, uchar red green blue.
void write_synthetic_ply(const std::string& path, const SyntheticPoints& points, PlyEncoding encoding, const PlyOptions& options = PlyOptions());

}  // namespace glim_localization::test
