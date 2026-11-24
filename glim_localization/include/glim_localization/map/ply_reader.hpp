#pragma once

#include <string>

#include <glim_localization/map/pcd_map.hpp>

namespace glim_localization {

/// Read x, y and z of the `vertex` element from a PLY file (`format ascii 1.0` or `binary_little_endian 1.0`).
/// x, y and z must be scalar `float`/`float32` or `double`/`float64` properties. Other vertex properties
/// (intensity, normals, colors, lists) are skipped, and elements before `vertex` are skipped; elements after it
/// (for example faces) are not read. `binary_big_endian` is rejected, and so is a header longer than 64 KiB.
/// In ascii files each vertex is one line. Points with a non-finite coordinate are dropped. The result has w = 1.
/// On failure, `points` is unchanged.
bool read_ply(const std::string& path, PcdPoints& points);

}  // namespace glim_localization
