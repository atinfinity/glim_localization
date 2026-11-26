#include <glim_localization/map/pcd_map.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <utility>

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

namespace glim_localization {
namespace {

// Header values are untrusted. These bounds keep every size computation far from overflow.
constexpr long long kMaxFieldCount = 1 << 20;   ///< COUNT of one field
constexpr size_t kMaxPointBytes = 1 << 24;      ///< Bytes of one binary point (all fields)
constexpr size_t kMaxValuesPerPoint = 1 << 24;  ///< Values of one ascii point (all fields)

struct PcdField {
  std::string name;
  size_t size = 0;
  char type = 0;
  size_t count = 1;
  size_t offset = 0;  ///< Byte offset in a binary point
};

struct PcdHeader {
  std::vector<PcdField> fields;
  size_t num_points = 0;
  std::string data;
  int xyz[3] = {-1, -1, -1};  ///< Field index of x, y, z
  size_t point_bytes = 0;
  size_t values_per_point = 0;
};

std::vector<std::string> split(const std::string& line) {
  std::istringstream iss(line);
  std::vector<std::string> tokens;
  std::string token;
  while (iss >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

std::optional<long long> parse_int(const std::string& text) {
  try {
    size_t pos = 0;
    const long long value = std::stoll(text, &pos);
    if (pos != text.size()) {
      return std::nullopt;
    }
    return value;
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

/// Reads up to and including the DATA line, leaving `ifs` at the first data byte.
bool read_header(std::ifstream& ifs, const std::string& path, PcdHeader& header) {
  std::vector<std::string> names, sizes, types, counts;
  std::optional<long long> width, height, points;

  std::string line;
  while (std::getline(ifs, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    const auto tokens = split(line);
    if (tokens.empty() || tokens[0][0] == '#') {
      continue;
    }
    const std::string& key = tokens[0];
    const std::vector<std::string> values(tokens.begin() + 1, tokens.end());
    const auto single_int = [&](std::optional<long long>& out) {
      if (values.size() == 1) {
        out = parse_int(values[0]);
      }
      return out.has_value() && *out >= 0;
    };

    if (key == "FIELDS") {
      names = values;
    } else if (key == "SIZE") {
      sizes = values;
    } else if (key == "TYPE") {
      types = values;
    } else if (key == "COUNT") {
      counts = values;
    } else if (key == "WIDTH") {
      if (!single_int(width)) {
        spdlog::error("invalid WIDTH in {}", path);
        return false;
      }
    } else if (key == "HEIGHT") {
      if (!single_int(height)) {
        spdlog::error("invalid HEIGHT in {}", path);
        return false;
      }
    } else if (key == "POINTS") {
      if (!single_int(points)) {
        spdlog::error("invalid POINTS in {}", path);
        return false;
      }
    } else if (key == "DATA") {
      if (values.size() != 1) {
        spdlog::error("invalid DATA in {}", path);
        return false;
      }
      header.data = values[0];
      break;
    }
    // VERSION and VIEWPOINT are not needed.
  }

  if (header.data.empty()) {
    spdlog::error("no DATA line in {}", path);
    return false;
  }
  if (header.data != "ascii" && header.data != "binary" && header.data != "binary_compressed") {
    spdlog::error("unsupported PCD DATA {} in {} (only ascii, binary and binary_compressed)", header.data, path);
    return false;
  }
  if (names.empty() || sizes.size() != names.size() || types.size() != names.size() || (!counts.empty() && counts.size() != names.size())) {
    spdlog::error("inconsistent FIELDS, SIZE, TYPE or COUNT in {}", path);
    return false;
  }

  std::set<std::string> seen_names;
  for (size_t i = 0; i < names.size(); i++) {
    PcdField field;
    field.name = names[i];
    // PCL names padding fields "_", which may repeat. Any other repeat would make x, y or z ambiguous.
    if (field.name != "_" && !seen_names.insert(field.name).second) {
      spdlog::error("field {} appears more than once in {}", field.name, path);
      return false;
    }
    const auto size = parse_int(sizes[i]);
    const auto count = counts.empty() ? std::optional<long long>(1) : parse_int(counts[i]);
    if (!size || (*size != 1 && *size != 2 && *size != 4 && *size != 8) || !count || *count < 1 || *count > kMaxFieldCount ||
        types[i].size() != 1 || std::string("FIU").find(types[i][0]) == std::string::npos) {
      spdlog::error("invalid field {} in {}", field.name, path);
      return false;
    }
    field.size = static_cast<size_t>(*size);
    field.count = static_cast<size_t>(*count);
    field.type = types[i][0];
    field.offset = header.point_bytes;
    // size <= 8 and count <= 2^20, so neither product nor sum can wrap before the check.
    header.point_bytes += field.size * field.count;
    header.values_per_point += field.count;
    if (header.point_bytes > kMaxPointBytes || header.values_per_point > kMaxValuesPerPoint) {
      spdlog::error("points in {} are too large ({} bytes, {} values)", path, header.point_bytes, header.values_per_point);
      return false;
    }

    const char* axes[3] = {"x", "y", "z"};
    for (int axis = 0; axis < 3; axis++) {
      if (field.name == axes[axis]) {
        if (field.type != 'F' || (field.size != 4 && field.size != 8) || field.count != 1) {
          spdlog::error("field {} in {} must be a single F4 or F8", field.name, path);
          return false;
        }
        header.xyz[axis] = static_cast<int>(i);
      }
    }
    header.fields.push_back(field);
  }
  if (header.xyz[0] < 0 || header.xyz[1] < 0 || header.xyz[2] < 0) {
    spdlog::error("missing x, y or z field in {}", path);
    return false;
  }
  for (int axis = 0; axis < 3; axis++) {
    const PcdField& field = header.fields[header.xyz[axis]];
    if (field.offset > header.point_bytes || field.size > header.point_bytes - field.offset) {
      spdlog::error("field {} in {} is outside the point", field.name, path);
      return false;
    }
  }

  // WIDTH and HEIGHT are non-negative, so a product above the maximum is the only overflow.
  std::optional<long long> width_x_height;
  if (width && height) {
    if (*width != 0 && *height > std::numeric_limits<long long>::max() / *width) {
      spdlog::error("WIDTH x HEIGHT overflows in {}", path);
      return false;
    }
    width_x_height = *width * *height;
  }

  if (points) {
    if (width_x_height && *width_x_height != *points) {
      spdlog::error("POINTS does not match WIDTH x HEIGHT in {}", path);
      return false;
    }
    header.num_points = static_cast<size_t>(*points);
  } else if (width_x_height) {
    header.num_points = static_cast<size_t>(*width_x_height);
  } else {
    spdlog::error("no POINTS in {}", path);
    return false;
  }
  return true;
}

/// Bytes from the current position to the end of the stream, which is left at the current position.
std::optional<size_t> remaining_bytes(std::ifstream& ifs) {
  const std::streampos pos = ifs.tellg();
  if (pos < 0 || !ifs.seekg(0, std::ios::end)) {
    return std::nullopt;
  }
  const std::streampos end = ifs.tellg();
  if (end < pos || !ifs.seekg(pos)) {
    return std::nullopt;
  }
  return static_cast<size_t>(end - pos);
}

// PCD binary data is little-endian, and read_float copies the bytes as they are.
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "binary PCD reading assumes a little-endian host");

double read_float(const char* bytes, size_t size) {
  if (size == 4) {
    float value;
    std::memcpy(&value, bytes, sizeof(value));
    return value;
  }
  double value;
  std::memcpy(&value, bytes, sizeof(value));
  return value;
}

/// gtsam_points::estimate_covariances takes the point count as an int.
bool fits_covariance_estimation(size_t num_points) {
  return num_points <= static_cast<size_t>(std::numeric_limits<int>::max());
}

bool read_binary(std::ifstream& ifs, const std::string& path, const PcdHeader& header, PcdPoints& points) {
  // point_bytes >= 12 (x, y and z). Compare with the file before allocating, so a huge POINTS cannot exhaust memory.
  const auto remaining = remaining_bytes(ifs);
  if (!remaining) {
    spdlog::error("failed to get the size of {}", path);
    return false;
  }
  if (header.num_points > *remaining / header.point_bytes) {
    spdlog::error("truncated binary data in {}: {} points of {} bytes need more than the {} bytes left", path, header.num_points,
                  header.point_bytes, *remaining);
    return false;
  }
  std::vector<char> buffer(header.num_points * header.point_bytes);
  ifs.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
  if (static_cast<size_t>(ifs.gcount()) != buffer.size()) {
    spdlog::error("truncated binary data in {}", path);
    return false;
  }

  points.reserve(header.num_points);
  for (size_t i = 0; i < header.num_points; i++) {
    const char* point = buffer.data() + i * header.point_bytes;
    Eigen::Vector4d p(0.0, 0.0, 0.0, 1.0);
    for (int axis = 0; axis < 3; axis++) {
      const PcdField& field = header.fields[header.xyz[axis]];
      p[axis] = read_float(point + field.offset, field.size);
    }
    points.push_back(p);
  }
  return true;
}

/// PCL's binary_compressed: two little-endian uint32 (compressed size, uncompressed size), then an LZF stream.
/// The uncompressed data is field-major (all values of the first field, then of the next, ...) and, unlike binary
/// data, leaves out the padding fields named "_".
bool read_binary_compressed(std::ifstream& ifs, const std::string& path, const PcdHeader& header, PcdPoints& points) {
  uint32_t sizes[2];
  ifs.read(reinterpret_cast<char*>(sizes), sizeof(sizes));
  if (static_cast<size_t>(ifs.gcount()) != sizeof(sizes)) {
    spdlog::error("truncated binary_compressed header in {}", path);
    return false;
  }
  const size_t compressed_size = sizes[0];
  const size_t uncompressed_size = sizes[1];

  // Byte offset of each field within one packed point (padding left out). Field f starts at field_offset[f] * num_points.
  std::vector<size_t> field_offset(header.fields.size(), 0);
  size_t packed_point_bytes = 0;
  for (size_t i = 0; i < header.fields.size(); i++) {
    if (header.fields[i].name == "_") {
      continue;
    }
    field_offset[i] = packed_point_bytes;
    packed_point_bytes += header.fields[i].size * header.fields[i].count;
  }
  // packed_point_bytes >= 12 (x, y and z). The sizes are uint32, so a larger cloud cannot be stored this way.
  if (header.num_points > std::numeric_limits<uint32_t>::max() / packed_point_bytes) {
    spdlog::error("{} points of {} bytes in {} exceed the 4 GiB limit of binary_compressed", header.num_points, packed_point_bytes, path);
    return false;
  }
  if (uncompressed_size != header.num_points * packed_point_bytes) {
    spdlog::error("binary_compressed data in {} holds {} bytes, but {} points of {} bytes need {}", path, uncompressed_size,
                  header.num_points, packed_point_bytes, header.num_points * packed_point_bytes);
    return false;
  }
  if (uncompressed_size == 0) {
    return true;
  }

  // Check both sizes before allocating: the compressed data must be in the file, and LZF expands at most 88 times
  // (a 3-byte back reference copies at most 264 bytes).
  constexpr size_t kMaxLzfExpansion = 88;
  const auto remaining = remaining_bytes(ifs);
  if (!remaining) {
    spdlog::error("failed to get the size of {}", path);
    return false;
  }
  if (compressed_size > *remaining) {
    spdlog::error("truncated binary_compressed data in {}: {} compressed bytes, {} left", path, compressed_size, *remaining);
    return false;
  }
  if (uncompressed_size > compressed_size * kMaxLzfExpansion) {
    spdlog::error("corrupt binary_compressed data in {}: {} bytes cannot expand to {}", path, compressed_size, uncompressed_size);
    return false;
  }

  std::vector<char> compressed(compressed_size);
  ifs.read(compressed.data(), static_cast<std::streamsize>(compressed.size()));
  if (static_cast<size_t>(ifs.gcount()) != compressed.size()) {
    spdlog::error("truncated binary_compressed data in {}", path);
    return false;
  }
  std::vector<char> buffer(uncompressed_size);
  if (!lzf_decompress(compressed.data(), compressed.size(), buffer.data(), buffer.size())) {
    spdlog::error("corrupt binary_compressed data in {}", path);
    return false;
  }

  points.reserve(header.num_points);
  for (size_t i = 0; i < header.num_points; i++) {
    Eigen::Vector4d p(0.0, 0.0, 0.0, 1.0);
    for (int axis = 0; axis < 3; axis++) {
      const int f = header.xyz[axis];
      const PcdField& field = header.fields[f];
      p[axis] = read_float(buffer.data() + field_offset[f] * header.num_points + i * field.size, field.size);
    }
    points.push_back(p);
  }
  return true;
}

bool read_ascii(std::ifstream& ifs, const std::string& path, const PcdHeader& header, PcdPoints& points) {
  // Value index of x, y, z within a point
  size_t xyz_value[3];
  size_t value_index = 0;
  for (size_t i = 0; i < header.fields.size(); i++) {
    for (int axis = 0; axis < 3; axis++) {
      if (header.xyz[axis] == static_cast<int>(i)) {
        xyz_value[axis] = value_index;
      }
    }
    value_index += header.fields[i].count;
  }
  for (int axis = 0; axis < 3; axis++) {
    if (xyz_value[axis] >= header.values_per_point) {
      spdlog::error("x, y or z value index is outside the point in {}", path);
      return false;
    }
  }

  // Each value takes at least one character and one separator (the last line may lack its newline).
  // Reject a huge POINTS before reserving.
  const auto remaining = remaining_bytes(ifs);
  if (!remaining) {
    spdlog::error("failed to get the size of {}", path);
    return false;
  }
  if (header.num_points > (*remaining + 1) / (2 * header.values_per_point)) {
    spdlog::error("truncated ascii data in {}: {} points of {} values need more than the {} bytes left", path, header.num_points,
                  header.values_per_point, *remaining);
    return false;
  }
  points.reserve(header.num_points);
  std::string line;
  for (size_t i = 0; i < header.num_points;) {
    if (!std::getline(ifs, line)) {
      spdlog::error("truncated ascii data in {}: {} of {} points", path, i, header.num_points);
      return false;
    }
    const auto tokens = split(line);
    if (tokens.empty()) {
      continue;
    }
    if (tokens.size() != header.values_per_point) {
      spdlog::error("point {} in {} has {} values, expected {}", i, path, tokens.size(), header.values_per_point);
      return false;
    }
    Eigen::Vector4d p(0.0, 0.0, 0.0, 1.0);
    for (int axis = 0; axis < 3; axis++) {
      const std::string& text = tokens[xyz_value[axis]];
      char* end = nullptr;
      p[axis] = std::strtod(text.c_str(), &end);
      if (end != text.c_str() + text.size()) {
        spdlog::error("invalid value {} at point {} in {}", text, i, path);
        return false;
      }
    }
    points.push_back(p);
    i++;
  }
  return true;
}

}  // namespace

bool lzf_decompress(const char* in, size_t in_size, char* out, size_t out_size) {
  const auto* ip = reinterpret_cast<const unsigned char*>(in);
  const unsigned char* const in_end = ip + in_size;
  size_t op = 0;
  while (ip < in_end) {
    const unsigned int ctrl = *ip++;
    if (ctrl < 32) {
      // A literal run of ctrl + 1 bytes
      const size_t len = ctrl + 1;
      if (len > static_cast<size_t>(in_end - ip) || len > out_size - op) {
        return false;
      }
      std::memcpy(out + op, ip, len);
      ip += len;
      op += len;
      continue;
    }
    // A back reference: length ctrl >> 5 (one more byte is added if it is 7) plus 2, offset of 13 bits plus 1
    size_t len = ctrl >> 5;
    if (len == 7) {
      if (ip >= in_end) {
        return false;
      }
      len += *ip++;
    }
    if (ip >= in_end) {
      return false;
    }
    const size_t offset = ((static_cast<size_t>(ctrl & 0x1f) << 8) | *ip++) + 1;
    len += 2;
    if (offset > op || len > out_size - op) {
      return false;
    }
    // Byte by byte: the source may overlap the bytes being written (a repeated pattern).
    for (size_t j = 0; j < len; j++, op++) {
      out[op] = out[op - offset];
    }
  }
  return op == out_size;
}

bool read_pcd(const std::string& path, PcdPoints& points) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    spdlog::error("failed to open {}", path);
    return false;
  }

  PcdHeader header;
  if (!read_header(ifs, path, header)) {
    return false;
  }

  PcdPoints all;
  try {
    const bool read = header.data == "binary"              ? read_binary(ifs, path, header, all)
                      : header.data == "binary_compressed" ? read_binary_compressed(ifs, path, header, all)
                                                           : read_ascii(ifs, path, header, all);
    if (!read) {
      return false;
    }
  } catch (const std::exception& e) {
    spdlog::error("failed to read {}: {}", path, e.what());
    return false;
  }

  PcdPoints result;
  result.reserve(all.size());
  for (const auto& p : all) {
    if (p.allFinite()) {
      result.push_back(p);
    }
  }
  if (result.size() != all.size()) {
    spdlog::info("dropped {} points with non-finite coordinates from {}", all.size() - result.size(), path);
  }
  points = std::move(result);
  return true;
}

std::string check_pcd_map_params(const PcdMapParams& params) {
  // The negated comparisons also reject NaN.
  if (!std::isfinite(params.grid_size) || !(params.grid_size >= kMinPcdGridSize)) {
    return fmt::format("grid size {} must be finite and at least {} m", params.grid_size, kMinPcdGridSize);
  }
  if (!std::isfinite(params.voxel_size) || !(params.voxel_size == 0.0 || params.voxel_size >= kMinPcdVoxelSize)) {
    return fmt::format("voxel size {} must be 0 (off) or a finite value of at least {} m", params.voxel_size, kMinPcdVoxelSize);
  }
  if (params.k_neighbors < kMinPcdKNeighbors) {
    return fmt::format("k_neighbors {} must be at least {}", params.k_neighbors, kMinPcdKNeighbors);
  }
  if (params.num_threads < 1) {
    return fmt::format("num_threads {} must be at least 1", params.num_threads);
  }
  return "";
}

bool build_pcd_submaps(const PcdPoints& points, const PcdMapParams& params, LoadedSubmaps& submaps) {
  if (const std::string error = check_pcd_map_params(params); !error.empty()) {
    spdlog::error("invalid PCD map parameters: {}", error);
    return false;
  }
  if (points.empty()) {
    spdlog::error("no points to split");
    return false;
  }

  // Cell indices are int64, so keep coordinates far inside its range.
  constexpr double kMaxCell = 1e15;
  for (const auto& p : points) {
    if (!p.allFinite() || std::abs(p.x() / params.grid_size) > kMaxCell || std::abs(p.y() / params.grid_size) > kMaxCell) {
      spdlog::error("point out of range: {} {} {}", p.x(), p.y(), p.z());
      return false;
    }
  }

  gtsam_points::PointCloudCPU::Ptr cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
  if (params.voxel_size > 0.0) {
    // voxelgrid_sampling packs voxel coordinates into 21 bits per axis and silently merges points outside
    // +-2^20 voxels into made-up ones. Downsample around the bounding-box centre and check the extent.
    Eigen::Vector3d min_pt = points.front().head<3>();
    Eigen::Vector3d max_pt = min_pt;
    for (const auto& p : points) {
      min_pt = min_pt.cwiseMin(p.head<3>());
      max_pt = max_pt.cwiseMax(p.head<3>());
    }
    const Eigen::Vector3d center = (min_pt + max_pt) / 2.0;
    constexpr double kMaxVoxelsFromCenter = (1 << 20) - 2;  // Margin for rounding at the edges
    const double max_half_extent = ((max_pt - min_pt) / 2.0).maxCoeff();
    if (max_half_extent / params.voxel_size > kMaxVoxelsFromCenter) {
      spdlog::error("the cloud spans {} m, too wide for voxel size {} m (at most {} m); use a larger voxel size or split the cloud",
                    2.0 * max_half_extent, params.voxel_size, 2.0 * kMaxVoxelsFromCenter * params.voxel_size);
      return false;
    }

    PcdPoints local(points.size());
    for (size_t i = 0; i < points.size(); i++) {
      local[i] << points[i].head<3>() - center, 1.0;
    }
    cloud = gtsam_points::voxelgrid_sampling(std::make_shared<gtsam_points::PointCloudCPU>(local), params.voxel_size, params.num_threads);
    for (size_t i = 0; i < cloud->size(); i++) {
      cloud->points[i].head<3>() += center;
    }
  }
  if (cloud->size() < static_cast<size_t>(params.k_neighbors)) {
    spdlog::error("{} points are fewer than k_neighbors {}", cloud->size(), params.k_neighbors);
    return false;
  }
  if (!fits_covariance_estimation(cloud->size())) {
    spdlog::error("{} points after downsampling exceed the {} points that covariance estimation takes at once; use a larger voxel size "
                  "or split the cloud",
                  cloud->size(), std::numeric_limits<int>::max());
    return false;
  }

  const auto covs = gtsam_points::estimate_covariances(cloud->points, cloud->size(), params.k_neighbors, params.num_threads);

  std::map<std::pair<int64_t, int64_t>, std::vector<size_t>> cells;
  for (size_t i = 0; i < cloud->size(); i++) {
    const Eigen::Vector4d& p = cloud->points[i];
    const auto ix = static_cast<int64_t>(std::floor(p.x() / params.grid_size));
    const auto iy = static_cast<int64_t>(std::floor(p.y() / params.grid_size));
    cells[{ix, iy}].push_back(i);
  }

  LoadedSubmaps result;
  result.reserve(cells.size());
  for (const auto& [cell, indices] : cells) {
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const size_t i : indices) {
      centroid += cloud->points[i].head<3>();
    }
    centroid /= static_cast<double>(indices.size());

    PcdPoints local(indices.size());
    std::vector<Eigen::Matrix4d> local_covs(indices.size());
    for (size_t j = 0; j < indices.size(); j++) {
      local[j] << cloud->points[indices[j]].head<3>() - centroid, 1.0;
      local_covs[j] = covs[indices[j]];  // The rotation is identity, so covariances stay as they are.
    }

    auto tile = std::make_shared<gtsam_points::PointCloudCPU>(local);
    tile->add_covs(local_covs);

    LoadedSubmap submap;
    submap.id = static_cast<int>(result.size());
    submap.T_world_origin.setIdentity();
    submap.T_world_origin.translation() = centroid;
    submap.points = tile;
    result.push_back(std::move(submap));
  }

  spdlog::info("split {} points into {} tiles of {} m", cloud->size(), result.size(), params.grid_size);
  submaps = std::move(result);
  return true;
}

bool recompute_covariances(LoadedSubmaps& submaps, int k_neighbors, int num_threads) {
  if (k_neighbors < kMinPcdKNeighbors) {
    spdlog::error("k_neighbors {} must be at least {}", k_neighbors, kMinPcdKNeighbors);
    return false;
  }
  if (num_threads < 1) {
    spdlog::error("num_threads {} must be at least 1", num_threads);
    return false;
  }

  std::vector<gtsam_points::PointCloud::Ptr> clouds;
  clouds.reserve(submaps.size());
  size_t total = 0;
  for (const auto& submap : submaps) {
    if (!submap.points || submap.points->size() < static_cast<size_t>(k_neighbors)) {
      spdlog::error("submap {} has {} points, fewer than k_neighbors {}", submap.id, submap.points ? submap.points->size() : 0, k_neighbors);
      return false;
    }
    if (!fits_covariance_estimation(submap.points->size())) {
      spdlog::error("submap {} has {} points, more than the {} that covariance estimation takes at once", submap.id, submap.points->size(),
                    std::numeric_limits<int>::max());
      return false;
    }
    auto cloud = gtsam_points::PointCloudCPU::clone(*submap.points);
    // A rigid transform keeps the neighbours, so estimating in the origin frame gives R^T * C_world * R.
    cloud->add_covs(gtsam_points::estimate_covariances(cloud->points, cloud->size(), k_neighbors, num_threads));
    total += cloud->size();
    clouds.push_back(cloud);
  }

  for (size_t i = 0; i < submaps.size(); i++) {
    submaps[i].points = clouds[i];
  }
  spdlog::info("recomputed the covariances of {} points in {} submaps (k_neighbors {})", total, submaps.size(), k_neighbors);
  return true;
}

}  // namespace glim_localization
