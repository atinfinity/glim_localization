#include <glim_localization/map/occupancy_grid.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <utility>

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

namespace glim_localization {
namespace {

// map_server trinary mode with negate 0 reads a pixel p as occupancy (255 - p) / 255:
// 0 -> 1.0 (occupied), 205 -> 0.196 (unknown), 254 -> 0.004 (free). The thresholds keep each value well clear.
constexpr uint8_t kPixelOccupied = 0;
constexpr uint8_t kPixelUnknown = 205;
constexpr uint8_t kPixelFree = 254;
constexpr double kOccupiedThresh = 0.65;
constexpr double kFreeThresh = 0.1;

/// Cell indices are int64, so keep coordinates far inside its range.
constexpr double kMaxCell = 1e15;

constexpr int8_t kSeenBelow = 1;
constexpr int8_t kSeenInBand = 2;

/// A YAML double-quoted scalar. check_occupancy_grid_prefix rules out control characters, so only \ and " need escaping.
std::string yaml_quoted(const std::string& text) {
  std::string quoted = "\"";
  for (const char c : text) {
    if (c == '\\' || c == '"') {
      quoted.push_back('\\');
    }
    quoted.push_back(c);
  }
  quoted.push_back('"');
  return quoted;
}

}  // namespace

std::string check_occupancy_grid_prefix(const std::string& prefix) {
  const std::string name = std::filesystem::path(prefix).filename().string();
  if (name.empty() || name == "." || name == "..") {
    return fmt::format("occupancy grid prefix \"{}\" has no file name (for example out/map writes out/map.pgm and out/map.yaml)", prefix);
  }
  for (const char c : name) {
    if (static_cast<unsigned char>(c) < 0x20 || c == 0x7f) {
      return fmt::format("occupancy grid prefix \"{}\" has a control character in its file name", prefix);
    }
  }
  return "";
}

std::string check_occupancy_grid_params(const OccupancyGridParams& params) {
  // The negated comparisons also reject NaN.
  if (!std::isfinite(params.resolution) || !(params.resolution >= kMinOccupancyResolution)) {
    return fmt::format("occupancy resolution {} must be finite and at least {} m", params.resolution, kMinOccupancyResolution);
  }
  if (!std::isfinite(params.z_min) || !std::isfinite(params.z_max)) {
    return fmt::format("occupancy z_min {} and z_max {} must both be set and finite", params.z_min, params.z_max);
  }
  if (!(params.z_min < params.z_max)) {
    return fmt::format("occupancy z_min {} must be below z_max {}", params.z_min, params.z_max);
  }
  return "";
}

bool build_occupancy_grid(const LoadedSubmaps& submaps, const OccupancyGridParams& params, OccupancyGrid& grid) {
  if (const std::string error = check_occupancy_grid_params(params); !error.empty()) {
    spdlog::error("invalid occupancy grid parameters: {}", error);
    return false;
  }

  // Two passes over the map-frame points below z_max: the extent, then the cells.
  const auto for_each_point = [&](const auto& visit) {
    for (const auto& submap : submaps) {
      if (!submap.points) {
        continue;
      }
      for (size_t i = 0; i < submap.points->size(); i++) {
        const Eigen::Vector3d p = submap.T_world_origin * submap.points->points[i].head<3>();
        if (p.allFinite() && p.z() < params.z_max && !visit(p)) {
          return false;
        }
      }
    }
    return true;
  };
  const auto cell_of = [&](const Eigen::Vector3d& p) {
    return std::make_pair(static_cast<int64_t>(std::floor(p.x() / params.resolution)), static_cast<int64_t>(std::floor(p.y() / params.resolution)));
  };

  bool any = false;
  int64_t min_x = 0, min_y = 0, max_x = 0, max_y = 0;
  const bool in_range = for_each_point([&](const Eigen::Vector3d& p) {
    if (std::abs(p.x() / params.resolution) > kMaxCell || std::abs(p.y() / params.resolution) > kMaxCell) {
      spdlog::error("point out of range for the occupancy grid: {} {} {}", p.x(), p.y(), p.z());
      return false;
    }
    const auto [x, y] = cell_of(p);
    if (!any) {
      min_x = max_x = x;
      min_y = max_y = y;
      any = true;
    }
    min_x = std::min(min_x, x);
    max_x = std::max(max_x, x);
    min_y = std::min(min_y, y);
    max_y = std::max(max_y, y);
    return true;
  });
  if (!in_range) {
    return false;
  }
  if (!any) {
    spdlog::error("no points below occupancy z_max {} for the occupancy grid", params.z_max);
    return false;
  }

  // Each extent is at most 2e15 + 1, so the product is checked by division.
  const auto width = static_cast<uint64_t>(max_x - min_x + 1);
  const auto height = static_cast<uint64_t>(max_y - min_y + 1);
  if (width > kMaxOccupancyCells || height > kMaxOccupancyCells / width) {
    spdlog::error("the occupancy grid would be {} x {} cells, more than {}; use a larger resolution", width, height, kMaxOccupancyCells);
    return false;
  }

  OccupancyGrid result;
  result.resolution = params.resolution;
  result.origin = Eigen::Vector2d(static_cast<double>(min_x), static_cast<double>(min_y)) * params.resolution;
  result.width = static_cast<int>(width);
  result.height = static_cast<int>(height);

  // One buffer (one byte per cell): first the seen flags, then the cell values in place.
  std::vector<int8_t>& cells = result.data;
  cells.assign(width * height, 0);
  for_each_point([&](const Eigen::Vector3d& p) {
    const auto [x, y] = cell_of(p);
    cells[static_cast<size_t>(y - min_y) * width + static_cast<size_t>(x - min_x)] |= p.z() < params.z_min ? kSeenBelow : kSeenInBand;
    return true;
  });
  size_t num_occupied = 0, num_free = 0;
  for (int8_t& cell : cells) {
    if (cell & kSeenInBand) {
      cell = kOccupancyOccupied;
      num_occupied++;
    } else if (cell & kSeenBelow) {
      cell = kOccupancyFree;
      num_free++;
    } else {
      cell = kOccupancyUnknown;
    }
  }

  spdlog::info("occupancy grid {} x {} cells of {} m: {} occupied, {} free, {} unknown", result.width, result.height, result.resolution,
               num_occupied, num_free, cells.size() - num_occupied - num_free);
  grid = std::move(result);
  return true;
}

std::string occupancy_grid_pgm_path(const std::string& prefix) {
  return prefix + ".pgm";
}

std::string occupancy_grid_yaml_path(const std::string& prefix) {
  return prefix + ".yaml";
}

bool save_occupancy_grid(const std::string& prefix, const OccupancyGrid& grid) {
  namespace fs = std::filesystem;
  const fs::path pgm_path = occupancy_grid_pgm_path(prefix);
  const fs::path yaml_path = occupancy_grid_yaml_path(prefix);
  if (const std::string error = check_occupancy_grid_prefix(prefix); !error.empty()) {
    spdlog::error("{}", error);
    return false;
  }
  if (grid.width <= 0 || grid.height <= 0 || grid.data.size() != static_cast<size_t>(grid.width) * grid.height) {
    spdlog::error("invalid occupancy grid ({} x {}, {} cells)", grid.width, grid.height, grid.data.size());
    return false;
  }

  std::error_code ec;
  for (const auto& path : {pgm_path, yaml_path}) {
    if (fs::exists(path, ec) || ec) {
      spdlog::error("{} already exists", path.string());
      return false;
    }
  }
  if (pgm_path.has_parent_path()) {
    fs::create_directories(pgm_path.parent_path(), ec);
    if (ec) {
      spdlog::error("failed to create {}: {}", pgm_path.parent_path().string(), ec.message());
      return false;
    }
  }

  // Written under temporary names and renamed, so a failure does not leave a half-written map behind.
  const fs::path pgm_tmp = pgm_path.string() + ".tmp";
  const fs::path yaml_tmp = yaml_path.string() + ".tmp";
  const auto cleanup = [&] {
    fs::remove(pgm_tmp, ec);
    fs::remove(yaml_tmp, ec);
  };

  {
    std::ofstream ofs(pgm_tmp, std::ios::binary | std::ios::trunc);
    ofs << "P5\n# glim_localization map_compiler occupancy grid, resolution " << grid.resolution << " m\n"
        << grid.width << " " << grid.height << "\n255\n";
    std::vector<char> row(grid.width);
    // PGM rows run from top to bottom: the first row is the highest y.
    for (int y = grid.height - 1; y >= 0; y--) {
      for (int x = 0; x < grid.width; x++) {
        const int8_t cell = grid.at(x, y);
        row[x] = static_cast<char>(cell == kOccupancyOccupied ? kPixelOccupied : cell == kOccupancyFree ? kPixelFree : kPixelUnknown);
      }
      ofs.write(row.data(), static_cast<std::streamsize>(row.size()));
    }
    if (!ofs.flush()) {
      spdlog::error("failed to write {}", pgm_tmp.string());
      cleanup();
      return false;
    }
  }
  {
    // Fixed notation keeps every number a YAML float (an exponent without a dot is a string in YAML 1.1).
    // The image name is quoted, so names such as "1.5", "true", "a: b" or "#x" stay strings.
    std::ofstream ofs(yaml_tmp, std::ios::trunc);
    ofs << fmt::format(
      "image: {}\n"
      "mode: trinary\n"
      "resolution: {:.9f}\n"
      "origin: [{:.9f}, {:.9f}, 0.0]\n"
      "negate: 0\n"
      "occupied_thresh: {}\n"
      "free_thresh: {}\n",
      yaml_quoted(pgm_path.filename().string()), grid.resolution, grid.origin.x(), grid.origin.y(), kOccupiedThresh, kFreeThresh);
    if (!ofs.flush()) {
      spdlog::error("failed to write {}", yaml_tmp.string());
      cleanup();
      return false;
    }
  }

  fs::rename(pgm_tmp, pgm_path, ec);
  if (!ec) {
    fs::rename(yaml_tmp, yaml_path, ec);
    if (ec) {
      std::error_code ignored;
      fs::remove(pgm_path, ignored);
    }
  }
  if (ec) {
    spdlog::error("failed to write {}: {}", prefix, ec.message());
    cleanup();
    return false;
  }
  return true;
}

}  // namespace glim_localization
