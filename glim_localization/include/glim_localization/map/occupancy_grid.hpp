#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <glim_localization/map/map_io.hpp>

namespace glim_localization {

/// Cell values, as in nav_msgs/OccupancyGrid
constexpr int8_t kOccupancyUnknown = -1;
constexpr int8_t kOccupancyFree = 0;
constexpr int8_t kOccupancyOccupied = 100;

/// Bounds of OccupancyGridParams
constexpr double kMinOccupancyResolution = 0.01;
constexpr size_t kMaxOccupancyCells = size_t(1) << 30;  ///< One byte per cell in memory and in the PGM

/// How a 3D map becomes a 2D occupancy grid. Heights are map-frame z.
/// A cell is occupied if it holds a point with z_min <= z < z_max, otherwise free if it holds a point with
/// z < z_min (ground seen below the band), otherwise unknown. Points at or above z_max (ceilings) are ignored.
struct OccupancyGridParams {
  double resolution = 0.05;                                  ///< Cell edge [m], >= kMinOccupancyResolution
  double z_min = std::numeric_limits<double>::quiet_NaN();  ///< Lower bound of the obstacle band [m], required
  double z_max = std::numeric_limits<double>::quiet_NaN();  ///< Upper bound of the obstacle band [m], required
};

/// A 2D grid in the map frame, laid out like nav_msgs/OccupancyGrid.
struct OccupancyGrid {
  double resolution = 0.0;
  Eigen::Vector2d origin = Eigen::Vector2d::Zero();  ///< Map-frame x, y of the lower-left corner of cell (0, 0)
  int width = 0;                                     ///< Cells along x
  int height = 0;                                    ///< Cells along y
  std::vector<int8_t> data;                          ///< Row-major, row 0 at the lowest y

  int8_t at(int x, int y) const { return data[static_cast<size_t>(y) * width + x]; }
};

/// Empty if `params` is valid, otherwise a message that names the bad parameter.
std::string check_occupancy_grid_params(const OccupancyGridParams& params);

/// Project the map's points (in the map frame) onto a grid. Cell edges are multiples of the resolution, and the
/// grid covers the x, y extent of the points below z_max. Fails if there are no such points or the grid would
/// exceed kMaxOccupancyCells. On failure, `grid` is unchanged.
bool build_occupancy_grid(const LoadedSubmaps& submaps, const OccupancyGridParams& params, OccupancyGrid& grid);

/// Write `<prefix>.pgm` and `<prefix>.yaml` in the map_server format (trinary, negate 0): occupied cells are 0,
/// free cells 254 and unknown cells 205; the top PGM row is the highest y. Parent directories are created.
/// Fails without writing if either file exists.
bool save_occupancy_grid(const std::string& prefix, const OccupancyGrid& grid);

/// The files save_occupancy_grid writes for `prefix`
std::string occupancy_grid_pgm_path(const std::string& prefix);
std::string occupancy_grid_yaml_path(const std::string& prefix);

}  // namespace glim_localization
