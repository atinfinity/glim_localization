#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

#include <spdlog/spdlog.h>

#include <glim_localization/map/map_io.hpp>
#include <glim_localization/map/occupancy_grid.hpp>
#include <glim_localization/map/pcd_map.hpp>
#include <glim_localization/map/ply_reader.hpp>

namespace {

void print_usage() {
  std::cerr << "usage: map_compiler <input> <output_dir> [options]\n"
            << "  <input>  a GLIM dump, a compiled map, or a .pcd or .ply file (x, y, z in the map frame)\n"
            << "           (PCD: DATA ascii, binary or binary_compressed)\n"
            << "options for .pcd and .ply input:\n"
            << "  --grid-size <m>    tile edge along x and y, at least 1 (default 20)\n"
            << "  --voxel-size <m>   voxel grid downsampling before tiling, 0 disables it, otherwise at least 0.01 (default 0)\n"
            << "  --k-neighbors <n>  neighbours for covariance estimation, 5 to 1000 (default 10)\n"
            << "options for a GLIM dump or a compiled map (a compiled map keeps the source in its map_index.json):\n"
            << "  --recompute-covariances  re-estimate the per-point covariances from the k nearest neighbours within each\n"
            << "                           submap, as for .pcd and .ply input (default off: keep the stored ones)\n"
            << "  --k-neighbors <n>        with --recompute-covariances: neighbours, 5 to 1000 (default 10)\n"
            << "point cloud export (any input):\n"
            << "  --export-pcd <file>  also write the points of the compiled map in the map frame as one PCD file (binary,\n"
            << "                       double x y z, no covariances), for viewing and editing. Points in the overlap of\n"
            << "                       GLIM submaps appear once per submap; --voxel-size merges them when compiling it again\n"
            << "2D occupancy grid for Nav2 (any input; map_server PGM + YAML, heights in the map frame):\n"
            << "  --occupancy-grid <prefix>       write <prefix>.pgm and <prefix>.yaml\n"
            << "  --occupancy-resolution <m>      cell edge, at least 0.01 (default 0.05)\n"
            << "  --occupancy-z-min <m>           lower bound of the obstacle band (required with --occupancy-grid)\n"
            << "  --occupancy-z-max <m>           upper bound of the obstacle band (required with --occupancy-grid)\n"
            << "  A cell is occupied if it holds a point with z-min <= z < z-max, free if it holds only points below z-min,\n"
            << "  and unknown otherwise. Points at or above z-max are ignored.\n"
            << "MapManager uses up to map_max_submaps (default 5) tiles nearest to the robot by centroid, so 20 m tiles\n"
            << "cover only about 40 x 40 m. For long-range LiDAR, raise --grid-size (for example 50 to 100) or map_max_submaps."
            << std::endl;
}

bool parse_double(const char* text, double& value) {
  char* end = nullptr;
  value = std::strtod(text, &end);
  return end != text && *end == '\0';
}

bool parse_int(const char* text, int& value) {
  char* end = nullptr;
  const long parsed = std::strtol(text, &end, 10);
  if (end == text || *end != '\0' || parsed < 1 || parsed > 1000) {
    return false;
  }
  value = static_cast<int>(parsed);
  return true;
}

/// "pcd" or "ply" for a point cloud file, empty for a map directory.
/// Decided by the extension, so a missing .pcd or .ply file is reported as such and not as a broken GLIM dump.
/// An existing directory is always a map, whatever its name.
std::string point_cloud_format(const std::string& path) {
  std::error_code ec;
  if (std::filesystem::is_directory(path, ec)) {
    return "";
  }
  std::string extension = std::filesystem::path(path).extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char c) { return std::tolower(c); });
  if (extension == ".pcd" || extension == ".ply") {
    return extension.substr(1);
  }
  return "";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    print_usage();
    return 1;
  }
  const std::string input = argv[1];
  const std::string output = argv[2];

  glim_localization::PcdMapParams params;
  params.num_threads = std::max(1u, std::thread::hardware_concurrency());
  glim_localization::OccupancyGridParams grid_params;
  std::string grid_prefix;
  std::string export_pcd;
  bool recompute_covariances = false;
  bool has_k_neighbors = false;
  bool has_tiling_options = false;  // --grid-size or --voxel-size
  bool has_grid_options = false;
  for (int i = 3; i < argc; i += 2) {
    const std::string option = argv[i];
    if (option == "--recompute-covariances") {
      recompute_covariances = true;
      i--;  // A flag without a value
      continue;
    }
    if (i + 1 >= argc) {
      std::cerr << "missing value for " << option << std::endl;
      print_usage();
      return 1;
    }
    const char* value = argv[i + 1];
    bool ok = false;
    if (option == "--grid-size") {
      ok = parse_double(value, params.grid_size);
    } else if (option == "--voxel-size") {
      ok = parse_double(value, params.voxel_size);
    } else if (option == "--k-neighbors") {
      ok = parse_int(value, params.k_neighbors);
    } else if (option == "--export-pcd") {
      export_pcd = value;
      ok = !export_pcd.empty();
    } else if (option == "--occupancy-grid") {
      grid_prefix = value;
      ok = !grid_prefix.empty();
    } else if (option == "--occupancy-resolution") {
      ok = parse_double(value, grid_params.resolution);
    } else if (option == "--occupancy-z-min") {
      ok = parse_double(value, grid_params.z_min);
    } else if (option == "--occupancy-z-max") {
      ok = parse_double(value, grid_params.z_max);
    } else {
      std::cerr << "unknown option " << option << std::endl;
      print_usage();
      return 1;
    }
    if (!ok) {
      std::cerr << "invalid value for " << option << ": " << value << std::endl;
      return 1;
    }
    if (option == "--export-pcd") {
      // Any input
    } else if (option.rfind("--occupancy-", 0) == 0) {
      has_grid_options = true;
    } else if (option == "--k-neighbors") {
      has_k_neighbors = true;
    } else {
      has_tiling_options = true;
    }
  }

  // Check the outputs and everything about the occupancy grid before the slow part, and build the grid before writing
  // anything. save_compiled_map checks the output again; this only fails early.
  {
    std::error_code ec;
    const std::filesystem::path output_path(output);
    if (std::filesystem::exists(output_path, ec) && (!std::filesystem::is_directory(output_path, ec) || !std::filesystem::is_empty(output_path, ec))) {
      std::cerr << "output exists and is not an empty directory: " << output << std::endl;
      return 1;
    }
  }
  if (!export_pcd.empty()) {
    std::error_code ec;
    const bool exists = std::filesystem::exists(export_pcd, ec);
    if (ec) {
      std::cerr << "failed to check " << export_pcd << ": " << ec.message() << std::endl;
      return 1;
    }
    if (exists) {
      std::cerr << export_pcd << " already exists" << std::endl;
      return 1;
    }
  }
  if (has_grid_options) {
    if (grid_prefix.empty()) {
      std::cerr << "--occupancy-* options need --occupancy-grid <prefix>" << std::endl;
      return 1;
    }
    if (const std::string error = glim_localization::check_occupancy_grid_prefix(grid_prefix); !error.empty()) {
      std::cerr << "invalid options: " << error << std::endl;
      return 1;
    }
    if (const std::string error = glim_localization::check_occupancy_grid_params(grid_params); !error.empty()) {
      std::cerr << "invalid options: " << error << " (set them with --occupancy-z-min and --occupancy-z-max)" << std::endl;
      return 1;
    }
    std::error_code ec;
    for (const auto& path : {glim_localization::occupancy_grid_pgm_path(grid_prefix), glim_localization::occupancy_grid_yaml_path(grid_prefix)}) {
      if (std::filesystem::exists(path, ec)) {
        std::cerr << path << " already exists" << std::endl;
        return 1;
      }
    }
  }

  glim_localization::LoadedSubmaps submaps;
  std::string source;  // Set by the input below
  const std::string format = point_cloud_format(input);
  if (!format.empty()) {
    if (recompute_covariances) {
      std::cerr << "--recompute-covariances applies only to a GLIM dump or a compiled map; .pcd and .ply input always estimates "
                   "covariances (set k with --k-neighbors)"
                << std::endl;
      return 1;
    }
    if (const std::string error = glim_localization::check_pcd_map_params(params); !error.empty()) {
      std::cerr << "invalid options: " << error << std::endl;
      return 1;
    }
    std::error_code ec;
    if (!std::filesystem::is_regular_file(input, ec)) {
      std::cerr << "file not found: " << input << std::endl;
      return 1;
    }
    glim_localization::PcdPoints points;
    const bool read = format == "ply" ? glim_localization::read_ply(input, points) : glim_localization::read_pcd(input, points);
    if (!read || !glim_localization::build_pcd_submaps(points, params, submaps)) {
      spdlog::error("failed to build a map from {}", input);
      return 1;
    }
    source = format;
  } else {
    if (has_tiling_options) {
      std::cerr << "--grid-size and --voxel-size apply only to .pcd and .ply input" << std::endl;
      return 1;
    }
    if (has_k_neighbors && !recompute_covariances) {
      std::cerr << "--k-neighbors applies to a GLIM dump or a compiled map only with --recompute-covariances" << std::endl;
      return 1;
    }
    if (recompute_covariances && params.k_neighbors < glim_localization::kMinPcdKNeighbors) {
      std::cerr << "invalid options: k_neighbors " << params.k_neighbors << " must be at least " << glim_localization::kMinPcdKNeighbors << std::endl;
      return 1;
    }
    // A compiled map keeps the source it was compiled from (for example "pcd"), also with --recompute-covariances.
    if (!glim_localization::load_map(input, submaps, &source)) {
      spdlog::error("failed to load {}", input);
      return 1;
    }
    if (recompute_covariances && !glim_localization::recompute_covariances(submaps, params.k_neighbors, params.num_threads)) {
      spdlog::error("failed to recompute the covariances of {}", input);
      return 1;
    }
  }

  glim_localization::OccupancyGrid grid;
  if (!grid_prefix.empty() && !glim_localization::build_occupancy_grid(submaps, grid_params, grid)) {
    spdlog::error("failed to build the occupancy grid");
    return 1;
  }

  if (!glim_localization::save_compiled_map(output, submaps, source)) {
    spdlog::error("failed to write {}", output);
    return 1;
  }
  spdlog::info("wrote {} submaps to {}", submaps.size(), output);

  if (!grid_prefix.empty()) {
    if (!glim_localization::save_occupancy_grid(grid_prefix, grid)) {
      spdlog::error("failed to write the occupancy grid {}; the compiled map in {} was written and is complete", grid_prefix, output);
      return 1;
    }
    spdlog::info("wrote the occupancy grid {}", glim_localization::occupancy_grid_yaml_path(grid_prefix));
  }

  // Last, so a failed export leaves the compiled map and the occupancy grid complete.
  if (!export_pcd.empty()) {
    if (!glim_localization::write_pcd(export_pcd, submaps)) {
      spdlog::error("failed to write the point cloud {}; the other outputs were written and are complete", export_pcd);
      return 1;
    }
    spdlog::info("wrote the point cloud {}", export_pcd);
  }
  return 0;
}
