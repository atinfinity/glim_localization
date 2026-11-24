#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

#include <spdlog/spdlog.h>

#include <glim_localization/map/map_io.hpp>
#include <glim_localization/map/pcd_map.hpp>

namespace {

void print_usage() {
  std::cerr << "usage: map_compiler <input> <output_dir> [options]\n"
            << "  <input>  a GLIM dump, a compiled map, or a .pcd file (x, y, z in the map frame)\n"
            << "options for .pcd input:\n"
            << "  --grid-size <m>    tile edge along x and y, at least 1 (default 20)\n"
            << "  --voxel-size <m>   voxel grid downsampling before tiling, 0 disables it, otherwise at least 0.01 (default 0)\n"
            << "  --k-neighbors <n>  neighbours for covariance estimation, 5 to 1000 (default 10)\n"
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

/// Decided by the extension, so a missing .pcd file is reported as such and not as a broken GLIM dump.
/// An existing directory is always a map, whatever its name.
bool is_pcd_path(const std::string& path) {
  std::error_code ec;
  if (std::filesystem::is_directory(path, ec)) {
    return false;
  }
  std::string extension = std::filesystem::path(path).extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char c) { return std::tolower(c); });
  return extension == ".pcd";
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
  bool has_options = false;
  for (int i = 3; i < argc; i += 2) {
    const std::string option = argv[i];
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
    } else {
      std::cerr << "unknown option " << option << std::endl;
      print_usage();
      return 1;
    }
    if (!ok) {
      std::cerr << "invalid value for " << option << ": " << value << std::endl;
      return 1;
    }
    has_options = true;
  }

  glim_localization::LoadedSubmaps submaps;
  std::string source = "glim_dump";
  if (is_pcd_path(input)) {
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
    if (!glim_localization::read_pcd(input, points) || !glim_localization::build_pcd_submaps(points, params, submaps)) {
      spdlog::error("failed to build a map from {}", input);
      return 1;
    }
    source = "pcd";
  } else {
    if (has_options) {
      std::cerr << "options apply only to .pcd input" << std::endl;
      return 1;
    }
    if (!glim_localization::load_map(input, submaps)) {
      spdlog::error("failed to load {}", input);
      return 1;
    }
  }

  if (!glim_localization::save_compiled_map(output, submaps, source)) {
    spdlog::error("failed to write {}", output);
    return 1;
  }

  spdlog::info("wrote {} submaps to {}", submaps.size(), output);
  return 0;
}
