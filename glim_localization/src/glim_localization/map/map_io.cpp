#include <glim_localization/map/map_io.hpp>

#include <unistd.h>

#include <array>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>

#include <boost/format.hpp>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <gtsam/base/serialization.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include <glim/mapping/sub_map.hpp>

namespace glim_localization {
namespace {

namespace fs = std::filesystem;
using gtsam::symbol_shorthand::X;

std::string submap_dir(const std::string& path, int id) {
  return (boost::format("%s/%06d") % path % id).str();
}

/// Number of submaps from graph.txt, or consecutive directories when it is absent.
std::optional<int> count_submaps(const std::string& path) {
  if (fs::exists(path + "/graph.txt")) {
    std::ifstream ifs(path + "/graph.txt");
    std::string token;
    int count = 0;
    if (!(ifs >> token >> count) || token != "num_submaps:" || count < 0) {
      spdlog::error("invalid graph.txt in {}", path);
      return std::nullopt;
    }
    return count;
  }

  int count = 0;
  while (fs::is_directory(submap_dir(path, count))) {
    count++;
  }
  return count;
}

/// SubMap::load dereferences covariances without checking them, so require the files first.
bool has_required_files(const std::string& dir) {
  const auto exists = [&dir](const char* name) { return fs::exists(dir + "/" + name); };
  if (!exists("data.txt")) {
    spdlog::error("missing data.txt in {}", dir);
    return false;
  }
  if (!exists("points.bin") && !exists("points_compact.bin")) {
    spdlog::error("missing point file in {}", dir);
    return false;
  }
  if (!exists("covs.bin") && !exists("covs_compact.bin")) {
    spdlog::error("missing covariance file in {}", dir);
    return false;
  }
  return true;
}

bool is_valid_pose(const Eigen::Isometry3d& T) {
  const Eigen::Matrix4d& m = T.matrix();
  return m.allFinite() && m.row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0));
}

/// values.bin keeps full precision, while data.txt rounds poses to six significant digits.
bool load_values(const std::string& path, std::optional<gtsam::Values>& values) {
  const std::string file = path + "/values.bin";
  if (!fs::exists(file)) {
    spdlog::warn("{} not found, using poses rounded to 6 digits in data.txt", file);
    return true;
  }
  try {
    gtsam::Values loaded;
    gtsam::deserializeFromBinaryFile(file, loaded);
    values = std::move(loaded);
    return true;
  } catch (const std::exception& e) {
    spdlog::error("failed to deserialize {}: {}", file, e.what());
    return false;
  }
}

bool load_submap(const std::string& path, int id, const std::optional<gtsam::Values>& values, LoadedSubmap& loaded) {
  const std::string dir = submap_dir(path, id);
  if (!has_required_files(dir)) {
    return false;
  }

  glim::SubMap::Ptr submap;
  try {
    submap = glim::SubMap::load(dir);
  } catch (const std::exception& e) {
    spdlog::error("failed to load {}: {}", dir, e.what());
    return false;
  }
  if (!submap || !submap->frame) {
    spdlog::error("failed to load {}", dir);
    return false;
  }
  if (submap->id != id) {
    spdlog::error("submap id mismatch in {}: {}", dir, submap->id);
    return false;
  }
  if (!is_valid_pose(submap->T_world_origin)) {
    spdlog::error("invalid T_world_origin in {}", dir);
    return false;
  }

  loaded.id = id;
  loaded.T_world_origin = submap->T_world_origin;
  loaded.points = submap->frame;

  if (values) {
    try {
      loaded.T_world_origin = Eigen::Isometry3d(values->at<gtsam::Pose3>(X(id)).matrix());
    } catch (const std::exception& e) {
      spdlog::error("no pose for submap {} in values.bin: {}", id, e.what());
      return false;
    }
  }

  if (loaded.points->size() && !loaded.points->has_covs()) {
    spdlog::error("no covariances in {}", dir);
    return false;
  }
  return true;
}

constexpr const char* kIndexFile = "map_index.json";
constexpr const char* kIndexFormat = "glim_localization_map";
constexpr int kIndexVersion = 1;

/// The index holds exact poses because data.txt rounds them.
bool load_compiled_map(const std::string& path, LoadedSubmaps& submaps) {
  const std::string file = path + "/" + kIndexFile;
  try {
    std::ifstream ifs(file);
    const nlohmann::json index = nlohmann::json::parse(ifs);
    if (index.at("format").get<std::string>() != kIndexFormat || index.at("version").get<int>() != kIndexVersion) {
      spdlog::error("unsupported map format in {}", file);
      return false;
    }

    const auto& entries = index.at("submaps");
    if (!entries.is_array() || entries.empty() || index.at("num_submaps").get<size_t>() != entries.size()) {
      spdlog::error("inconsistent submap count in {}", file);
      return false;
    }

    LoadedSubmaps result;
    result.reserve(entries.size());
    int previous_id = -1;
    for (const auto& entry : entries) {
      const int id = entry.at("id").get<int>();
      // Paths are derived from ids, so an index cannot point outside the map directory.
      if (id <= previous_id || entry.at("path").get<std::string>() != (boost::format("%06d") % id).str()) {
        spdlog::error("invalid submap entry {} in {}", id, file);
        return false;
      }
      previous_id = id;

      const auto elements = entry.at("T_world_origin").get<std::vector<double>>();
      if (elements.size() != 16) {
        spdlog::error("invalid T_world_origin for submap {} in {}", id, file);
        return false;
      }
      Eigen::Isometry3d T_world_origin;
      T_world_origin.matrix() = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(elements.data());
      if (!is_valid_pose(T_world_origin)) {
        spdlog::error("invalid T_world_origin for submap {} in {}", id, file);
        return false;
      }

      LoadedSubmap loaded;
      if (!load_submap(path, id, std::nullopt, loaded)) {
        return false;
      }
      const size_t num_points = entry.at("num_points").get<size_t>();
      if (num_points == 0 || loaded.points->size() != num_points) {
        spdlog::error("point count mismatch for submap {} in {}", id, file);
        return false;
      }
      loaded.T_world_origin = T_world_origin;
      result.push_back(std::move(loaded));
    }

    submaps = std::move(result);
    return true;
  } catch (const std::exception& e) {
    spdlog::error("failed to read {}: {}", file, e.what());
    return false;
  }
}

nlohmann::json make_index_entry(const LoadedSubmap& submap) {
  const double inf = std::numeric_limits<double>::infinity();
  Eigen::Vector3d min = Eigen::Vector3d::Constant(inf);
  Eigen::Vector3d max = Eigen::Vector3d::Constant(-inf);
  for (size_t i = 0; i < submap.points->size(); i++) {
    const Eigen::Vector3d p = submap.T_world_origin * submap.points->points[i].head<3>();
    min = min.cwiseMin(p);
    max = max.cwiseMax(p);
  }

  std::vector<double> pose(16);
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(pose.data()) = submap.T_world_origin.matrix();

  return {
    {"id", submap.id},
    {"path", (boost::format("%06d") % submap.id).str()},
    {"num_points", submap.points->size()},
    {"T_world_origin", pose},
    {"bbox_min", std::array<double, 3>{min.x(), min.y(), min.z()}},
    {"bbox_max", std::array<double, 3>{max.x(), max.y(), max.z()}},
  };
}

/// Write one submap with upstream SubMap::save, which needs one frame to stay loadable.
void write_compiled_submap(const std::string& path, const LoadedSubmap& loaded) {
  glim::EstimationFrame::Ptr frame(new glim::EstimationFrame);
  frame->id = loaded.id;
  frame->stamp = 0.0;
  frame->T_lidar_imu.setIdentity();
  frame->T_world_lidar = loaded.T_world_origin;
  frame->T_world_imu = loaded.T_world_origin;
  frame->v_world_imu.setZero();
  frame->imu_bias.setZero();
  frame->frame_id = glim::FrameID::IMU;

  glim::SubMap submap;
  submap.id = loaded.id;
  submap.T_world_origin = loaded.T_world_origin;
  submap.T_origin_endpoint_L.setIdentity();
  submap.T_origin_endpoint_R.setIdentity();
  submap.frame = loaded.points;
  submap.frames = {frame};
  submap.odom_frames = {frame};

  const std::string dir = submap_dir(path, loaded.id);
  submap.save(dir);
  fs::remove(dir + "/imu_rate.txt");
}

}  // namespace

bool save_compiled_map(const std::string& path, const LoadedSubmaps& submaps, const std::string& source) {
  if (submaps.empty()) {
    spdlog::error("no submaps to write");
    return false;
  }

  fs::path temp;
  try {
    fs::path output = fs::absolute(path).lexically_normal();
    if (!output.has_filename()) {
      output = output.parent_path();
    }
    if (fs::exists(output) && (!fs::is_directory(output) || !fs::is_empty(output))) {
      spdlog::error("output exists and is not an empty directory: {}", output.string());
      return false;
    }
    fs::create_directories(output.parent_path());

    // Build next to the output, so a failed run leaves nothing at the destination.
    temp = output;
    temp += ".tmp." + std::to_string(::getpid());
    fs::remove_all(temp);
    fs::create_directories(temp);

    nlohmann::json entries = nlohmann::json::array();
    for (const auto& submap : submaps) {
      write_compiled_submap(temp.string(), submap);
      entries.push_back(make_index_entry(submap));
    }
    const nlohmann::json index = {
      {"format", kIndexFormat},
      {"version", kIndexVersion},
      {"source", source},
      {"num_submaps", submaps.size()},
      {"submaps", entries},
    };
    {
      std::ofstream ofs(temp / kIndexFile);
      ofs << index.dump(2) << std::endl;
      if (!ofs) {
        throw std::runtime_error("failed to write map_index.json");
      }
    }

    // SubMap::save does not report write errors, so read the result back.
    LoadedSubmaps check;
    if (!load_map(temp.string(), check) || check.size() != submaps.size()) {
      throw std::runtime_error("written map failed verification");
    }

    fs::rename(temp, output);
    return true;
  } catch (const std::exception& e) {
    spdlog::error("failed to write {}: {}", path, e.what());
    if (!temp.empty()) {
      std::error_code ec;
      fs::remove_all(temp, ec);
    }
    return false;
  }
}

bool load_map(const std::string& path, LoadedSubmaps& submaps) {
  std::error_code ec;
  if (!fs::is_directory(path, ec)) {
    spdlog::error("map directory not found: {}", path);
    return false;
  }
  if (fs::exists(path + "/" + kIndexFile)) {
    return load_compiled_map(path, submaps);
  }

  const auto count = count_submaps(path);
  if (!count) {
    return false;
  }
  if (*count == 0) {
    spdlog::error("no submaps in {}", path);
    return false;
  }

  std::optional<gtsam::Values> values;
  if (!load_values(path, values)) {
    return false;
  }

  LoadedSubmaps result;
  result.reserve(*count);
  for (int id = 0; id < *count; id++) {
    LoadedSubmap loaded;
    if (!load_submap(path, id, values, loaded)) {
      return false;
    }
    if (loaded.points->size() == 0) {
      spdlog::warn("skipping submap {} without points", id);
      continue;
    }
    result.push_back(std::move(loaded));
  }
  if (result.empty()) {
    spdlog::error("no submaps with points in {}", path);
    return false;
  }

  submaps = std::move(result);
  return true;
}

}  // namespace glim_localization
