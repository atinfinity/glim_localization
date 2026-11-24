#include <glim_localization/map/map_manager.hpp>

#include <algorithm>
#include <functional>
#include <chrono>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>

#include <spdlog/spdlog.h>

#include <gtsam_points/ann/kdtree.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>

#include <glim_localization/map/map_io.hpp>

namespace glim_localization {

struct MapManager::State {
  std::vector<MapTile::ConstPtr> tiles;
  std::unordered_map<int, MapTile::ConstPtr> tiles_by_id;
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> origins;  // Referenced by kdtree
  std::unique_ptr<gtsam_points::KdTree> kdtree;
};

namespace {

MapTile::ConstPtr make_tile(const LoadedSubmap& submap, const std::vector<double>& resolutions) {
  auto tile = std::make_shared<MapTile>();
  tile->id = submap.id;
  tile->T_world_origin = submap.T_world_origin;
  tile->points = submap.points;

  for (const double resolution : resolutions) {
    auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
    voxelmap->insert(*submap.points);
    tile->voxelmaps.push_back(voxelmap);
  }

  Eigen::Vector3d min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d max = -min;
  for (size_t i = 0; i < submap.points->size(); i++) {
    const Eigen::Vector3d p = submap.T_world_origin * submap.points->points[i].head<3>();
    min = min.cwiseMin(p);
    max = max.cwiseMax(p);
  }
  tile->bbox_min = min;
  tile->bbox_max = max;
  return tile;
}

}  // namespace

MapManager::MapManager(const MapManagerParams& params) : params_(params) {}

MapManager::~MapManager() = default;

std::shared_ptr<const MapManager::State> MapManager::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool MapManager::load(const std::string& path) {
  const auto& resolutions = params_.voxel_resolutions;
  if (resolutions.empty() || std::any_of(resolutions.begin(), resolutions.end(), [](double r) { return !(r > 0.0) || !std::isfinite(r); })) {
    spdlog::error("voxel_resolutions must be positive");
    return false;
  }
  // Map factors take the first level as the finest and the last as the coarsest.
  if (std::adjacent_find(resolutions.begin(), resolutions.end(), std::greater_equal<double>()) != resolutions.end()) {
    spdlog::error("voxel_resolutions must be strictly ascending");
    return false;
  }

  LoadedSubmaps submaps;
  if (!load_map(path, submaps)) {
    return false;
  }

  const auto start = std::chrono::steady_clock::now();
  auto state = std::make_shared<State>();
  state->tiles.reserve(submaps.size());
  state->origins.reserve(submaps.size());
  for (const auto& submap : submaps) {
    auto tile = make_tile(submap, resolutions);
    state->tiles_by_id.emplace(tile->id, tile);
    state->origins.emplace_back(submap.T_world_origin.translation().homogeneous());
    state->tiles.push_back(std::move(tile));
  }
  state->kdtree = std::make_unique<gtsam_points::KdTree>(state->origins.data(), static_cast<int>(state->origins.size()));

  const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;
  spdlog::info("loaded {} map tiles from {} (voxelization {:.3f} s)", state->tiles.size(), path, elapsed.count());

  std::lock_guard<std::mutex> lock(mutex_);
  state_ = std::move(state);
  return true;
}

size_t MapManager::size() const {
  const auto state = snapshot();
  return state ? state->tiles.size() : 0;
}

MapTile::ConstPtr MapManager::tile(int id) const {
  const auto state = snapshot();
  if (!state) {
    return nullptr;
  }
  const auto found = state->tiles_by_id.find(id);
  return found == state->tiles_by_id.end() ? nullptr : found->second;
}

std::vector<MapTile::ConstPtr> MapManager::tiles() const {
  const auto state = snapshot();
  if (!state) {
    return {};
  }
  std::vector<MapTile::ConstPtr> result = state->tiles;
  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) { return a->id < b->id; });
  return result;
}

std::vector<MapTile::ConstPtr> MapManager::query(const Eigen::Isometry3d& T_world_imu, double radius, int k) const {
  const auto state = snapshot();
  const Eigen::Vector3d center = T_world_imu.translation();
  if (!state || !(radius > 0.0) || !std::isfinite(radius) || k <= 0 || !center.allFinite()) {
    return {};
  }

  // Search the whole radius before truncating, so ties at the k-th tile resolve by id.
  const Eigen::Vector4d pt = center.homogeneous();
  std::vector<size_t> indices;
  std::vector<double> sq_dists;
  const size_t found = state->kdtree->radius_search(pt.data(), radius, indices, sq_dists);

  const double sq_radius = radius * radius;
  std::vector<std::pair<double, const MapTile::ConstPtr*>> candidates;
  candidates.reserve(found);
  for (size_t i = 0; i < found; i++) {
    const double sq_dist = (state->origins[indices[i]].head<3>() - center).squaredNorm();
    if (sq_dist <= sq_radius) {
      candidates.emplace_back(sq_dist, &state->tiles[indices[i]]);
    }
  }
  std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
    return a.first != b.first ? a.first < b.first : (*a.second)->id < (*b.second)->id;
  });

  std::vector<MapTile::ConstPtr> result;
  const size_t count = std::min(candidates.size(), static_cast<size_t>(k));
  result.reserve(count);
  for (size_t i = 0; i < count; i++) {
    result.push_back(*candidates[i].second);
  }
  return result;
}

}  // namespace glim_localization
