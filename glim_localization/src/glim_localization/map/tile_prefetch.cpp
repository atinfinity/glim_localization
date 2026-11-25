#include <glim_localization/map/tile_prefetch.hpp>

#include <algorithm>
#include <cmath>
#include <tuple>
#include <unordered_set>

namespace glim_localization {

namespace {

double distance_to_segment(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  const Eigen::Vector3d ab = b - a;
  const double length_sq = ab.squaredNorm();
  const double t = length_sq > 0.0 ? std::clamp((p - a).dot(ab) / length_sq, 0.0, 1.0) : 0.0;
  return (a + t * ab - p).norm();
}

}  // namespace

std::vector<MapTile::ConstPtr> select_prefetch_tiles(
  const MapManager& map,
  const Eigen::Isometry3d& T_world_imu,
  const Eigen::Vector3d& v_world_imu,
  const TilePrefetchParams& params) {
  const Eigen::Vector3d start = T_world_imu.translation();
  if (params.max_tiles < 1 || params.max_submaps < 1 || !(params.search_radius > 0.0) || !std::isfinite(params.search_radius) || !start.allFinite()) {
    return {};
  }
  const double lookahead = std::isfinite(params.lookahead) ? std::max(params.lookahead, 0.0) : 0.0;
  const Eigen::Vector3d velocity = v_world_imu.allFinite() ? v_world_imu : Eigen::Vector3d::Zero();
  Eigen::Vector3d end = start + velocity * lookahead;
  if (!end.allFinite()) {
    end = start;
  }

  // Samples along the segment, no farther apart than half the search radius, so no stretch of the path is missed.
  const double length = (end - start).norm();
  const int steps = std::min(1000, static_cast<int>(std::ceil(length / (0.5 * params.search_radius))));
  std::vector<MapTile::ConstPtr> candidates;
  std::unordered_set<int> seen;
  for (int i = 0; i <= steps; i++) {
    Eigen::Isometry3d T = T_world_imu;
    T.translation() = steps > 0 ? start + (end - start) * (static_cast<double>(i) / steps) : start;
    for (const auto& tile : map.query(T, params.search_radius, params.max_submaps)) {
      if (seen.insert(tile->id).second) {
        candidates.push_back(tile);
      }
    }
  }

  // Distances in whole millimeters, so rounding errors do not decide between tiles at the same distance.
  const auto mm = [](double d) { return std::llround(d * 1e3); };
  const auto key = [&](const MapTile::ConstPtr& tile) {
    const Eigen::Vector3d origin = tile->T_world_origin.translation();
    return std::make_tuple(mm(distance_to_segment(origin, start, end)), mm((origin - start).norm()), tile->id);
  };
  std::sort(candidates.begin(), candidates.end(), [&](const auto& a, const auto& b) { return key(a) < key(b); });
  if (candidates.size() > static_cast<size_t>(params.max_tiles)) {
    candidates.resize(params.max_tiles);
  }
  return candidates;
}

}  // namespace glim_localization
