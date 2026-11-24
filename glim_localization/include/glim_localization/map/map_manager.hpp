#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim_localization/map/map_tile.hpp>

namespace glim_localization {

struct MapManagerParams {
  std::vector<double> voxel_resolutions = {0.5, 1.0, 2.0};  ///< Fine to coarse [m]
};

/// Holds a prebuilt map as an immutable snapshot.
/// All methods are thread-safe, and returned tiles stay valid after a reload.
class MapManager {
public:
  explicit MapManager(const MapManagerParams& params = MapManagerParams());
  ~MapManager();

  MapManager(const MapManager&) = delete;
  MapManager& operator=(const MapManager&) = delete;

  /// Load a map directory. On failure, returns false and keeps the current map.
  bool load(const std::string& path);

  size_t size() const;

  /// @return The tile with the given dump id, or nullptr
  MapTile::ConstPtr tile(int id) const;

  /// Find up to k tiles whose origins are within radius of the pose.
  /// @return Tiles sorted by distance, then by id
  std::vector<MapTile::ConstPtr> query(const Eigen::Isometry3d& T_world_imu, double radius, int k) const;

private:
  struct State;

  std::shared_ptr<const State> snapshot() const;

  MapManagerParams params_;
  mutable std::mutex mutex_;
  std::shared_ptr<const State> state_;
};

}  // namespace glim_localization
