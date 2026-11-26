#pragma once

#include <string>

#include "synthetic_map.hpp"
#include "synthetic_sensor.hpp"

namespace glim_localization::test {

/// The corridor used by the localization tests: five submaps with cross walls.
SyntheticMapParams replay_map_params();

/// Write the synthetic trajectory as a replay dataset in `dir`: LiDAR frames at 10 Hz,
/// IMU at 100 Hz up to t_end + 0.05 s, wheel odometry at 50 Hz over the same span (base frame = IMU frame, small
/// noise), and the ground truth as TUM (traj_gt.txt).
/// Stamps are `round(t * 1e9)` ns. The LiDAR and IMU frames coincide.
/// @return The number of frames written
int write_synthetic_replay(const std::string& dir, double t_end, const SyntheticMapParams& map_params = replay_map_params());

struct SyntheticReplayOptions {
  SyntheticLidarParams lidar;    ///< lidar.scan_duration > 0 gives a rolling-shutter scan
  bool write_point_times = false;  ///< Store the per-point times (false: the same points, but the times are lost)
};

/// As above, with a rolling-shutter lidar (see SyntheticLidarParams::scan_duration). The IMU is extended by the scan duration
/// so that the last sweep is covered.
int write_synthetic_replay(const std::string& dir, double t_end, const SyntheticMapParams& map_params, const SyntheticReplayOptions& options);

}  // namespace glim_localization::test
