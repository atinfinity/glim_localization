#include "synthetic_replay.hpp"

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>

#include <glim_localization/replay/replay_dataset.hpp>

#include "synthetic_scene.hpp"
#include "synthetic_sensor.hpp"

namespace glim_localization::test {
namespace {

int64_t to_ns(double seconds) {
  return std::llround(seconds * 1e9);
}

}  // namespace

SyntheticMapParams replay_map_params() {
  SyntheticMapParams params;
  params.num_submaps = 5;
  params.cross_walls = true;  // The plain corridor is nearly degenerate along x
  return params;
}

int write_synthetic_replay(const std::string& dir, double t_end, const SyntheticMapParams& map_params) {
  return write_synthetic_replay(dir, t_end, map_params, SyntheticReplayOptions());
}

int write_synthetic_replay(const std::string& dir, double t_end, const SyntheticMapParams& map_params, const SyntheticReplayOptions& options) {
  ReplayDatasetWriter writer(dir);
  const bool with_times = options.write_point_times;

  std::ofstream gt(dir + "/traj_gt.txt", std::ios::trunc);
  int count = 0;
  for (const auto& scan : make_synthetic_scans(make_scene_boxes(map_params), 0.0, t_end, options.lidar)) {
    std::vector<ReplayPoint> points;
    points.reserve(scan->size());
    for (const auto& p : scan->points) {
      points.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()), 0.0f, 0.0f});
    }
    if (with_times) {
      for (size_t k = 0; k < points.size(); k++) {
        points[k].t = static_cast<float>(scan->times[k]);
      }
    }
    const int64_t stamp_ns = to_ns(scan->stamp);
    writer.write_cloud(stamp_ns, points, with_times ? kReplayFlagPointTime : 0);

    const Eigen::Isometry3d T = synthetic_T_world_imu(scan->stamp);
    const Eigen::Quaterniond q(T.linear());
    char line[256];
    std::snprintf(
      line, sizeof(line), "%s %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n", format_stamp_ns(stamp_ns).c_str(), T.translation().x(), T.translation().y(),
      T.translation().z(), q.x(), q.y(), q.z(), q.w());
    gt << line;
    count++;
  }
  if (!gt) {
    throw std::runtime_error("cannot write traj_gt.txt in " + dir);
  }

  std::vector<ReplayImu> imu;
  for (const auto& sample : make_synthetic_imu(0.0, t_end + 0.05 + options.lidar.scan_duration)) {
    imu.push_back({to_ns(sample.stamp), sample.linear_acc, sample.angular_vel});
  }
  writer.write_imu(imu);
  writer.finish(with_times, R"({"source":"synthetic"})");
  return count;
}

}  // namespace glim_localization::test
