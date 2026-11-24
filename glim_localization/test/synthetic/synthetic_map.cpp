#include "synthetic_map.hpp"
#include "synthetic_scene.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <stdexcept>

#include <boost/format.hpp>

#include <gtsam/base/serialization.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/mapping/sub_map.hpp>

namespace glim_localization::test {
namespace {

using PointVector = std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>;

constexpr double kHalfWidth = 5.0;
constexpr double kHeight = 3.0;
constexpr double kMargin = 20.0;
constexpr double kPillarInterval = 7.0;
constexpr double kPillarHalfSize = 0.3;
constexpr double kPillarInset = 1.0;
constexpr double kCrossWallOffset = 3.0;
constexpr double kCrossWallInterval = 5.3;
constexpr double kCrossWallLength = 2.5;  // Leaves the sensor path (|y| <= 1 m) clear
// Thinner than the voxels on purpose. At 0.4 m, voxels holding both faces average them into
// a plane at the wall center, which biases VGICP along x by up to 0.3 m.
constexpr double kCrossWallThickness = 0.05;

void add_box_surface(PointVector& points, const Eigen::Vector3d& min, const Eigen::Vector3d& max, double step) {
  const auto samples = [step](double lo, double hi) {
    return std::max(2, static_cast<int>(std::ceil((hi - lo) / step)) + 1);
  };
  const Eigen::Vector3i n(samples(min.x(), max.x()), samples(min.y(), max.y()), samples(min.z(), max.z()));
  const Eigen::Vector3d delta = (max - min).cwiseQuotient((n - Eigen::Vector3i::Ones()).cast<double>());

  for (int axis = 0; axis < 3; axis++) {
    const int u = (axis + 1) % 3;
    const int v = (axis + 2) % 3;
    for (const double fixed : {min[axis], max[axis]}) {
      for (int i = 0; i < n[u]; i++) {
        for (int j = 0; j < n[v]; j++) {
          Eigen::Vector4d p(0.0, 0.0, 0.0, 1.0);
          p[axis] = fixed;
          p[u] = min[u] + i * delta[u];
          p[v] = min[v] + j * delta[v];
          points.push_back(p);
        }
      }
    }
  }
}

// Pillars and boxes break the translational symmetry along the corridor.
std::vector<SceneBox> scene_obstacles(const SyntheticMapParams& params) {
  const double x_min = -kMargin;
  const double x_max = (params.num_submaps - 1) * params.submap_spacing + kMargin;
  std::vector<SceneBox> boxes;

  const double pillar_y = kHalfWidth - kPillarInset;
  int index = 0;
  for (double x = x_min + kPillarInterval; x < x_max; x += kPillarInterval, index++) {
    const double y = index % 2 == 0 ? pillar_y : -pillar_y;
    const Eigen::Vector3d half(kPillarHalfSize, kPillarHalfSize, 0.0);
    boxes.push_back({Eigen::Vector3d(x, y, 0.0) - half, Eigen::Vector3d(x, y, kHeight) + half});
  }
  for (double x = x_min + 11.0; x < x_max; x += 17.0) {
    const double y = std::fmod(x * 0.37, 2.0) - 1.0;
    boxes.push_back({Eigen::Vector3d(x, y, 0.0), Eigen::Vector3d(x + 1.2, y + 0.8, 0.9)});
  }
  if (params.cross_walls) {
    bool positive_side = true;
    for (double x = x_min + kCrossWallOffset; x < x_max; x += kCrossWallInterval, positive_side = !positive_side) {
      const double y_min = positive_side ? kHalfWidth - kCrossWallLength : -kHalfWidth;
      boxes.push_back({Eigen::Vector3d(x, y_min, 0.0), Eigen::Vector3d(x + kCrossWallThickness, y_min + kCrossWallLength, kHeight)});
    }
  }
  return boxes;
}

PointVector make_scene(const SyntheticMapParams& params) {
  const double x_min = -kMargin;
  const double x_max = (params.num_submaps - 1) * params.submap_spacing + kMargin;
  const double step = params.point_spacing;
  PointVector points;

  for (double x = x_min; x <= x_max; x += step) {
    for (double y = -kHalfWidth; y <= kHalfWidth; y += step) {
      points.emplace_back(x, y, 0.0, 1.0);
    }
    for (double z = step; z <= kHeight; z += step) {
      points.emplace_back(x, -kHalfWidth, z, 1.0);
      points.emplace_back(x, kHalfWidth, z, 1.0);
    }
  }

  for (const auto& box : scene_obstacles(params)) {
    add_box_surface(points, box.min, box.max, step);
  }

  std::mt19937 rng(params.seed);
  std::normal_distribution<double> noise(0.0, params.noise_sigma);
  for (auto& p : points) {
    p.head<3>() += Eigen::Vector3d(noise(rng), noise(rng), noise(rng));
  }
  return points;
}

Eigen::Isometry3d origin_pose(const SyntheticMapParams& params, int id) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(0.1 * id - 0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = Eigen::Vector3d(id * params.submap_spacing, 0.0, 1.0);
  return T;
}

glim::EstimationFrame::Ptr make_origin_frame(int id, const Eigen::Isometry3d& T_world_origin) {
  glim::EstimationFrame::Ptr frame(new glim::EstimationFrame);
  frame->id = id;
  frame->stamp = static_cast<double>(id);
  frame->T_lidar_imu.setIdentity();
  frame->T_world_lidar = T_world_origin;
  frame->T_world_imu = T_world_origin;
  frame->v_world_imu.setZero();
  frame->imu_bias.setZero();
  frame->frame_id = glim::FrameID::IMU;
  return frame;
}

}  // namespace

std::vector<SceneBox> make_scene_boxes(const SyntheticMapParams& params) {
  const double x_min = -kMargin;
  const double x_max = (params.num_submaps - 1) * params.submap_spacing + kMargin;

  std::vector<SceneBox> boxes;
  boxes.push_back({Eigen::Vector3d(x_min, -kHalfWidth, 0.0), Eigen::Vector3d(x_max, kHalfWidth, 0.0)});
  boxes.push_back({Eigen::Vector3d(x_min, -kHalfWidth, 0.0), Eigen::Vector3d(x_max, -kHalfWidth, kHeight)});
  boxes.push_back({Eigen::Vector3d(x_min, kHalfWidth, 0.0), Eigen::Vector3d(x_max, kHalfWidth, kHeight)});
  const auto obstacles = scene_obstacles(params);
  boxes.insert(boxes.end(), obstacles.begin(), obstacles.end());
  return boxes;
}

SyntheticPoints make_synthetic_world_points(const SyntheticMapParams& params) {
  if (params.num_submaps <= 0 || params.point_spacing <= 0.0) {
    throw std::invalid_argument("invalid synthetic map parameters");
  }
  // Submap origins are scene poses shifted by world_offset, so the world frame is the scene frame shifted.
  SyntheticPoints points = make_scene(params);
  for (auto& p : points) {
    p.head<3>() += params.world_offset;
  }
  return points;
}

void write_synthetic_pcd(const std::string& path, const SyntheticPoints& points, PcdEncoding encoding, bool double_xyz) {
  std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
  const int size = double_xyz ? 8 : 4;
  ofs << "# .PCD v0.7 - synthetic test data\n"
      << "VERSION 0.7\n"
      << "FIELDS x y z intensity\n"
      << "SIZE " << size << " " << size << " " << size << " 4\n"
      << "TYPE F F F F\n"
      << "COUNT 1 1 1 1\n"
      << "WIDTH " << points.size() << "\n"
      << "HEIGHT 1\n"
      << "VIEWPOINT 0 0 0 1 0 0 0\n"
      << "POINTS " << points.size() << "\n"
      << "DATA " << (encoding == PcdEncoding::Ascii ? "ascii" : "binary") << "\n";

  for (size_t i = 0; i < points.size(); i++) {
    const float intensity = static_cast<float>(i % 256);
    if (encoding == PcdEncoding::Ascii) {
      ofs << (boost::format("%.17g %.17g %.17g %g\n") % points[i].x() % points[i].y() % points[i].z() % intensity).str();
      continue;
    }
    for (int axis = 0; axis < 3; axis++) {
      if (double_xyz) {
        const double value = points[i][axis];
        ofs.write(reinterpret_cast<const char*>(&value), sizeof(value));
      } else {
        const float value = static_cast<float>(points[i][axis]);
        ofs.write(reinterpret_cast<const char*>(&value), sizeof(value));
      }
    }
    ofs.write(reinterpret_cast<const char*>(&intensity), sizeof(intensity));
  }
  if (!ofs) {
    throw std::runtime_error("failed to write " + path);
  }
}

SyntheticMap write_synthetic_dump(const std::string& path, const SyntheticMapParams& params) {
  if (params.num_submaps <= 0 || params.point_spacing <= 0.0) {
    throw std::invalid_argument("invalid synthetic map parameters");
  }
  std::filesystem::create_directories(path);

  const PointVector scene = make_scene(params);
  const double sq_radius = params.submap_radius * params.submap_radius;

  SyntheticMap map;
  gtsam::Values values;
  for (int i = 0; i < params.num_submaps; i++) {
    const Eigen::Isometry3d T_scene_origin = origin_pose(params, i);
    Eigen::Isometry3d T_world_origin = T_scene_origin;
    T_world_origin.translation() += params.world_offset;

    const Eigen::Isometry3d T_origin_scene = T_scene_origin.inverse();
    PointVector local;
    for (const auto& p : scene) {
      if ((p.head<3>() - T_scene_origin.translation()).squaredNorm() < sq_radius) {
        local.emplace_back(T_origin_scene * p);
      }
    }

    auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(local);
    cloud->add_covs(gtsam_points::estimate_covariances(cloud->points, cloud->size()));

    glim::SubMap submap;
    submap.id = i;
    submap.T_world_origin = T_world_origin;
    submap.T_origin_endpoint_L.setIdentity();
    submap.T_origin_endpoint_R.setIdentity();
    submap.frame = cloud;
    // SubMap::load always parses per-frame fields, so one origin frame is required.
    const auto frame = make_origin_frame(i, T_world_origin);
    submap.frames = {frame};
    submap.odom_frames = {frame};
    submap.save((boost::format("%s/%06d") % path % i).str());

    using gtsam::symbol_shorthand::B;
    using gtsam::symbol_shorthand::E;
    using gtsam::symbol_shorthand::V;
    using gtsam::symbol_shorthand::X;
    const gtsam::Pose3 pose(T_world_origin.matrix());
    values.insert(X(i), pose);
    for (int side = 0; side < 2; side++) {
      values.insert(E(2 * i + side), pose);
      values.insert(V(2 * i + side), gtsam::Vector3(gtsam::Vector3::Zero()));
      values.insert(B(2 * i + side), gtsam::imuBias::ConstantBias());
    }

    SyntheticSubmapTruth truth;
    truth.id = i;
    truth.T_world_origin = T_world_origin;
    truth.num_points = cloud->size();
    map.submaps.push_back(truth);
  }

  gtsam::serializeToBinaryFile(values, path + "/values.bin");

  std::ofstream graph(path + "/graph.txt");
  graph << "num_submaps: " << params.num_submaps << "\n"
        << "num_all_frames: " << params.num_submaps << "\n"
        << "num_matching_cost_factors: 0\n";
  if (!graph) {
    throw std::runtime_error("failed to write graph.txt");
  }
  return map;
}

}  // namespace glim_localization::test
