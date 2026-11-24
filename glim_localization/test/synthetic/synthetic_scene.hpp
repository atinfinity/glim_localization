#pragma once

#include <vector>

#include <Eigen/Core>

#include "synthetic_map.hpp"

namespace glim_localization::test {

/// Axis-aligned box in the scene frame. A zero extent along one axis is a rectangle.
struct SceneBox {
  Eigen::Vector3d min;
  Eigen::Vector3d max;
};

/// Analytic geometry of the corridor sampled by write_synthetic_dump (floor, walls, pillars, boxes).
std::vector<SceneBox> make_scene_boxes(const SyntheticMapParams& params = SyntheticMapParams());

}  // namespace glim_localization::test
