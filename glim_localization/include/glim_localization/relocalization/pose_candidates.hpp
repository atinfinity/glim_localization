#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam_points/types/point_cloud.hpp>

#include <glim_localization/map/map_manager.hpp>

namespace glim_localization {

/// A pose hypothesis for the first frame after an initial pose, to be scored against the map.
struct PoseCandidate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
  /// Yaw of this candidate relative to the hint, about the world z axis [rad]. 0 for a generator without a hint.
  double yaw_offset = 0.0;
};

using PoseCandidates = std::vector<PoseCandidate, Eigen::aligned_allocator<PoseCandidate>>;

/// What a candidate generator may use. A local generator (yaw search) needs the hint; a global one (for example
/// place recognition or feature matching, not implemented) would use the frame and the map instead.
struct CandidateQuery {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::optional<Eigen::Isometry3d> hint;     ///< The given pose (/initialpose, ~/initialize, the configured pose)
  gtsam_points::PointCloud::ConstPtr frame;  ///< The first frame in the IMU frame, with covariances
  std::shared_ptr<const MapManager> map;     ///< The map the candidates are scored against
};

/// Produces pose candidates. Candidates are scored in the returned order, and a time budget may stop the scoring
/// early, so the most likely candidate comes first.
class PoseCandidateGenerator {
public:
  virtual ~PoseCandidateGenerator() = default;

  virtual const char* name() const = 0;

  /// @return the candidates, most likely first; empty if the query lacks what the generator needs
  virtual PoseCandidates generate(const CandidateQuery& query) const = 0;
};

/// Yaw offsets in [-range, range] at multiples of step, ordered by distance from 0 (0, +step, -step, +2 step, ...).
/// With a range of 180 deg or more the offset +180 deg is kept and -180 deg is dropped (the same yaw).
/// @param range  Half width [rad], at least 0
/// @param step   Step [rad], positive
/// @throw std::invalid_argument for a negative or non-finite range or a step that is not positive and finite
std::vector<double> yaw_search_offsets(double range, double step);

/// The hint rotated about the world z axis through its own position by each of yaw_search_offsets(range, step).
/// Roll, pitch and the position are those of the hint.
class YawCandidateGenerator : public PoseCandidateGenerator {
public:
  /// @throw std::invalid_argument as yaw_search_offsets
  YawCandidateGenerator(double range, double step);

  const char* name() const override { return "yaw"; }
  PoseCandidates generate(const CandidateQuery& query) const override;

private:
  std::vector<double> offsets_;
};

}  // namespace glim_localization
