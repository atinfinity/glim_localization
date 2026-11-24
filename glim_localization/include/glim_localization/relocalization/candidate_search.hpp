#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam_points/types/point_cloud.hpp>

#include <glim_localization/map/map_manager.hpp>
#include <glim_localization/odometry/map_factor_builder.hpp>
#include <glim_localization/relocalization/pose_candidates.hpp>

namespace glim_localization {

struct CandidateScorerParams {
  MapFactorParams map_factors;  ///< Tile search around each candidate, as for the map factors
  int max_iterations = 10;      ///< Levenberg-Marquardt iterations of the scan-to-map registration
};

/// A candidate after scan-to-map registration.
struct CandidateScore {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PoseCandidate candidate;
  Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();  ///< The registered pose
  /// Yaw of the registered pose relative to the hint [rad], wrapped to [-pi, pi]: candidate.yaw_offset plus the yaw
  /// the registration added. Neighboring candidates often converge to the same pose, so this, not the candidate's
  /// offset, is the correction that was applied.
  double yaw_offset = 0.0;
  double score = 0.0;     ///< Inlier ratio at the registered pose (map_inlier_ratio), in [0, 1]
  double residual = 0.0;  ///< Finest-level error per inlier point at the registered pose. Reported only
  int num_tiles = 0;      ///< Tiles that passed the overlap filter at the candidate pose
};

/// Register the frame to the map from the candidate with unary VGICP factors on every voxel level of the tiles that
/// overlap at the candidate pose (the same factors as the odometry), then score the registered pose by its inlier ratio.
/// A candidate with no overlapping tile scores 0 and keeps its pose.
CandidateScore score_candidate(
  const MapManager& map,
  const gtsam_points::PointCloud::ConstPtr& frame,
  const PoseCandidate& candidate,
  const CandidateScorerParams& params);

/// Index of the highest score that is at least min_score; ties go to the earlier candidate (closer to the hint).
/// Non-finite scores never pass. Empty if none passes.
std::optional<size_t> select_best_candidate(const std::vector<CandidateScore, Eigen::aligned_allocator<CandidateScore>>& scores, double min_score);

struct CandidateSearchParams {
  CandidateScorerParams scorer;
  double min_score = 0.5;    ///< The best candidate must reach this inlier ratio
  double time_budget = 2.0;  ///< Seconds. No candidate is started after it; the first one is always scored
};

struct CandidateSearchResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool accepted = false;
  std::optional<CandidateScore> best;  ///< The highest score, also when it is below min_score
  int num_candidates = 0;              ///< Generated
  int num_evaluated = 0;               ///< Scored before the time budget ran out
  double elapsed = 0.0;                ///< Seconds
  std::string message;                 ///< Why it was rejected, or a summary of the accepted candidate
};

/// Score the candidates in order until the time budget runs out and select the best one.
CandidateSearchResult search_candidates(
  const MapManager& map,
  const gtsam_points::PointCloud::ConstPtr& frame,
  const PoseCandidates& candidates,
  const CandidateSearchParams& params);

}  // namespace glim_localization
