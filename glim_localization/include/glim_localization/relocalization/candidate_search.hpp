#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
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
  int num_tiles = 0;      ///< Tiles selected again at the registered pose and scored against
  bool completed = true;  ///< False if the deadline stopped the registration (the score is then 0)
};

/// Register the frame to the map from the candidate with unary VGICP factors on every voxel level of the tiles that
/// overlap at the candidate pose (the same factors as the odometry). The tiles are then selected again at the
/// registered pose, and the registered pose is scored by its inlier ratio against them. A candidate with no
/// overlapping tile scores 0 and keeps its pose.
/// @param deadline  If set, the registration stops at the first iteration that ends after it, and the candidate is
///                  not completed (score 0)
CandidateScore score_candidate(
  const MapManager& map,
  const gtsam_points::PointCloud::ConstPtr& frame,
  const PoseCandidate& candidate,
  const CandidateScorerParams& params,
  std::optional<std::chrono::steady_clock::time_point> deadline = std::nullopt);

/// Index of the highest score that is at least min_score; ties go to the earlier candidate (closer to the hint).
/// Non-finite scores never pass. Empty if none passes.
std::optional<size_t> select_best_candidate(const std::vector<CandidateScore, Eigen::aligned_allocator<CandidateScore>>& scores, double min_score);

/// Index of the highest finite score among the candidates whose registered yaw differs from that of scores[best] by
/// more than distinct_yaw (wrapped); ties go to the earlier candidate. Empty if there is none.
std::optional<size_t> select_runner_up(const std::vector<CandidateScore, Eigen::aligned_allocator<CandidateScore>>& scores, size_t best, double distinct_yaw);

/// Yaw separation [rad] beyond which two registered poses of a yaw search with this step are different hypotheses:
/// two steps, capped at 90 deg so that a coarse step (90 deg or more) still has candidates far enough apart for the
/// ambiguity check.
inline double distinct_yaw_for_step(double step) {
  return std::min(2.0 * step, 0.5 * M_PI);
}

struct CandidateSearchParams {
  CandidateScorerParams scorer;
  double min_score = 0.7;  ///< The best candidate must reach this inlier ratio
  /// The best score must exceed the runner-up (the best candidate at a clearly different yaw) by more than this, or the
  /// search is ambiguous. 0 accepts any strictly better best; a negative value disables the check.
  double min_score_gap = 0.05;
  double distinct_yaw = 30.0 * M_PI / 180.0;  ///< Registered yaws further apart than this [rad] are different hypotheses
  /// Seconds. Every candidate must be scored within it, or the search is rejected as truncated. It is checked before
  /// each candidate and at each registration iteration. The first candidate is always started.
  double time_budget = 10.0;
};

/// Why a search accepted or rejected the given pose.
enum class CandidateSearchOutcome {
  ACCEPTED,
  NO_CANDIDATE,  ///< The generator returned nothing to score
  TRUNCATED,     ///< The time budget ran out before every candidate was scored: an unscored one could be better
  LOW_SCORE,     ///< The best score is below min_score
  AMBIGUOUS      ///< A candidate at a clearly different yaw scores within min_score_gap of the best
};

const char* to_string(CandidateSearchOutcome outcome);

struct CandidateSearchResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool accepted = false;
  CandidateSearchOutcome outcome = CandidateSearchOutcome::NO_CANDIDATE;
  std::optional<CandidateScore> best;       ///< The highest score among the scored candidates, also when rejected
  std::optional<CandidateScore> runner_up;  ///< The highest score at a yaw more than distinct_yaw from the best
  int num_candidates = 0;                   ///< Generated
  int num_evaluated = 0;                    ///< Completely scored before the time budget ran out
  double elapsed = 0.0;                     ///< Seconds
  std::string message;                      ///< Why it was rejected, or a summary of the accepted candidate

  bool truncated() const { return num_evaluated < num_candidates; }
};

/// Score the candidates in order and accept the best one only if every candidate was scored within the time budget,
/// the best reaches min_score, and no candidate at a clearly different yaw comes within min_score_gap of it.
CandidateSearchResult search_candidates(
  const MapManager& map,
  const gtsam_points::PointCloud::ConstPtr& frame,
  const PoseCandidates& candidates,
  const CandidateSearchParams& params);

}  // namespace glim_localization
