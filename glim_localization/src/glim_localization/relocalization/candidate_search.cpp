#include <glim_localization/relocalization/candidate_search.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

namespace glim_localization {

namespace {

constexpr gtsam::Key kPoseKey = 0;

double finest_residual(const std::vector<MapTile::ConstPtr>& tiles, const gtsam_points::PointCloud::ConstPtr& frame, const Eigen::Isometry3d& T_world_imu, int num_threads) {
  gtsam::Values values;
  values.insert(kPoseKey, gtsam::Pose3(T_world_imu.matrix()));
  double error = 0.0;
  int num_inliers = 0;
  for (const auto& tile : tiles) {
    gtsam_points::IntegratedVGICPFactor factor(gtsam::Pose3(tile->T_world_origin.matrix()), kPoseKey, tile->voxelmaps.front(), frame);
    factor.set_num_threads(num_threads);
    factor.linearize(values);
    error += factor.error(values);
    num_inliers += factor.num_inliers();
  }
  return num_inliers > 0 ? error / num_inliers : 0.0;
}

}  // namespace

CandidateScore score_candidate(
  const MapManager& map,
  const gtsam_points::PointCloud::ConstPtr& frame,
  const PoseCandidate& candidate,
  const CandidateScorerParams& params) {
  CandidateScore result;
  result.candidate = candidate;
  result.T_world_imu = candidate.T_world_imu;
  result.yaw_offset = candidate.yaw_offset;

  MapFactors factors = create_map_factors(map, kPoseKey, frame, candidate.T_world_imu, params.map_factors);
  result.num_tiles = static_cast<int>(factors.tiles.size());
  if (factors.tiles.empty()) {
    return result;
  }

  gtsam::Values values;
  values.insert(kPoseKey, gtsam::Pose3(candidate.T_world_imu.matrix()));
  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(params.max_iterations);
  gtsam_points::LevenbergMarquardtOptimizerExt optimizer(factors.factors, values, lm_params);
  values = optimizer.optimize();

  Eigen::Isometry3d T(values.at<gtsam::Pose3>(kPoseKey).matrix());
  T.linear() = Eigen::Quaterniond(T.linear()).normalized().toRotationMatrix();
  if (!T.matrix().allFinite()) {
    return result;  // Score 0: a failed solve is not a hypothesis
  }
  result.T_world_imu = T;
  // Yaw added by the registration, about world z (the column of the world x axis seen in the rotated frame)
  const Eigen::Matrix3d turn = T.linear() * candidate.T_world_imu.linear().transpose();
  const double added = std::atan2(turn(1, 0), turn(0, 0));
  result.yaw_offset = std::atan2(std::sin(candidate.yaw_offset + added), std::cos(candidate.yaw_offset + added));
  result.score = map_inlier_ratio(factors.tiles, frame, T);
  result.residual = finest_residual(factors.tiles, frame, T, params.map_factors.num_threads);
  return result;
}

std::optional<size_t> select_best_candidate(const std::vector<CandidateScore, Eigen::aligned_allocator<CandidateScore>>& scores, double min_score) {
  std::optional<size_t> best;
  for (size_t i = 0; i < scores.size(); i++) {
    const double score = scores[i].score;
    if (!std::isfinite(score) || score < min_score) {
      continue;
    }
    if (!best || score > scores[*best].score) {
      best = i;
    }
  }
  return best;
}

CandidateSearchResult search_candidates(
  const MapManager& map,
  const gtsam_points::PointCloud::ConstPtr& frame,
  const PoseCandidates& candidates,
  const CandidateSearchParams& params) {
  const auto begin = std::chrono::steady_clock::now();
  const auto elapsed = [&]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count(); };

  CandidateSearchResult result;
  result.num_candidates = static_cast<int>(candidates.size());
  std::vector<CandidateScore, Eigen::aligned_allocator<CandidateScore>> scores;
  for (const auto& candidate : candidates) {
    if (!scores.empty() && elapsed() >= params.time_budget) {
      break;
    }
    scores.push_back(score_candidate(map, frame, candidate, params.scorer));
  }
  result.num_evaluated = static_cast<int>(scores.size());

  // The best score, whether or not it passes, for the report
  const auto best_any = select_best_candidate(scores, -std::numeric_limits<double>::infinity());
  if (best_any) {
    result.best = scores[*best_any];
  }
  const auto best = select_best_candidate(scores, params.min_score);
  result.accepted = best.has_value();
  result.elapsed = elapsed();

  char text[256];
  if (scores.empty()) {
    std::snprintf(text, sizeof(text), "no candidate to score");
  } else if (result.accepted) {
    std::snprintf(
      text, sizeof(text), "accepted a yaw correction of %.1f deg with inlier ratio %.3f (%d of %d candidates in %.3f s)", result.best->yaw_offset * 180.0 / M_PI,
      result.best->score, result.num_evaluated, result.num_candidates, result.elapsed);
  } else {
    std::snprintf(
      text, sizeof(text), "best inlier ratio %.3f (yaw correction %.1f deg) is below %.3f (%d of %d candidates in %.3f s)", result.best ? result.best->score : 0.0,
      result.best ? result.best->yaw_offset * 180.0 / M_PI : 0.0, params.min_score, result.num_evaluated, result.num_candidates, result.elapsed);
  }
  result.message = text;
  return result;
}

}  // namespace glim_localization
