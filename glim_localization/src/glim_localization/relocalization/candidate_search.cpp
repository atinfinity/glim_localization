#include <glim_localization/relocalization/candidate_search.hpp>

#include <algorithm>
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

double wrap_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

CandidateScore score_candidate(
  const MapManager& map,
  const gtsam_points::PointCloud::ConstPtr& frame,
  const PoseCandidate& candidate,
  const CandidateScorerParams& params,
  std::optional<std::chrono::steady_clock::time_point> deadline) {
  CandidateScore result;
  result.candidate = candidate;
  result.T_world_imu = candidate.T_world_imu;
  result.yaw_offset = candidate.yaw_offset;

  MapFactors factors = create_map_factors(map, kPoseKey, frame, candidate.T_world_imu, params.map_factors);
  if (factors.tiles.empty()) {
    return result;
  }

  gtsam::Values values;
  values.insert(kPoseKey, gtsam::Pose3(candidate.T_world_imu.matrix()));
  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(params.max_iterations);
  bool interrupted = false;
  const gtsam_points::LevenbergMarquardtOptimizerExt* running = nullptr;
  double previous_error = 0.0;
  if (deadline) {
    // Checked after every iteration, so one registration overruns the deadline by at most one iteration. The deadline
    // interrupts only a registration that the optimizer would have continued: one that converged or reached
    // max_iterations on that same iteration is complete (the same tests as LevenbergMarquardtOptimizerExt::optimize).
    lm_params.termination_criteria = [&](const gtsam::Values&) {
      const double error = running->error();
      const double delta = std::abs(previous_error - error);
      const bool stops_anyway = running->iterations() > lm_params.maxIterations || !std::isfinite(error) || delta < lm_params.absoluteErrorTol ||
                                delta / std::abs(previous_error) < lm_params.relativeErrorTol || error < lm_params.errorTol;
      previous_error = error;
      if (std::chrono::steady_clock::now() < *deadline) {
        return false;
      }
      interrupted = !stops_anyway;
      return true;
    };
  }
  gtsam_points::LevenbergMarquardtOptimizerExt optimizer(factors.factors, values, lm_params);
  running = &optimizer;
  previous_error = optimizer.error();
  values = optimizer.optimize();
  if (interrupted) {
    result.completed = false;
    return result;  // Score 0: a registration that did not finish is not a hypothesis
  }

  Eigen::Isometry3d T(values.at<gtsam::Pose3>(kPoseKey).matrix());
  T.linear() = Eigen::Quaterniond(T.linear()).normalized().toRotationMatrix();
  if (!T.matrix().allFinite()) {
    return result;  // Score 0: a failed solve is not a hypothesis
  }
  result.T_world_imu = T;
  // Yaw added by the registration, about world z (the column of the world x axis seen in the rotated frame). It also
  // contains any roll and pitch change, so it is approximate; it is reported, not applied (the pose is).
  const Eigen::Matrix3d turn = T.linear() * candidate.T_world_imu.linear().transpose();
  const double added = std::atan2(turn(1, 0), turn(0, 0));
  result.yaw_offset = wrap_angle(candidate.yaw_offset + added);
  // The registration may move the pose by meters: score against the tiles that overlap where it ended, as the
  // odometry does for each estimate, not against those chosen at the candidate pose
  const auto tiles = select_map_tiles(map, frame, T, params.map_factors);
  result.num_tiles = static_cast<int>(tiles.size());
  result.score = map_inlier_ratio(tiles, frame, T);
  result.residual = finest_residual(tiles, frame, T, params.map_factors.num_threads);
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

std::optional<size_t> select_runner_up(const std::vector<CandidateScore, Eigen::aligned_allocator<CandidateScore>>& scores, size_t best, double distinct_yaw) {
  std::optional<size_t> runner_up;
  if (best >= scores.size()) {
    return runner_up;
  }
  for (size_t i = 0; i < scores.size(); i++) {
    const double score = scores[i].score;
    if (i == best || !std::isfinite(score) || std::abs(wrap_angle(scores[i].yaw_offset - scores[best].yaw_offset)) <= distinct_yaw) {
      continue;
    }
    if (!runner_up || score > scores[*runner_up].score) {
      runner_up = i;
    }
  }
  return runner_up;
}

const char* to_string(CandidateSearchOutcome outcome) {
  switch (outcome) {
    case CandidateSearchOutcome::ACCEPTED:
      return "accepted";
    case CandidateSearchOutcome::NO_CANDIDATE:
      return "no candidate";
    case CandidateSearchOutcome::TRUNCATED:
      return "truncated";
    case CandidateSearchOutcome::LOW_SCORE:
      return "low score";
    case CandidateSearchOutcome::AMBIGUOUS:
      return "ambiguous";
    case CandidateSearchOutcome::OUT_OF_RANGE:
      return "out of range";
  }
  return "unknown";
}

CandidateSearchResult search_candidates(
  const MapManager& map,
  const gtsam_points::PointCloud::ConstPtr& frame,
  const PoseCandidates& candidates,
  const CandidateSearchParams& params) {
  const auto begin = std::chrono::steady_clock::now();
  constexpr double kMaxBudget = 1e6;  // Keeps the deadline representable
  // A non-finite budget counts as 0 (std::clamp would pass NaN through)
  const double budget = std::isfinite(params.time_budget) ? std::clamp(params.time_budget, 0.0, kMaxBudget) : 0.0;
  const auto deadline = begin + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(budget));
  const auto elapsed = [&]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count(); };

  std::vector<CandidateScore, Eigen::aligned_allocator<CandidateScore>> scores;
  for (const auto& candidate : candidates) {
    if (!scores.empty() && std::chrono::steady_clock::now() >= deadline) {
      break;
    }
    // The first candidate (the hint) is always scored to the end, so that even a search cut short reports a score;
    // max_iterations bounds it. The others stop inside the registration when the deadline passes.
    auto score = score_candidate(map, frame, candidate, params.scorer, scores.empty() ? std::nullopt : std::make_optional(deadline));
    if (!score.completed) {
      break;  // Stopped by the deadline: not scored
    }
    scores.push_back(std::move(score));
  }
  return decide_candidates(scores, static_cast<int>(candidates.size()), params, elapsed());
}

CandidateSearchResult decide_candidates(
  const std::vector<CandidateScore, Eigen::aligned_allocator<CandidateScore>>& scores,
  const int num_candidates,
  const CandidateSearchParams& params,
  const double elapsed) {
  CandidateSearchResult result;
  result.num_candidates = num_candidates;
  result.num_evaluated = static_cast<int>(scores.size());
  result.elapsed = elapsed;

  // Split by the translation bound; without it every candidate is within. The order is kept, so ties still go to
  // the candidate closer to the hint.
  const bool bounded = params.hint_position && params.max_translation > 0.0;
  std::vector<CandidateScore, Eigen::aligned_allocator<CandidateScore>> within;
  std::vector<CandidateScore, Eigen::aligned_allocator<CandidateScore>> beyond;
  for (CandidateScore score : scores) {
    if (params.hint_position) {
      score.translation = (score.T_world_imu.translation() - *params.hint_position).head<2>().norm();
    }
    (bounded && !(score.translation <= params.max_translation) ? beyond : within).push_back(std::move(score));
  }

  // The best score and its runner-up, whether or not they pass, for the report
  const auto best = select_best_candidate(within, -std::numeric_limits<double>::infinity());
  if (best) {
    result.best = within[*best];
    const auto runner_up = select_runner_up(within, *best, params.distinct_yaw);
    if (runner_up) {
      result.runner_up = within[*runner_up];
    }
  }
  const auto out_of_range = select_best_candidate(beyond, -std::numeric_limits<double>::infinity());
  if (out_of_range) {
    result.out_of_range = beyond[*out_of_range];
  }
  const double runner_up_score = result.runner_up ? result.runner_up->score : 0.0;

  // A truncated search is rejected even if its best candidate passes: the unscored candidates may score higher, and
  // the runner-up is only known over the whole set (a wrong yaw near the hint often reaches 0.5 on its own).
  // A match beyond the bound is checked before the score: when it is the best match, the given pose is further off
  // than stated, which is the reason to report, not the low score of what is left within the bound.
  if (num_candidates <= 0) {
    result.outcome = CandidateSearchOutcome::NO_CANDIDATE;
  } else if (result.truncated()) {
    result.outcome = CandidateSearchOutcome::TRUNCATED;
  } else if (result.out_of_range && result.out_of_range->score >= params.min_score && (!result.best || !(result.best->score > result.out_of_range->score))) {
    result.outcome = CandidateSearchOutcome::OUT_OF_RANGE;
  } else if (!result.best || !(result.best->score >= params.min_score)) {
    result.outcome = CandidateSearchOutcome::LOW_SCORE;
  } else if (params.min_score_gap >= 0.0 && result.runner_up && !(result.best->score - runner_up_score > params.min_score_gap)) {
    result.outcome = CandidateSearchOutcome::AMBIGUOUS;
  } else {
    result.outcome = CandidateSearchOutcome::ACCEPTED;
  }
  result.accepted = result.outcome == CandidateSearchOutcome::ACCEPTED;

  constexpr double kToDeg = 180.0 / M_PI;
  const double best_score = result.best ? result.best->score : 0.0;
  const double best_yaw = result.best ? result.best->yaw_offset * kToDeg : 0.0;
  const double runner_up_yaw = result.runner_up ? result.runner_up->yaw_offset * kToDeg : 0.0;
  char counts[96];
  std::snprintf(counts, sizeof(counts), "%d of %d candidates in %.3f s", result.num_evaluated, result.num_candidates, result.elapsed);
  char text[384];
  switch (result.outcome) {
    case CandidateSearchOutcome::NO_CANDIDATE:
      std::snprintf(text, sizeof(text), "no candidate to score");
      break;
    case CandidateSearchOutcome::TRUNCATED:
      std::snprintf(text, sizeof(text), "the time budget of %.3f s ran out after %s (best inlier ratio %.3f at a yaw correction of %.1f deg)", params.time_budget, counts, best_score, best_yaw);
      break;
    case CandidateSearchOutcome::LOW_SCORE:
      std::snprintf(text, sizeof(text), "best inlier ratio %.3f (yaw correction %.1f deg) is below %.3f (%s)", best_score, best_yaw, params.min_score, counts);
      break;
    case CandidateSearchOutcome::AMBIGUOUS:
      std::snprintf(
        text, sizeof(text), "ambiguous: inlier ratio %.3f at a yaw correction of %.1f deg and %.3f at %.1f deg differ by no more than %.3f (%s)", best_score, best_yaw, runner_up_score,
        runner_up_yaw, params.min_score_gap, counts);
      break;
    case CandidateSearchOutcome::OUT_OF_RANGE:
      std::snprintf(
        text, sizeof(text), "out of range: inlier ratio %.3f at a yaw correction of %.1f deg lies %.2f m from the given pose, beyond %.2f m (best within %.3f; %s)",
        result.out_of_range->score, result.out_of_range->yaw_offset * kToDeg, result.out_of_range->translation, params.max_translation, best_score, counts);
      break;
    case CandidateSearchOutcome::ACCEPTED:
      std::snprintf(
        text, sizeof(text), "accepted a yaw correction of %.1f deg with inlier ratio %.3f (runner-up %.3f at %.1f deg; %s)", best_yaw, best_score, runner_up_score, runner_up_yaw,
        counts);
      break;
  }
  result.message = text;
  return result;
}

}  // namespace glim_localization
