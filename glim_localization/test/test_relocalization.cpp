// Unit tests of the initial pose search: candidate generation, scoring and selection.
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

#include <glim/common/cloud_covariance_estimation.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>

#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim_localization/map/map_io.hpp>
#include <glim_localization/map/map_manager.hpp>
#include <glim_localization/map/pcd_map.hpp>
#include <glim_localization/relocalization/candidate_search.hpp>
#include <glim_localization/relocalization/pose_candidates.hpp>

#include "synthetic/synthetic_map.hpp"
#include "synthetic/synthetic_replay.hpp"
#include "synthetic/synthetic_scene.hpp"
#include "synthetic/synthetic_sensor.hpp"

namespace fs = std::filesystem;
using namespace glim_localization;
using namespace glim_localization::test;

namespace {

constexpr double kDeg = M_PI / 180.0;

double yaw_of(const Eigen::Isometry3d& T) {
  const Eigen::Vector3d x = T.linear().col(0);
  return std::atan2(x.y(), x.x());
}

double wrap(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double rotation_error_deg(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle() / kDeg;
}

Eigen::Isometry3d with_yaw_error(const Eigen::Isometry3d& T, double yaw) {
  Eigen::Isometry3d result = T;
  result.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * T.linear();
  return result;
}

std::vector<double> to_deg(const std::vector<double>& radians) {
  std::vector<double> degrees;
  for (const double r : radians) {
    degrees.push_back(std::round(r / kDeg * 1e6) / 1e6);
  }
  return degrees;
}

using Scores = std::vector<CandidateScore, Eigen::aligned_allocator<CandidateScore>>;

CandidateScore scored(double score, double yaw_offset) {
  CandidateScore result;
  result.score = score;
  result.candidate.yaw_offset = yaw_offset;
  return result;
}

}  // namespace

TEST(YawSearchOffsets, OrderedByDistanceFromZero) {
  EXPECT_EQ(to_deg(yaw_search_offsets(30.0 * kDeg, 15.0 * kDeg)), (std::vector<double>{0.0, 15.0, -15.0, 30.0, -30.0}));
  // A range that is not a multiple of the step stops at the last multiple inside it
  EXPECT_EQ(to_deg(yaw_search_offsets(20.0 * kDeg, 15.0 * kDeg)), (std::vector<double>{0.0, 15.0, -15.0}));
  EXPECT_EQ(to_deg(yaw_search_offsets(0.0, 15.0 * kDeg)), (std::vector<double>{0.0}));
}

TEST(YawSearchOffsets, FullTurnHasNoDuplicate) {
  // +180 and -180 deg are the same yaw: only +180 is kept, and a range beyond 180 deg adds nothing
  EXPECT_EQ(to_deg(yaw_search_offsets(M_PI, 90.0 * kDeg)), (std::vector<double>{0.0, 90.0, -90.0, 180.0}));
  EXPECT_EQ(to_deg(yaw_search_offsets(4.0, 90.0 * kDeg)), (std::vector<double>{0.0, 90.0, -90.0, 180.0}));
  EXPECT_EQ(yaw_search_offsets(M_PI, 15.0 * kDeg).size(), 24u);
}

TEST(YawSearchOffsets, RejectsInvalidParameters) {
  EXPECT_THROW(yaw_search_offsets(-1.0, 0.1), std::invalid_argument);
  EXPECT_THROW(yaw_search_offsets(std::nan(""), 0.1), std::invalid_argument);
  EXPECT_THROW(yaw_search_offsets(1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(yaw_search_offsets(1.0, -0.1), std::invalid_argument);
  EXPECT_THROW(yaw_search_offsets(1.0, std::numeric_limits<double>::infinity()), std::invalid_argument);
  EXPECT_THROW(YawCandidateGenerator(1.0, 0.0), std::invalid_argument);
  // A step below 0.5 deg (for example radians given as degrees) would explode the candidate count
  EXPECT_THROW(yaw_search_offsets(M_PI, 0.4 * kDeg), std::invalid_argument);
  EXPECT_THROW(yaw_search_offsets(M_PI, 15.0 * kDeg * kDeg), std::invalid_argument);
  EXPECT_EQ(yaw_search_offsets(M_PI, kMinYawSearchStep).size(), 720u) << "the largest candidate set";
}

TEST(YawCandidateGenerator, RotatesTheHintAboutWorldZ) {
  YawCandidateGenerator generator(90.0 * kDeg, 45.0 * kDeg);
  EXPECT_STREQ(generator.name(), "yaw");

  CandidateQuery query;
  EXPECT_TRUE(generator.generate(query).empty()) << "no hint, no candidates";

  Eigen::Isometry3d hint = Eigen::Isometry3d::Identity();
  hint.translation() = Eigen::Vector3d(3.0, -2.0, 1.5);
  hint.linear() = (Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(-0.03, Eigen::Vector3d::UnitX()))
                    .toRotationMatrix();
  query.hint = hint;
  const auto candidates = generator.generate(query);
  ASSERT_EQ(candidates.size(), 5u);
  EXPECT_DOUBLE_EQ(candidates.front().yaw_offset, 0.0);
  EXPECT_TRUE(candidates.front().T_world_imu.isApprox(hint));
  for (const auto& candidate : candidates) {
    // Same position, same gravity direction (roll and pitch), yaw turned by the offset
    EXPECT_TRUE(candidate.T_world_imu.translation().isApprox(hint.translation()));
    EXPECT_NEAR((candidate.T_world_imu.linear().transpose() * Eigen::Vector3d::UnitZ() - hint.linear().transpose() * Eigen::Vector3d::UnitZ()).norm(), 0.0, 1e-12);
    EXPECT_NEAR(wrap(yaw_of(candidate.T_world_imu) - yaw_of(hint) - candidate.yaw_offset), 0.0, 1e-9);
  }
}

TEST(SelectBestCandidate, HighestPassingScoreEarliestOnTies) {
  EXPECT_FALSE(select_best_candidate(Scores(), 0.0));

  Scores scores = {scored(0.4, 0.0), scored(0.9, 0.1), scored(0.9, -0.1), scored(0.6, 0.2)};
  ASSERT_TRUE(select_best_candidate(scores, 0.5));
  EXPECT_EQ(*select_best_candidate(scores, 0.5), 1u);
  EXPECT_FALSE(select_best_candidate(scores, 0.95));

  scores = {scored(std::nan(""), 0.0), scored(0.55, 0.1)};
  ASSERT_TRUE(select_best_candidate(scores, 0.5));
  EXPECT_EQ(*select_best_candidate(scores, 0.5), 1u) << "NaN never wins";
  EXPECT_FALSE(select_best_candidate({scored(std::nan(""), 0.0)}, -1.0));
}

// A score whose registered pose ended at (x, 0, z), against a hint at the origin
CandidateScore scored_at(double score, double yaw_offset, double x, double z = 0.0) {
  CandidateScore result = scored(score, yaw_offset);
  result.yaw_offset = yaw_offset;
  result.T_world_imu.translation() = Eigen::Vector3d(x, 0.0, z);
  return result;
}

CandidateSearchParams bounded_params(double max_translation) {
  CandidateSearchParams params;
  params.distinct_yaw = 30.0 * kDeg;
  params.hint_position = Eigen::Vector3d::Zero();
  params.max_translation = max_translation;
  return params;
}

TEST(DecideCandidates, ZeroBoundKeepsTheUnboundedDecision) {
  // A match far from the hint is accepted as before; the distance is still reported
  const Scores scores = {scored_at(0.5, 0.0, 0.0), scored_at(0.95, 90.0 * kDeg, 3.0), scored_at(0.6, 180.0 * kDeg, 0.0)};
  for (const bool with_hint : {false, true}) {
    auto params = bounded_params(0.0);
    if (!with_hint) {
      params.hint_position.reset();
    }
    const auto result = decide_candidates(scores, 3, params);
    EXPECT_EQ(result.outcome, CandidateSearchOutcome::ACCEPTED) << to_string(result.outcome);
    ASSERT_TRUE(result.best);
    EXPECT_EQ(result.best->score, 0.95);
    EXPECT_NEAR(result.best->translation, with_hint ? 3.0 : 0.0, 1e-12);
    EXPECT_FALSE(result.out_of_range);
  }
  // A bound without a hint position does nothing
  auto params = bounded_params(1.0);
  params.hint_position.reset();
  EXPECT_EQ(decide_candidates(scores, 3, params).outcome, CandidateSearchOutcome::ACCEPTED);
}

TEST(DecideCandidates, LowerMatchBeyondTheBoundIsIgnored) {
  const Scores scores = {scored_at(0.95, 0.0, 0.4), scored_at(0.8, 180.0 * kDeg, 3.0)};
  const auto result = decide_candidates(scores, 2, bounded_params(2.5));
  EXPECT_EQ(result.outcome, CandidateSearchOutcome::ACCEPTED) << result.message;
  ASSERT_TRUE(result.out_of_range);
  EXPECT_EQ(result.out_of_range->score, 0.8);
  EXPECT_FALSE(result.runner_up) << "a pose beyond the bound is never the runner-up";
}

TEST(DecideCandidates, BetterMatchBeyondTheBoundRejects) {
  // At or above both the minimum and the best within the bound, including a tie
  for (const double within : {0.9, 0.959, 0.3}) {
    const Scores scores = {scored_at(within, 0.0, 1.0), scored_at(0.959, 180.0 * kDeg, 2.98)};
    const auto result = decide_candidates(scores, 2, bounded_params(2.5));
    EXPECT_EQ(result.outcome, CandidateSearchOutcome::OUT_OF_RANGE) << within << ": " << result.message;
    EXPECT_FALSE(result.accepted);
    ASSERT_TRUE(result.out_of_range);
    EXPECT_NEAR(result.out_of_range->translation, 2.98, 1e-12);
    EXPECT_EQ(result.message.rfind("out of range:", 0), 0u) << result.message;
  }
  // Nothing within the bound at all
  const auto result = decide_candidates({scored_at(0.9, 0.0, 4.0)}, 1, bounded_params(2.5));
  EXPECT_EQ(result.outcome, CandidateSearchOutcome::OUT_OF_RANGE) << result.message;
  EXPECT_FALSE(result.best);
}

TEST(DecideCandidates, MatchBeyondTheBoundBelowTheMinimumFallsThroughToLowScore) {
  const Scores scores = {scored_at(0.3, 0.0, 0.0), scored_at(0.6, 90.0 * kDeg, 5.0)};
  const auto result = decide_candidates(scores, 2, bounded_params(2.5));
  EXPECT_EQ(result.outcome, CandidateSearchOutcome::LOW_SCORE) << result.message;
  // The message names the excluded match, so that it is not hidden behind the low score within the bound
  EXPECT_NE(result.message.find("best inlier ratio 0.300 (yaw correction 0.0 deg) is below 0.700 (beyond 2.50 m: 0.600 at 5.00 m;"), std::string::npos) << result.message;
  const auto nothing_within = decide_candidates({scored_at(0.6, 0.0, 5.0)}, 1, bounded_params(2.5));
  EXPECT_EQ(nothing_within.outcome, CandidateSearchOutcome::LOW_SCORE);
  EXPECT_NE(nothing_within.message.find("(beyond 2.50 m: 0.600 at 5.00 m;"), std::string::npos) << nothing_within.message;
}

TEST(DecideCandidates, AmbiguityOnlyAmongPosesWithinTheBound) {
  auto result = decide_candidates({scored_at(0.95, 0.0, 0.0), scored_at(0.93, 180.0 * kDeg, 1.0)}, 2, bounded_params(2.5));
  EXPECT_EQ(result.outcome, CandidateSearchOutcome::AMBIGUOUS) << result.message;
  // The same runner-up beyond the bound: lower than the best, so ignored
  result = decide_candidates({scored_at(0.95, 0.0, 0.0), scored_at(0.93, 180.0 * kDeg, 3.0)}, 2, bounded_params(2.5));
  EXPECT_EQ(result.outcome, CandidateSearchOutcome::ACCEPTED) << result.message;
}

TEST(DecideCandidates, BoundIsInclusiveAndIgnoresZ) {
  const auto result = decide_candidates({scored_at(0.95, 0.0, 2.5, 10.0)}, 1, bounded_params(2.5));
  EXPECT_EQ(result.outcome, CandidateSearchOutcome::ACCEPTED) << result.message;
  EXPECT_NEAR(result.best->translation, 2.5, 1e-12);
}

TEST(DecideCandidates, TruncationAndNoCandidateComeFirst) {
  const Scores scores = {scored_at(0.9, 0.0, 4.0)};
  EXPECT_EQ(decide_candidates(scores, 24, bounded_params(2.5)).outcome, CandidateSearchOutcome::TRUNCATED);
  EXPECT_EQ(decide_candidates({}, 0, bounded_params(2.5)).outcome, CandidateSearchOutcome::NO_CANDIDATE);
}

// Scoring against the corridor of the localization tests, with one synthetic scan at rest
class CandidateScoringTest : public testing::Test {
protected:
  static void SetUpTestSuite() {
    dir_ = new fs::path(fs::temp_directory_path() / ("glim_loc_relocalization_" + std::to_string(::getpid())));
    fs::remove_all(*dir_);
    const SyntheticMapParams map_params = replay_map_params();
    write_synthetic_dump(dir_->string(), map_params);
    auto map = std::make_shared<MapManager>();
    ASSERT_TRUE(map->load(dir_->string()));
    map_ = new std::shared_ptr<const MapManager>(map);

    glim::CloudPreprocessorParams preprocess_params;
    preprocess_params.global_shutter = true;
    glim::CloudPreprocessor preprocessor(preprocess_params);
    const auto scans = make_synthetic_scans(make_scene_boxes(map_params), 0.5, 0.5);
    ASSERT_EQ(scans.size(), 1u);
    const auto raw = preprocessor.preprocess(scans.front());
    // The LiDAR and IMU frames coincide in the synthetic data
    glim::CloudCovarianceEstimation covariance_estimation(2);
    std::vector<Eigen::Vector4d> normals;
    std::vector<Eigen::Matrix4d> covs;
    covariance_estimation.estimate(raw->points, raw->neighbors, normals, covs);
    auto frame = std::make_shared<gtsam_points::PointCloudCPU>(raw->points);
    frame->add_covs(covs);
    frame_ = new gtsam_points::PointCloud::ConstPtr(frame);
    truth_ = new Eigen::Isometry3d(synthetic_T_world_imu(0.5));
  }

  static void TearDownTestSuite() {
    delete truth_;
    delete frame_;
    delete map_;
    fs::remove_all(*dir_);
    delete dir_;
  }

  static CandidateSearchParams search_params() {
    CandidateSearchParams params;
    params.scorer.map_factors.num_threads = 2;
    params.time_budget = 1e6;  // Not the subject here; a loaded CI machine must not cut the search short (ctest TIMEOUT bounds it)
    return params;
  }

  static fs::path* dir_;
  static std::shared_ptr<const MapManager>* map_;
  static gtsam_points::PointCloud::ConstPtr* frame_;
  static Eigen::Isometry3d* truth_;
};

fs::path* CandidateScoringTest::dir_ = nullptr;
std::shared_ptr<const MapManager>* CandidateScoringTest::map_ = nullptr;
gtsam_points::PointCloud::ConstPtr* CandidateScoringTest::frame_ = nullptr;
Eigen::Isometry3d* CandidateScoringTest::truth_ = nullptr;

TEST_F(CandidateScoringTest, RightYawScoresHighWrongYawLow) {
  const auto params = search_params().scorer;
  PoseCandidate right;
  right.T_world_imu = with_yaw_error(*truth_, 5.0 * kDeg);
  right.T_world_imu.translation() += Eigen::Vector3d(0.3, -0.2, 0.1);
  const auto good = score_candidate(**map_, *frame_, right, params);
  EXPECT_GT(good.num_tiles, 0);
  EXPECT_GT(good.score, 0.8);
  EXPECT_LT((good.T_world_imu.translation() - truth_->translation()).norm(), 0.1);
  EXPECT_LT(rotation_error_deg(good.T_world_imu, *truth_), 1.0);

  PoseCandidate wrong;
  wrong.T_world_imu = with_yaw_error(*truth_, 75.0 * kDeg);
  const auto bad = score_candidate(**map_, *frame_, wrong, params);
  std::cout << "score right=" << good.score << " (residual " << good.residual << "), wrong 75 deg=" << bad.score << " (residual " << bad.residual << ")"
            << std::endl;
  EXPECT_LT(bad.score, good.score - 0.2);
}

TEST_F(CandidateScoringTest, CandidateOffTheMapScoresZero) {
  PoseCandidate off;
  off.T_world_imu = *truth_;
  off.T_world_imu.translation() += Eigen::Vector3d(500.0, 500.0, 0.0);
  const auto result = score_candidate(**map_, *frame_, off, search_params().scorer);
  EXPECT_EQ(result.num_tiles, 0);
  EXPECT_EQ(result.score, 0.0);
  EXPECT_TRUE(result.T_world_imu.isApprox(off.T_world_imu));
}

TEST_F(CandidateScoringTest, SearchRecoversLargeYawError) {
  for (const double error_deg : {60.0, 75.0, 90.0, -80.0}) {
    CandidateQuery query;
    query.hint = with_yaw_error(*truth_, error_deg * kDeg);
    const auto candidates = YawCandidateGenerator(M_PI, 15.0 * kDeg).generate(query);
    const auto result = search_candidates(**map_, *frame_, candidates, search_params());
    ASSERT_TRUE(result.accepted) << error_deg << " deg: " << result.message;
    ASSERT_TRUE(result.best);
    const double translation = (result.best->T_world_imu.translation() - truth_->translation()).norm();
    const double rotation = rotation_error_deg(result.best->T_world_imu, *truth_);
    std::cout << "yaw error " << error_deg << " deg: " << result.message << ", error after registration " << translation << " m, " << rotation << " deg"
              << std::endl;
    EXPECT_EQ(result.num_evaluated, 24);
    EXPECT_FALSE(result.truncated());
    EXPECT_EQ(result.outcome, CandidateSearchOutcome::ACCEPTED);
    EXPECT_NEAR(wrap(result.best->yaw_offset + error_deg * kDeg) / kDeg, 0.0, 1.0);
    EXPECT_LT(translation, 0.1);
    EXPECT_LT(rotation, 1.0);
  }
}

TEST_F(CandidateScoringTest, RejectsWhenNoCandidatePasses) {
  CandidateQuery query;
  query.hint = *truth_;
  query.hint->translation() += Eigen::Vector3d(500.0, 0.0, 0.0);  // Off the map
  const auto candidates = YawCandidateGenerator(M_PI, 90.0 * kDeg).generate(query);
  const auto result = search_candidates(**map_, *frame_, candidates, search_params());
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.num_evaluated, 4);
  ASSERT_TRUE(result.best);
  EXPECT_EQ(result.best->score, 0.0);
  EXPECT_EQ(result.outcome, CandidateSearchOutcome::LOW_SCORE);
  EXPECT_NE(result.message.find("below"), std::string::npos) << result.message;

  // On the map at a wrong yaw, with the hint as the only candidate: the wrong minimum does not reach the default 0.7
  query.hint = with_yaw_error(*truth_, 75.0 * kDeg);
  const auto wrong = search_candidates(**map_, *frame_, YawCandidateGenerator(0.0, 1.0).generate(query), search_params());
  std::cout << "wrong yaw, no search: " << wrong.message << std::endl;
  EXPECT_FALSE(wrong.accepted);
  EXPECT_EQ(wrong.outcome, CandidateSearchOutcome::LOW_SCORE);
  ASSERT_TRUE(wrong.best);
  EXPECT_GT(wrong.best->num_tiles, 0) << "on the map";

  // On the map, but the threshold is out of reach
  query.hint = *truth_;
  auto params = search_params();
  params.min_score = 1.0 + 1e-9;
  const auto strict = search_candidates(**map_, *frame_, YawCandidateGenerator(0.0, 1.0).generate(query), params);
  EXPECT_FALSE(strict.accepted);
  ASSERT_TRUE(strict.best);
  EXPECT_GT(strict.best->score, 0.8) << "the best score is reported also when it is rejected";
}

TEST_F(CandidateScoringTest, TruncatedSearchIsRejected) {
  // 75 deg off: the hint and its neighbors register into wrong minima that still reach an inlier ratio of 0.5.
  // A search that the budget stops there must not accept them.
  CandidateQuery query;
  query.hint = with_yaw_error(*truth_, 75.0 * kDeg);
  const auto candidates = YawCandidateGenerator(M_PI, 15.0 * kDeg).generate(query);
  auto params = search_params();
  params.min_score = 0.5;
  params.time_budget = 0.0;
  const auto result = search_candidates(**map_, *frame_, candidates, params);
  std::cout << "budget 0: " << result.message << std::endl;
  EXPECT_EQ(result.num_candidates, 24);
  EXPECT_EQ(result.num_evaluated, 1) << "the first candidate (the hint) is always scored";
  EXPECT_TRUE(result.truncated());
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.outcome, CandidateSearchOutcome::TRUNCATED);
  ASSERT_TRUE(result.best);
  EXPECT_GT(result.best->score, params.min_score) << "the wrong yaw would pass the threshold on its own";
  EXPECT_GT(rotation_error_deg(result.best->T_world_imu, *truth_), 30.0);
  EXPECT_NE(result.message.find("time budget"), std::string::npos) << result.message;

  // Also when the truncated search found the right yaw: the unscored candidates are unknown
  query.hint = *truth_;
  const auto right = search_candidates(**map_, *frame_, YawCandidateGenerator(M_PI, 15.0 * kDeg).generate(query), params);
  EXPECT_FALSE(right.accepted);
  EXPECT_EQ(right.outcome, CandidateSearchOutcome::TRUNCATED);

  // A deadline that passes inside a registration stops it after the current iteration: not scored
  const auto begin = std::chrono::steady_clock::now();
  const auto stopped = score_candidate(**map_, *frame_, candidates.front(), params.scorer, begin);
  EXPECT_FALSE(stopped.completed);
  EXPECT_EQ(stopped.score, 0.0);
  // Loose: one iteration, not the whole registration (a slow CI runner must not fail it)
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count(), 2.0);
  const auto unbounded = score_candidate(**map_, *frame_, candidates.front(), params.scorer);
  EXPECT_TRUE(unbounded.completed);

  const auto none = search_candidates(**map_, *frame_, PoseCandidates(), search_params());
  EXPECT_FALSE(none.accepted);
  EXPECT_EQ(none.outcome, CandidateSearchOutcome::NO_CANDIDATE);
  EXPECT_EQ(none.num_evaluated, 0);
  EXPECT_FALSE(none.best);

  // A non-finite budget counts as 0: only the first candidate is started, and the search is truncated
  auto nan_budget = search_params();
  nan_budget.time_budget = std::numeric_limits<double>::quiet_NaN();
  const auto nan = search_candidates(**map_, *frame_, candidates, nan_budget);
  EXPECT_FALSE(nan.accepted);
  EXPECT_EQ(nan.outcome, CandidateSearchOutcome::TRUNCATED);
  EXPECT_LE(nan.num_evaluated, 1);
}

TEST(DistinctYawForStep, TwoStepsAtMost90Degrees) {
  EXPECT_NEAR(distinct_yaw_for_step(15.0 * kDeg), 30.0 * kDeg, 1e-12);
  EXPECT_NEAR(distinct_yaw_for_step(45.0 * kDeg), 90.0 * kDeg, 1e-12);
  // A coarse step would otherwise leave no candidate further away than distinct_yaw, so no runner-up
  EXPECT_NEAR(distinct_yaw_for_step(90.0 * kDeg), 90.0 * kDeg, 1e-12);
  EXPECT_NEAR(distinct_yaw_for_step(120.0 * kDeg), 90.0 * kDeg, 1e-12);
}

TEST_F(CandidateScoringTest, SearchReportsTheRunnerUp) {
  // The corridor is nearly symmetric under a half turn: the flipped pose explains most of the scan, but not all of it
  CandidateQuery query;
  query.hint = with_yaw_error(*truth_, 75.0 * kDeg);
  const auto result = search_candidates(**map_, *frame_, YawCandidateGenerator(M_PI, 15.0 * kDeg).generate(query), search_params());
  ASSERT_TRUE(result.accepted) << result.message;
  ASSERT_TRUE(result.runner_up);
  std::cout << "runner-up: " << result.message << std::endl;
  EXPECT_GT(std::abs(wrap(result.runner_up->yaw_offset - result.best->yaw_offset)), 30.0 * kDeg);
  EXPECT_GT(result.best->score - result.runner_up->score, search_params().min_score_gap);

  // A gap larger than the one the corridor gives makes the same search ambiguous
  auto params = search_params();
  params.min_score_gap = result.best->score - result.runner_up->score + 0.01;
  const auto strict = search_candidates(**map_, *frame_, YawCandidateGenerator(M_PI, 15.0 * kDeg).generate(query), params);
  EXPECT_FALSE(strict.accepted);
  EXPECT_EQ(strict.outcome, CandidateSearchOutcome::AMBIGUOUS);
  EXPECT_NE(strict.message.find("ambiguous"), std::string::npos) << strict.message;
}

TEST_F(CandidateScoringTest, DeadlineCheckKeepsTheConvergence) {
  // The deadline goes in through termination_criteria; a deadline far away must not change when the registration stops
  PoseCandidate candidate;
  for (const double error_deg : {5.0, 60.0}) {
    candidate.T_world_imu = with_yaw_error(*truth_, error_deg * kDeg);
    const auto params = search_params().scorer;
    const auto t0 = std::chrono::steady_clock::now();
    const auto plain = score_candidate(**map_, *frame_, candidate, params);
    const auto t1 = std::chrono::steady_clock::now();
    const auto bounded = score_candidate(**map_, *frame_, candidate, params, t1 + std::chrono::hours(1));
    const auto t2 = std::chrono::steady_clock::now();
    std::cout << error_deg << " deg: " << std::chrono::duration<double>(t1 - t0).count() << " s without a deadline, "
              << std::chrono::duration<double>(t2 - t1).count() << " s with one" << std::endl;
    EXPECT_TRUE(bounded.completed);
    EXPECT_NEAR(plain.score, bounded.score, 1e-6);
    EXPECT_TRUE(plain.T_world_imu.isApprox(bounded.T_world_imu, 1e-9));
  }
}

TEST(SelectRunnerUp, OnlyCandidatesAtADifferentYaw) {
  // Registered yaws: 0 (best), 10 deg (the same hypothesis), 180 deg and -90 deg
  Scores scores = {scored(0.95, 0.0), scored(0.94, 0.0), scored(0.6, 0.0), scored(0.7, 0.0)};
  scores[1].yaw_offset = 10.0 * kDeg;
  scores[2].yaw_offset = M_PI;
  scores[3].yaw_offset = -90.0 * kDeg;
  ASSERT_TRUE(select_runner_up(scores, 0, 30.0 * kDeg));
  EXPECT_EQ(*select_runner_up(scores, 0, 30.0 * kDeg), 3u);
  // Wrap-around: 170 deg and -170 deg are 20 deg apart
  scores = {scored(0.95, 0.0), scored(0.9, 0.0)};
  scores[0].yaw_offset = 170.0 * kDeg;
  scores[1].yaw_offset = -170.0 * kDeg;
  EXPECT_FALSE(select_runner_up(scores, 0, 30.0 * kDeg));
  EXPECT_FALSE(select_runner_up(scores, 5, 30.0 * kDeg));
}

// A rectangular room with two pillars placed point-symmetrically about the sensor: a half turn maps the scene onto
// itself, so the true yaw and the flipped one cannot be told apart
class SymmetricSceneTest : public testing::Test {
protected:
  static void SetUpTestSuite() {
    dir_ = new fs::path(fs::temp_directory_path() / ("glim_loc_relocalization_symmetric_" + std::to_string(::getpid())));
    fs::remove_all(*dir_);
    truth_ = new Eigen::Isometry3d(synthetic_T_world_imu(0.5));
    const Eigen::Vector3d c = truth_->translation();
    std::vector<SceneBox> scene;
    const double x0 = c.x() - 7.0, x1 = c.x() + 7.0, y0 = c.y() - 4.5, y1 = c.y() + 4.5, h = 3.0;
    scene.push_back({Eigen::Vector3d(x0, y0, 0.0), Eigen::Vector3d(x1, y1, 0.0)});
    scene.push_back({Eigen::Vector3d(x0, y0, 0.0), Eigen::Vector3d(x1, y0, h)});
    scene.push_back({Eigen::Vector3d(x0, y1, 0.0), Eigen::Vector3d(x1, y1, h)});
    scene.push_back({Eigen::Vector3d(x0, y0, 0.0), Eigen::Vector3d(x0, y1, h)});
    scene.push_back({Eigen::Vector3d(x1, y0, 0.0), Eigen::Vector3d(x1, y1, h)});
    for (const double sign : {1.0, -1.0}) {
      const Eigen::Vector3d p(c.x() + sign * 3.0, c.y() + sign * 2.0, 0.0);
      scene.push_back({p - Eigen::Vector3d(0.4, 0.4, 0.0), p + Eigen::Vector3d(0.4, 0.4, h)});
    }

    // The map: the surfaces sampled every 0.1 m, as one PCD tile
    PcdPoints points;
    constexpr double kStep = 0.1;
    for (const auto& box : scene) {
      for (int axis = 0; axis < 3; axis++) {
        const int u = (axis + 1) % 3, v = (axis + 2) % 3;
        const bool flat = box.max[axis] - box.min[axis] < 1e-9;
        for (int side = 0; side < (flat ? 1 : 2); side++) {  // A rectangle has one face
          const double fixed = side == 0 ? box.min[axis] : box.max[axis];
          for (double a = box.min[u]; a <= box.max[u] + 1e-9; a += kStep) {
            for (double b = box.min[v]; b <= box.max[v] + 1e-9; b += kStep) {
              Eigen::Vector4d p(0.0, 0.0, 0.0, 1.0);
              p[axis] = fixed;
              p[u] = a;
              p[v] = b;
              points.push_back(p);
            }
          }
        }
      }
    }
    PcdMapParams pcd_params;
    pcd_params.grid_size = 40.0;
    pcd_params.voxel_size = 0.05;
    LoadedSubmaps submaps;
    ASSERT_TRUE(build_pcd_submaps(points, pcd_params, submaps));
    ASSERT_TRUE(save_compiled_map((*dir_ / "map").string(), submaps, "pcd"));
    auto map = std::make_shared<MapManager>();
    ASSERT_TRUE(map->load((*dir_ / "map").string()));
    map_ = new std::shared_ptr<const MapManager>(map);

    glim::CloudPreprocessorParams preprocess_params;
    preprocess_params.global_shutter = true;
    glim::CloudPreprocessor preprocessor(preprocess_params);
    const auto scans = make_synthetic_scans(scene, 0.5, 0.5);
    ASSERT_EQ(scans.size(), 1u);
    const auto raw = preprocessor.preprocess(scans.front());
    glim::CloudCovarianceEstimation covariance_estimation(2);
    std::vector<Eigen::Vector4d> normals;
    std::vector<Eigen::Matrix4d> covs;
    covariance_estimation.estimate(raw->points, raw->neighbors, normals, covs);
    auto frame = std::make_shared<gtsam_points::PointCloudCPU>(raw->points);
    frame->add_covs(covs);
    frame_ = new gtsam_points::PointCloud::ConstPtr(frame);
  }

  static void TearDownTestSuite() {
    delete frame_;
    delete map_;
    delete truth_;
    fs::remove_all(*dir_);
    delete dir_;
  }

  static fs::path* dir_;
  static std::shared_ptr<const MapManager>* map_;
  static gtsam_points::PointCloud::ConstPtr* frame_;
  static Eigen::Isometry3d* truth_;
};

fs::path* SymmetricSceneTest::dir_ = nullptr;
std::shared_ptr<const MapManager>* SymmetricSceneTest::map_ = nullptr;
gtsam_points::PointCloud::ConstPtr* SymmetricSceneTest::frame_ = nullptr;
Eigen::Isometry3d* SymmetricSceneTest::truth_ = nullptr;

TEST_F(SymmetricSceneTest, HalfTurnAmbiguityIsRejected) {
  CandidateSearchParams params;
  params.scorer.map_factors.num_threads = 2;
  params.time_budget = 1e6;  // A loaded CI machine must not cut the search short (ctest TIMEOUT bounds it)
  CandidateQuery query;
  query.hint = with_yaw_error(*truth_, 20.0 * kDeg);
  const auto result = search_candidates(**map_, *frame_, YawCandidateGenerator(M_PI, 15.0 * kDeg).generate(query), params);
  std::cout << "symmetric room: " << result.message << std::endl;
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.outcome, CandidateSearchOutcome::AMBIGUOUS);
  EXPECT_FALSE(result.truncated());
  ASSERT_TRUE(result.best);
  ASSERT_TRUE(result.runner_up);
  EXPECT_GT(result.best->score, params.min_score) << "each of the two yaws alone would pass";
  EXPECT_GT(result.runner_up->score, params.min_score);
  EXPECT_NEAR(std::abs(wrap(result.runner_up->yaw_offset - result.best->yaw_offset)) / kDeg, 180.0, 2.0);
}

TEST_F(SymmetricSceneTest, MatchBeyondTheBoundIsRejected) {
  // The given pose 1 m off: both half-turn matches register back to the truth, beyond a 0.5 m bound
  CandidateSearchParams params;
  params.scorer.map_factors.num_threads = 2;
  params.time_budget = 1e6;
  CandidateQuery query;
  Eigen::Isometry3d hint = with_yaw_error(*truth_, 20.0 * kDeg);
  hint.translation() += Eigen::Vector3d(1.0, 0.0, 0.0);
  query.hint = hint;
  params.hint_position = hint.translation();
  params.max_translation = 0.5;
  const auto result = search_candidates(**map_, *frame_, YawCandidateGenerator(M_PI, 15.0 * kDeg).generate(query), params);
  std::cout << "symmetric room, bound 0.5 m: " << result.message << std::endl;
  EXPECT_EQ(result.outcome, CandidateSearchOutcome::OUT_OF_RANGE) << result.message;
  ASSERT_TRUE(result.out_of_range);
  EXPECT_GT(result.out_of_range->score, params.min_score);
  EXPECT_NEAR(result.out_of_range->translation, 1.0, 0.2);
}
