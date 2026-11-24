// Unit tests of the initial pose search: candidate generation, scoring and selection.
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

#include <glim_localization/map/map_manager.hpp>
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
    params.min_score = 0.5;
    params.time_budget = 30.0;
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
  EXPECT_NE(result.message.find("below"), std::string::npos) << result.message;

  // On the map, but the threshold is out of reach
  query.hint = *truth_;
  auto params = search_params();
  params.min_score = 1.0 + 1e-9;
  const auto strict = search_candidates(**map_, *frame_, YawCandidateGenerator(0.0, 1.0).generate(query), params);
  EXPECT_FALSE(strict.accepted);
  ASSERT_TRUE(strict.best);
  EXPECT_GT(strict.best->score, 0.8) << "the best score is reported also when it is rejected";
}

TEST_F(CandidateScoringTest, TimeBudgetStopsAfterTheFirstCandidate) {
  CandidateQuery query;
  query.hint = *truth_;
  const auto candidates = YawCandidateGenerator(M_PI, 15.0 * kDeg).generate(query);
  auto params = search_params();
  params.time_budget = 0.0;
  const auto result = search_candidates(**map_, *frame_, candidates, params);
  EXPECT_EQ(result.num_candidates, 24);
  EXPECT_EQ(result.num_evaluated, 1) << "the first candidate (the hint) is always scored";
  EXPECT_TRUE(result.accepted);
  EXPECT_DOUBLE_EQ(result.best->candidate.yaw_offset, 0.0);

  const auto none = search_candidates(**map_, *frame_, PoseCandidates(), search_params());
  EXPECT_FALSE(none.accepted);
  EXPECT_EQ(none.num_evaluated, 0);
  EXPECT_FALSE(none.best);
}
