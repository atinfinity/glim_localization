#include <glim_localization/relocalization/pose_candidates.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace glim_localization {

std::vector<double> yaw_search_offsets(double range, double step) {
  if (!std::isfinite(range) || range < 0.0) {
    throw std::invalid_argument("the yaw search range must be finite and not negative");
  }
  if (!std::isfinite(step) || !(step > 0.0)) {
    throw std::invalid_argument("the yaw search step must be finite and positive");
  }
  constexpr double kTolerance = 1e-9;
  const double half_turn = M_PI;
  const double limit = std::min(range, half_turn);
  const int count = static_cast<int>(std::floor(limit / step + kTolerance));

  std::vector<double> offsets = {0.0};
  for (int i = 1; i <= count; i++) {
    const double offset = i * step;
    offsets.push_back(offset);
    if (offset < half_turn - kTolerance) {
      offsets.push_back(-offset);  // -180 deg is the same yaw as +180 deg
    }
  }
  return offsets;
}

YawCandidateGenerator::YawCandidateGenerator(double range, double step) : offsets_(yaw_search_offsets(range, step)) {}

PoseCandidates YawCandidateGenerator::generate(const CandidateQuery& query) const {
  PoseCandidates candidates;
  if (!query.hint) {
    return candidates;
  }
  candidates.reserve(offsets_.size());
  for (const double offset : offsets_) {
    PoseCandidate candidate;
    candidate.T_world_imu = *query.hint;
    candidate.T_world_imu.linear() = Eigen::AngleAxisd(offset, Eigen::Vector3d::UnitZ()).toRotationMatrix() * query.hint->linear();
    candidate.yaw_offset = offset;
    candidates.push_back(candidate);
  }
  return candidates;
}

}  // namespace glim_localization
