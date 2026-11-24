#include <glim_localization/odometry/weighted_vgicp_factor_gpu.hpp>

#include <stdexcept>
#include <utility>

#include <gtsam/linear/HessianFactor.h>

namespace glim_localization {

namespace {

double require_positive(const double weight) {
  if (!(weight > 0.0)) {
    throw std::invalid_argument("WeightedVGICPFactorGPU requires a positive weight");
  }
  return weight;
}

}  // namespace

WeightedVGICPFactorGPU::WeightedVGICPFactorGPU(
  const gtsam::Pose3& fixed_target_pose,
  const gtsam::Key source_key,
  const gtsam_points::GaussianVoxelMap::ConstPtr& target,
  const gtsam_points::PointCloud::ConstPtr& source,
  const double weight,
  CUstream_st* stream,
  std::shared_ptr<gtsam_points::TempBufferManager> temp_buffer)
: gtsam_points::IntegratedVGICPFactorGPU(fixed_target_pose, source_key, target, source, stream, std::move(temp_buffer)),
  fixed_target_pose_(fixed_target_pose),
  target_(target),
  source_(source),
  weight_(require_positive(weight)) {}

WeightedVGICPFactorGPU::~WeightedVGICPFactorGPU() {}

double WeightedVGICPFactorGPU::error(const gtsam::Values& values) const {
  return weight_ * gtsam_points::IntegratedVGICPFactorGPU::error(values);
}

gtsam::GaussianFactor::shared_ptr WeightedVGICPFactorGPU::linearize(const gtsam::Values& values) const {
  const auto linearized = gtsam_points::IntegratedVGICPFactorGPU::linearize(values);
  const auto hessian = std::dynamic_pointer_cast<gtsam::HessianFactor>(linearized);
  if (!hessian) {
    throw std::runtime_error("IntegratedVGICPFactorGPU did not linearize to a HessianFactor");
  }
  const gtsam::Matrix G = weight_ * hessian->information();
  const gtsam::Vector g = weight_ * hessian->linearTerm();
  return std::make_shared<gtsam::HessianFactor>(keys()[0], G, g, weight_ * hessian->constantTerm());
}

gtsam::NonlinearFactor::shared_ptr WeightedVGICPFactorGPU::clone() const {
  // Like the base clone: no stream or buffer, which makes standalone evaluation synchronous.
  return std::make_shared<WeightedVGICPFactorGPU>(fixed_target_pose_, keys()[0], target_, source_, weight_);
}

}  // namespace glim_localization
