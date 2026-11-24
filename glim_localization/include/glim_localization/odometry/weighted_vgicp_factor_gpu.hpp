#pragma once

#include <memory>

#include <gtsam/geometry/Pose3.h>

#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>

namespace glim_localization {

/// Unary IntegratedVGICPFactorGPU scaled by a constant weight (CUDA builds only).
///
/// WeightedFactor cannot wrap a GPU factor: the smoother batches only factors that are NonlinearFactorGPU, and a
/// wrapper is not one. This subclass stays one: the GPU batch computes the unweighted result, then error() and
/// linearize() scale it as WeightedFactor does (information, linear and constant terms times the weight).
class WeightedVGICPFactorGPU : public gtsam_points::IntegratedVGICPFactorGPU {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<WeightedVGICPFactorGPU>;

  /// @throw std::invalid_argument if the weight is not positive
  WeightedVGICPFactorGPU(
    const gtsam::Pose3& fixed_target_pose,
    gtsam::Key source_key,
    const gtsam_points::GaussianVoxelMap::ConstPtr& target,
    const gtsam_points::PointCloud::ConstPtr& source,
    double weight,
    CUstream_st* stream = nullptr,
    std::shared_ptr<gtsam_points::TempBufferManager> temp_buffer = nullptr);
  ~WeightedVGICPFactorGPU() override;

  double weight() const { return weight_; }

  double error(const gtsam::Values& values) const override;
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override;
  gtsam::NonlinearFactor::shared_ptr clone() const override;

private:
  const gtsam::Pose3 fixed_target_pose_;
  const gtsam_points::GaussianVoxelMap::ConstPtr target_;
  const gtsam_points::PointCloud::ConstPtr source_;
  const double weight_;
};

}  // namespace glim_localization
