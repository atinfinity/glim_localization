#pragma once

#include <memory>

#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace glim_localization {

/// Scales the error and the information of a unary factor that linearizes to a HessianFactor.
class WeightedFactor : public gtsam::NonlinearFactor {
public:
  /// @throw std::invalid_argument if the factor is null or not unary, or the weight is not positive
  WeightedFactor(const gtsam::NonlinearFactor::shared_ptr& factor, double weight);

  size_t dim() const override { return factor_->dim(); }
  double error(const gtsam::Values& values) const override;
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override;
  gtsam::NonlinearFactor::shared_ptr clone() const override;

  double weight() const { return weight_; }
  const gtsam::NonlinearFactor::shared_ptr& factor() const { return factor_; }

private:
  gtsam::NonlinearFactor::shared_ptr factor_;
  double weight_;
};

}  // namespace glim_localization
