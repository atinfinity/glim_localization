#include <glim_localization/odometry/weighted_factor.hpp>

#include <stdexcept>

#include <gtsam/linear/HessianFactor.h>

namespace glim_localization {

WeightedFactor::WeightedFactor(const gtsam::NonlinearFactor::shared_ptr& factor, double weight)
: gtsam::NonlinearFactor(factor ? factor->keys() : gtsam::KeyVector()), factor_(factor), weight_(weight) {
  if (!factor_ || factor_->size() != 1) {
    throw std::invalid_argument("WeightedFactor requires a unary factor");
  }
  if (!(weight_ > 0.0)) {
    throw std::invalid_argument("WeightedFactor requires a positive weight");
  }
}

double WeightedFactor::error(const gtsam::Values& values) const {
  return weight_ * factor_->error(values);
}

gtsam::GaussianFactor::shared_ptr WeightedFactor::linearize(const gtsam::Values& values) const {
  const auto linearized = factor_->linearize(values);
  const auto hessian = std::dynamic_pointer_cast<gtsam::HessianFactor>(linearized);
  if (!hessian) {
    throw std::runtime_error("WeightedFactor requires a factor that linearizes to a HessianFactor");
  }
  const gtsam::Matrix G = weight_ * hessian->information();
  const gtsam::Vector g = weight_ * hessian->linearTerm();
  return std::make_shared<gtsam::HessianFactor>(keys()[0], G, g, weight_ * hessian->constantTerm());
}

gtsam::NonlinearFactor::shared_ptr WeightedFactor::clone() const {
  return std::make_shared<WeightedFactor>(factor_->clone(), weight_);
}

}  // namespace glim_localization
