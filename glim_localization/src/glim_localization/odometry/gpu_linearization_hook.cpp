#include <glim_localization/odometry/gpu_linearization_hook.hpp>

#include <memory>
#include <mutex>

#include <gtsam/nonlinear/Values.h>

#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>
#include <gtsam_points/factors/nonlinear_factor_gpu.hpp>
#include <gtsam_points/optimizers/linearization_hook.hpp>

namespace glim_localization {

namespace {

/// Never evaluated: only offered to the registered factor sets to see whether one takes GPU factors.
class ProbeFactorGPU : public gtsam_points::NonlinearFactorGPU {
public:
  ProbeFactorGPU() : gtsam_points::NonlinearFactorGPU(gtsam::KeyVector{0}) {}

  size_t dim() const override { return 6; }
  double error(const gtsam::Values&) const override { return 0.0; }
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values&) const override { return nullptr; }

  size_t linearization_input_size() const override { return 0; }
  size_t linearization_output_size() const override { return 0; }
  size_t evaluation_input_size() const override { return 0; }
  size_t evaluation_output_size() const override { return 0; }
  void set_linearization_point(const gtsam::Values&, void*) override {}
  void issue_linearize(const void*, const void*, void*) override {}
  void store_linearized(const void*) override {}
  void set_evaluation_point(const gtsam::Values&, void*) override {}
  void issue_compute_error(const void*, const void*, const void*, const void*, void*) override {}
  void store_computed_error(const void*) override {}
  void sync() override {}
};

}  // namespace

bool ensure_gpu_linearization_hook() {
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);

  gtsam_points::LinearizationHook probe_hook;
  if (probe_hook.add(std::make_shared<ProbeFactorGPU>())) {
    return false;
  }
  gtsam_points::LinearizationHook::register_hook([] { return gtsam_points::create_nonlinear_factor_set_gpu(); });
  return true;
}

}  // namespace glim_localization
