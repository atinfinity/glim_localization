#include <glim_localization/version.hpp>

#include <glim/odometry/odometry_estimation_base.hpp>

namespace glim_localization {

std::string version() {
  return GLIM_LOCALIZATION_VERSION;
}

bool is_glim_linked() {
  // A real reference keeps libglim from being dropped by --as-needed.
  volatile auto load_module = &glim::OdometryEstimationBase::load_module;
  return load_module != nullptr;
}

}  // namespace glim_localization
