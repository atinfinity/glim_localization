#include <glim_localization/odometry/localization_backend.hpp>

#include <stdexcept>
#include <utility>

#include <glim/util/config.hpp>

#include <glim_localization/odometry/odometry_estimation_localization_cpu.hpp>
#ifdef GLIM_LOCALIZATION_WITH_CUDA
#include <glim_localization/odometry/odometry_estimation_localization_gpu.hpp>
#endif

namespace glim_localization {

std::string to_string(LocalizationBackend backend) {
  switch (backend) {
    case LocalizationBackend::CPU:
      return "cpu";
    case LocalizationBackend::GPU:
      return "gpu";
  }
  return "unknown";
}

bool gpu_backend_available() {
#ifdef GLIM_LOCALIZATION_WITH_CUDA
  return true;
#else
  return false;
#endif
}

LocalizationBackend localization_backend_from_config() {
  glim::Config config(glim::GlobalConfig::get_config_path("config_odometry"));
  const std::string so_name = config.param<std::string>("odometry_estimation", "so_name", "");
  if (so_name == "libodometry_localization_cpu.so") {
    return LocalizationBackend::CPU;
  }
  if (so_name == "libodometry_localization_gpu.so") {
    return LocalizationBackend::GPU;
  }
  throw std::runtime_error("config_odometry so_name is not a localization module: " + so_name);
}

std::unique_ptr<OdometryEstimationLocalizationParams> make_localization_params(LocalizationBackend backend) {
  switch (backend) {
    case LocalizationBackend::CPU:
      return std::make_unique<OdometryEstimationLocalizationCPU::Params>();
    case LocalizationBackend::GPU:
#ifdef GLIM_LOCALIZATION_WITH_CUDA
      return std::make_unique<OdometryEstimationLocalizationGPU::Params>();
#else
      throw std::runtime_error("the GPU backend needs a build with BUILD_WITH_CUDA=ON");
#endif
  }
  throw std::runtime_error("unknown localization backend");
}

std::unique_ptr<OdometryEstimationLocalization> make_localization_odometry(
  const OdometryEstimationLocalizationParams& params,
  std::shared_ptr<const MapManager> map) {
#ifdef GLIM_LOCALIZATION_WITH_CUDA
  if (const auto* gpu = dynamic_cast<const OdometryEstimationLocalizationGPU::Params*>(&params)) {
    return std::make_unique<OdometryEstimationLocalizationGPU>(*gpu, std::move(map));
  }
#endif
  if (const auto* cpu = dynamic_cast<const OdometryEstimationLocalizationCPU::Params*>(&params)) {
    return std::make_unique<OdometryEstimationLocalizationCPU>(*cpu, std::move(map));
  }
  throw std::invalid_argument("params are not from make_localization_params");
}

}  // namespace glim_localization
