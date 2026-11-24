#pragma once

#include <memory>
#include <string>

#include <glim_localization/odometry/odometry_estimation_localization.hpp>

namespace glim_localization {

/// Odometry backend of the localization module
enum class LocalizationBackend { CPU, GPU };

std::string to_string(LocalizationBackend backend);

/// True if this build has the GPU backend (BUILD_WITH_CUDA=ON)
bool gpu_backend_available();

/// The backend selected by so_name in the config_odometry file of the current GlobalConfig
/// (libodometry_localization_cpu.so or libodometry_localization_gpu.so).
/// @throw std::runtime_error if the so_name is not a localization module
LocalizationBackend localization_backend_from_config();

/// Read the parameters of the backend from the current GlobalConfig.
/// @throw std::runtime_error for GPU in a build without CUDA
std::unique_ptr<OdometryEstimationLocalizationParams> make_localization_params(LocalizationBackend backend);

/// Create the module for params made by make_localization_params, with an already loaded map.
/// @throw std::invalid_argument if the map is null or a parameter is invalid
std::unique_ptr<OdometryEstimationLocalization> make_localization_odometry(
  const OdometryEstimationLocalizationParams& params,
  std::shared_ptr<const MapManager> map);

}  // namespace glim_localization
