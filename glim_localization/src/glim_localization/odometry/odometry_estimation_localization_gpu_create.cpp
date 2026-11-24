// Derived from koide3/glim src/glim/odometry/odometry_estimation_gpu_create.cpp at commit 1a197c999e4865f5cab7c9cd25474128413ebbca (MIT License, Copyright (c) 2024 koide3)
#include <cstdlib>
#include <exception>

#include <spdlog/spdlog.h>

#include <glim_localization/odometry/odometry_estimation_localization_gpu.hpp>

extern "C" glim::OdometryEstimationBase* create_odometry_estimation_module() {
  // An exception must not cross the C entry point that GLIM calls through dlsym.
  try {
    glim_localization::OdometryEstimationLocalizationGPU::Params params;
    return new glim_localization::OdometryEstimationLocalizationGPU(params);
  } catch (const std::exception& e) {
    spdlog::error("failed to create odometry_localization_gpu: {}", e.what());
    spdlog::default_logger()->flush();
    std::abort();
  }
}
