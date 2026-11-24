#pragma once

namespace glim_localization {

/// Make sure that the smoother batches GPU factors on the GPU (CUDA builds only).
///
/// gtsam_points' ISAM2 linearizes NonlinearFactorGPU in batches only when a GPU factor set is registered with
/// LinearizationHook; otherwise each factor runs alone in sync mode. glim_ros2 registers it before it loads the
/// modules, but the replay tool, R2 and tests do not. This registers it once per process unless a registered set
/// already accepts GPU factors. Thread-safe.
/// @return true if this call registered it
bool ensure_gpu_linearization_hook();

}  // namespace glim_localization
