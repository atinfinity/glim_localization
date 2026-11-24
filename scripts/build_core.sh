#!/bin/bash
# Build, test, and install the ROS-independent core with plain CMake (inside the container).
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir=/ws/build/core
install_dir=/ws/build/core_install

# ccache only when opted in with CCACHE_DIR (CI). Set on every configure, so a build directory
# configured with ccache once does not keep using it.
launcher=(-DCMAKE_C_COMPILER_LAUNCHER= -DCMAKE_CXX_COMPILER_LAUNCHER=)
if [[ -n "${CCACHE_DIR:-}" ]]; then
  launcher=(-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache)
fi

# CUDA mode: the CUDA image sets GLIM_LOC_BUILD_WITH_CUDA=ON (docker/Dockerfile.cuda). Set on every
# configure, like the launcher, and the same in build_colcon.sh.
with_cuda="${GLIM_LOC_BUILD_WITH_CUDA:-OFF}"
[[ "${with_cuda}" == ON || "${with_cuda}" == OFF ]] || { echo "FAIL: GLIM_LOC_BUILD_WITH_CUDA must be ON or OFF, got ${with_cuda}" >&2; exit 1; }

# build_colcon.sh reuses this build directory and configures it with colcon's install prefix, so the
# prefix is set here too: the plain configure is the same whether colcon ran before or not.
cmake -S "${repo_root}/glim_localization" -B "${build_dir}" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON -DGLIM_LOCALIZATION_USE_AMENT=OFF \
  -DBUILD_WITH_CUDA="${with_cuda}" \
  -DCMAKE_INSTALL_PREFIX="${install_dir}" "${launcher[@]}"
cmake --build "${build_dir}"
# Parallel jobs: CTEST_JOBS, default the CPU count (test temp dirs are per-PID).
ctest --test-dir "${build_dir}" -j"${CTEST_JOBS:-$(nproc)}" --output-on-failure --no-tests=error
rm -rf "${install_dir}"
cmake --install "${build_dir}" --prefix "${install_dir}"
test -f "${install_dir}/lib/cmake/glim_localization/glim_localization-config.cmake"

test -f "${install_dir}/lib/libodometry_localization_cpu.so"
test -x "${install_dir}/lib/glim_localization/glim_localization_replay"
if [[ "${with_cuda}" == ON ]]; then
  grep -q '^BUILD_WITH_CUDA:BOOL=ON$' "${build_dir}/CMakeCache.txt" || { echo "FAIL: the core is not configured with BUILD_WITH_CUDA=ON" >&2; exit 1; }
  test -x "${build_dir}/test/test_gpu_smoke"
  # The GPU module: without this check a CMake change that drops it would pass, because the GPU tests skip without a GPU
  test -f "${install_dir}/lib/libodometry_localization_gpu.so" || { echo "FAIL: the GPU module is not installed" >&2; exit 1; }
  test -f "${install_dir}/include/glim_localization/odometry/odometry_estimation_localization_gpu.hpp"
else
  # The GPU headers need the CUDA gtsam_points, so a CPU build does not install them
  if find "${install_dir}/include" -name '*gpu*' | grep -q .; then
    echo "FAIL: a CPU build installs GPU headers" >&2
    exit 1
  fi
fi

modules=("${build_dir}/libodometry_localization_cpu.so")
[[ "${with_cuda}" == ON ]] && modules+=("${build_dir}/libodometry_localization_gpu.so")
for library in "${build_dir}/libglim_localization.so" "${modules[@]}" "${build_dir}/tools/glim_localization_replay"; do
  libraries="$(ldd "${library}")"
  grep -q 'libglim\.so' <<< "${libraries}" || { echo "FAIL: libglim is not linked to ${library}" >&2; exit 1; }
  if grep -E 'Iridescence|glfw|libGL|libX11' <<< "${libraries}"; then
    echo "FAIL: ${library} links GUI libraries" >&2
    exit 1
  fi
  if grep -E 'librclcpp|librcl\.so' <<< "${libraries}"; then
    echo "FAIL: ${library} links ROS libraries" >&2
    exit 1
  fi
done

echo "PASS: core build"
