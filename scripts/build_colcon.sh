#!/bin/bash
# Build and install all packages with colcon (inside the container, after build_core.sh).
# The core tests run once, in build_core.sh.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
base=/ws/build/colcon
core_build=/ws/build/core
paths=(--base-paths "${repo_root}" --build-base "${base}/build" --install-base "${base}/install")

# make gets -jN, N from CMAKE_BUILD_PARALLEL_LEVEL (the image sets it to its JOBS), instead of colcon's
# default of the CPU count. Two packages build at once, but the core is already compiled, so only
# glim_localization_msgs compiles next to it. No -l: right after the parallel core tests the load average
# stays high for a minute and would hold glim_localization_ros2 to one job. MAKEFLAGS set by the caller
# is kept.
if [[ -z "${MAKEFLAGS:-}" ]]; then
  export MAKEFLAGS="-j${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
fi

# ccache only when opted in with CCACHE_DIR (CI). Set on every configure, so a build directory
# configured with ccache once does not keep using it.
launcher=(-DCMAKE_C_COMPILER_LAUNCHER= -DCMAKE_CXX_COMPILER_LAUNCHER=)
if [[ -n "${CCACHE_DIR:-}" ]]; then
  launcher=(-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache)
fi

# The core is compiled once. colcon builds glim_localization in the build directory of build_core.sh
# (a link), with the same compile options (Release, tests and tools on, the same launcher), so make finds
# it up to date and only the ament export (GLIM_LOCALIZATION_USE_AMENT=ON) and the install run here.
# build_core.sh configures that directory with GLIM_LOCALIZATION_USE_AMENT=OFF, so colcon must configure
# it again on every run: removing colcon's record of its last CMake arguments makes it do so.
[[ -f "${core_build}/CMakeCache.txt" ]] || { echo "FAIL: ${core_build} is not configured (run build_core.sh first)" >&2; exit 1; }
mkdir -p "${base}/build"
if [[ "$(readlink "${base}/build/glim_localization" || true)" != "${core_build}" ]]; then
  rm -rf "${base}/build/glim_localization"
  ln -s "${core_build}" "${base}/build/glim_localization"
fi
rm -f "${core_build}/cmake_args.last"

# CUDA mode: the same BUILD_WITH_CUDA=ON as build_core.sh, so make finds the core up to date. colcon
# passes --cmake-args to every package, so the CPU mode passes nothing (the core keeps the OFF that
# build_core.sh configured) and the other packages get no unused variable.
with_cuda="${GLIM_LOC_BUILD_WITH_CUDA:-OFF}"
[[ "${with_cuda}" == ON || "${with_cuda}" == OFF ]] || { echo "FAIL: GLIM_LOC_BUILD_WITH_CUDA must be ON or OFF, got ${with_cuda}" >&2; exit 1; }
cuda_args=()
if [[ "${with_cuda}" == ON ]]; then
  cuda_args=(-DBUILD_WITH_CUDA=ON)
fi

colcon --log-base "${base}/log" build "${paths[@]}" --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON -DBUILD_TESTING=ON -DGLIM_LOCALIZATION_USE_AMENT=ON \
  ${cuda_args[@]+"${cuda_args[@]}"} "${launcher[@]}"

# The downstream packages found the core through its ament package config, not a plain CMake one.
grep -q '^GLIM_LOCALIZATION_USE_AMENT:BOOL=ON$' "${core_build}/CMakeCache.txt" \
  || { echo "FAIL: colcon did not configure the core with GLIM_LOCALIZATION_USE_AMENT=ON" >&2; exit 1; }
core_install="${base}/install/glim_localization"
test -f "${core_install}/share/glim_localization/cmake/glim_localizationConfig.cmake"
test -f "${core_install}/share/ament_index/resource_index/packages/glim_localization"
grep -q "glim_localization_DIR:PATH=${core_install}/share/glim_localization/cmake" "${base}/build/glim_localization_ros2/CMakeCache.txt" \
  || { echo "FAIL: glim_localization_ros2 did not use the ament package config of the core" >&2; exit 1; }

echo "PASS: colcon build"
