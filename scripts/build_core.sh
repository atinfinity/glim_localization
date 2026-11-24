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

# build_colcon.sh reuses this build directory and configures it with colcon's install prefix, so the
# prefix is set here too: the plain configure is the same whether colcon ran before or not.
cmake -S "${repo_root}/glim_localization" -B "${build_dir}" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON -DGLIM_LOCALIZATION_USE_AMENT=OFF \
  -DCMAKE_INSTALL_PREFIX="${install_dir}" "${launcher[@]}"
cmake --build "${build_dir}"
# Parallel jobs: CTEST_JOBS, default the CPU count (test temp dirs are per-PID).
ctest --test-dir "${build_dir}" -j"${CTEST_JOBS:-$(nproc)}" --output-on-failure --no-tests=error
rm -rf "${install_dir}"
cmake --install "${build_dir}" --prefix "${install_dir}"
test -f "${install_dir}/lib/cmake/glim_localization/glim_localization-config.cmake"

test -f "${install_dir}/lib/libodometry_localization_cpu.so"
test -x "${install_dir}/lib/glim_localization/glim_localization_replay"

for library in "${build_dir}/libglim_localization.so" "${build_dir}/libodometry_localization_cpu.so" "${build_dir}/tools/glim_localization_replay"; do
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
