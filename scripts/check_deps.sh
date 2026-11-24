#!/bin/bash
# Dependency check (system libraries, GLIM, glim_ros2 and the Python tooling). Run inside the container:
#   scripts/docker_run.sh scripts/check_deps.sh
set -euo pipefail

GLIM_COMMIT=1a197c999e4865f5cab7c9cd25474128413ebbca
GTSAM_VERSION=4.3.0
GTSAM_POINTS_VERSION=1.2.2

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir=/ws/build/check_deps

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

echo "== Pinned versions"
actual="$(dpkg-query -W -f='${Version}' libgtsam-notbb-dev)"
[[ "${actual}" == "${GTSAM_VERSION}"* ]] || fail "libgtsam-notbb-dev is ${actual}"
echo "libgtsam-notbb-dev ${actual}"
# The CUDA image (GLIM_LOC_BUILD_WITH_CUDA=ON, docker/Dockerfile.cuda) installs the PPA's CUDA 13.1 build
# of gtsam_points instead of libgtsam-points-dev; both install the same files, so exactly one may be present.
with_cuda="${GLIM_LOC_BUILD_WITH_CUDA:-OFF}"
gtsam_points_package=libgtsam-points-dev
other_package=libgtsam-points-cuda13.1-dev
if [[ "${with_cuda}" == ON ]]; then
  gtsam_points_package=libgtsam-points-cuda13.1-dev
  other_package=libgtsam-points-dev
fi
actual="$(dpkg-query -W -f='${Version}' "${gtsam_points_package}")"
[[ "${actual}" == "${GTSAM_POINTS_VERSION}"* ]] || fail "${gtsam_points_package} is ${actual}"
echo "${gtsam_points_package} ${actual}"
if dpkg-query -W -f='${db:Status-Abbrev}' "${other_package}" 2>/dev/null | grep -q '^ii'; then
  fail "${other_package} is installed next to ${gtsam_points_package}"
fi
if [[ "${with_cuda}" == ON ]]; then
  CUDA_VERSION=13.1
  grep -Eq '^#define GTSAM_POINTS_USE_CUDA$|^#define GTSAM_POINTS_USE_CUDA 1$' /usr/local/include/gtsam_points/config.hpp \
    || fail "gtsam_points is not built with CUDA"
  actual="$(nvcc --version | sed -n 's/.*release \([0-9.]*\),.*/\1/p')"
  [[ "${actual}" == "${CUDA_VERSION}" ]] || fail "nvcc is CUDA ${actual}, expected ${CUDA_VERSION}"
  echo "nvcc ${actual}"
  [[ -f /opt/glim/lib/libodometry_estimation_gpu.so ]] || fail "GLIM was built without CUDA (no libodometry_estimation_gpu.so)"
  echo "libodometry_estimation_gpu.so installed"
fi
actual="$(git -c safe.directory=/opt/glim_ws/src/glim -C /opt/glim_ws/src/glim rev-parse HEAD)"
[[ "${actual}" == "${GLIM_COMMIT}" ]] || fail "glim is at ${actual}"
echo "glim ${actual}"

GLIM_ROS2_COMMIT=4d4ec524ccf1b02aa09b0af2af767ecc54343798
actual="$(git -c safe.directory=/opt/glim_ws/src/glim_ros2 -C /opt/glim_ws/src/glim_ros2 rev-parse HEAD)"
[[ "${actual}" == "${GLIM_ROS2_COMMIT}" ]] || fail "glim_ros2 is at ${actual}"
echo "glim_ros2 ${actual}"
for exe in glim_rosnode glim_rosbag; do
  [[ -x "/opt/glim/lib/glim_ros/${exe}" ]] || fail "${exe} is not installed"
done
[[ -f /opt/glim/lib/libglim_ros.so ]] || fail "libglim_ros.so is not installed"
echo "glim_rosnode, glim_rosbag, libglim_ros.so installed"

# ldd must succeed, resolve every dependency, and show no GUI library. A failure of ldd itself or an
# unresolved dependency (which hides what it would pull in) is an error, not a pass.
check_no_gui_libs() {
  local lib="$1" out linked
  out="$(ldd "${lib}" 2>&1)" || fail "ldd failed on ${lib}: ${out}"
  ! grep -q 'not found' <<<"${out}" || fail "${lib} has unresolved dependencies: $(grep 'not found' <<<"${out}")"
  linked="$(grep -E "$2" <<<"${out}" || true)"
  [[ -z "${linked}" ]] || fail "$(basename "${lib}") links GUI libraries: ${linked}"
}

echo "== Headless environment"
check_no_gui_libs /opt/glim/lib/libglim_ros.so 'Iridescence|glfw|libGL|libX11|libQt|libvtk|opencv'
packages="$(dpkg-query -W -f='${db:Status-Abbrev} ${Package}\n' | awk '$1 == "ii" {print $2}' | grep -E 'opencv|cv-bridge' || true)"
[[ -z "${packages}" ]] || fail "OpenCV packages installed: ${packages}"
[[ -z "${DISPLAY:-}" ]] || fail "DISPLAY is set"
forbidden="$(find /opt/glim/lib -maxdepth 1 \( -name '*viewer*' -o -name 'libmemory_monitor*' -o -name 'libmap_editor*' \) -print)"
[[ -z "${forbidden}" ]] || fail "viewer libraries installed: ${forbidden}"
check_no_gui_libs /opt/glim/lib/libglim.so 'Iridescence|glfw|libGL|libX11'
packages="$(dpkg-query -W -f='${db:Status-Abbrev} ${Package}\n' | awk '$1 == "ii" {print $2}' | grep -E 'iridescence|rviz|libglfw' || true)"
[[ -z "${packages}" ]] || fail "GUI packages installed: ${packages}"
echo "no GUI libraries or packages"

echo "== glim_ros consumer"
# find_package(glim_ros) must work for downstream packages (the viewer library is removed from the
# install, so a stale export would break every consumer). Throwaway project, removed on exit.
consumer_dir="$(mktemp -d)"
trap 'rm -rf "${consumer_dir}"' EXIT
cat > "${consumer_dir}/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(glim_ros_consumer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(glim_ros REQUIRED)
add_executable(consumer main.cpp)
target_include_directories(consumer PRIVATE ${glim_ros_INCLUDE_DIRS})
target_link_libraries(consumer ${glim_ros_LIBRARIES})
CMAKE
cat > "${consumer_dir}/main.cpp" <<'CPP'
#include <glim_ros/glim_ros.hpp>
// Reference a real symbol so the linker cannot drop libglim_ros.
int main() {
  void (glim::GlimROS::*volatile f)(bool) = &glim::GlimROS::wait;
  return f == nullptr;
}
CPP
cmake -S "${consumer_dir}" -B "${consumer_dir}/build" -DCMAKE_BUILD_TYPE=Release >/dev/null \
  || fail "find_package(glim_ros) does not configure"
cmake --build "${consumer_dir}/build" >/dev/null || fail "a consumer of glim_ros does not build"
"${consumer_dir}/build/consumer" || fail "the glim_ros consumer does not run"
echo "find_package(glim_ros) consumer builds and runs"

echo "== Python tooling"
[[ -x "${GLIM_LOC_PYTHON:-}" ]] || fail "GLIM_LOC_PYTHON is not set to an executable"
check_python_package() {
  local package="$1" expected="$2" actual
  actual="$("${GLIM_LOC_PYTHON}" -c "import importlib.metadata as m; import ${package}; print(m.version('${package}'))")" \
    || fail "cannot import ${package}"
  [[ "${actual}" == "${expected}" ]] || fail "${package} is ${actual}, expected ${expected}"
  echo "${package} ${actual}"
}
# The expected versions come from the pinned requirements, not from a second list.
for package in rosbags evo pytest; do
  expected="$(grep -iE "^${package}==" "${repo_root}/scripts/requirements.in" | cut -d= -f3)"
  [[ -n "${expected}" ]] || fail "${package} is not pinned in requirements.in"
  check_python_package "${package}" "${expected}"
done
"${GLIM_LOC_PYTHON}" -c "import sys; assert sys.prefix != sys.base_prefix" || fail "GLIM_LOC_PYTHON is not a venv interpreter"
[[ "$(command -v python3)" != "${GLIM_LOC_PYTHON}" ]] || fail "the venv must not be first on PATH"

echo "== Minimal project"
cmake -S "${repo_root}/scripts/check_deps" -B "${build_dir}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${build_dir}"
"${build_dir}/check_deps"

echo "PASS: Phase 0 dependency check"
