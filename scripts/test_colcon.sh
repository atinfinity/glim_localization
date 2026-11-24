#!/bin/bash
# Run the tests of the ROS packages with colcon (inside the container, after build_colcon.sh).
# The core tests run once, in build_core.sh; the core is not tested here.
# Headless: no display. The gtests run in process; the launch tests run glim_rosbag (R1) and glim_localization_node
# with ros2 bag play (R2) over synthetic bags.
set -euo pipefail

base=/ws/build/colcon
core_build=/ws/build/core
fixture=/ws/out/r1_fixture
: "${GLIM_LOC_PYTHON:?GLIM_LOC_PYTHON is not set (run inside the dev container)}"
export PYTHONDONTWRITEBYTECODE=1

[[ -f "${base}/install/setup.bash" ]] || { echo "FAIL: ${base}/install/setup.bash not found (run build_colcon.sh)" >&2; exit 1; }
# ROS setup scripts reference unset variables.
set +u
# shellcheck disable=SC1091
source "${base}/install/setup.bash"
set -u

# Same base paths as build_colcon.sh, so colcon finds the build directories.
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
paths=(--base-paths "${repo_root}" --build-base "${base}/build" --install-base "${base}/install")

# Fixture for the R1 launch test: the synthetic map and replay dataset (12 s), and the same data as a bag.
# Two /initialpose messages are in the bag: the first frame (offset 0.3 m, -0.2 m, 0.1 m and 3 deg) and exact ground truth at 5 s.
generator="${core_build}/test/synthetic_replay_dataset"
[[ -x "${generator}" ]] || { echo "FAIL: ${generator} not built (run build_core.sh first)" >&2; exit 1; }
rm -rf "${fixture}"
mkdir -p "${fixture}"
"${generator}" "${fixture}/input" 12
env -u PYTHONPATH "${GLIM_LOC_PYTHON}" "${repo_root}/scripts/replay_to_bag.py" "${fixture}/input/data" --out "${fixture}/bag" \
  --initial-pose "0:0.3,-0.2,0.1,3" --initial-pose "5"
export GLIM_R1_BAG="${fixture}/bag" GLIM_R1_DATA="${fixture}/input/data" GLIM_R1_MAP="${fixture}/input/map"
# Fixture for the R2 IMU gap launch test: the same data with no IMU sample for 0.3 s from 6.0 s (GAP_START_S and
# GAP_S in test_r2_imu_gap.py) and no /initialpose (the test sets the pose through ~/initialize).
env -u PYTHONPATH "${GLIM_LOC_PYTHON}" "${repo_root}/scripts/replay_to_bag.py" "${fixture}/input/data" --out "${fixture}/gap_bag" \
  --drop-imu "6.0:0.3"
export GLIM_R2_GAP_BAG="${fixture}/gap_bag"

pkg_build="${base}/build/glim_localization_ros2"
# Every gtest that must run. Add the name of a new test here.
expected=(test_message_constants test_localization_publisher test_gyro_buffer test_init_pose_delay)
# Every launch test that must run (the file name without .py). Add the name of a new test here.
expected_launch=(test_r1_extension test_r2_node test_r2_imu_gap)

# A build with BUILD_TESTING off registers no tests, and old results must not count.
grep -q '^BUILD_TESTING:BOOL=ON$' "${pkg_build}/CMakeCache.txt" || { echo "FAIL: BUILD_TESTING is not ON in ${pkg_build}/CMakeCache.txt (run build_colcon.sh)" >&2; exit 1; }
rm -rf "${pkg_build}/test_results" "${pkg_build}/Testing"

# Keep DDS discovery on this host, so that tests in other containers on one network do not meet.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# The tests run in parallel (CTEST_JOBS, default: the CPU count). Each test that uses ROS reserves its own
# ROS_DOMAIN_ID through run_test_isolated.py (see glim_localization_ros2/CMakeLists.txt), which keeps a
# ROS_DOMAIN_ID that is already set, so it must not be set here.
unset ROS_DOMAIN_ID
jobs="${CTEST_JOBS:-$(nproc)}"

colcon --log-base "${base}/log" test "${paths[@]}" --packages-select glim_localization_ros2 --return-code-on-test-failure \
  --event-handlers console_direct+ --ctest-args -j"${jobs}"
colcon test-result --test-result-base "${base}/build" --verbose

# Each expected gtest must have written a result file in this run, with at least one test case.
for name in "${expected[@]}"; do
  xml="${pkg_build}/test_results/glim_localization_ros2/${name}.gtest.xml"
  [[ -f "${xml}" ]] || { echo "FAIL: no result file for ${name} (${xml})" >&2; exit 1; }
  count="$(sed -n 's/.*<testsuites[^>]* tests="\([0-9]*\)".*/\1/p' "${xml}" | head -n 1)"
  [[ -n "${count}" && "${count}" -ge 1 ]] || { echo "FAIL: ${name} ran no test cases" >&2; exit 1; }
  echo "ok: ${name} ran ${count} test cases"
done

# Each launch test writes an xunit file; it must have run in this run, with at least one test case.
for name in "${expected_launch[@]}"; do
  xml="${pkg_build}/test_results/glim_localization_ros2/test_${name}.py.xunit.xml"
  [[ -f "${xml}" ]] || { echo "FAIL: no result file for the launch test ${name} (${xml})" >&2; exit 1; }
  count="$(sed -n 's/.*<testsuite [^>]*tests="\([0-9]*\)".*/\1/p' "${xml}" | head -n 1)"
  [[ -n "${count}" && "${count}" -ge 1 ]] || { echo "FAIL: the launch test ${name} ran no test cases" >&2; exit 1; }
  echo "ok: ${name} ran ${count} test cases"
done

echo "PASS: colcon test"
