#!/bin/bash
# Check that the generated glim_localization_msgs interfaces exist and are usable from the CLI,
# C++ and Python (inside the container, after build_colcon.sh). Headless; no ROS graph is needed.
# Usage: scripts/test_interfaces.sh [colcon-install-dir]
set -euo pipefail

install_dir="${1:-/ws/build/colcon/install}"

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

[[ -f "${install_dir}/setup.bash" ]] || fail "${install_dir}/setup.bash not found (run build_colcon.sh)"
# ROS setup scripts reference unset variables.
set +u
# shellcheck disable=SC1091
source "${install_dir}/setup.bash"
set -u

# The check must test the install it was given, not one that was already sourced or left over
# (a package skipped by COLCON_IGNORE keeps its old install in a persistent build volume).
pkg=glim_localization_msgs
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
expected_prefix="$(realpath "${install_dir}/${pkg}")"
prefix="$(ros2 pkg prefix "${pkg}" 2>/dev/null)" || fail "${pkg} is not found after sourcing ${install_dir}"
[[ "$(realpath "${prefix}")" == "${expected_prefix}" ]] || fail "${pkg} resolves to ${prefix}, not ${expected_prefix}"
module_path="$(python3 -c "import ${pkg}; print(${pkg}.__file__)")" || fail "cannot import ${pkg}"
[[ "$(realpath "${module_path}")" == "${expected_prefix}"/* ]] || fail "python ${pkg} is ${module_path}, not under ${expected_prefix}"
for file in msg/LocalizationStatus.msg srv/Initialize.srv; do
  cmp -s "${repo_root}/${pkg}/${file}" "${expected_prefix}/share/${pkg}/${file}" \
    || fail "installed ${file} differs from the source (stale install?)"
done
echo "installed from ${expected_prefix}"

work_dir="$(mktemp -d)"
trap 'rm -rf "${work_dir}"' EXIT

echo "== Interface definitions"
interfaces="$(ros2 interface list 2>/dev/null)" || fail "ros2 interface list failed"
for name in glim_localization_msgs/msg/LocalizationStatus glim_localization_msgs/srv/Initialize; do
  grep -qx "[[:space:]]*${name}" <<<"${interfaces}" || fail "${name} is not installed"
  ros2 interface show "${name}" >/dev/null || fail "ros2 interface show ${name} failed"
  echo "${name}"
done

# Pin the wire API: top-level fields and constants in order (nested types come from other packages).
show_api() { ros2 interface show "$1" --no-comments | grep -v '^[[:space:]]'; }
status_api="$(show_api glim_localization_msgs/msg/LocalizationStatus)" || fail "cannot show LocalizationStatus"
expected_status='std_msgs/Header header
uint8 INITIALIZING=0
uint8 TRACKING=1
uint8 DEGRADED=2
uint8 LOST=3
uint8 state
int32 reset_count
int32 num_tiles
int32 num_map_factors
float64 inlier_ratio
float64 residual
float64 processing_time
uint8 RESET_NONE=0
uint8 RESET_EXTERNAL=1
uint8 RESET_IMU_GAP=2
uint8 last_reset_reason
uint8 INIT_SEARCH_NONE=0
uint8 INIT_SEARCH_ACCEPTED=1
uint8 INIT_SEARCH_REJECTED=2
uint8 init_search
uint8 INIT_SEARCH_REASON_NONE=0
uint8 INIT_SEARCH_REASON_TRUNCATED=1
uint8 INIT_SEARCH_REASON_LOW_SCORE=2
uint8 INIT_SEARCH_REASON_AMBIGUOUS=3
uint8 INIT_SEARCH_REASON_NO_CANDIDATE=4
uint8 init_search_reason
int32 init_search_candidates
int32 init_search_candidates_total
float64 init_search_score
float64 init_search_yaw_offset
float64 init_search_runner_up_score
float64 init_search_runner_up_yaw_offset
float64 init_search_time'
[[ "${status_api}" == "${expected_status}" ]] || fail "LocalizationStatus changed:
$(diff <(echo "${expected_status}") <(echo "${status_api}") || true)"
initialize_api="$(show_api glim_localization_msgs/srv/Initialize)" || fail "cannot show Initialize"
expected_initialize='geometry_msgs/PoseWithCovarianceStamped pose
bool global_search
---
bool success
string message
float64 score'
[[ "${initialize_api}" == "${expected_initialize}" ]] || fail "Initialize changed:
$(diff <(echo "${expected_initialize}") <(echo "${initialize_api}") || true)"
echo "API matches"

echo "== C++ consumer"
mkdir -p "${work_dir}/cpp"
cat > "${work_dir}/cpp/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(interfaces_cpp LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(glim_localization_msgs REQUIRED)
add_executable(consumer main.cpp)
ament_target_dependencies(consumer rclcpp glim_localization_msgs)
CMAKE
cat > "${work_dir}/cpp/main.cpp" <<'CPP'
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <glim_localization_msgs/msg/localization_status.hpp>
#include <glim_localization_msgs/srv/initialize.hpp>

int main() {
  using Status = glim_localization_msgs::msg::LocalizationStatus;
  Status status;
  status.state = Status::TRACKING;
  status.reset_count = 2;
  status.num_tiles = 3;
  status.num_map_factors = 4;
  status.inlier_ratio = 0.5;
  status.residual = 0.1;
  status.processing_time = 0.02;
  status.last_reset_reason = Status::RESET_IMU_GAP;

  glim_localization_msgs::srv::Initialize::Request request;
  request.global_search = false;
  request.pose.pose.pose.position.x = 1.0;
  glim_localization_msgs::srv::Initialize::Response response;
  response.success = true;
  response.message = "ok";
  response.score = 0.9;

  // Load the generated type support libraries: serialize and deserialize, and look up the service.
  rclcpp::Serialization<Status> status_serialization;
  rclcpp::SerializedMessage serialized;
  status_serialization.serialize_message(&status, &serialized);
  Status status_copy;
  status_serialization.deserialize_message(&serialized, &status_copy);
  rclcpp::Serialization<glim_localization_msgs::srv::Initialize::Request> request_serialization;
  rclcpp::SerializedMessage serialized_request;
  request_serialization.serialize_message(&request, &serialized_request);
  glim_localization_msgs::srv::Initialize::Request request_copy;
  request_serialization.deserialize_message(&serialized_request, &request_copy);
  const auto* service_support = rosidl_typesupport_cpp::get_service_type_support_handle<glim_localization_msgs::srv::Initialize>();

  const bool ok = service_support != nullptr && status_copy.reset_count == 2 && status_copy.last_reset_reason == 2 &&
                  request_copy.pose.pose.pose.position.x == 1.0 && status.state == 1 && Status::LOST == 3 && status.last_reset_reason == 2 &&
                  request.pose.pose.pose.position.x == 1.0 && response.score == 0.9;
  return ok ? 0 : 1;
}
CPP
cmake -S "${work_dir}/cpp" -B "${work_dir}/cpp/build" -DCMAKE_BUILD_TYPE=Release >/dev/null \
  || fail "find_package(glim_localization_msgs) does not configure"
cmake --build "${work_dir}/cpp/build" >/dev/null || fail "the C++ consumer does not build"
"${work_dir}/cpp/build/consumer" || fail "the C++ consumer does not run"
echo "C++ ok"

echo "== Python consumer"
python3 - <<'PY' || fail "the Python consumer failed"
from glim_localization_msgs.msg import LocalizationStatus
from glim_localization_msgs.srv import Initialize

status = LocalizationStatus()
status.state = LocalizationStatus.DEGRADED
status.last_reset_reason = LocalizationStatus.RESET_EXTERNAL
status.processing_time = 0.02
status.init_search = LocalizationStatus.INIT_SEARCH_REJECTED
status.init_search_candidates = 1
status.init_search_candidates_total = 24
status.init_search_runner_up_score = 0.85
request = Initialize.Request()
request.global_search = True
request.pose.pose.pose.position.x = 1.0
response = Initialize.Response()
response.success = False
response.message = "no relocalizer"
response.score = 0.0
# Round trips load the generated type support extensions, as any publisher or service would.
from rclpy.serialization import deserialize_message, serialize_message
status_copy = deserialize_message(serialize_message(status), LocalizationStatus)
assert status_copy.state == 2 and status_copy.last_reset_reason == 1
assert abs(status_copy.processing_time - 0.02) < 1e-12
assert status_copy.init_search == 2 and status_copy.init_search_candidates == 1
assert status_copy.init_search_candidates_total == 24 and status_copy.init_search_runner_up_score == 0.85
request_copy = deserialize_message(serialize_message(request), Initialize.Request)
assert request_copy.global_search is True and request_copy.pose.pose.pose.position.x == 1.0
response_copy = deserialize_message(serialize_message(response), Initialize.Response)
assert response_copy.success is False and response_copy.message == "no relocalizer"
assert status.state == 2 and LocalizationStatus.LOST == 3
assert request.global_search is True
assert response.message == "no relocalizer"
PY
echo "Python ok"

echo "PASS: interfaces"
