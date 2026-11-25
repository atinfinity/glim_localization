#!/bin/bash
# Record the scripted Gazebo run to a rosbag2 (inside the simulation image, docker/sim/).
#   sim/scripts/record.sh <out dir>
# Writes <out>/bag (mcap: LiDAR points, IMU, IMU link pose), <out>/traj_gt.txt (TUM, T_world_imu),
# <out>/record.json (wall time, real-time factor, Gazebo version) and <out>/logs/. The out dir must not
# hold a bag. Environment: SIM_DURATION (simulation seconds, default 60), SIM_SEED (gz sim --seed,
# default 1), SIM_TIMEOUT (wall seconds allowed for gz sim, default 900).
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
out="${1:?usage: record.sh <out dir>}"
world="${repo_root}/sim/worlds/sim_eval_room.sdf"
world_name=sim_eval_room
duration="${SIM_DURATION:-60}"  # simulation seconds; the profile in the world is 60 s
step=0.001                      # max_step_size of the world
iterations="$(python3 -c "print(round(${duration} / ${step}))")"
seed="${SIM_SEED:-1}"
sim_timeout="${SIM_TIMEOUT:-900}"
plugin_build=/ws/build_sim/scripted_motion

test ! -e "${out}/bag" || { echo "FAIL: ${out}/bag exists" >&2; exit 1; }
mkdir -p "${out}/logs"

# The ScriptedMotion system plugin
{ cmake -S "${repo_root}/sim/plugins/scripted_motion" -B "${plugin_build}" -DCMAKE_BUILD_TYPE=Release \
    && cmake --build "${plugin_build}" -j "$(nproc)"; } > "${out}/logs/plugin_build.log" 2>&1 \
  || { cat "${out}/logs/plugin_build.log" >&2; echo "FAIL: building the ScriptedMotion plugin" >&2; exit 1; }
export GZ_SIM_SYSTEM_PLUGIN_PATH="${plugin_build}${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export GZ_SIM_RESOURCE_PATH="${repo_root}/sim/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
# Keep Gazebo transport inside the container and the Fuel cache out of the repository
export GZ_IP=127.0.0.1
export HOME="${HOME:-/root}"

pids=()
recorder_pid=""
recorder_status=unknown
# Background jobs of a non-interactive shell ignore SIGINT, so they are stopped with SIGTERM (the
# recorder closes the bag on it), and killed if they do not exit within 20 s. The recorder's exit
# status is kept: killed or failed, its bag may be incomplete.
cleanup() {
  pkill -TERM -f "gz sim" 2>/dev/null || true
  for pid in "${pids[@]}"; do kill -TERM "${pid}" 2>/dev/null || true; done
  for _ in $(seq 20); do
    alive=false
    for pid in "${pids[@]}"; do kill -0 "${pid}" 2>/dev/null && alive=true; done
    [[ "${alive}" == false ]] && break
    sleep 1
  done
  for pid in "${pids[@]}"; do kill -KILL "${pid}" 2>/dev/null || true; done
  if [[ -n "${recorder_pid}" ]]; then
    recorder_status=0
    wait "${recorder_pid}" 2>/dev/null || recorder_status=$?
  fi
  wait 2>/dev/null || true
}
trap cleanup EXIT

# The bridge and the recorder start first, so that they record from the start of the simulation.
# No message may be lost on the way to the bag. gz-transport (ZeroMQ) drops messages beyond its high-water
# mark (default 1000 queued messages per socket): with the server running about 12x faster than real time,
# a few IMU samples and poses were lost in most runs while the bridge was busy with a point cloud. 0 removes
# the limit, for the server and the bridge (both inherit it). The bridge and the recorder also use deep
# ROS 2 queues (sim/config/bridge.yaml, record_qos.yaml) instead of the default depth of 10.
export GZ_TRANSPORT_RCVHWM=0 GZ_TRANSPORT_SNDHWM=0
"$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge" \
  --ros-args -p "config_file:=${repo_root}/sim/config/bridge.yaml" \
  > "${out}/logs/bridge.log" 2>&1 &
pids+=($!)

topics=(/sim_eval/lidar/points /sim_eval/imu /sim_eval/imu_pose)
ros2 bag record -s mcap -o "${out}/bag" --qos-profile-overrides-path "${repo_root}/sim/config/record_qos.yaml" \
  "${topics[@]}" > "${out}/logs/bag_record.log" 2>&1 &
recorder_pid=$!
pids+=("${recorder_pid}")

for _ in $(seq 60); do
  grep -q 'All requested topics are subscribed' "${out}/logs/bag_record.log" && break
  sleep 1
done
grep -q 'All requested topics are subscribed' "${out}/logs/bag_record.log" \
  || { echo "FAIL: the recorder did not subscribe to all topics (logs/bag_record.log)" >&2; exit 1; }
# Server only, started paused, sensors rendered offscreen with EGL (llvmpipe without a GPU). The paused
# server creates the sensors and advertises their topics, but publishes nothing.
gz_status_file="${out}/logs/gz_server.status"
( gz_status=0
  timeout --kill-after=20 "${sim_timeout}" \
    gz sim -s --headless-rendering --seed "${seed}" -v 3 "${world}" > "${out}/logs/gz_server.log" 2>&1 || gz_status=$?
  echo "${gz_status}" > "${gz_status_file}" ) &
pids+=($!)
gz_failed() {
  if [[ -f "${gz_status_file}" ]]; then
    tail -n 20 "${out}/logs/gz_server.log" >&2
    local status
    status="$(cat "${gz_status_file}")"
    if [[ "${status}" == 124 || "${status}" == 137 ]]; then
      echo "FAIL: gz sim did not finish ${duration} s of simulation within ${sim_timeout} s of wall time" \
        "(raise SIM_TIMEOUT; logs/gz_server.log)" >&2
    else
      echo "FAIL: gz sim exited with ${status} before the end of the run (logs/gz_server.log)" >&2
    fi
    exit 1
  fi
}
ready=false
for _ in $(seq 240); do
  gz_failed
  if gz service -l 2>/dev/null | grep -qx "/world/${world_name}/control" \
      && [[ "$(gz topic -l 2>/dev/null | grep -cxE '/sim_eval/(imu|imu_pose|lidar/points)')" -eq 3 ]]; then
    ready=true
    break
  fi
  sleep 0.5
done
[[ "${ready}" == true ]] || { echo "FAIL: the server did not advertise the sensor topics (logs/gz_server.log)" >&2; exit 1; }
# The recorder is subscribed to the bridge's ROS 2 publishers, but the bridge finds the Gazebo publishers
# only through gz-transport discovery after the server advertises them. A server started running would
# publish the first steps before discovery finished, and those messages were lost (the first IMU sample
# in some runs). The server waits paused for discovery instead; check_data.py still checks the counts and
# that the first stamps are within one period of 0, so a lost message fails the run.
sleep 3

# Run exactly <iterations> steps (WorldControl multi_step; the server pauses again after them) and wait
# for the last one. (gz sim --iterations counts paused iterations, so it cannot be combined with a paused
# start.)
echo "record: running ${duration} s of simulation (${iterations} steps, seed ${seed})"
start_ns="$(date +%s%N)"
gz service -s "/world/${world_name}/control" --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 10000 --req "pause: true, multi_step: ${iterations}" > "${out}/logs/gz_control.log" 2>&1 \
  || { cat "${out}/logs/gz_control.log" >&2; echo "FAIL: the world control request failed" >&2; exit 1; }
while true; do
  gz_failed
  done_iterations="$(gz topic -e -n 1 -t "/world/${world_name}/stats" 2>/dev/null \
    | sed -n 's/^iterations: \([0-9]*\)$/\1/p' | head -n 1)"
  [[ -n "${done_iterations}" && "${done_iterations}" -ge "${iterations}" ]] && break
  sleep 0.5
done
end_ns="$(date +%s%N)"
if [[ "${done_iterations}" -ne "${iterations}" ]]; then
  echo "FAIL: the server ran ${done_iterations} steps, not ${iterations}" >&2
  exit 1
fi
# Let the bridge and the recorder drain their queues, then stop them.
sleep 5
cleanup
trap - EXIT
if [[ "${recorder_status}" != 0 || ! -f "${out}/bag/metadata.yaml" ]]; then
  echo "FAIL: the recorder exited with ${recorder_status} or left no metadata.yaml (logs/bag_record.log)" >&2
  exit 1
fi
gz_version="$(gz sim --versions 2>/dev/null | tail -n 1)"

wall="$(python3 -c "print((${end_ns} - ${start_ns}) / 1e9)")"
# The real-time factor while running: simulation time over receive (wall) time of the pose messages,
# from the second message on (the first one, at t = 0, is sent while the server waits paused). The wall
# time of the stepping (to 0.5 s) is recorded as well.
python3 "${repo_root}/sim/scripts/pose_to_tum.py" "${out}/bag" --topic /sim_eval/imu_pose --out "${out}/traj_gt.txt" \
  --stats "${out}/record.json" --stats-extra "wall_time_s=${wall}" "seed=${seed}" \
  "sim_duration_s=${duration}" "gz_sim_version=${gz_version}"
ros2 bag info "${out}/bag" > "${out}/logs/bag_info.txt"
cat "${out}/logs/bag_info.txt"
