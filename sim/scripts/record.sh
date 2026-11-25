#!/bin/bash
# Record the scripted Gazebo run to a rosbag2 (inside the simulation image, docker/sim/).
#   sim/scripts/record.sh <out dir>
# Writes <out>/bag (mcap: LiDAR points, IMU, IMU link pose), <out>/traj_gt.txt (TUM, T_world_imu),
# <out>/record.json (wall time and real-time factor) and <out>/logs/. The out dir must not hold a bag.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
out="${1:?usage: record.sh <out dir>}"
world="${repo_root}/sim/worlds/sim_eval_room.sdf"
world_name=sim_eval_room
duration="${SIM_DURATION:-60}"  # simulation seconds; the profile in the world is 60 s
step=0.001                      # max_step_size of the world
iterations="$(python3 -c "print(round(${duration} / ${step}))")"
seed="${SIM_SEED:-1}"
plugin_build=/ws/build_sim/scripted_motion

test ! -e "${out}/bag" || { echo "FAIL: ${out}/bag exists" >&2; exit 1; }
mkdir -p "${out}/logs"

# The ScriptedMotion system plugin
cmake -S "${repo_root}/sim/plugins/scripted_motion" -B "${plugin_build}" -DCMAKE_BUILD_TYPE=Release > "${out}/logs/plugin_build.log"
cmake --build "${plugin_build}" -j "$(nproc)" >> "${out}/logs/plugin_build.log"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${plugin_build}${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export GZ_SIM_RESOURCE_PATH="${repo_root}/sim/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
# Keep Gazebo transport inside the container and the Fuel cache out of the repository
export GZ_IP=127.0.0.1
export HOME="${HOME:-/root}"

pids=()
# Background jobs of a non-interactive shell ignore SIGINT, so they are stopped with SIGTERM (the
# recorder closes the bag on it), and killed if they do not exit within 20 s.
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
  wait 2>/dev/null || true
}
trap cleanup EXIT

# The bridge and the recorder start first, so that they record from the start of the simulation.
"$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge" \
  /sim_eval/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
  /sim_eval/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
  /sim_eval/imu_pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose \
  > "${out}/logs/bridge.log" 2>&1 &
pids+=($!)

topics=(/sim_eval/lidar/points /sim_eval/imu /sim_eval/imu_pose)
ros2 bag record -s mcap -o "${out}/bag" "${topics[@]}" > "${out}/logs/bag_record.log" 2>&1 &
pids+=($!)

for _ in $(seq 60); do
  grep -q 'All requested topics are subscribed' "${out}/logs/bag_record.log" && break
  sleep 1
done
grep -q 'All requested topics are subscribed' "${out}/logs/bag_record.log" \
  || { echo "FAIL: the recorder did not subscribe to all topics (logs/bag_record.log)" >&2; exit 1; }
sleep 2

# Server only, running, sensors rendered offscreen with EGL (llvmpipe without a GPU); it exits after
# <iterations> steps. (Started paused, the server would count the paused iterations too.)
echo "record: running ${duration} s of simulation (${iterations} steps)"
start_ns="$(date +%s%N)"
gz sim -s -r --headless-rendering --iterations "${iterations}" --seed "${seed}" -v 3 "${world}" \
  > "${out}/logs/gz_server.log" 2>&1
end_ns="$(date +%s%N)"
# Let the bridge and the recorder drain their queues, then stop them.
sleep 5
cleanup
trap - EXIT

wall="$(python3 -c "print((${end_ns} - ${start_ns}) / 1e9)")"
# The real-time factor while running: simulation time over receive (wall) time of the pose messages,
# which excludes the server start-up. The whole wall time, with start-up, is recorded as well.
python3 "${repo_root}/sim/scripts/pose_to_tum.py" "${out}/bag" --topic /sim_eval/imu_pose --out "${out}/traj_gt.txt" \
  --stats "${out}/record.json" --stats-extra "wall_time_with_startup_s=${wall}" "seed=${seed}" "sim_duration_s=${duration}"
ros2 bag info "${out}/bag" > "${out}/logs/bag_info.txt"
cat "${out}/logs/bag_info.txt"
