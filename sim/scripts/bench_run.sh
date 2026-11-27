#!/bin/bash
# One benchmark run of one method on the Gazebo bag (scripts/sim_benchmark.sh, docs/benchmark.md):
#   sim/scripts/bench_run.sh <out dir> <glim|lidarloc|lidarloc_noimu> <run>
# glim runs in the development image (glim_localization:dev, after scripts/build_core.sh and
# scripts/build_colcon.sh), lidarloc and lidarloc_noimu in the baseline image
# (glim_localization:baseline-lidarloc). Needs the sim_eval.sh output in <out> (config, map, map_export.pcd,
# traj_gt.txt) and the restamped bag in <out>/bench/bag. Writes <out>/bench/<method>/run<run>: the config or
# the parameters, the logs, and the files of sim/scripts/bench_recorder.py.
# Steps: start the node from the ground truth at the first frame, wait until it is active, start the
# recorder, play the bag at 1x with --clock (the two sensor topics only), stop.
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
usage="usage: bench_run.sh <out dir> <glim|lidarloc|lidarloc_noimu> <run>"
out="${1:?${usage}}"
method="${2:?${usage}}"
run="${3:?${usage}}"
bag="${out}/bench/bag"
dir="${out}/bench/${method}/run${run}"
cloud_topic=/sim_eval/lidar/points
imu_topic=/sim_eval/imu
for f in config map map_export.pcd traj_gt.txt bench/bag; do
  test -e "${out}/${f}" || { echo "FAIL: ${out}/${f} is missing (run scripts/sim_benchmark.sh)" >&2; exit 1; }
done
rm -rf "${dir}"
mkdir -p "${dir}"

case "${method}" in
  glim)
    : "${GLIM_LOC_PYTHON:?GLIM_LOC_PYTHON is not set (glim runs in the dev container)}"
    test -f /ws/build/colcon/install/setup.bash || { echo "FAIL: the ROS packages are not built" >&2; exit 1; }
    source /ws/build/colcon/install/setup.bash
    env -u PYTHONPATH "${GLIM_LOC_PYTHON}" "${repo_root}/sim/scripts/ros_config.py" "${out}" "${dir}/config"
    launch=(ros2 launch glim_localization_ros2 localization.launch.py use_sim_time:=true
            config_path:="${dir}/config" map_path:="${out}/map")
    exe=lib/glim_localization_ros2/glim_localization_node
    pose_topic=/glim_localization/pose
    ready() { grep -q "active: waiting for" "${dir}/node.log"; }
    ;;
  lidarloc | lidarloc_noimu)
    test -f /opt/baseline_ws/install/setup.bash || { echo "FAIL: not in the baseline image" >&2; exit 1; }
    source /opt/baseline_ws/install/setup.bash
    share="$(ros2 pkg prefix --share lidar_localization_ros2)"
    no_imu=()
    [[ "${method}" == lidarloc_noimu ]] && no_imu=(--no-imu)
    frame="$(python3 "${repo_root}/sim/scripts/bench_params.py" "${share}/param/localization.yaml" "${out}" \
      "${bag}" "${dir}/params" ${no_imu[@]+"${no_imu[@]}"})"
    # The LiDAR in the IMU frame (the base frame), same axes: T_IMU_LIDAR of sim/scripts/localize.sh.
    launch=(ros2 launch lidar_localization_ros2 lidar_localization.launch.py
            localization_param_dir:="${dir}/params/localization.yaml"
            cloud_topic:="${cloud_topic}" imu_topic:="${imu_topic}"
            lidar_frame_id:="${frame}" lidar_tf_x:=0.10 lidar_tf_y:=0.0 lidar_tf_z:=0.20)
    exe=lib/lidar_localization_ros2/lidar_localization_node
    pose_topic=/pcl_pose
    ready() { ros2 lifecycle get /lidar_localization 2> /dev/null | grep -q '^active'; }
    ;;
  *)
    echo "${usage}" >&2
    exit 2
    ;;
esac
printf '%q ' "${launch[@]}" > "${dir}/launch.txt"
echo >> "${dir}/launch.txt"

# Background jobs of a script ignore SIGINT, so they are stopped with SIGTERM.
"${launch[@]}" > "${dir}/node.log" 2>&1 &
launch_pid=$!
recorder=
cleanup() {
  if [[ -n "${recorder}" ]]; then kill -TERM "${recorder}" 2> /dev/null || true; fi
  kill -TERM "${launch_pid}" 2> /dev/null || true
  wait "${launch_pid}" 2> /dev/null || true
}
trap cleanup EXIT
# bash is PID 1 in the container, which ignores signals without a handler: exit so that the trap runs.
trap 'exit 130' INT TERM
for _ in $(seq 240); do
  ready && break
  kill -0 "${launch_pid}" 2> /dev/null || { tail -n 30 "${dir}/node.log" >&2; echo "FAIL: the launch exited" >&2; exit 1; }
  sleep 0.5
done
ready || { tail -n 30 "${dir}/node.log" >&2; echo "FAIL: the node did not become active" >&2; exit 1; }
node_pid="$(pgrep -f "/${exe}( |$)" | head -n 1)"
[[ -n "${node_pid}" ]] || { echo "FAIL: no ${exe} process" >&2; exit 1; }
echo "bench_run: ${method} run ${run}: node pid ${node_pid} active"

"${repo_root}/sim/scripts/bench_recorder.py" --pid "${node_pid}" --cloud-topic "${cloud_topic}" \
  --pose-topic "${pose_topic}" --out "${dir}" --ready-file "${dir}/.recorder_ready" > "${dir}/recorder.log" 2>&1 &
recorder=$!
for _ in $(seq 60); do
  test -e "${dir}/.recorder_ready" && break
  sleep 0.5
done
test -e "${dir}/.recorder_ready" || { cat "${dir}/recorder.log" >&2; echo "FAIL: the recorder did not start" >&2; exit 1; }
sleep 2  # discovery between the recorder, the node and the player
ros2 bag play "${bag}" --clock 100 --rate 1.0 --delay 2 \
  --qos-profile-overrides-path "${repo_root}/sim/config/bench_play_qos.yaml" \
  --topics "${cloud_topic}" "${imu_topic}" > "${dir}/play.log" 2>&1
sleep 3  # the last scans
kill -0 "${node_pid}" 2> /dev/null || echo "bench_run: WARNING: the node exited during the run" >&2
kill -TERM "${recorder}"
wait "${recorder}" || { cat "${dir}/recorder.log" >&2; echo "FAIL: the recorder" >&2; exit 1; }
recorder=
tail -n 1 "${dir}/recorder.log"
