#!/bin/bash
# Localize with glim_localization_node (R2) on the restamped bag, for the RViz2 capture of
# scripts/sim_viz.sh (inside the development image, glim_localization:dev):
#   sim/scripts/ros_localize.sh <out dir> <rate>
# Needs <out>/config and <out>/map from localize.sh and <out>/viz/bag from restamp_bag.py. Builds the
# core and the ROS packages first when they are not built. Writes <out>/viz/config (the localize.sh
# config with the R2 config_ros.json, the sim topics and the initial pose) and <out>/viz/logs.
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
out="${1:?usage: ros_localize.sh <out dir> <rate>}"
rate="${2:?usage: ros_localize.sh <out dir> <rate>}"
viz="${out}/viz"
: "${GLIM_LOC_PYTHON:?GLIM_LOC_PYTHON is not set (run inside the dev container)}"
for f in config map traj_gt.txt viz/bag; do
  test -e "${out}/${f}" || { echo "FAIL: ${out}/${f} is missing (run scripts/sim_eval.sh first)" >&2; exit 1; }
done
mkdir -p "${viz}/logs"

if [[ ! -f /ws/build/colcon/install/setup.bash ]]; then
  echo "ros_localize: building the core and the ROS packages (logs/build.log)"
  { "${repo_root}/scripts/build_core.sh" && "${repo_root}/scripts/build_colcon.sh"; } \
    > "${viz}/logs/build.log" 2>&1 || { echo "FAIL: build (${viz}/logs/build.log)" >&2; exit 1; }
fi
source /ws/build/colcon/install/setup.bash

# Config: the localize.sh one (rig extrinsic) with the R2 config_ros.json. The gpu_lidar clouds have
# no time field: a global shutter LiDAR. The initial pose is the ground truth at the first frame. ~/map
# keeps one point per 0.2 m voxel instead of 0.5 m, for a denser map in the picture.
rm -rf "${viz}/config"
cp -r "${out}/config" "${viz}/config"
cp "${repo_root}/glim_localization_ros2/config/r2/config_ros.json" "${viz}/config/"
env -u PYTHONPATH "${GLIM_LOC_PYTHON}" - "${viz}/config" "${out}/traj_gt.txt" <<'PY'
import re
import sys
d, gt = sys.argv[1], sys.argv[2]
row = next(l for l in open(gt) if l.strip() and not l.startswith("#")).split()
edits = {
    "config_ros.json": [(r'"imu_topic": "[^"]*"', '"imu_topic": "/sim_eval/imu"'),
                        (r'"map_cloud_voxel": [0-9.]+', '"map_cloud_voxel": 0.2'),
                        (r'"points_topic": "[^"]*"', '"points_topic": "/sim_eval/lidar/points"')],
    "config_sensors.json": [(r'"global_shutter_lidar": \w+', '"global_shutter_lidar": true')],
    "config_odometry_localization_cpu.json": [
        (r'"map_path": "[^"]*",', '"map_path": "",\n    "init_T_world_imu": [' + ", ".join(row[1:8]) + "],")],
}
for name, subs in edits.items():
    path = f"{d}/{name}"
    text = open(path).read()
    for pattern, repl in subs:
        text, n = re.subn(pattern, repl, text, count=1)
        assert n == 1, (name, pattern)
    open(path, "w").write(text)
PY

# Background jobs of a script ignore SIGINT, so they are stopped with SIGTERM.
ros2 launch glim_localization_ros2 localization.launch.py use_sim_time:=true \
  config_path:="${viz}/config" map_path:="${out}/map" > "${viz}/logs/node.log" 2>&1 &
node=$!
trap 'kill -TERM ${node} 2>/dev/null || true' EXIT
# bash is PID 1 in the container, which ignores signals without a handler: exit so that the trap runs.
trap 'exit 130' INT TERM
for _ in $(seq 120); do
  grep -q "active: waiting for" "${viz}/logs/node.log" && break
  kill -0 ${node} 2>/dev/null || { cat "${viz}/logs/node.log" >&2; exit 1; }
  sleep 0.5
done
grep -q "active: waiting for" "${viz}/logs/node.log" || { echo "FAIL: the node did not start" >&2; exit 1; }
# Time for RViz2 in the other container to discover the node and take the latched ~/map
sleep "${SIM_VIZ_SETTLE:-5}"
echo "ros_localize: playing the bag at rate ${rate}"
ros2 bag play "${viz}/bag" --clock 100 --rate "${rate}" > "${viz}/logs/play.log" 2>&1
sleep 2
kill -0 ${node} 2>/dev/null || { tail -n 20 "${viz}/logs/node.log" >&2; echo "FAIL: the node exited" >&2; exit 1; }
grep -E "\[glim_localization\]: (state|LOST|reset)" "${viz}/logs/node.log" | tail -n 5 || true
echo "ros_localize: done"
