#!/bin/bash
# RViz2 capture of glim_localization_node on the Gazebo bag (docs/sim_eval.md). Runs on the host, never
# in CI, after scripts/sim_eval.sh:
#   scripts/sim_viz.sh [out name] [rate]
# Uses out/<out name> (default out/sim_eval) of sim_eval.sh and writes out/<out name>/viz:
#   1. restamp (sim image)  restamp_bag.py: the bag with log time = header stamp -> viz/bag
#   2. capture (sim image, detached)  viz_capture.sh: Xvfb + rviz2 + viz_paths.py, records the screen
#   3. localize (dev image)  ros_localize.sh: glim_localization_node, `ros2 bag play --clock` at <rate>
#      (default 1.0)
# Result: viz/sim_viz.webp (three times as fast as recorded, SIM_VIZ_SPEEDUP) and viz/sim_viz.png. The two
# containers are on the compose network and discover each other with ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
name="${1:-sim_eval}"
rate="${2:-1.0}"
[[ "${name}" =~ ^[A-Za-z0-9_.-]+$ && "${name}" != . && "${name}" != .. ]] \
  || { echo "usage: sim_viz.sh [out name] [rate] (name: letters, digits, _ . -)" >&2; exit 2; }
[[ "${rate}" =~ ^[0-9]+(\.[0-9]+)?$ && ! "${rate}" =~ ^0+(\.0+)?$ ]] \
  || { echo "sim_viz: rate must be a positive number" >&2; exit 2; }
host_out="${repo_root}/out/${name}"
out="/ws/out/${name}"
for f in .sim_eval_out record.json config map traj_gt.txt bag; do
  test -e "${host_out}/${f}" || { echo "FAIL: ${host_out}/${f} is missing (run scripts/sim_eval.sh ${name})" >&2; exit 1; }
done
compose=(docker compose -f "${repo_root}/docker/sim/compose.yaml")
project="${COMPOSE_PROJECT_NAME:-glim_localization_sim}"
ros_env=(-e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET)
capture="${project}-viz-capture"
run() {  # <service> <command...>
  "${compose[@]}" run --rm -T "$@"
}

duration="$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["sim_duration_s"])' "${host_out}/record.json")"
seconds="$(python3 -c 'import sys; print(round(float(sys.argv[1]) / float(sys.argv[2]) + 2, 1))' "${duration}" "${rate}")"
speedup="${SIM_VIZ_SPEEDUP:-3}"

"${compose[@]}" build sim
start=${SECONDS}
echo "sim_viz: 1/3 restamp the bag"
run sim bash -c 'rm -rf "$1/viz/bag" && mkdir -p "$1/viz" && python3 sim/scripts/restamp_bag.py "$1/bag" "$1/viz/bag"' _ "${out}"

echo "sim_viz: 2/3 start the capture (${seconds} s)"
docker rm -f "${capture}" > /dev/null 2>&1 || true
trap 'docker rm -f "${capture}" > /dev/null 2>&1 || true' EXIT
"${compose[@]}" run -d --name "${capture}" "${ros_env[@]}" \
  -e SIM_VIZ_CROP -e SIM_VIZ_QUALITY -e SIM_VIZ_WAIT \
  sim sim/scripts/viz_capture.sh "${out}" "${seconds}" "${speedup}" > /dev/null

echo "sim_viz: 3/3 localize with glim_localization_node (rate ${rate})"
run "${ros_env[@]}" -e SIM_VIZ_SETTLE loc sim/scripts/ros_localize.sh "${out}" "${rate}"

# The playback is over: the capture needs its recording time and the encoding, not the /clock wait.
# Without a limit, a capture that never saw /clock (no discovery) would block for SIM_VIZ_WAIT.
status="$(timeout "$(python3 -c 'import sys; print(int(float(sys.argv[1])) + 300)' "${seconds}")" \
  docker wait "${capture}")" || { echo "FAIL: the capture did not finish (out/${name}/viz/logs)" >&2; exit 1; }
docker logs "${capture}" 2>&1 | tail -n 5
[[ "${status}" == 0 ]] || { echo "FAIL: the capture exited with ${status} (out/${name}/viz/logs)" >&2; exit 1; }
echo "sim_viz: done in $((SECONDS - start)) s; out/${name}/viz/sim_viz.webp"
