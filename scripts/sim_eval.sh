#!/bin/bash
# Gazebo simulation evaluation, Step 1 of issue 9 (docs/sim_eval.md). Runs on the host, never in CI:
#   scripts/sim_eval.sh [out name]
# From the world to eval.json in out/<out name> (default out/sim_eval), which is removed first:
#   1. record   (sim image)  Gazebo Harmonic, scripted 60 s run -> rosbag2 + ground truth (TUM, T_world_imu)
#   2. map      (sim image)  sdf2map: the world SDF -> map.pcd
#   3. localize (dev image)  bag_to_replay.py --gt, data checks, map_compiler, replay, evaluate.py (no alignment)
# Images: glim_localization:sim (docker/sim/Dockerfile, built when missing) and glim_localization:dev.
# Set COMPOSE_PROJECT_NAME to use another compose project (volumes and network) than glim_localization_sim.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
name="${1:-sim_eval}"
[[ "${name}" =~ ^[A-Za-z0-9_.-]+$ && "${name}" != . && "${name}" != .. ]] \
  || { echo "usage: sim_eval.sh [out name] (letters, digits, _ . -)" >&2; exit 2; }
host_out="${repo_root}/out/${name}"
out="/ws/out/${name}"
compose=(docker compose -f "${repo_root}/docker/sim/compose.yaml")

tty_args=()
if [[ ! -t 0 ]]; then
  tty_args=(-T)
fi
run() {  # <service> <command...>
  "${compose[@]}" run --rm ${tty_args[@]+"${tty_args[@]}"} "$@"
}

# Only a directory this script created (marker) is removed, so nothing else under out/ is ever deleted.
marker="${host_out}/.sim_eval_out"
if [[ -e "${host_out}" && ! -f "${marker}" ]]; then
  echo "FAIL: ${host_out} exists and was not created by sim_eval.sh; choose another out name" >&2
  exit 1
fi
rm -rf "${host_out}"
mkdir -p "${host_out}"
echo "created by scripts/sim_eval.sh; removed at the start of every run" > "${marker}"

if ! docker image inspect "${GLIM_LOC_SIM_IMAGE:-glim_localization:sim}" > /dev/null 2>&1; then
  echo "sim_eval: building the simulation image"
  "${compose[@]}" build sim
fi

start=${SECONDS}
echo "sim_eval: 1/3 record"
run sim sim/scripts/record.sh "${out}"
echo "sim_eval: 2/3 map (sdf2map)"
run sim ros2 run sdf2map sdf2map --input sim/worlds/sim_eval_room.sdf --output "${out}/map.pcd" \
  --model-path sim/models --exclude '^rig$' --no-download --density 400 --voxel 0.05 --seed 42
echo "sim_eval: 3/3 localize and evaluate"
run loc sim/scripts/localize.sh "${out}"
echo "sim_eval: done in $((SECONDS - start)) s; results in out/${name}/run (eval.json, eval.txt)"
