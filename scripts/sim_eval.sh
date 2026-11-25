#!/bin/bash
# Gazebo simulation evaluation, Step 1 of issue 9 (docs/sim_eval.md). Runs on the host, never in CI:
#   scripts/sim_eval.sh [out name]
# From the world to eval.json in out/<out name> (default out/sim_eval), which is emptied first:
#   1. record   (sim image)  Gazebo Harmonic, scripted 60 s run -> rosbag2 + ground truth (TUM, T_world_imu)
#   2. map      (sim image)  sdf2map: the world SDF -> map.pcd
#   3. localize (dev image)  bag_to_replay.py --gt, data checks, map_compiler, replay, evaluate.py (no alignment)
# Images: glim_localization:sim (docker/sim/Dockerfile, built or refreshed from the cache every run) and
# glim_localization:dev. Environment passed to the containers: SIM_DURATION, SIM_SEED, SIM_TIMEOUT
# (record.sh), SIM_EVAL_MAX_TRANS, SIM_EVAL_MAX_ROT_DEG (localize.sh). COMPOSE_PROJECT_NAME selects the
# compose project (its volumes and network); the default is glim_localization_sim.
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

# Only a directory this script created (marker) is emptied, so nothing else under out/ is ever deleted.
# The outputs belong to the container user (root with rootful Docker on Linux), so they are deleted inside
# a container, not on the host.
marker_name=.sim_eval_out
marker="${host_out}/${marker_name}"
if [[ -L "${host_out}" || -L "${marker}" ]]; then
  echo "FAIL: ${host_out} or its marker is a symlink" >&2
  exit 1
fi
if [[ -e "${host_out}" && ! -f "${marker}" ]]; then
  echo "FAIL: ${host_out} exists and was not created by sim_eval.sh (no ${marker_name}); choose another out name" >&2
  exit 1
fi
mkdir -p "${repo_root}/out"

"${compose[@]}" build sim  # cheap when cached; picks up changes of docker/sim/

run sim bash -c 'set -e; rm -rf "$1"; mkdir -p "$1"
  echo "created by scripts/sim_eval.sh; emptied at the start of every run" > "$1/$2"' _ "${out}" "${marker_name}"

start=${SECONDS}
echo "sim_eval: 1/3 record"
run sim sim/scripts/record.sh "${out}"
echo "sim_eval: 2/3 map (sdf2map)"
run sim ros2 run sdf2map sdf2map --input sim/worlds/sim_eval_room.sdf --output "${out}/map.pcd" \
  --model-path sim/models --exclude '^rig$' --no-download --density 400 --voxel 0.05 --seed 42
echo "sim_eval: 3/3 localize and evaluate"
run loc sim/scripts/localize.sh "${out}"
echo "sim_eval: done in $((SECONDS - start)) s; results in out/${name}/run (eval.json, eval.txt)"
