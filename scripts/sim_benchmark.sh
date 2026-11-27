#!/bin/bash
# Benchmark against lidar_localization_ros2 (NDT) on the Gazebo bag, step 1 of issue 80 (docs/benchmark.md).
# Runs on the host, never in CI, after scripts/sim_eval.sh:
#   scripts/sim_benchmark.sh [out name] [runs]
# Uses out/<out name> (default out/sim_eval) of sim_eval.sh and writes out/<out name>/bench, which is
# replaced:
#   1. images   build glim_localization:baseline-lidarloc (docker/baseline/Dockerfile) and refresh the sim image
#   2. restamp  (sim image) restamp_bag.py: the bag with log time = header stamp -> bench/bag, once
#   3. build    (dev image) scripts/build_core.sh and scripts/build_colcon.sh, incremental
#   4. runs     sim/scripts/bench_run.sh for glim (dev image), lidarloc and lidarloc_noimu (baseline image),
#               <runs> times each (default 3), one at a time: run 1 of every method, then run 2, ...
#   5. compare  (dev image) scripts/compare_methods.py -> bench/benchmark.json, benchmark.md, benchmark.svg
# Every run plays the bag at 1x with its node, the recorder and the player in one container
# (ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST). bench/host.json records the CPU, nproc and the image IDs.
# COMPOSE_PROJECT_NAME selects the compose project (its volumes); the default is glim_localization_sim.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
name="${1:-sim_eval}"
runs="${2:-3}"
[[ "${name}" =~ ^[A-Za-z0-9_.-]+$ && "${name}" != . && "${name}" != .. ]] \
  || { echo "usage: sim_benchmark.sh [out name] [runs] (name: letters, digits, _ . -)" >&2; exit 2; }
[[ "${runs}" =~ ^[1-9][0-9]*$ ]] || { echo "sim_benchmark: runs must be a positive integer" >&2; exit 2; }
host_out="${repo_root}/out/${name}"
out="/ws/out/${name}"
for f in .sim_eval_out config map map_export.pcd traj_gt.txt bag; do
  test -e "${host_out}/${f}" \
    || { echo "FAIL: ${host_out}/${f} is missing (run scripts/sim_eval.sh ${name})" >&2; exit 1; }
done
compose=(docker compose -f "${repo_root}/docker/sim/compose.yaml")
local_ros=(-e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
run() {  # <service> <command...>
  "${compose[@]}" run --rm -T "$@"
}

start=${SECONDS}
echo "sim_benchmark: 1/5 images"
"${compose[@]}" build sim baseline
echo "sim_benchmark: 2/5 restamp the bag"
run sim bash -c 'rm -rf "$1/bench" && mkdir -p "$1/bench" && python3 sim/scripts/restamp_bag.py "$1/bag" "$1/bench/bag"' \
  _ "${out}"
echo "sim_benchmark: 3/5 build the core and the ROS packages (bench/build.log)"
run loc bash -c '{ scripts/build_core.sh && scripts/build_colcon.sh; } > "$1/bench/build.log" 2>&1' _ "${out}" \
  || { echo "FAIL: build (out/${name}/bench/build.log)" >&2; exit 1; }

# The machine and the images, written inside a container: out/ belongs to the container user.
image_id() { docker image inspect --format '{{.Id}}' "$1"; }
dev_image="${GLIM_LOC_IMAGE:-glim_localization:dev}"
baseline_image="${GLIM_LOC_BASELINE_IMAGE:-glim_localization:baseline-lidarloc}"
python3 - "$(nproc)" "$(LC_ALL=C lscpu | sed -n 's/^Model name: *//p' | head -n 1)" "${dev_image}" "$(image_id "${dev_image}")" \
  "${baseline_image}" "$(image_id "${baseline_image}")" "$(git -C "${repo_root}" rev-parse HEAD)" "${runs}" <<'PY' \
  | run sim bash -c 'cat > "$1/bench/host.json"' _ "${out}"
import json
import sys
nproc, cpu, dev, dev_id, base, base_id, commit, runs = sys.argv[1:]
print(json.dumps({"cpu": cpu, "nproc": int(nproc), "images": {dev: dev_id, base: base_id}, "commit": commit,
                  "runs": int(runs)}, indent=2))
PY

echo "sim_benchmark: 4/5 runs (${runs} per method, one at a time)"
for k in $(seq "${runs}"); do
  run "${local_ros[@]}" loc sim/scripts/bench_run.sh "${out}" glim "${k}"
  run baseline sim/scripts/bench_run.sh "${out}" lidarloc "${k}"
  run baseline sim/scripts/bench_run.sh "${out}" lidarloc_noimu "${k}"
done

echo "sim_benchmark: 5/5 compare"
run loc bash -c 'env -u PYTHONPATH "${GLIM_LOC_PYTHON}" scripts/compare_methods.py --bench "$1/bench" \
  --gt "$1/traj_gt.txt" --settle 2 --svg "$1/bench/benchmark.svg"' _ "${out}"
echo "sim_benchmark: done in $((SECONDS - start)) s; out/${name}/bench/benchmark.md"
