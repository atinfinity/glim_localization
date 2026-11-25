#!/bin/bash
# Convert, compile the map, localize and evaluate (inside the development image, glim_localization:dev).
#   sim/scripts/localize.sh <out dir>
# Needs <out>/bag and <out>/traj_gt.txt from record.sh and <out>/map.pcd from sdf2map. Writes
# <out>/dataset, <out>/map, <out>/config, <out>/run (replay output, eval.json, eval.txt) and
# <out>/run/status_sequence.json. Builds the core first when it is not built.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
out="${1:?usage: localize.sh <out dir>}"
build_dir=/ws/build/core
: "${GLIM_LOC_PYTHON:?GLIM_LOC_PYTHON is not set (run inside the dev container)}"
export PYTHONDONTWRITEBYTECODE=1
py() { env -u PYTHONPATH "${GLIM_LOC_PYTHON}" "$@"; }

for f in bag traj_gt.txt map.pcd; do
  test -e "${out}/${f}" || { echo "FAIL: ${out}/${f} is missing" >&2; exit 1; }
done
rm -rf "${out}/dataset" "${out}/map" "${out}/config" "${out}/run"
mkdir -p "${out}/logs"

if [[ ! -x "${build_dir}/tools/glim_localization_replay" || ! -x "${build_dir}/tools/map_compiler" ]]; then
  echo "localize: building the core (scripts/build_core.sh)"
  "${repo_root}/scripts/build_core.sh" > "${out}/logs/build_core.log" 2>&1 \
    || { echo "FAIL: build_core.sh (logs/build_core.log)" >&2; exit 1; }
fi

# 1. Replay dataset. gpu_lidar clouds have no time field (instantaneous scan, all t = 0) and non-finite
#    points for missing returns, which bag_to_replay.py drops. The ground truth is copied as traj_gt.txt.
py "${repo_root}/scripts/bag_to_replay.py" "${out}/bag" --out "${out}/dataset" \
  --points-topic /sim_eval/lidar/points --imu-topic /sim_eval/imu --gt "${out}/traj_gt.txt" \
  > "${out}/logs/bag_to_replay.log" 2>&1 || { cat "${out}/logs/bag_to_replay.log" >&2; exit 1; }
tail -n 3 "${out}/logs/bag_to_replay.log"

# 2. Map from the sdf2map PCD (world frame = map frame)
"${build_dir}/tools/map_compiler" "${out}/map.pcd" "${out}/map" --grid-size 50 --voxel-size 0.1 \
  > "${out}/logs/map_compiler.log" 2>&1 || { cat "${out}/logs/map_compiler.log" >&2; exit 1; }

# Data checks: sensor rates in simulation time, and scans on the map at the ground-truth poses
py "${repo_root}/sim/scripts/check_data.py" "${out}/dataset" "${out}/map.pcd" --out "${out}/check_data.json" \
  || { echo "FAIL: data checks (check_data.json)" >&2; exit 1; }

# 3. Config: the shipped CPU config with the rig extrinsic (sim/models/sim_eval_rig/model.sdf): the LiDAR
#    is at (0.10, 0, 0.20) in the IMU frame with the same axes, so T_lidar_imu = (-0.10, 0, -0.20).
mkdir "${out}/config"
cp "${repo_root}"/glim_localization/config/*.json "${out}/config/"
py - "${out}/config/config_sensors.json" <<'PY'
import re
import sys
path = sys.argv[1]
text = open(path).read()
new, n = re.subn(r'"T_lidar_imu"\s*:\s*\[[^\]]*\]', '"T_lidar_imu": [-0.10, 0.0, -0.20, 0.0, 0.0, 0.0, 1.0]', text)
assert n == 1, "T_lidar_imu not found"
open(path, "w").write(new)
PY

# 4. Localize from the exact initial pose: the ground truth at the first frame (the rig is still for 2 s).
init_pose="$(py - "${out}/dataset/traj_gt.txt" <<'PY'
import sys
row = next(l for l in open(sys.argv[1]) if l.strip() and not l.startswith("#")).split()
print(" ".join(row[1:8]))
PY
)"
echo "localize: initial pose ${init_pose}"
replay_status=0
# shellcheck disable=SC2086
"${build_dir}/tools/glim_localization_replay" --data "${out}/dataset" --config "${out}/config" \
  --map "${out}/map" --out "${out}/run" --init-pose ${init_pose} > "${out}/logs/replay.log" 2>&1 || replay_status=$?
echo "localize: replay exit code ${replay_status}"

# 5. Status sequence (run-length encoded states of frames.csv)
py - "${out}/run/frames.csv" "${out}/run/status_sequence.json" <<'PY'
import csv
import json
import sys
rows = list(csv.DictReader(open(sys.argv[1])))
runs = []
for r in rows:
    t = int(r["stamp_ns"]) * 1e-9
    if runs and runs[-1]["state"] == r["state"]:
        runs[-1]["frames"] += 1
        runs[-1]["end_s"] = t
    else:
        runs.append({"state": r["state"], "frames": 1, "start_s": t, "end_s": t})
json.dump(runs, open(sys.argv[2], "w"), indent=2)
print("localize: states " + " -> ".join(f"{r['state']} x{r['frames']}" for r in runs))
PY

# 6. Evaluate without alignment (evaluate.py never aligns; the map frame is the world frame)
eval_status=0
py "${repo_root}/scripts/evaluate.py" --est "${out}/run/traj_localization.txt" --gt "${out}/dataset/traj_gt.txt" \
  --frames "${out}/run/frames.csv" --out "${out}/run" --plot || eval_status=$?
if [[ "${replay_status}" -ne 0 || "${eval_status}" -ne 0 ]]; then
  echo "FAIL: replay exit ${replay_status}, evaluate exit ${eval_status}" >&2
  exit 1
fi
echo "PASS: localize and evaluate (${out}/run/eval.json)"
