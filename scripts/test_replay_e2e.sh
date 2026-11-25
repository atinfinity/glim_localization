#!/bin/bash
# End to end: synthetic dataset -> glim_localization_replay -> evaluate.py (inside the container).
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir=/ws/build/core
out=/ws/out/phase4_e2e
: "${GLIM_LOC_PYTHON:?GLIM_LOC_PYTHON is not set (run inside the dev container)}"
export PYTHONDONTWRITEBYTECODE=1

py() { env -u PYTHONPATH "${GLIM_LOC_PYTHON}" "$@"; }

replay="${build_dir}/tools/glim_localization_replay"
test -x "${replay}" || { echo "FAIL: ${replay} not built (run build_core.sh first)" >&2; exit 1; }

rm -rf "${out}"
mkdir -p "${out}"
"${build_dir}/test/synthetic_replay_dataset" "${out}/input" 12

# Initial pose: the ground truth at the first frame, offset by (0.3, -0.2, 0.1) m and 3 degrees about z.
init_pose="$(py - "${out}/input/data/traj_gt.txt" <<'PY'
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
row = next(l for l in open(sys.argv[1]) if l.strip() and not l.startswith("#")).split()
p = np.array(row[1:4], dtype=float)
r = R.from_quat([float(v) for v in row[4:8]])
p = p + np.array([0.3, -0.2, 0.1])
r = R.from_euler("z", 3.0, degrees=True) * r
print(" ".join(f"{v:.17g}" for v in [*p, *r.as_quat()]))
PY
)"

run_replay() {  # <out dir> [extra args...]
  local dest="$1"; shift
  "${replay}" --data "${out}/input/data" --config "${repo_root}/glim_localization/config" \
    --map "${out}/input/map" --out "${dest}" "$@" > "${dest}.log" 2>&1
}

# shellcheck disable=SC2086
run_replay "${out}/run1" --init-pose ${init_pose}
run_replay "${out}/run2" --init-pose ${init_pose}

# Accuracy
py "${repo_root}/scripts/evaluate.py" --est "${out}/run1/traj_localization.txt" --gt "${out}/input/data/traj_gt.txt" \
  --frames "${out}/run1/frames.csv" --out "${out}/run1" --settle 3 --max-trans 0.1 --max-rot-deg 1

# The same scene as a PCD: map_compiler tiles it and estimates covariances, and replay localizes on the result.
"${build_dir}/tools/map_compiler" "${out}/input/map.pcd" "${out}/pcd_map" > "${out}/pcd_map.log" 2>&1
# shellcheck disable=SC2086
"${replay}" --data "${out}/input/data" --config "${repo_root}/glim_localization/config" \
  --map "${out}/pcd_map" --out "${out}/run_pcd" --init-pose ${init_pose} > "${out}/run_pcd.log" 2>&1
py "${repo_root}/scripts/evaluate.py" --est "${out}/run_pcd/traj_localization.txt" --gt "${out}/input/data/traj_gt.txt" \
  --frames "${out}/run_pcd/frames.csv" --out "${out}/run_pcd" --settle 3 --max-trans 0.1 --max-rot-deg 1

# Determinism: identical stamp/result columns, poses within 1e-3 m
py - "${out}/run1" "${out}/run2" <<'PY'
import csv
import sys
import numpy as np

def frames(d):
    with open(f"{d}/frames.csv") as f:
        rows = list(csv.DictReader(f))
    return [(r["stamp_ns"], r["result"]) for r in rows]

def poses(d):
    return np.loadtxt(f"{d}/traj_localization.txt")

a, b = sys.argv[1:3]
assert frames(a) == frames(b), "frames.csv stamp_ns,result differ between runs"
pa, pb = poses(a), poses(b)
assert pa.shape == pb.shape, "different number of poses"
diff = np.abs(pa[:, 1:4] - pb[:, 1:4]).max()
assert diff < 1e-3, f"poses differ by {diff} m"
print(f"determinism ok: {len(pa)} poses, max difference {diff:.3g} m")
PY

# --init-stamp: earlier frames are dropped and the first pose is at or after the stamp
run_replay "${out}/run_stamp" --init-pose ${init_pose} --init-stamp 2.0
py - "${out}/run_stamp" <<'PY'
import csv
import sys
import numpy as np
d = sys.argv[1]
rows = list(csv.DictReader(open(f"{d}/frames.csv")))
early = [r for r in rows if int(r["stamp_ns"]) < 2_000_000_000]
assert early and all(r["result"] == "dropped" for r in early), "frames before --init-stamp are not all dropped"
first = np.loadtxt(f"{d}/traj_localization.txt")[0, 0]
assert first >= 2.0, f"first pose at {first} s"
print(f"init-stamp ok: {len(early)} dropped frames, first pose at {first:.3f} s")
PY

# --init-stamp with an IMU that does not cover the earlier frames: they are `dropped` (not skipped_no_imu),
# the settle window starts at init-stamp + settle, and evaluate passes with full coverage.
init_pose_at() {  # <t seconds>: the ground truth pose nearest to t, offset like init_pose
  py - "${out}/input/data/traj_gt.txt" "$1" <<'PY'
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
rows = [l.split() for l in open(sys.argv[1]) if l.strip() and not l.startswith("#")]
row = min(rows, key=lambda r: abs(float(r[0]) - float(sys.argv[2])))
p = np.array(row[1:4], dtype=float) + np.array([0.3, -0.2, 0.1])
r = R.from_euler("z", 3.0, degrees=True) * R.from_quat([float(v) for v in row[4:8]])
print(" ".join(f"{v:.17g}" for v in [*p, *r.as_quat()]))
PY
}
imu_variant() {  # <name> <python expression on the stamp in ns: keep the sample if true>
  rm -rf "${out}/$1"
  cp -r "${out}/input/data" "${out}/$1"
  py - "${out}/$1" "$2" <<'PY'
import json
import sys
d, keep = sys.argv[1:3]
lines = open(f"{d}/imu.csv").read().splitlines()
rows = [l for l in lines[1:] if eval(keep, {"ns": int(l.split(",")[0])})]
open(f"{d}/imu.csv", "w").write("\n".join([lines[0], *rows]) + "\n")
meta = json.load(open(f"{d}/meta.json"))
meta["num_imu"] = len(rows)
json.dump(meta, open(f"{d}/meta.json", "w"))
PY
}
check_init_stamp_hole() {  # <name> <init stamp> <expected window start>
  local data="${out}/$1" dest="${out}/run_$1"
  local pose
  pose="$(init_pose_at "$2")"
  # shellcheck disable=SC2086
  "${replay}" --data "${data}" --config "${repo_root}/glim_localization/config" --map "${out}/input/map" --out "${dest}" \
    --init-pose ${pose} --init-stamp "$2" > "${dest}.log" 2>&1
  py "${repo_root}/scripts/evaluate.py" --est "${dest}/traj_localization.txt" --gt "${out}/input/data/traj_gt.txt" \
    --frames "${dest}/frames.csv" --out "${dest}" --settle 3 --max-trans 0.1 --max-rot-deg 1 > "${dest}.eval.log"
  py - "${dest}" "$3" <<'PY'
import csv
import json
import sys
d, window_start = sys.argv[1], float(sys.argv[2])
rows = list(csv.DictReader(open(f"{d}/frames.csv")))
assert not any(r["result"] == "skipped_no_imu" for r in rows), "frames before --init-stamp are skipped_no_imu"
report = json.load(open(f"{d}/eval.json"))["trajectory"]
assert report["coverage"] == 1.0, f"coverage {report['coverage']}"
first_in_window = min(int(r["stamp_ns"]) for r in rows if int(r["stamp_ns"]) * 1e-9 >= window_start - 1e-9 and r["result"] == "estimated")
assert abs(first_in_window * 1e-9 - window_start) < 1e-6, f"window starts at {first_in_window * 1e-9}"
print(f"init-stamp hole ok: coverage {report['coverage']}, window starts at {window_start} s")
PY
}
imu_variant imu_from_5 "ns >= 5000000000"
check_init_stamp_hole imu_from_5 5.0 8.0
imu_variant imu_hole_4_6 "ns <= 4000000000 or ns >= 6000000000"
check_init_stamp_hole imu_hole_4_6 6.0 9.0

# GPU backend (CUDA image with a visible GPU): the same run with config/gpu, the same accuracy limits, and
# the per-frame odometry time of both backends. ci.sh sets GLIM_LOC_REQUIRE_GPU=1 when nvidia-smi lists a GPU.
if [[ "${GLIM_LOC_BUILD_WITH_CUDA:-OFF}" == ON && "${GLIM_LOC_REQUIRE_GPU:-0}" == 1 ]]; then
  # shellcheck disable=SC2086
  "${replay}" --data "${out}/input/data" --config "${repo_root}/glim_localization/config/gpu" \
    --map "${out}/input/map" --out "${out}/run_gpu" --init-pose ${init_pose} > "${out}/run_gpu.log" 2>&1
  py "${repo_root}/scripts/evaluate.py" --est "${out}/run_gpu/traj_localization.txt" --gt "${out}/input/data/traj_gt.txt" \
    --frames "${out}/run_gpu/frames.csv" --out "${out}/run_gpu" --settle 3 --max-trans 0.1 --max-rot-deg 1
  py - "${out}/run1" "${out}/run_gpu" <<'PY'
import json
import sys
cpu, gpu = sys.argv[1:3]
assert json.load(open(f"{cpu}/replay_args.json"))["effective"]["backend"] == "cpu"
assert json.load(open(f"{gpu}/replay_args.json"))["effective"]["backend"] == "gpu"
for name, d in (("cpu", cpu), ("gpu", gpu)):
    report = json.load(open(f"{d}/eval.json"))
    t, r, o = report["errors"]["ate_trans_m"], report["errors"]["ate_rot_deg"], report["timing"]["odometry_ms"]
    print(f"{name}: ATE trans rmse {t['rmse']:.4f} m, max {t['max']:.4f} m, rot max {r['max']:.3f} deg; "
          f"odometry mean {o['mean']:.1f} ms, p95 {o['p95']:.1f} ms")

# Same factors, same optimum: the two backends must agree far closer than the accuracy limit, so a GPU regression
# that stays inside it (a wrong level, a dropped tile, a weight not applied) still fails.
import numpy as np
a = np.loadtxt(f"{cpu}/traj_localization.txt", ndmin=2)
b = np.loadtxt(f"{gpu}/traj_localization.txt", ndmin=2)
assert a.shape == b.shape and len(a) > 0, f"cpu {a.shape} vs gpu {b.shape} poses"
assert np.allclose(a[:, 0], b[:, 0], rtol=0.0, atol=1e-6), "the backends estimated different frames"
dp = np.linalg.norm(a[:, 1:4] - b[:, 1:4], axis=1).max()
dot = np.clip(np.abs(np.sum(a[:, 4:8] * b[:, 4:8], axis=1)) / (np.linalg.norm(a[:, 4:8], axis=1) * np.linalg.norm(b[:, 4:8], axis=1)), 0.0, 1.0)
dr = np.degrees(2.0 * np.arccos(dot)).max()
print(f"cpu vs gpu over {len(a)} poses: max position diff {dp:.3e} m, max rotation diff {dr:.3e} deg")
assert dp < 1e-3, f"cpu and gpu trajectories differ by {dp} m"
assert dr < 0.05, f"cpu and gpu trajectories differ by {dr} deg"
PY
  echo "gpu replay ok"

  # Tile streaming: the corridor in 4 m tiles, so the tiles of the map factors change along the trajectory. The GPU
  # module with prefetch, and with a byte budget that holds about two frames' tiles (evicting the rest), must agree
  # with the CPU module on the same map as closely as above.
  "${build_dir}/tools/map_compiler" "${out}/input/map.pcd" "${out}/small_tiles" --grid-size 4 > "${out}/small_tiles.log" 2>&1
  small_tiles_config() {  # <dest> <key=value>...: a copy of the config with these odometry keys replaced or added
    local dest="$1"; shift
    rm -rf "${dest}"
    cp -r "${repo_root}/glim_localization/config" "${dest}"
    py - "${dest}" "$@" <<'PY'
import re
import sys
dest, *pairs = sys.argv[1:]
for name in ("config_odometry_localization_cpu.json", "config_odometry_localization_gpu.json"):
    text = open(f"{dest}/{name}").read()
    for key, value in (pair.split("=", 1) for pair in pairs):
        if key.startswith("map_gpu_") and "_cpu" in name:
            continue
        text, n = re.subn(rf'("{key}"\s*:\s*)[^,\n]+', lambda m: m.group(1) + value, text)
        assert n == 1, f"{key} not in {name}"
    open(f"{dest}/{name}", "w").write(text)
PY
  }
  small_common=(map_search_radius=12.0 map_max_submaps=12 map_gpu_max_tiles=48 map_gpu_prefetch_max_tiles=24)
  small_tiles_config "${out}/config_small_tiles" "${small_common[@]}"
  small_tiles_config "${out}/config_small_budget" "${small_common[@]}" "map_gpu_memory_budget_mb=${GLIM_LOC_E2E_SMALL_BUDGET_MB:-24}"
  for run in cpu:config_small_tiles gpu:config_small_tiles/gpu gpu_budget:config_small_budget/gpu; do
    # shellcheck disable=SC2086
    "${replay}" --data "${out}/input/data" --config "${out}/${run#*:}" --map "${out}/small_tiles" \
      --out "${out}/run_tiles_${run%%:*}" --init-pose ${init_pose} > "${out}/run_tiles_${run%%:*}.log" 2>&1
  done
  py - "${out}" <<'PY'
import re
import sys
import numpy as np
out = sys.argv[1]
cpu = np.loadtxt(f"{out}/run_tiles_cpu/traj_localization.txt", ndmin=2)
for name in ("gpu", "gpu_budget"):
    log = open(f"{out}/run_tiles_{name}.log").read()
    m = re.search(r"GPU map tiles: (.*)", log)
    assert m, f"no tile counters in run_tiles_{name}.log"
    stats = dict(kv.split("=") for kv in m.group(1).split())
    b = np.loadtxt(f"{out}/run_tiles_{name}/traj_localization.txt", ndmin=2)
    assert cpu.shape == b.shape and len(b) > 0, f"cpu {cpu.shape} vs {name} {b.shape} poses"
    dp = np.linalg.norm(cpu[:, 1:4] - b[:, 1:4], axis=1).max()
    qa, qb = cpu[:, 4:8], b[:, 4:8]
    dot = np.clip(np.abs(np.sum(qa * qb, axis=1)) / (np.linalg.norm(qa, axis=1) * np.linalg.norm(qb, axis=1)), 0.0, 1.0)
    dr = np.degrees(2.0 * np.arccos(dot)).max()
    print(f"small tiles, {name}: max diff to cpu {dp:.3e} m, {dr:.3e} deg; {m.group(1)}")
    assert dp < 1e-3, f"{name}: cpu and gpu differ by {dp} m"
    assert dr < 0.05, f"{name}: cpu and gpu differ by {dr} deg"
    assert int(stats["prefetched"]) > 0, f"{name}: nothing was prefetched"
    if name == "gpu_budget":
        assert int(stats["evictions"]) > 0, "the small budget evicted nothing"
        assert float(stats["peak_mb"]) <= float(stats["budget_mb"]) + 1e-6 or int(stats["over_budget"]) > 0, "over the budget"
PY
  echo "gpu tile streaming ok"
else
  echo "SKIPPED: GPU replay (not a CUDA build with a visible GPU)"
fi

# Exit codes
set +e
"${replay}" --data "${out}/input/data" --config "${repo_root}/glim_localization/config" --map "${out}/input/map" --out "${out}/no_init" >/dev/null 2>&1
rc_no_init=$?
"${replay}" --data "${out}/input/data" --config "${repo_root}/glim_localization/config" --map "${out}/input/map" --out "${out}/late" \
  --init-pose ${init_pose} --init-stamp 1000 >/dev/null 2>&1
rc_late=$?
set -e
[ "${rc_no_init}" = 2 ] || { echo "FAIL: missing initial pose exited ${rc_no_init}, expected 2" >&2; exit 1; }
[ "${rc_late}" = 4 ] || { echo "FAIL: --init-stamp after the end exited ${rc_late}, expected 4" >&2; exit 1; }

echo "PASS: replay e2e"
