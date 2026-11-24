#!/bin/bash
# Real-data smoke test: map a converted OS1-128 sequence with GLIM through the headless mapper,
# then localize on the compiled map and compare against GLIM's own trajectory. Run inside the container, never in CI:
#   scripts/docker_run.sh scripts/smoke_os1_128.sh <replay dataset dir> [out dir]
# The dataset comes from scripts/bag_to_replay.py. Output goes to the out dir (default /ws/out/smoke_os1_128/run).
#
# The reference is GLIM's own trajectory, so every number is a consistency figure against GLIM, not an accuracy.
# Environment: PRIOR_END (seconds after the first frame, default 80) and TAIL_START (default 85) set the held-out split;
# MAPPER_ARGS and CONFIG_PATCH are for the fallbacks (for example a smaller random_downsample_target).
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir=/ws/build/core
data="${1:?usage: smoke_os1_128.sh <replay dataset dir> [out dir]}"
out="${2:-/ws/out/smoke_os1_128/run}"
prior_end="${PRIOR_END:-80}"
tail_start="${TAIL_START:-85}"
: "${GLIM_LOC_PYTHON:?GLIM_LOC_PYTHON is not set (run inside the dev container)}"
export PYTHONDONTWRITEBYTECODE=1
py() { env -u PYTHONPATH "${GLIM_LOC_PYTHON}" "$@"; }

mapper="${build_dir}/tools/glim_localization_mapper"
compiler="${build_dir}/tools/map_compiler"
replay="${build_dir}/tools/glim_localization_replay"
for tool in "${mapper}" "${compiler}" "${replay}"; do
  test -x "${tool}" || { echo "FAIL: ${tool} not built (run build_core.sh first)" >&2; exit 1; }
done
test -f "${data}/imu.csv" || { echo "FAIL: ${data} is not a replay dataset" >&2; exit 1; }

# The out directory is canonical before any check (symlinks, "..", "//", relative paths)
out="$(realpath -m -- "${out}")"
data="$(realpath -- "${data}")"
within() {  # <path> <dir>: is <path> the dir or below it?
  [ "$1" = "$2" ] || case "$1" in "$2"/*) return 0 ;; *) return 1 ;; esac
}
if [ "${out}" = "/" ]; then
  echo "FAIL: refusing the output directory '/'" >&2; exit 1
fi
# Refuse the protected paths, and any directory that contains one of them (that includes every ancestor)
for protected in / /ws /ws/out "${repo_root}" "${HOME:-/nonexistent}" "${data}"; do
  if within "${protected}" "${out}"; then
    echo "FAIL: refusing the output directory '${out}': it is or contains '${protected}'" >&2; exit 1
  fi
done
if within "${out}" "${data}"; then
  echo "FAIL: refusing the output directory '${out}': it is inside the dataset" >&2; exit 1
fi
# Only a directory this script created (marker file) or an empty one is cleaned, so nothing else is ever deleted
marker="${out}/.smoke_os1_128_out"
if [ -e "${out}" ] && [ ! -d "${out}" ]; then
  echo "FAIL: '${out}' is not a directory" >&2; exit 1
fi
if [ -d "${out}" ] && [ ! -f "${marker}" ] && [ -n "$(ls -A -- "${out}")" ]; then
  echo "FAIL: '${out}' is not empty and was not created by this script (no marker); use a new out directory" >&2; exit 1
fi
if [ -L "${marker}" ]; then
  echo "FAIL: '${marker}' is a symlink" >&2; exit 1
fi
mkdir -p "${out}"
echo "created by scripts/smoke_os1_128.sh; its fixed output names are removed at the start of every run" > "${marker}"
# Remove only the fixed names this script writes (rm does not follow symlinks), so an earlier run's output can never mask a failed step
rm -rf "${out}/config_mapping" "${out}/config_localization" "${out}/map_full" "${out}/map_prefix" "${out}/replay_heldout" "${out}/replay_self" \
  "${out}/mapper_full.log" "${out}/mapper_prefix.log" "${out}/replay_heldout.log" "${out}/replay_self.log"
# Both configs get the OS1 extrinsic T_lidar_imu = (os_sensor -> os_imu from /tf_static); the shipped localization config has zeros
cp -r "${repo_root}/glim_localization/config/mapping_cpu" "${out}/config_mapping"
mkdir "${out}/config_localization"
cp "${repo_root}"/glim_localization/config/*.json "${out}/config_localization/"
py - "${out}/config_localization/config_sensors.json" <<'PY'
import re, sys
path = sys.argv[1]
text = open(path).read()
new, n = re.subn(r'"T_lidar_imu"\s*:\s*\[[^\]]*\]', '"T_lidar_imu": [0.006253, -0.011775, 0.007645, 0.0, 0.0, 0.0, 1.0]', text)
assert n == 1, "T_lidar_imu not found"
open(path, "w").write(new)
PY
if [ -n "${CONFIG_PATCH:-}" ]; then  # fallback hook, for example: random_downsample_target 5000
  py - "${out}/config_mapping/config_preprocess.json" ${CONFIG_PATCH} <<'PY'
import re, sys
path, key, value = sys.argv[1:4]
text = open(path).read()
new, n = re.subn(r'("%s"\s*:\s*)[^,\n]+' % re.escape(key), r'\g<1>%s' % value, text)
assert n == 1, key + " not found"
open(path, "w").write(new)
PY
fi

# First frame stamp [s], from the point file names (<stamp_ns>.bin)
t0="$(py - "${data}" <<'PY'
import os, sys
names = sorted(n for n in os.listdir(os.path.join(sys.argv[1], "points")) if n.endswith(".bin"))
print("%.9f" % (int(names[0][:-4]) * 1e-9))
PY
)"
at() { py -c "print('%.9f' % (${t0} + $1))"; }
echo "first frame ${t0}"

echo "== mapper: full sequence"
# Acceptance gates (see the real-data smoke test in docs/PROGRESS.md): at least 95 % of the frames in the trajectory and of the frames after initialization
check_mapper_coverage() {  # <label> <summary.json>
  py - "$1" "$2" <<'PY' || { echo "FAIL: the mapper ($1) trajectory_coverage gate (>= 0.95) was not met" >&2; exit 1; }
import json, math, sys
label, path = sys.argv[1:3]
value = json.load(open(path)).get("trajectory_coverage")
if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
    print("trajectory_coverage is missing or not a finite number: %r" % (value,))
    sys.exit(1)
print("mapper (%s) trajectory_coverage %.4f against the gate 0.95" % (label, value))
sys.exit(0 if value >= 0.95 else 1)
PY
}

check_replay_frames() {  # <label> <frames.csv>
  py - "$1" "$2" <<'PY' || { echo "FAIL: the replay ($1) estimated-frames gate (>= 95 % after initialization) was not met" >&2; exit 1; }
import csv, sys
label, path = sys.argv[1:3]
results = [row["result"] for row in csv.DictReader(open(path))]
# Frames from the first one that is not `dropped` (before --init-stamp) to the end; skipped_no_imu and diverged count as missing
start = next((i for i, r in enumerate(results) if r != "dropped"), None)
if start is None:
    print("no frame after initialization")
    sys.exit(1)
after = results[start:]
estimated = sum(1 for r in after if r == "estimated")
print("replay (%s): %d of %d frames after initialization are estimated (gate 95 %%)" % (label, estimated, len(after)))
sys.exit(0 if estimated * 100 >= 95 * len(after) else 1)
PY
}

run_mapper() {  # <label> <out dir> [mapper args...]
  local label="$1" dir="$2"; shift 2
  local status=0
  "${mapper}" --data "${data}" --config "${out}/config_mapping" --out "${dir}" "$@" ${MAPPER_ARGS:-} > "${out}/mapper_${label}.log" 2>&1 || status=$?
  grep -v "param \|use default_value" "${out}/mapper_${label}.log" | tail -n 15 || true
  if [ "${status}" -ne 0 ] || [ ! -f "${dir}/mapper_summary.json" ] || [ ! -d "${dir}/dump" ] || grep -q '"failed": true' "${dir}/mapper_summary.json"; then
    echo "FAIL: the mapper (${label}) exited ${status}, left no dump and summary, or reported failed" >&2
    exit 1
  fi
}
run_mapper full "${out}/map_full"
check_mapper_coverage full "${out}/map_full/mapper_summary.json"
cat "${out}/map_full/mapper_summary.json"

# Is there a revisit? The tail must pass within 10 m of the prefix poses for at least 15 s
revisit="$(py - "${out}/map_full/traj_glim.txt" "$(at 0)" "$(at "${prior_end}")" "$(at "${tail_start}")" <<'PY'
import sys
import numpy as np
traj = np.loadtxt(sys.argv[1])
t0, t_prior, t_tail = (float(v) for v in sys.argv[2:5])
prior = traj[(traj[:, 0] >= t0) & (traj[:, 0] <= t_prior)]
tail = traj[traj[:, 0] >= t_tail]
if len(prior) == 0 or len(tail) == 0:
    print("no 0")
    sys.exit(0)
dist = np.linalg.norm(tail[:, None, 1:4] - prior[None, :, 1:4], axis=2).min(axis=1)
near = dist <= 10.0
seconds = near.sum() * np.median(np.diff(tail[:, 0]))
print("%s %.1f" % ("yes" if seconds >= 15.0 else "no", seconds))
PY
)"
echo "revisit check (tail within 10 m of the prefix, >= 15 s): ${revisit}"

echo "== map_compiler: full"
"${compiler}" "${out}/map_full/dump" "${out}/map_full/compiled" 2>&1 | tail -3

evaluate() {  # <label> <replay out dir> [gate: ATE rmse limit in m]; fails the script when evaluate.py fails or the gate is exceeded
  local label="$1" dir="$2" gate="${3:-}"
  rm -f "${dir}/eval.json" "${dir}/eval.txt"
  local status=0
  # evaluate.py prints its own RESULT line; this script prints the final one only after every gate has passed
  py "${repo_root}/scripts/evaluate.py" --est "${dir}/traj_localization.txt" --gt "${out}/map_full/traj_glim.txt" \
    --frames "${dir}/frames.csv" --out "${dir}" 2>&1 | { grep -v '^RESULT: PASS' || true; } || status=${PIPESTATUS[0]}
  if [ "${status}" -ne 0 ] || [ ! -f "${dir}/eval.json" ]; then
    echo "FAIL: evaluate.py (${label}) exited ${status}" >&2
    exit 1
  fi
  { grep -v '^RESULT: PASS' "${dir}/eval.txt" || true; }
  if [ -n "${gate}" ]; then
    py - "${dir}/eval.json" "${gate}" <<'PY' || { echo "FAIL: the ${label} ATE rmse gate was not met" >&2; exit 1; }
import json, sys
rmse = json.load(open(sys.argv[1]))["errors"]["ate_trans_m"]["rmse"]
limit = float(sys.argv[2])
print("self-localization ATE rmse %.4f m against the provisional gate %.2f m (consistency with GLIM, not accuracy)" % (rmse, limit))
sys.exit(0 if rmse < limit else 1)
PY
  fi
}

init_pose() {  # <first stamp>: GLIM's pose at that stamp, as x y z qx qy qz qw
  py - "${out}/map_full/traj_glim.txt" "$1" <<'PY'
import sys
import numpy as np
traj = np.loadtxt(sys.argv[1])
row = traj[traj[:, 0] >= float(sys.argv[2]) - 1e-6][0]
print(" ".join("%.17g" % v for v in row[1:8]))
PY
}

if [ "${revisit%% *}" = "yes" ]; then
  echo "== mapper: prefix (end ${prior_end} s) for the held-out run"
  run_mapper prefix "${out}/map_prefix" --end "$(at "${prior_end}")"
  cat "${out}/map_prefix/mapper_summary.json"
  check_mapper_coverage prefix "${out}/map_prefix/mapper_summary.json"
  "${compiler}" "${out}/map_prefix/dump" "${out}/map_prefix/compiled" 2>&1 | tail -3
  # The full and prefix runs must share a world frame for the initial pose to transfer: report their disagreement on the overlap
  py - "${out}/map_full/traj_glim.txt" "${out}/map_prefix/traj_glim.txt" <<'PY'
import sys
import numpy as np
a, b = (np.loadtxt(p) for p in sys.argv[1:3])
ia = {round(t, 3): i for i, t in enumerate(a[:, 0])}
pairs = [(ia[round(t, 3)], j) for j, t in enumerate(b[:, 0]) if round(t, 3) in ia]
d = np.linalg.norm(a[[p[0] for p in pairs], 1:4] - b[[p[1] for p in pairs], 1:4], axis=1)
print("full vs prefix GLIM runs on the overlap: %d poses, translation difference rms %.3f m, max %.3f m" % (len(d), np.sqrt((d ** 2).mean()), d.max()))
PY
  tail_stamp="$(at "${tail_start}")"
  echo "== replay: held-out tail from ${tail_stamp} on the prefix map"
  "${replay}" --data "${data}" --config "${out}/config_localization" --map "${out}/map_prefix/compiled" --out "${out}/replay_heldout" \
    --start "${tail_stamp}" --init-pose $(init_pose "${tail_stamp}") 2>&1 | tee "${out}/replay_heldout.log" | tail -5
  evaluate heldout "${out}/replay_heldout"
  check_replay_frames heldout "${out}/replay_heldout/frames.csv"
else
  echo "== replay: no revisit, self-localization of the full sequence on the full map"
  first_stamp="$(py -c "
import numpy as np
print('%.9f' % np.loadtxt('${out}/map_full/traj_glim.txt')[0, 0])")"
  "${replay}" --data "${data}" --config "${out}/config_localization" --map "${out}/map_full/compiled" --out "${out}/replay_self" \
    --init-stamp "${first_stamp}" --init-pose $(init_pose "${first_stamp}") 2>&1 | tee "${out}/replay_self.log" | tail -5
  evaluate self "${out}/replay_self" 0.3
  check_replay_frames self "${out}/replay_self/frames.csv"
fi
echo "RESULT: PASS (every step succeeded and every gate passed; consistency figures against GLIM's trajectory, not accuracy)"
