#!/bin/bash
# Real-data smoke test: map a converted OS1-128 sequence with GLIM through the headless mapper,
# then localize on the compiled map and compare against GLIM's own trajectory. Run inside the container, never in CI:
#   scripts/docker_run.sh scripts/smoke_os1_128.sh [--force] <replay dataset dir> [out dir]
# The dataset comes from scripts/bag_to_replay.py. Output goes to the out dir (default /ws/out/smoke_os1_128/run).
#
# The reference is GLIM's own trajectory, so every number is a consistency figure against GLIM, not an accuracy.
# Environment: PRIOR_END (seconds after the first frame, default 80) and TAIL_START (default 85) set the held-out split;
# MAPPER_ARGS and CONFIG_PATCH are for the fallbacks (for example a smaller random_downsample_target). MAPPER_ARGS goes
# into the cache key as text only: it must not add data inputs (a file it names is not hashed), or use --force.
#
# Map cache: map_full and map_prefix (mapper output, compiled map and GLIM's reference trajectory) are kept in
# SMOKE_CACHE_DIR (default /ws/out/smoke_os1_128/cache, or cache-cuda when GLIM_LOC_BUILD_WITH_CUDA=ON) and reused when the key matches: a hash of the dataset, the
# mapping config (after CONFIG_PATCH), its extrinsics, the mapper, map_compiler and GLIM binaries and libraries, and the
# mapper and split arguments. A cached map is validated (manifest of sizes and SHA-256, the mapper summary and the
# trajectory) before use. A build goes to a temporary directory and is renamed into the cache only after it passed, so
# a partial or failed build is never reused. --force rebuilds both maps. The localization always runs.
# The two maps are built one after the other, never in parallel (about 6 GB peak together).
set -euo pipefail

usage="usage: smoke_os1_128.sh [--force] <replay dataset dir> [out dir]"
force=false
positional=()
for arg in "$@"; do  # --force in any position; any other leading dash (an option, or such a path) is refused
  case "${arg}" in
    --force) force=true ;;
    -*) echo "${usage}" >&2; exit 2 ;;
    *) positional+=("${arg}") ;;
  esac
done
if [ "${#positional[@]}" -lt 1 ] || [ "${#positional[@]}" -gt 2 ]; then
  echo "${usage}" >&2; exit 2
fi
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${SMOKE_BUILD_DIR:-/ws/build/core}"  # override only for the tests (fake tools)
data="${positional[0]}"
out="${positional[1]:-/ws/out/smoke_os1_128/run}"
# The CPU (dev) and CUDA (dev-cuda) containers share /ws/out but build different binaries, so each flavor has its own
# default cache; with one cache they would rebuild each other's maps on every turn
cache_suffix=""
[ "${GLIM_LOC_BUILD_WITH_CUDA:-OFF}" = ON ] && cache_suffix="-cuda"
cache="${SMOKE_CACHE_DIR:-/ws/out/smoke_os1_128/cache${cache_suffix}}"
prior_end="${PRIOR_END:-80}"
tail_start="${TAIL_START:-85}"
: "${GLIM_LOC_PYTHON:?GLIM_LOC_PYTHON is not set (run inside the dev container)}"
export PYTHONDONTWRITEBYTECODE=1
py() { env -u PYTHONPATH "${GLIM_LOC_PYTHON}" "$@"; }
cache_tool="${repo_root}/scripts/smoke_map_cache.py"
# OS1 extrinsic T_lidar_imu (os_sensor -> os_imu from /tf_static); config/mapping_cpu has the same value
t_lidar_imu="0.006253, -0.011775, 0.007645, 0.0, 0.0, 0.0, 1.0"
run_start="${EPOCHREALTIME}"

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
# The map cache gets the same checks: canonical, no protected path, not inside the dataset, no overlap with the out
# directory, and a marker before anything in it is removed
cache="$(realpath -m -- "${cache}")"
for protected in / /ws /ws/out "${repo_root}" "${HOME:-/nonexistent}" "${data}"; do
  if within "${protected}" "${cache}"; then
    echo "FAIL: refusing the cache directory '${cache}': it is or contains '${protected}'" >&2; exit 1
  fi
done
if within "${cache}" "${data}"; then
  echo "FAIL: refusing the cache directory '${cache}': it is inside the dataset" >&2; exit 1
fi
if within "${cache}" "${out}" || within "${out}" "${cache}"; then
  echo "FAIL: the cache directory '${cache}' and the output directory '${out}' must not contain each other" >&2; exit 1
fi
cache_marker="${cache}/.smoke_os1_128_cache"
if [ -e "${cache}" ] && [ ! -d "${cache}" ]; then
  echo "FAIL: '${cache}' is not a directory" >&2; exit 1
fi
if [ -d "${cache}" ] && [ ! -f "${cache_marker}" ] && [ -n "$(ls -A -- "${cache}")" ]; then
  echo "FAIL: '${cache}' is not empty and was not created by this script (no marker); use a new cache directory" >&2; exit 1
fi
if [ -L "${cache_marker}" ] || [ -L "${cache}/.lock" ]; then
  echo "FAIL: '${cache_marker}' or '${cache}/.lock' is a symlink" >&2; exit 1
fi

# One run at a time per cache: a second run would rebuild or read the entries under the first one. The lock is taken
# before anything is written (opened for append, so a waiting run does not even change the lock file)
mkdir -p "${cache}"
exec 9>> "${cache}/.lock"
flock -n 9 || { echo "FAIL: another smoke_os1_128.sh run holds '${cache}/.lock'" >&2; exit 1; }
[ -f "${cache_marker}" ] || echo "map cache of scripts/smoke_os1_128.sh; remove it to drop every cached map" > "${cache_marker}"
mkdir -p "${out}"
echo "created by scripts/smoke_os1_128.sh; its fixed output names are removed at the start of every run" > "${marker}"
# Builds of an interrupted earlier run are never reused; an entry displaced by an interrupted install is put back (and
# checked like any other). This run's own builds are removed when it exits
py "${cache_tool}" cleanup --cache "${cache}"
trap 'rm -rf "${cache}"/.tmp-map_*-$$' EXIT
# Remove only the fixed names this script writes (rm does not follow symlinks), so an earlier run's output can never mask a failed step
rm -rf "${out}/config_mapping" "${out}/config_localization" "${out}/map_full" "${out}/map_prefix" "${out}/replay_heldout" "${out}/replay_self" \
  "${out}/mapper_full.log" "${out}/mapper_prefix.log" "${out}/replay_heldout.log" "${out}/replay_self.log" \
  "${out}/cache_key_full.json" "${out}/cache_key_prefix.json" "${out}/timings.tsv" "${out}/maps.tsv" "${out}/smoke_summary.json"
# Both configs get the OS1 extrinsic T_lidar_imu = (os_sensor -> os_imu from /tf_static); the shipped localization config has zeros
cp -r "${repo_root}/glim_localization/config/mapping_cpu" "${out}/config_mapping"
mkdir "${out}/config_localization"
cp "${repo_root}"/glim_localization/config/*.json "${out}/config_localization/"
py - "${out}/config_localization/config_sensors.json" "${t_lidar_imu}" <<'PY'
import re, sys
path, value = sys.argv[1:3]
text = open(path).read()
new, n = re.subn(r'"T_lidar_imu"\s*:\s*\[[^\]]*\]', '"T_lidar_imu": [%s]' % value, text)
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

# Wall time of each step, in timings.tsv (step, seconds) and smoke_summary.json at the end
elapsed() { awk -v a="$1" -v b="${EPOCHREALTIME}" 'BEGIN { printf "%.1f", b - a }'; }
timing() {  # <step> <start>
  local seconds
  seconds="$(elapsed "$2")"
  printf '%s\t%s\n' "$1" "${seconds}" >> "${out}/timings.tsv"
  echo "time: $1 ${seconds} s"
}

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

# The mapping config's extrinsics, a key component of its own so the log can name it (the config hash covers it too)
mapping_extrinsics="$(py - "${out}/config_mapping/config_sensors.json" <<'PY'
import re, sys
found = re.findall(r'^\s*"T_lidar_imu"\s*:\s*\[([^\]]*)\]', open(sys.argv[1]).read(), re.M)
assert len(found) == 1, "T_lidar_imu not found in the mapping config"
print(" ".join(v.strip() for v in found[0].split(",")))
PY
)"

map_step() {  # <label> [mapper args...]: reuse the cached map_<label> when its key matches and it is valid, else build it
  local label="$1"; shift
  local entry="${cache}/map_${label}" key="${out}/cache_key_${label}.json"
  local start="${EPOCHREALTIME}" status=0 reason="" split=()
  [ "${label}" = prefix ] && split=(--value "prior_end=${prior_end}")
  py "${cache_tool}" key --out "${key}" --input "data=${data}" --input "mapping_config=${out}/config_mapping" \
    --value "extrinsics=${mapping_extrinsics}" --value "mapper_args=$* ${MAPPER_ARGS:-}" "${split[@]}" \
    --binary "${mapper}" --binary "${compiler}" --modules "${out}/config_mapping" > /dev/null
  if [ "${force}" = true ]; then
    reason="--force"
  else
    reason="$(py "${cache_tool}" check --entry "${entry}" --key "${key}")" || status=$?
    if [ "${status}" -eq 0 ]; then
      reason=""
    elif [ -z "${reason}" ]; then
      reason="the cache check failed (exit ${status})"
    fi
  fi
  timing "cache_key_and_check_${label}" "${start}"
  # One line, one tsv field: escape backslashes, tabs and newlines (a file name in the reason may hold them)
  reason="${reason//\\/\\\\}"
  reason="${reason//$'\t'/\\t}"
  reason="${reason//$'\n'/\\n}"
  if [ -z "${reason}" ]; then
    echo "== map_${label}: REUSED the cached map (key $(py -c "import json, sys; print(json.load(open(sys.argv[1]))['key'][:16])" "${key}"), ${entry})"
    printf '%s\treused\t%s\n' "${label}" "key matches and the cached map is valid" >> "${out}/maps.tsv"
  else
    echo "== map_${label}: REBUILDING (${reason})"
    printf '%s\trebuilt\t%s\n' "${label}" "${reason}" >> "${out}/maps.tsv"
    local build="${cache}/.tmp-map_${label}-$$"
    rm -rf "${build}"
    echo "== mapper: ${label}${*:+ ($*)}"
    start="${EPOCHREALTIME}"
    run_mapper "${label}" "${build}" "$@"
    timing "mapper_${label}" "${start}"
    check_mapper_coverage "${label}" "${build}/mapper_summary.json"
    echo "== map_compiler: ${label}"
    start="${EPOCHREALTIME}"
    "${compiler}" "${build}/dump" "${build}/compiled" 2>&1 | tail -3
    timing "map_compiler_${label}" "${start}"
    cp "${out}/mapper_${label}.log" "${build}/mapper.log"
    # Validated and sealed (manifest of every file), then renamed over the old entry: only a complete build enters the cache
    start="${EPOCHREALTIME}"
    py "${cache_tool}" seal --entry "${build}" --key "${key}" || { echo "FAIL: the new map_${label} failed validation" >&2; exit 1; }
    py "${cache_tool}" install --build "${build}" --entry "${entry}"
    timing "cache_store_${label}" "${start}"
  fi
  # The gate runs on every run, cached or not
  check_mapper_coverage "${label}" "${entry}/mapper_summary.json"
}

map_full="${cache}/map_full"
map_prefix="${cache}/map_prefix"
map_step full
cat "${map_full}/mapper_summary.json"

# Is there a revisit? The tail must pass within 10 m of the prefix poses for at least 15 s
revisit="$(py - "${map_full}/traj_glim.txt" "$(at 0)" "$(at "${prior_end}")" "$(at "${tail_start}")" <<'PY'
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

evaluate() {  # <label> <replay out dir> [gate: ATE rmse limit in m]; fails the script when evaluate.py fails or the gate is exceeded
  local label="$1" dir="$2" gate="${3:-}"
  rm -f "${dir}/eval.json" "${dir}/eval.txt"
  local status=0 start="${EPOCHREALTIME}"
  # evaluate.py prints its own RESULT line; this script prints the final one only after every gate has passed
  py "${repo_root}/scripts/evaluate.py" --est "${dir}/traj_localization.txt" --gt "${map_full}/traj_glim.txt" \
    --frames "${dir}/frames.csv" --out "${dir}" 2>&1 | { grep -v '^RESULT: PASS' || true; } || status=${PIPESTATUS[0]}
  timing "evaluate_${label}" "${start}"
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
  py - "${map_full}/traj_glim.txt" "$1" <<'PY'
import sys
import numpy as np
traj = np.loadtxt(sys.argv[1])
row = traj[traj[:, 0] >= float(sys.argv[2]) - 1e-6][0]
print(" ".join("%.17g" % v for v in row[1:8]))
PY
}

if [ "${revisit%% *}" = "yes" ]; then
  echo "== map_prefix (end ${prior_end} s) for the held-out run"
  map_step prefix --end "$(at "${prior_end}")"
  cat "${map_prefix}/mapper_summary.json"
  # The full and prefix runs must share a world frame for the initial pose to transfer: report their disagreement on the overlap
  py - "${map_full}/traj_glim.txt" "${map_prefix}/traj_glim.txt" <<'PY'
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
  start="${EPOCHREALTIME}"
  "${replay}" --data "${data}" --config "${out}/config_localization" --map "${map_prefix}/compiled" --out "${out}/replay_heldout" \
    --start "${tail_stamp}" --init-pose $(init_pose "${tail_stamp}") 2>&1 | tee "${out}/replay_heldout.log" | tail -5
  timing replay_heldout "${start}"
  evaluate heldout "${out}/replay_heldout"
  check_replay_frames heldout "${out}/replay_heldout/frames.csv"
else
  echo "== replay: no revisit, self-localization of the full sequence on the full map"
  first_stamp="$(py -c "
import sys
import numpy as np
print('%.9f' % np.loadtxt(sys.argv[1])[0, 0])" "${map_full}/traj_glim.txt")"
  start="${EPOCHREALTIME}"
  "${replay}" --data "${data}" --config "${out}/config_localization" --map "${map_full}/compiled" --out "${out}/replay_self" \
    --init-stamp "${first_stamp}" --init-pose $(init_pose "${first_stamp}") 2>&1 | tee "${out}/replay_self.log" | tail -5
  timing replay_self "${start}"
  evaluate self "${out}/replay_self" 0.3
  check_replay_frames self "${out}/replay_self/frames.csv"
fi
timing total "${run_start}"
# Which maps were reused or rebuilt and why, and the time of every step
py - "${out}" <<'PY'
import csv, json, os, sys
out = sys.argv[1]
maps = {row[0]: {"action": row[1], "reason": row[2]} for row in csv.reader(open(os.path.join(out, "maps.tsv")), delimiter="\t")}
timings = {row[0]: float(row[1]) for row in csv.reader(open(os.path.join(out, "timings.tsv")), delimiter="\t")}
json.dump({"maps": maps, "timings_s": timings}, open(os.path.join(out, "smoke_summary.json"), "w"), indent=2)
for label, value in maps.items():
    print("map_%s: %s (%s)" % (label, value["action"], value["reason"]))
print("timings [s]: " + ", ".join("%s %.1f" % item for item in timings.items()))
PY
echo "RESULT: PASS (every step succeeded and every gate passed; consistency figures against GLIM's trajectory, not accuracy)"
