#!/bin/bash
# GPU map tile streaming benchmark on synthetic data (inside the CUDA container, with a visible GPU).
#
# The synthetic corridor is compiled into small tiles, so the tiles the map factors use change several times along the
# trajectory. The same replay runs with the CPU module and with the GPU module (default budget with prefetch, without
# prefetch, and with a small byte budget). Reports the per-frame odometry time over all frames and over the frames at
# a tile transition, the GPU tile counters, and the largest pose difference to the CPU run.
#
# Usage: scripts/bench_tile_streaming.sh [--build-dir DIR] [--out DIR] [--grid-size M] [--duration S] [--small-budget-mb MB]
#                                        [--runs N] [--cpu-only]
#   --build-dir  core build tree with tools/ and test/ (default /ws/build/core). Point it at a build of another
#                commit to compare: keys that build does not know are ignored, so only the default GPU run means the
#                same there.
#   --runs       repeat each GPU run N times and keep the per-frame median time (default 3)
#   --cpu-only   only the CPU run (to check the script without a GPU)
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir=/ws/build/core
out=/ws/out/bench_tile_streaming
grid_size=4
duration=12
small_budget_mb=8
runs=3
cpu_only=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir) build_dir="$2"; shift 2 ;;
    --out) out="$2"; shift 2 ;;
    --grid-size) grid_size="$2"; shift 2 ;;
    --duration) duration="$2"; shift 2 ;;
    --small-budget-mb) small_budget_mb="$2"; shift 2 ;;
    --runs) runs="$2"; shift 2 ;;
    --cpu-only) cpu_only=true; shift ;;
    *) sed -n '2,16p' "${BASH_SOURCE[0]}" >&2; exit 2 ;;
  esac
done
: "${GLIM_LOC_PYTHON:?GLIM_LOC_PYTHON is not set (run inside the dev container)}"
export PYTHONDONTWRITEBYTECODE=1
py() { env -u PYTHONPATH "${GLIM_LOC_PYTHON}" "$@"; }

replay="${build_dir}/tools/glim_localization_replay"
test -x "${replay}" || { echo "FAIL: ${replay} not built" >&2; exit 1; }

rm -rf "${out}"
mkdir -p "${out}"
"${build_dir}/test/synthetic_replay_dataset" "${out}/input" "${duration}" > /dev/null
"${build_dir}/tools/map_compiler" "${out}/input/map.pcd" "${out}/map" --grid-size "${grid_size}" > "${out}/map.log" 2>&1

# The config of this tree with settings for small tiles: a radius that holds a few rows of tiles and a frame's worth
# of them per query. Both backends use the same values.
tiles_config() {  # <dest> <key=value>...: a copy of the config with the given odometry keys replaced
  local dest="$1"; shift
  rm -rf "${dest}"
  cp -r "${repo_root}/glim_localization/config" "${dest}"
  py - "${dest}" "$@" <<'PY'
import re
import sys
dest, *pairs = sys.argv[1:]
for name in ("config_odometry_localization_cpu.json", "config_odometry_localization_gpu.json"):
    path = f"{dest}/{name}"
    text = open(path).read()
    for pair in pairs:
        key, value = pair.split("=", 1)
        text, n = re.subn(rf'("{key}"\s*:\s*)[^,\n]+', lambda m: m.group(1) + value, text)
        if n == 0:  # Not in this file (a GPU key in the CPU file): add it
            if name.endswith("_gpu.json") or not key.startswith("map_gpu_"):
                text = text.replace('"odometry_estimation": {', f'"odometry_estimation": {{\n    "{key}": {value},', 1)
    open(path, "w").write(text)
PY
}
common=(map_search_radius=12.0 map_max_submaps=12 map_gpu_max_tiles=48 map_gpu_prefetch_max_tiles=24)
tiles_config "${out}/config_default" "${common[@]}"
tiles_config "${out}/config_no_prefetch" "${common[@]}" map_gpu_prefetch=false
tiles_config "${out}/config_small" "${common[@]}" "map_gpu_memory_budget_mb=${small_budget_mb}"

init_pose="$(py - "${out}/input/data/traj_gt.txt" <<'PY'
import sys
row = next(l for l in open(sys.argv[1]) if l.strip() and not l.startswith("#")).split()
print(" ".join(row[1:8]))
PY
)"

run() {  # <name> <config dir>
  local dest="${out}/$1"
  # shellcheck disable=SC2086
  "${replay}" --data "${out}/input/data" --config "$2" --map "${out}/map" --out "${dest}" --init-pose ${init_pose} > "${dest}.log" 2>&1
}

run cpu "${out}/config_default"
names=(cpu)
if [[ "${cpu_only}" == false ]]; then
  for variant in default no_prefetch small; do
    for i in $(seq 1 "${runs}"); do
      run "gpu_${variant}_${i}" "${out}/config_${variant}/gpu"
    done
    names+=("gpu_${variant}")
  done
fi

py - "${out}" "${runs}" "${names[@]}" <<'PY'
import csv
import glob
import json
import re
import sys
import numpy as np

out, runs, *names = sys.argv[1:]
runs = int(runs)

def config_value(key):
    text = open(f"{out}/config_default/config_odometry_localization_gpu.json").read()
    return float(re.search(rf'"{key}"\s*:\s*([^,\n]+)', text).group(1))

radius, k = config_value("map_search_radius"), int(config_value("map_max_submaps"))

# Tile origins from the compiled map (data.txt: "T_world_origin:" and four matrix rows)
origins = []
for path in sorted(glob.glob(f"{out}/map/*/data.txt")):
    lines = open(path).read().splitlines()
    i = next(i for i, l in enumerate(lines) if l.strip().startswith("T_world_origin"))
    m = np.array([[float(v) for v in lines[i + 1 + r].split()] for r in range(4)])
    origins.append(m[:3, 3])
origins = np.array(origins)

gt = np.loadtxt(f"{out}/input/data/traj_gt.txt", ndmin=2)

def nearest(p):
    d = np.linalg.norm(origins - p, axis=1)
    idx = [i for i in np.argsort(d, kind="stable") if d[i] <= radius][:k]
    return frozenset(idx)

def frames(d):
    rows = [r for r in csv.DictReader(open(f"{d}/frames.csv")) if r["result"] == "estimated"]
    return [(int(r["stamp_ns"]), float(r["t_odometry_ms"])) for r in rows]

def run_dirs(name):
    return [f"{out}/{name}"] if name == "cpu" else [f"{out}/{name}_{i}" for i in range(1, runs + 1)]

# A frame is at a transition when the tiles near its true position differ from those of the previous frame.
base = frames(f"{out}/cpu")
stamps = [s for s, _ in base]
gt_index = [int(np.argmin(np.abs(gt[:, 0] - s * 1e-9))) for s in stamps]
sets = [nearest(gt[i, 1:4]) for i in gt_index]
transition = np.array([i > 0 and sets[i] != sets[i - 1] for i in range(len(sets))])
print(f"{len(origins)} tiles, {len(stamps)} frames, {int(transition.sum())} at a tile transition "
      f"(radius {radius} m, {k} tiles per frame)")

cpu_traj = np.loadtxt(f"{out}/cpu/traj_localization.txt", ndmin=2)
report = {"tiles": len(origins), "frames": len(stamps), "transition_frames": int(transition.sum()), "runs": {}}
for name in names:
    dirs = run_dirs(name)
    per_run = []
    for d in dirs:
        f = dict(frames(d))
        per_run.append([f.get(s, np.nan) for s in stamps])
    t = np.nanmedian(np.array(per_run), axis=0)
    entry = {}
    for label, mask in (("all", np.ones_like(transition)), ("transition", transition), ("steady", ~transition)):
        v = t[mask & ~np.isnan(t)]
        entry[label] = {"mean": float(v.mean()), "p95": float(np.percentile(v, 95)), "max": float(v.max())} if len(v) else None
    traj = np.loadtxt(f"{dirs[0]}/traj_localization.txt", ndmin=2)
    if traj.shape == cpu_traj.shape:
        entry["max_diff_to_cpu_m"] = float(np.linalg.norm(traj[:, 1:4] - cpu_traj[:, 1:4], axis=1).max())
        qa, qb = traj[:, 4:8], cpu_traj[:, 4:8]
        dot = np.clip(np.abs(np.sum(qa * qb, axis=1)) / (np.linalg.norm(qa, axis=1) * np.linalg.norm(qb, axis=1)), 0.0, 1.0)
        entry["max_diff_to_cpu_deg"] = float(np.degrees(2.0 * np.arccos(dot)).max())
    stats = re.search(r"GPU map tiles: (.*)", open(f"{dirs[0]}.log").read())
    entry["tiles"] = dict(kv.split("=") for kv in stats.group(1).split()) if stats else None
    report["runs"][name] = entry
    a, tr = entry["all"], entry["transition"]
    line = f"{name:16s} all: mean {a['mean']:6.2f} p95 {a['p95']:6.2f} max {a['max']:6.2f} ms"
    if tr:
        line += f" | transition: mean {tr['mean']:6.2f} p95 {tr['p95']:6.2f} max {tr['max']:6.2f} ms"
    if "max_diff_to_cpu_m" in entry:
        line += f" | vs cpu {entry['max_diff_to_cpu_m']:.2e} m {entry['max_diff_to_cpu_deg']:.2e} deg"
    print(line)
    if entry["tiles"]:
        print(f"{'':16s} tiles: " + " ".join(f"{k}={v}" for k, v in entry["tiles"].items()))
json.dump(report, open(f"{out}/bench.json", "w"), indent=2)
print(f"wrote {out}/bench.json")
PY
