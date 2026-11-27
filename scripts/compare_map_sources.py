#!/usr/bin/env python3
"""The same synthetic scene compiled from a GLIM dump and from its PCD export, replayed on the same data.

Runs inside the dev container after build_core.sh (issue #61, docs/getting_started.md "Which map to keep").

The synthetic dataset (synthetic_replay_dataset) has a GLIM dump of the corridor (overlapping submaps with
per-submap covariances). map_compiler compiles it as it is and exports it with --export-pcd, the way a user
would export a GLIM map; the export is then compiled as a point cloud (disjoint grid tiles, covariances
re-estimated over the whole cloud) with several --grid-size, --k-neighbors and --voxel-size values. The
generated scene cloud (map.pcd, every surface point once) is compiled too, as a reference without the
duplicated points of the submap overlaps.

Every map is replayed from the same initial poses: offsets from the first ground-truth pose with the yaw
search off, and a 60 degree yaw error with the yaw search on. Reported per map: ATE (after --settle s),
the mean inlier ratio of the estimated frames, and the yaw search scores of the true yaw and the best other one.
Maps use the shipped config except where MAP_CONFIG changes it (map_max_submaps 12 for one tiling).

Every seed (--seeds) is a new dataset: other noise on the map points and on the LiDAR ranges.

Outputs: <out>/results.json and <out>/results.md. Report only: a failed run is in the table (pass column), not
in the exit code. Exit codes: 0 reported, 2 a setup error.
"""
import argparse
import concurrent.futures
import csv
import json
import os
import re
import shutil
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(REPO_ROOT, "glim_localization", "config")
EVALUATE = os.path.join(REPO_ROOT, "scripts", "evaluate.py")

# name: (input, map_compiler options). "dump" is the GLIM dump, "export" its --export-pcd output, "scene" map.pcd.
MAPS = {
    "dump": ("dump", []),
    "dump_recov_k10": ("dump", ["--recompute-covariances", "--k-neighbors", "10"]),
    "dump_recov_k20": ("dump", ["--recompute-covariances", "--k-neighbors", "20"]),
    "export_g20_k10": ("export", []),
    "export_g10_k10": ("export", ["--grid-size", "10"]),
    "export_g50_k10": ("export", ["--grid-size", "50"]),
    "export_g20_k5": ("export", ["--k-neighbors", "5"]),
    "export_g20_k20": ("export", ["--k-neighbors", "20"]),
    "export_g20_k10_v0.05": ("export", ["--voxel-size", "0.05"]),
    "export_g10_k10_v0.05": ("export", ["--grid-size", "10", "--voxel-size", "0.05"]),
    "export_g50_k10_v0.05": ("export", ["--grid-size", "50", "--voxel-size", "0.05"]),
    "export_g20_k20_v0.05": ("export", ["--k-neighbors", "20", "--voxel-size", "0.05"]),
    "scene_g20_k10": ("scene", []),
    "scene_g10_k10": ("scene", ["--grid-size", "10"]),
    "scene_g50_k10": ("scene", ["--grid-size", "50"]),
    "scene_g20_k20": ("scene", ["--k-neighbors", "20"]),
    "scene_g10_k20": ("scene", ["--grid-size", "10", "--k-neighbors", "20"]),
    "scene_g50_k20": ("scene", ["--grid-size", "50", "--k-neighbors", "20"]),
    "scene_g20_k10_max12": ("scene", []),
}

# Odometry config keys changed for a map (JSON value text); the other maps use the shipped config.
MAP_CONFIG = {"scene_g20_k10_max12": {"map_max_submaps": "12"}}

# name: (translation [m], yaw [deg], yaw search)
CASES = {
    "e2e": ((0.3, -0.2, 0.1), 3.0, False),
    "offset_a": ((1.0, -0.5, 0.0), 8.0, False),
    "offset_b": ((-0.6, 0.6, 0.05), -6.0, False),
    "search_yaw60": ((0.3, -0.2, 0.0), 60.0, True),
}


def run(cmd, log_path):
    with open(log_path, "w") as log:
        return subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT).returncode


def first_gt_pose(gt_path):
    row = next(l for l in open(gt_path) if l.strip() and not l.startswith("#")).split()
    return np.array(row[1:4], dtype=float), Rotation.from_quat([float(v) for v in row[4:8]])


def init_pose(gt_path, translation, yaw_deg):
    p, r = first_gt_pose(gt_path)
    p = p + np.array(translation)
    r = Rotation.from_euler("z", yaw_deg, degrees=True) * r
    return [*p, *r.as_quat()]


def make_config(out, name, overrides):
    """A copy of the shipped config with these odometry keys replaced ({key: JSON value text})."""
    config = os.path.join(out, "configs", name)
    shutil.rmtree(config, ignore_errors=True)
    shutil.copytree(CONFIG, config, ignore=shutil.ignore_patterns("gpu", "mapping_cpu"))
    odometry = os.path.join(config, "config_odometry_localization_cpu.json")
    text = open(odometry).read()
    for key, value in overrides.items():
        text, n = re.subn(rf'("{key}"\s*:\s*)[^,\n]+', lambda m: m.group(1) + value, text)
        if n != 1:
            raise RuntimeError(f'{odometry}: "{key}" not found')
    open(odometry, "w").write(text)
    return config


def config_for(out, map_name, search):
    overrides = dict(MAP_CONFIG.get(map_name, {}))
    if search:
        overrides["init_yaw_search"] = "true"
    if not overrides:
        return CONFIG
    return make_config(out, f"{map_name}{'_search' if search else ''}", overrides)


def map_stats(path):
    index = json.load(open(os.path.join(path, "map_index.json")))
    return {"tiles": index["num_submaps"], "points": sum(s["num_points"] for s in index["submaps"])}


def run_case(ctx, seed, map_name, case_name):
    translation, yaw, search = CASES[case_name]
    root = ctx["roots"][seed]
    dest = os.path.join(root, "runs", f"{map_name}__{case_name}")
    shutil.rmtree(dest, ignore_errors=True)
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    data = os.path.join(root, "input", "data")
    gt = os.path.join(data, "traj_gt.txt")
    cmd = [ctx["replay"], "--data", data, "--config", ctx["configs"][(map_name, search)],
           "--map", os.path.join(root, "maps", map_name), "--out", dest,
           "--init-pose", *(f"{v:.17g}" for v in init_pose(gt, translation, yaw))]
    record = {"seed": seed, "map": map_name, "case": case_name, "replay_exit": run(cmd, dest + ".log")}
    summary_path = os.path.join(dest, "summary.json")
    summary = json.load(open(summary_path)) if os.path.isfile(summary_path) else {}
    record["init_search"] = summary.get("init_search")
    if search:
        record["yaw_search"] = yaw_search_scores(record["init_search"], dest + ".log", -yaw)
    frames = os.path.join(dest, "frames.csv")
    ratios = []
    if os.path.isfile(frames):
        with open(frames) as handle:
            ratios = [float(r["inlier_ratio"]) for r in csv.DictReader(handle) if r["result"] == "estimated"]
    record["mean_inlier_ratio"] = float(np.mean(ratios)) if ratios else None
    traj = os.path.join(dest, "traj_localization.txt")
    record["eval_exit"] = None
    if os.path.isfile(traj) and os.path.getsize(traj) > 0:
        cmd = [sys.executable, EVALUATE, "--est", traj, "--gt", gt, "--frames", frames, "--out", dest,
               "--settle", str(ctx["settle"]), "--max-trans", "0.1", "--max-rot-deg", "1"]
        if record["replay_exit"] != 0:
            cmd.append("--allow-incomplete")
        record["eval_exit"] = run(cmd, dest + ".eval.log")
        report_path = os.path.join(dest, "eval.json")
        if os.path.isfile(report_path):
            report = json.load(open(report_path))
            errors = report["errors"]
            record["ate_trans_rmse_m"] = errors["ate_trans_m"]["rmse"]
            record["ate_trans_max_m"] = errors["ate_trans_m"]["max"]
            record["ate_rot_rmse_deg"] = errors["ate_rot_deg"]["rmse"]
            record["ate_rot_max_deg"] = errors["ate_rot_deg"]["max"]
    record["passed"] = record["replay_exit"] == 0 and record["eval_exit"] == 0
    return record


def yaw_search_scores(init_search, log_path, true_correction_deg):
    """The inlier ratios of the yaw search: of the true correction and of the best other one, and whether it was
    accepted. An accepted search has them in summary.json; a rejected one stops the run before summary.json, so
    they come from its log line (an ambiguous rejection names the best two)."""
    def is_true(yaw):
        return abs((yaw - true_correction_deg + 180.0) % 360.0 - 180.0) < 5.0

    pairs = []
    accepted = False
    if init_search:
        accepted = init_search["result"].lower() == "accepted"
        pairs = [(init_search["score"], init_search["yaw_offset_deg"]),
                 (init_search["runner_up_score"], init_search["runner_up_yaw_offset_deg"])]
    else:
        for line in open(log_path, errors="replace"):
            m = re.search(r"inlier ratio ([\d.]+) at a yaw correction of (-?[\d.]+) deg and ([\d.]+) at (-?[\d.]+) deg", line)
            if "initial pose search" in line and m:
                pairs = [(float(m.group(1)), float(m.group(2))), (float(m.group(3)), float(m.group(4)))]
                break
    true = [score for score, yaw in pairs if is_true(yaw)]
    other = [score for score, yaw in pairs if not is_true(yaw)]
    if not true or not other:
        return {"accepted": accepted, "true_score": None, "other_score": None, "margin": None}
    return {"accepted": accepted and is_true(pairs[0][1]), "true_score": true[0], "other_score": other[0],
            "margin": true[0] - other[0]}


def summarize(records, stats):
    """Per map: means over the seeds and the cases with the yaw search off, maxima, and the yaw search scores."""
    def values(rows, key):
        return [r[key] for r in rows if r.get(key) is not None]

    def mean(v):
        return float(np.mean(v)) if v else None

    rows_out = {}
    for name in MAPS:
        rows = [r for r in records if r["map"] == name]
        tracked = [r for r in rows if not CASES[r["case"]][2]]
        converged = [r for r in tracked if r["passed"]]
        searches = [r["yaw_search"] for r in rows if CASES[r["case"]][2]]
        rows_out[name] = {
            "tiles": stats[name]["tiles"], "points": stats[name]["points"],
            "tracking_passed": len(converged), "tracking_runs": len(tracked),
            "failed": sorted({f"seed{r['seed']}/{r['case']}" for r in rows if not r["passed"]}),
            "ate_trans_rmse_m": mean(values(converged, "ate_trans_rmse_m")),
            "ate_trans_max_m": max(values(converged, "ate_trans_max_m"), default=None),
            "ate_rot_max_deg": max(values(converged, "ate_rot_max_deg"), default=None),
            "inlier_ratio": mean(values(converged, "mean_inlier_ratio")),
            "search_true_score": mean(values(searches, "true_score")),
            "search_other_score": mean(values(searches, "other_score")),
            "search_margin_min": min(values(searches, "margin"), default=None),
            "search_accepted": sum(s["accepted"] for s in searches), "search_runs": len(searches),
            "passed": sum(r["passed"] for r in rows), "runs": len(rows),
        }
    return rows_out


def report(summary, seeds):
    def fmt(value, digits, scale=1.0):
        return "-" if value is None else f"{value * scale:.{digits}f}"

    lines = [
        "| map | tiles | points | tracking pass | ATE trans rmse [mm] | ATE trans max [mm] | ATE rot max [deg] | "
        "inlier ratio | yaw search accepted | true yaw score | best other | min margin | all pass |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for name, s in summary.items():
        lines.append(
            f"| {name} | {s['tiles']} | {s['points']} | {s['tracking_passed']}/{s['tracking_runs']} | {fmt(s['ate_trans_rmse_m'], 1, 1e3)} | {fmt(s['ate_trans_max_m'], 1, 1e3)} | "
            f"{fmt(s['ate_rot_max_deg'], 3)} | {fmt(s['inlier_ratio'], 3)} | {s['search_accepted']}/{s['search_runs']} | "
            f"{fmt(s['search_true_score'], 3)} | {fmt(s['search_other_score'], 3)} | {fmt(s['search_margin_min'], 3)} | {s['passed']}/{s['runs']} |")
    lines.append("")
    lines.append(f"Seeds {', '.join(map(str, seeds))}. ATE and the inlier ratio over the cases with the yaw search off "
                 f"({', '.join(n for n, c in CASES.items() if not c[2])}), over the runs that pass (tracking pass): rmse and "
                 "the inlier ratio are means, max the largest. Tiles and points are those of the first seed. The yaw search columns (case "
                 f"{', '.join(n for n, c in CASES.items() if c[2])}): accepted at the true yaw, the mean inlier ratio of the true "
                 "yaw and of the best other yaw, and the smallest margin between them (negative: another yaw scored higher; "
                 "the search rejects margins up to init_yaw_search_min_score_gap 0.05 as ambiguous). pass: replay exit 0 and evaluate.py within 0.1 m and 1 deg.")
    return "\n".join(lines) + "\n"


def prepare_seed(tools, root, seed, duration):
    """The dataset of one seed, and every map compiled from it. Returns the map statistics."""
    os.makedirs(os.path.join(root, "maps"))
    inputs = os.path.join(root, "input")
    if run([tools["synthetic_replay_dataset"], inputs, f"{duration:g}", str(seed)], os.path.join(root, "dataset.log")) != 0:
        raise RuntimeError(f"synthetic_replay_dataset failed (see {root}/dataset.log)")
    # The dump compiled as it is, and exported the way a user would export it.
    export = os.path.join(root, "export.pcd")
    sources = {"dump": os.path.join(inputs, "map"), "export": export, "scene": os.path.join(inputs, "map.pcd")}
    for name, (source, options) in MAPS.items():
        cmd = [tools["map_compiler"], sources[source], os.path.join(root, "maps", name), *options]
        if name == "dump":
            cmd += ["--export-pcd", export]
        if run(cmd, os.path.join(root, "maps", name + ".log")) != 0:
            raise RuntimeError(f"map_compiler failed for {name} (see {root}/maps/{name}.log)")
    return {name: map_stats(os.path.join(root, "maps", name)) for name in MAPS}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--build-dir", default="/ws/build/core", help="core build directory (default /ws/build/core)")
    parser.add_argument("--out", required=True, help="output directory (replaced)")
    parser.add_argument("--seeds", type=int, nargs="+", default=[42, 43, 44],
                        help="synthetic_replay_dataset seeds: map point and LiDAR noise (default 42 43 44)")
    parser.add_argument("--duration", type=float, default=12.0, help="length of the synthetic dataset [s] (default 12)")
    parser.add_argument("--settle", type=float, default=3.0, help="evaluate.py --settle [s] (default 3)")
    parser.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2), help="parallel replays")
    args = parser.parse_args()

    tools = {name: os.path.join(args.build_dir, sub, name) for sub, name in
             (("tools", "glim_localization_replay"), ("tools", "map_compiler"), ("test", "synthetic_replay_dataset"))}
    for tool in tools.values():
        if not os.access(tool, os.X_OK):
            print(f"{tool} is not built (run scripts/build_core.sh first)", file=sys.stderr)
            return 2
    out = os.path.abspath(args.out)
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(out)
    roots = {seed: os.path.join(out, f"seed{seed}") for seed in args.seeds}
    try:
        stats = {seed: prepare_seed(tools, roots[seed], seed, args.duration) for seed in args.seeds}
        configs = {(m, s): config_for(out, m, s) for m in MAPS for s in (False, True)}
    except RuntimeError as e:
        print(e, file=sys.stderr)
        return 2

    ctx = {"roots": roots, "replay": tools["glim_localization_replay"], "configs": configs, "settle": args.settle}
    jobs = [(seed, m, c) for seed in args.seeds for m in MAPS for c in CASES]
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        records = list(pool.map(lambda job: run_case(ctx, *job), jobs))

    summary = summarize(records, stats[args.seeds[0]])
    json.dump({"seeds": args.seeds,
               "maps": {n: {"source": MAPS[n][0], "options": MAPS[n][1], "config": MAP_CONFIG.get(n, {})} for n in MAPS},
               "map_stats": {str(seed): stats[seed] for seed in args.seeds},
               "cases": {n: {"translation_m": c[0], "yaw_deg": c[1], "yaw_search": c[2]} for n, c in CASES.items()},
               "summary": summary, "records": records}, open(os.path.join(out, "results.json"), "w"), indent=2)
    table = report(summary, args.seeds)
    open(os.path.join(out, "results.md"), "w").write(table)
    print(table)
    return 0


if __name__ == "__main__":
    sys.exit(main())
