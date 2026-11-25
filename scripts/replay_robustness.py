#!/usr/bin/env python3
"""Robustness of the replay on synthetic data: perturbed initial poses and a changed map.

Runs inside the dev container after build_core.sh (see docs/replay.md, "Robustness on synthetic data").

Initial pose: the ground truth at the first frame, moved by a translation along a fixed direction in
the xy plane and rotated about the world z axis, replayed with the yaw search off and on. A case passes
when the replay exits 0 and evaluate.py meets the accuracy limits (default 0.1 m and 1 deg after 3 s).

Map change: the scans stay the same, the map PCD is changed (points removed at random, objects removed,
objects added) and compiled with map_compiler, and the replay starts from the small offset of
test_replay_e2e.sh. The report has the accuracy, the mean inlier ratio and the tracking ratio.

Suites:
  quick  a few cases with expected outcomes; exits 1 if one differs (run by scripts/ci.sh)
  full   the whole grid, reported only (manual; results are recorded in docs/PROGRESS.md)

Outputs: <out>/results.json and <out>/results.md. Exit codes: 0 ok, 1 an expectation failed, 2 bad arguments
or a setup error.
"""
import argparse
import concurrent.futures
import csv
import json
import math
import os
import shutil
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(REPO_ROOT, "glim_localization", "config")
EVALUATE = os.path.join(REPO_ROOT, "scripts", "evaluate.py")

# Direction of the translation offset in the map xy plane (diagonal across the corridor, which runs along x).
OFFSET_DIRECTION = np.array([1.0, -1.0, 0.0]) / math.sqrt(2.0)
E2E_OFFSET = ((0.3, -0.2, 0.1), 3.0)  # The offset of test_replay_e2e.sh: translation [m], yaw [deg]

FULL_TRANSLATIONS = (0.0, 0.5, 1.0, 1.5, 2.0, 3.0)
FULL_YAWS = (0.0, 5.0, 10.0, 15.0, 20.0, 30.0, 45.0, 60.0, 90.0, 135.0, 180.0)
FULL_MAP_CHANGES = (
    "none", "drop30", "drop50", "drop70", "drop90", "drop95", "drop98", "objects_removed", "objects_added", "combined"
)

# quick: (translation m, yaw deg, yaw search, expected pass). The expectations sit well inside or outside the
# boundary measured by the full suite (docs/PROGRESS.md), so noise between platforms does not flip them.
QUICK_POSES = (
    (0.5, 10.0, False, True),
    (0.0, 90.0, False, False),
    (0.5, 90.0, True, True),
    (3.0, 0.0, True, False),  # The search rejects the pose as ambiguous (the corridor flipped by 180 deg scores close)
)
QUICK_MAP_CHANGES = (("drop50", True), ("combined", True))

# Synthetic scene layout (glim_localization/test/synthetic/synthetic_map.cpp, cross_walls on): a corridor along x,
# floor at z = 0, side walls at |y| = 5, thin walls across it every 5.3 m from x = -17, alternating sides.
HALF_WIDTH = 5.0
CROSS_WALL_START, CROSS_WALL_INTERVAL, CROSS_WALL_THICKNESS = -17.0, 5.3, 0.05


def py(*args):
    return [sys.executable, *args]


def read_pcd(path):
    """Read the binary x y z intensity float32 PCD written by synthetic_replay_dataset."""
    with open(path, "rb") as handle:
        header = {}
        while True:
            line = handle.readline().decode("ascii").strip()
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(" ")
            header[key] = value
            if key == "DATA":
                break
        if header.get("FIELDS") != "x y z intensity" or header.get("TYPE") != "F F F F" or header.get("SIZE") != "4 4 4 4":
            raise ValueError(f"{path}: expected float32 x y z intensity")
        if header["DATA"] != "binary":
            raise ValueError(f"{path}: expected binary data")
        count = int(header["POINTS"])
        return np.frombuffer(handle.read(count * 16), dtype="<f4").reshape(count, 4).copy()


def write_pcd(path, points):
    with open(path, "wb") as handle:
        handle.write(
            (
                "# .PCD v0.7 - changed synthetic map\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
                f"COUNT 1 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n"
            ).encode("ascii")
        )
        handle.write(points.astype("<f4").tobytes())


def box_surface(lo, hi, step=0.2):
    """Points on the surface of an axis-aligned box, like add_box_surface of the synthetic map."""
    axes = [np.linspace(lo[i], hi[i], max(2, int(math.ceil((hi[i] - lo[i]) / step)) + 1)) for i in range(3)]
    grid = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
    on_face = np.zeros(len(grid), dtype=bool)
    for i in range(3):
        on_face |= np.isclose(grid[:, i], lo[i]) | np.isclose(grid[:, i], hi[i])
    return grid[on_face]


def object_mask(points):
    """Points of the pillars and the low boxes: not the floor, the side walls or the cross walls."""
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    structure = (z < 0.05) | (np.abs(y) > HALF_WIDTH - 0.05)
    phase = np.mod(x - CROSS_WALL_START, CROSS_WALL_INTERVAL)
    cross_wall = (phase < CROSS_WALL_THICKNESS + 0.03) | (phase > CROSS_WALL_INTERVAL - 0.03)
    return ~structure & ~cross_wall


def added_objects(rng, count=12):
    """Boxes that are in the map but not in the scans, beside the sensor path (|y| <= 1 m), within x in [-5, 45]."""
    clouds = []
    for _ in range(count):
        side = rng.choice([-1.0, 1.0])
        center = np.array([rng.uniform(-5.0, 45.0), side * rng.uniform(1.8, 3.2), 0.0])
        size = np.array([rng.uniform(0.6, 1.5), rng.uniform(0.6, 1.2), rng.uniform(0.8, 2.0)])
        lo = center - np.array([size[0] / 2, size[1] / 2, 0.0])
        clouds.append(box_surface(lo, lo + size))
    xyz = np.concatenate(clouds)
    xyz += rng.normal(0.0, 0.01, size=xyz.shape)
    return np.concatenate([xyz, np.zeros((len(xyz), 1))], axis=1).astype(np.float32)


def change_map(points, name, seed=7):
    """Return (changed points, description)."""
    rng = np.random.default_rng(seed)
    if name == "none":
        return points, "unchanged (compiled from the PCD)"
    if name.startswith("drop"):
        fraction = int(name[4:]) / 100.0
        keep = rng.random(len(points)) >= fraction
        return points[keep], f"{fraction:.0%} of the points removed at random"
    if name == "objects_removed":
        mask = object_mask(points)
        return points[~mask], f"pillars and low boxes removed ({mask.sum()} of {len(points)} points)"
    if name == "objects_added":
        extra = added_objects(rng)
        return np.concatenate([points, extra]), f"12 boxes added beside the path ({len(extra)} points)"
    if name == "combined":
        mask = object_mask(points)
        kept = points[~mask]
        kept = kept[rng.random(len(kept)) >= 0.3]
        extra = added_objects(rng)
        return np.concatenate([kept, extra]), "pillars and low boxes removed, 12 boxes added, then 30% of the points removed"
    raise ValueError(f"unknown map change {name}")


def first_gt_pose(data):
    with open(os.path.join(data, "traj_gt.txt")) as handle:
        row = next(line for line in handle if line.strip() and not line.startswith("#")).split()
    return np.array(row[1:4], dtype=float), Rotation.from_quat([float(v) for v in row[4:8]])


def perturbed_pose(data, translation, yaw_deg):
    position, rotation = first_gt_pose(data)
    if isinstance(translation, tuple):
        position = position + np.array(translation)
    else:
        position = position + translation * OFFSET_DIRECTION
    rotation = Rotation.from_euler("z", yaw_deg, degrees=True) * rotation
    return [*position, *rotation.as_quat()]


def run_case(case, ctx):
    """Replay and evaluate one case; returns the result record."""
    dest = os.path.join(ctx["out"], "runs", case["name"])
    shutil.rmtree(dest, ignore_errors=True)
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    config = ctx["config_search"] if case.get("yaw_search") else CONFIG
    cmd = [ctx["replay"], "--data", ctx["data"], "--config", config, "--map", case["map"], "--out", dest]
    cmd += ["--init-pose", *(f"{v:.17g}" for v in case["init_pose"])]
    with open(dest + ".log", "w") as log:
        replay_rc = subprocess.call(cmd, stdout=log, stderr=subprocess.STDOUT)
    record = {key: case[key] for key in case if key not in ("init_pose", "map")}
    record["replay_exit"] = replay_rc

    summary_path = os.path.join(dest, "summary.json")
    summary = json.load(open(summary_path)) if os.path.isfile(summary_path) else {}
    record["tracking_ratio"] = summary.get("tracking_ratio")
    search = summary.get("init_search")
    record["init_search"] = None if not search else {k: search[k] for k in ("result", "reason", "yaw_offset_deg", "score")}
    # A rejected search stops the run before summary.json; keep its log line (the reason, such as an ambiguous yaw)
    with open(dest + ".log") as log:
        rejected = [line.strip() for line in log if "initial pose search" in line and "rejected" in line]
    record["init_search_rejected"] = rejected[0] if rejected else None

    frames_path = os.path.join(dest, "frames.csv")
    ratios = []
    if os.path.isfile(frames_path):
        with open(frames_path) as handle:
            ratios = [float(r["inlier_ratio"]) for r in csv.DictReader(handle) if r["result"] == "estimated"]
    record["mean_inlier_ratio"] = float(np.mean(ratios)) if ratios else None

    traj = os.path.join(dest, "traj_localization.txt")
    record["eval_exit"] = None
    record["ate_trans_max_m"] = record["ate_trans_rmse_m"] = record["ate_rot_max_deg"] = None
    if os.path.isfile(traj) and os.path.getsize(traj) > 0:
        cmd = py(EVALUATE, "--est", traj, "--gt", os.path.join(ctx["data"], "traj_gt.txt"), "--frames", frames_path, "--out", dest)
        cmd += ["--settle", str(ctx["settle"]), "--max-trans", str(ctx["max_trans"]), "--max-rot-deg", str(ctx["max_rot_deg"])]
        if replay_rc != 0:
            cmd.append("--allow-incomplete")  # For the numbers only; the case fails on the replay exit code
        with open(dest + ".eval.log", "w") as log:
            record["eval_exit"] = subprocess.call(cmd, stdout=log, stderr=subprocess.STDOUT)
        report_path = os.path.join(dest, "eval.json")
        if os.path.isfile(report_path):
            errors = json.load(open(report_path))["errors"]
            record["ate_trans_max_m"] = errors["ate_trans_m"]["max"]
            record["ate_trans_rmse_m"] = errors["ate_trans_m"]["rmse"]
            record["ate_rot_max_deg"] = errors["ate_rot_deg"]["max"]
    record["passed"] = replay_rc == 0 and record["eval_exit"] == 0
    if not ctx["keep_runs"]:
        shutil.rmtree(dest, ignore_errors=True)
    return record


def prepare(args):
    out = args.out
    os.makedirs(out, exist_ok=True)
    build = args.build_dir
    replay = os.path.join(build, "tools", "glim_localization_replay")
    compiler = os.path.join(build, "tools", "map_compiler")
    generator = os.path.join(build, "test", "synthetic_replay_dataset")
    for tool in (replay, compiler, generator):
        if not os.access(tool, os.X_OK):
            raise RuntimeError(f"{tool} is not built (run scripts/build_core.sh first)")

    synthetic = args.input or os.path.join(out, "input")
    if args.input:
        for path in ("data/meta.json", "data/traj_gt.txt", "map.pcd"):
            if not os.path.isfile(os.path.join(synthetic, path)):
                raise RuntimeError(f"--input {synthetic} has no {path} (expected the output of synthetic_replay_dataset)")
    elif not os.path.isfile(os.path.join(synthetic, "data", "meta.json")):
        shutil.rmtree(synthetic, ignore_errors=True)
        subprocess.run([generator, synthetic, str(args.duration)], check=True, stdout=subprocess.DEVNULL)

    # A copy of the shipped config with the initial yaw search on
    config_search = os.path.join(out, "config_yaw_search")
    shutil.rmtree(config_search, ignore_errors=True)
    shutil.copytree(CONFIG, config_search, ignore=shutil.ignore_patterns("gpu", "mapping_cpu"))
    odometry = os.path.join(config_search, "config_odometry_localization_cpu.json")
    text = open(odometry).read()
    if '"init_yaw_search": false' not in text:
        raise RuntimeError(f"{odometry}: init_yaw_search not found")
    open(odometry, "w").write(text.replace('"init_yaw_search": false', '"init_yaw_search": true'))

    return {
        "out": out,
        "replay": replay,
        "compiler": compiler,
        "data": os.path.join(synthetic, "data"),
        "map_dump": os.path.join(synthetic, "map"),  # The GLIM dump, used by the initial pose cases as in test_replay_e2e.sh
        "map_pcd": os.path.join(synthetic, "map.pcd"),
        "config_search": config_search,
        "settle": args.settle,
        "max_trans": args.max_trans,
        "max_rot_deg": args.max_rot_deg,
        "keep_runs": args.keep_runs,
    }


def build_map(ctx, name):
    """Write and compile the changed map; returns (compiled map dir, description, points before, points after).

    The unchanged map ("none") also goes through map_compiler, so every row compares compiled PCD maps.
    """
    original = read_pcd(ctx["map_pcd"])
    changed, description = change_map(original, name)
    maps = os.path.join(ctx["out"], "maps")
    os.makedirs(maps, exist_ok=True)
    pcd = os.path.join(maps, f"{name}.pcd")
    compiled = os.path.join(maps, name)
    write_pcd(pcd, changed)
    shutil.rmtree(compiled, ignore_errors=True)
    with open(compiled + ".log", "w") as log:
        subprocess.run([ctx["compiler"], pcd, compiled], check=True, stdout=log, stderr=subprocess.STDOUT)
    os.remove(pcd)
    return compiled, description, len(original), len(changed)


def fmt(value, digits=3):
    return "-" if value is None else f"{value:.{digits}f}"


def pose_table(records):
    lines = []
    for search in (False, True):
        rows = [r for r in records if r["group"] == "pose" and r["yaw_search"] == search]
        if not rows:
            continue
        translations = sorted({r["translation_m"] for r in rows})
        yaws = sorted({r["yaw_deg"] for r in rows})
        cell = {(r["translation_m"], r["yaw_deg"]): r for r in rows}
        lines += [
            f"Initial pose, yaw search {'on' if search else 'off'} (P pass; xN replay exit code N; i replay ok but outside the accuracy limits)",
            "",
        ]
        lines.append("| translation \\ yaw | " + " | ".join(f"{y:g}" for y in yaws) + " |")
        lines.append("|---" * (len(yaws) + 1) + "|")
        for t in translations:
            marks = []
            for y in yaws:
                r = cell.get((t, y))
                if r is None:
                    marks.append(" ")
                elif r["passed"]:
                    marks.append("P")
                elif r["replay_exit"] != 0:
                    marks.append(f"x{r['replay_exit']}")
                else:
                    marks.append("i")
            lines.append(f"| {t:g} m | " + " | ".join(marks) + " |")
        lines.append("")
    return lines


def map_table(records):
    rows = [r for r in records if r["group"] == "map"]
    if not rows:
        return []
    lines = ["Map change (initial offset of test_replay_e2e.sh)", ""]
    lines.append("| map | change | points | passed | ATE rmse [m] | ATE max [m] | rot max [deg] | mean inlier ratio | tracking ratio |")
    lines.append("|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        points = "-" if r["map_points"] is None else f"{r['map_points']}/{r['map_points_original']}"
        lines.append(
            f"| {r['map_change']} | {r['map_description']} | {points} | {'yes' if r['passed'] else 'no'} | {fmt(r['ate_trans_rmse_m'], 4)} | "
            f"{fmt(r['ate_trans_max_m'], 4)} | {fmt(r['ate_rot_max_deg'])} | {fmt(r['mean_inlier_ratio'])} | {fmt(r['tracking_ratio'])} |"
        )
    lines.append("")
    return lines


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--suite", choices=("quick", "full"), default="quick")
    parser.add_argument("--build-dir", default="/ws/build/core")
    parser.add_argument("--out", default="/ws/out/robustness")
    parser.add_argument("--input", help="an existing synthetic_replay_dataset output to reuse (default: generate <out>/input)")
    parser.add_argument("--duration", type=float, default=12.0, help="length of a generated synthetic dataset [s] (default 12)")
    parser.add_argument("--jobs", type=int, default=1, help="replays run in parallel (default 1)")
    parser.add_argument("--settle", type=float, default=3.0)
    parser.add_argument("--max-trans", type=float, default=0.1)
    parser.add_argument("--max-rot-deg", type=float, default=1.0)
    parser.add_argument("--keep-runs", action="store_true", help="keep the replay outputs under <out>/runs")
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    if args.jobs < 1 or not args.duration > 0:
        parser.error("--jobs must be at least 1 and --duration positive")

    try:
        ctx = prepare(args)
    except (RuntimeError, subprocess.CalledProcessError) as error:
        sys.stderr.write(f"error: {error}\n")
        return 2

    cases = []
    if args.suite == "full":
        pose_grid = [(t, y, s, None) for s in (False, True) for t in FULL_TRANSLATIONS for y in FULL_YAWS]
        map_changes = [(m, None) for m in FULL_MAP_CHANGES]
    else:
        pose_grid = list(QUICK_POSES)
        map_changes = list(QUICK_MAP_CHANGES)
    for t, y, search, expected in pose_grid:
        cases.append(
            {
                "group": "pose",
                "name": f"pose_t{t:g}_yaw{y:g}_{'search' if search else 'nosearch'}",
                "translation_m": t,
                "yaw_deg": y,
                "yaw_search": search,
                "expected": expected,
                "map": ctx["map_dump"],
                "init_pose": perturbed_pose(ctx["data"], t, y),
            }
        )
    e2e_pose = perturbed_pose(ctx["data"], E2E_OFFSET[0], E2E_OFFSET[1])
    for name, expected in map_changes:
        try:
            compiled, description, before, after = build_map(ctx, name)
        except subprocess.CalledProcessError as error:
            sys.stderr.write(f"error: map_compiler failed for {name}: {error}\n")
            return 2
        cases.append(
            {
                "group": "map",
                "name": f"map_{name}",
                "map_change": name,
                "map_description": description,
                "map_points_original": before,
                "map_points": after,
                "yaw_search": False,
                "expected": expected,
                "map": compiled,
                "init_pose": e2e_pose,
            }
        )

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        records = list(pool.map(lambda case: run_case(case, ctx), cases))

    failures = []
    for r in records:
        status = "pass" if r["passed"] else f"fail (replay exit {r['replay_exit']}, evaluate exit {r['eval_exit']})"
        print(f"{r['name']}: {status}; ATE max {fmt(r['ate_trans_max_m'], 4)} m, inlier {fmt(r['mean_inlier_ratio'])}")
        if r["expected"] is not None and r["expected"] != r["passed"]:
            failures.append(f"{r['name']}: expected {'pass' if r['expected'] else 'fail'}, got {'pass' if r['passed'] else 'fail'}")

    report = {
        "suite": args.suite,
        "criteria": {"settle_s": args.settle, "max_trans_m": args.max_trans, "max_rot_deg": args.max_rot_deg, "replay_exit": 0},
        "offset_direction": OFFSET_DIRECTION.tolist(),
        "results": records,
        "failures": failures,
    }
    with open(os.path.join(args.out, "results.json"), "w") as handle:
        json.dump(report, handle, indent=2)
        handle.write("\n")
    lines = [f"# Replay robustness ({args.suite})", ""] + pose_table(records) + map_table(records)
    with open(os.path.join(args.out, "results.md"), "w") as handle:
        handle.write("\n".join(lines))
    print("\n".join(lines))

    if failures:
        for failure in failures:
            sys.stderr.write(f"FAIL: {failure}\n")
        return 1
    print(f"PASS: replay robustness ({args.suite})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
