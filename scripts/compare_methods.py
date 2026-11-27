#!/usr/bin/env python3
"""Compare localization methods on one bag: accuracy, latency, CPU and memory (docs/benchmark.md).

  compare_methods.py --bench <dir> --gt <traj_gt.txt> [--settle 2] [--svg <file>]

<bench> holds <method>/run<k>/ directories written by sim/scripts/bench_run.sh (clouds.csv, poses.csv,
traj.txt and resources.json of sim/scripts/bench_recorder.py). For every run:
- ATE and RPE against the ground truth with the functions of evaluate.py: no alignment, the window
  starts --settle seconds after the first cloud, RPE over 1 m;
- latency: the receive time of the pose minus the receive time of the cloud with the same stamp
  (within --stamp-tol), both on the monotonic clock of the recorder;
- poses: the fraction of the clouds that got a pose, and the poses that match no cloud;
- CPU: utime + stime of the node between the first and the last cloud, per scan and as a mean percentage
  of one core;
- memory: VmHWM (peak RSS) at the end, and the largest VmRSS sample.
Writes <bench>/benchmark.json (every run and the summary) and <bench>/benchmark.md (one row per method:
the median over the runs and the range in brackets), and with --svg a bar chart of latency and ATE
(plain SVG, as the other figures of docs/media).
"""
import argparse
import bisect
import copy
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import evaluate  # noqa: E402

LABELS = {
    "glim": "glim_localization",
    "lidarloc": "lidar_localization_ros2 NDT (IMU, default)",
    "lidarloc_noimu": "lidar_localization_ros2 NDT, no IMU",
}
ORDER = ["glim", "lidarloc", "lidarloc_noimu"]

# (key, column header, format) of the table
COLUMNS = [
    ("ate_trans_rmse_mm", "ATE trans RMSE [mm]", "{:.1f}"),
    ("ate_rot_rmse_deg", "ATE rot RMSE [deg]", "{:.3f}"),
    ("rpe_trans_rmse_mm", "RPE trans RMSE [mm/m]", "{:.1f}"),
    ("rpe_rot_rmse_deg", "RPE rot RMSE [deg/m]", "{:.3f}"),
    ("latency_median_ms", "Latency median [ms]", "{:.1f}"),
    ("latency_p95_ms", "Latency p95 [ms]", "{:.1f}"),
    ("latency_max_ms", "Latency max [ms]", "{:.1f}"),
    ("cpu_ms_per_scan", "CPU per scan [ms]", "{:.1f}"),
    ("cpu_percent", "CPU [% of a core]", "{:.0f}"),
    ("peak_rss_mib", "Peak RSS [MiB]", "{:.0f}"),
    ("pose_fraction", "Scans with a pose", "{:.3f}"),
]


def read_csv(path):
    with open(path) as handle:
        return list(csv.DictReader(handle))


def latency(clouds, poses, stamp_tol_ns):
    """Pair every cloud with the pose of the same stamp.

    clouds, poses: lists of (stamp_ns, recv_ns). A pose pairs with the cloud of the nearest stamp within
    stamp_tol_ns, each cloud with at most one pose (the first received). Returns the latencies in ms in
    the order of the clouds, the number of clouds without a pose and the number of poses without a cloud.
    """
    stamps = sorted(s for s, _ in clouds)
    recv = {}
    for s, r in clouds:
        recv.setdefault(s, r)
    pose_of = {}
    unmatched_poses = 0
    for s, r in poses:
        i = bisect.bisect_left(stamps, s)
        near = [stamps[j] for j in (i - 1, i) if 0 <= j < len(stamps)]
        best = min(near, key=lambda c: abs(c - s)) if near else None
        if best is None or abs(best - s) > stamp_tol_ns or best in pose_of:
            unmatched_poses += 1
            continue
        pose_of[best] = r
    latencies = [(pose_of[s] - recv[s]) * 1e-6 for s in sorted(recv) if s in pose_of]
    return latencies, len(recv) - len(pose_of), unmatched_poses


def cpu_usage(resources, num_clouds):
    """CPU ms per scan and the mean percentage of one core between the first and the last cloud."""
    first, last = resources.get("cpu_first"), resources.get("cpu_last")
    if not first or not last or num_clouds < 2 or last[0] <= first[0]:
        return None, None
    cpu_s = (last[1] - first[1]) / resources["clk_tck"]
    wall_s = (last[0] - first[0]) * 1e-9
    return cpu_s * 1000.0 / (num_clouds - 1), 100.0 * cpu_s / wall_s


def accuracy(gt, est_path, start_s, max_time_diff=0.01, rpe_delta_m=1.0):
    """ATE and RPE RMSE of the estimates from start_s, without alignment (evaluate.py functions)."""
    from evo.core import metrics, sync

    est = evaluate.read_tum(est_path, "estimate")
    gt_matched, est_matched = sync.associate_trajectories(gt, est, max_diff=max_time_diff)
    keep = np.where(est_matched.timestamps >= start_s - 1e-6)[0]
    if keep.size < 2:
        raise evaluate.InputError("fewer than 2 poses in the window")
    gt_eval, est_eval = copy.deepcopy(gt_matched), copy.deepcopy(est_matched)
    gt_eval.reduce_to_ids(keep)
    est_eval.reduce_to_ids(keep)
    trans, rot = metrics.PoseRelation.translation_part, metrics.PoseRelation.rotation_angle_deg
    ate_trans = evaluate.statistics(evaluate.absolute_error(gt_eval, est_eval, trans))
    result = {
        "num_evaluated": int(keep.size),
        "ate_trans_rmse_mm": 1000.0 * ate_trans["rmse"],
        "ate_trans_max_mm": 1000.0 * ate_trans["max"],
        "ate_rot_rmse_deg": evaluate.statistics(evaluate.absolute_error(gt_eval, est_eval, rot))["rmse"],
    }
    for key, relation, scale in (("rpe_trans_rmse_mm", trans, 1000.0), ("rpe_rot_rmse_deg", rot, 1.0)):
        try:
            result[key] = scale * evaluate.statistics(evaluate.relative_error(gt_eval, est_eval, relation, rpe_delta_m))["rmse"]
        except Exception:  # a trajectory shorter than the delta
            result[key] = None
    return result


def run_metrics(run_dir, gt, settle_s, stamp_tol_ns):
    clouds = [(int(r["stamp_ns"]), int(r["recv_ns"])) for r in read_csv(os.path.join(run_dir, "clouds.csv"))]
    pose_rows = read_csv(os.path.join(run_dir, "poses.csv"))
    poses = [(int(r["stamp_ns"]), int(r["recv_ns"])) for r in pose_rows]
    with open(os.path.join(run_dir, "resources.json")) as handle:
        resources = json.load(handle)
    lat, missing, unmatched = latency(clouds, poses, stamp_tol_ns)
    num_clouds = len({s for s, _ in clouds})
    cpu_per_scan, cpu_percent = cpu_usage(resources, len(clouds))
    rss = [kb for _, kb in resources.get("rss_kb", []) if kb is not None]
    result = {
        "num_clouds": num_clouds,
        "num_poses": len(poses),
        "missing_poses": missing,
        "unmatched_poses": unmatched,
        "pose_fraction": (num_clouds - missing) / num_clouds if num_clouds else 0.0,
        "latency_median_ms": float(np.median(lat)) if lat else None,
        "latency_p95_ms": float(np.percentile(lat, 95)) if lat else None,
        "latency_max_ms": float(np.max(lat)) if lat else None,
        "cpu_ms_per_scan": cpu_per_scan,
        "cpu_percent": cpu_percent,
        "peak_rss_mib": resources["vm_hwm_kb"] / 1024.0 if resources.get("vm_hwm_kb") else None,
        "max_sampled_rss_mib": max(rss) / 1024.0 if rss else None,
    }
    for key in ("ate_trans_rmse_mm", "ate_trans_max_mm", "ate_rot_rmse_deg", "rpe_trans_rmse_mm", "rpe_rot_rmse_deg"):
        result[key] = None
    if clouds and len(poses) >= 2:
        start_s = min(s for s, _ in clouds) * 1e-9 + settle_s
        try:
            result.update(accuracy(gt, os.path.join(run_dir, "traj.txt"), start_s))
        except Exception as error:  # InputError, or evo's own types on bad input
            result["accuracy_error"] = str(error)
    return result


def summarize(runs):
    """The median and the range over the runs of every metric; None where a run has no value."""
    summary = {}
    for key in runs[0]:
        values = [r[key] for r in runs]
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in values):
            continue
        summary[key] = {"median": float(np.median(values)), "min": float(min(values)), "max": float(max(values))}
    return summary


def cell(stat, fmt):
    if stat is None:
        return "n/a"
    text = fmt.format(stat["median"])
    if fmt.format(stat["min"]) != fmt.format(stat["max"]):
        text += f" ({fmt.format(stat['min'])} to {fmt.format(stat['max'])})"
    return text


def format_table(summaries, num_runs):
    """A Markdown table, one row per method: the median over the runs and the range in brackets."""
    header = ["Method", "Runs"] + [title for _, title, _ in COLUMNS]
    lines = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    for method, summary in summaries.items():
        row = [LABELS.get(method, method), str(num_runs[method])]
        row += [cell(summary.get(key), fmt) for key, _, fmt in COLUMNS]
        lines.append("| " + " | ".join(row) + " |")
    return "\n".join(lines) + "\n"


def nice_step(top, ticks=4):
    raw = top / ticks
    magnitude = 10 ** np.floor(np.log10(raw))
    return float(min(m * magnitude for m in (1, 2, 2.5, 5, 10) if m * magnitude >= raw))


def plot_svg(summaries, path):
    """Plain SVG bars, in the style of docs/media/gpu_profile_*.svg: latency (median, p95 whisker) and ATE."""
    colors = {"glim": "#2a78d6", "lidarloc": "#eb6834", "lidarloc_noimu": "#f2a57f"}
    methods = list(summaries)
    width, left, right, row_h = 760, 300, 680, 22
    panels = [("latency_median_ms", "latency_p95_ms", "Latency, scan in to pose out [ms]: median, whisker to p95"),
              ("ate_trans_rmse_mm", None, "ATE translation RMSE [mm]")]
    out = []
    y = 76
    for key, whisker_key, title in panels:
        values = [summaries[m].get(key, {}).get("median") for m in methods]
        whiskers = [summaries[m].get(whisker_key, {}).get("median") if whisker_key else None for m in methods]
        top = max([v for v in values + whiskers if v is not None] or [1.0])
        step = nice_step(top)
        span = step * np.ceil(top / step)
        scale = (right - left) / span
        out.append(f'<text x="16" y="{y}" font-size="13" font-weight="600" fill="#0b0b0b">{title}</text>')
        y0, y1 = y + 8, y + 8 + row_h * len(methods)
        for i in range(int(round(span / step)) + 1):
            x = left + i * step * scale
            out.append(f'<line x1="{x:.1f}" y1="{y0}" x2="{x:.1f}" y2="{y1}" stroke="#e4e3df" stroke-width="1"/>')
            out.append(f'<text x="{x:.1f}" y="{y1 + 16}" font-size="11" fill="#52514e" text-anchor="middle">{i * step:g}</text>')
        for i, (method, value, whisker) in enumerate(zip(methods, values, whiskers)):
            top_y = y0 + 4 + i * row_h
            label = LABELS.get(method, method)
            out.append(f'<text x="{left - 10}" y="{top_y + 11}" font-size="12" fill="#0b0b0b" text-anchor="end">{label}</text>')
            if value is None:
                out.append(f'<text x="{left + 6}" y="{top_y + 11}" font-size="11" fill="#52514e">n/a</text>')
                continue
            x_end = left + value * scale
            color = colors.get(method, "#8a8984")
            out.append(f'<rect x="{left}" y="{top_y}" width="{max(x_end - left, 1):.1f}" height="14" rx="3" fill="{color}">'
                       f'<title>{label}: {value:.2f}</title></rect>')
            text_x = x_end
            if whisker is not None:
                xw = left + whisker * scale
                out.append(f'<line x1="{x_end:.1f}" y1="{top_y + 7}" x2="{xw:.1f}" y2="{top_y + 7}" stroke="#0b0b0b" stroke-width="1"/>')
                out.append(f'<line x1="{xw:.1f}" y1="{top_y + 2}" x2="{xw:.1f}" y2="{top_y + 12}" stroke="#0b0b0b" stroke-width="1">'
                           f'<title>p95: {whisker:.2f}</title></line>')
                text_x = max(x_end, xw)
                text = f"{value:.1f} (p95 {whisker:.1f})"
            else:
                text = f"{value:.1f}"
            out.append(f'<text x="{text_x + 6:.1f}" y="{top_y + 11}" font-size="11" fill="#52514e">{text}</text>')
        out.append(f'<line x1="{left}" y1="{y0}" x2="{left}" y2="{y1}" stroke="#52514e" stroke-width="1"/>')
        y = y1 + 46
    height = y - 20
    head = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" '
            'font-family="-apple-system,Segoe UI,Helvetica,Arial,sans-serif" role="img" '
            'aria-label="Benchmark on the Gazebo bag: latency and ATE per method">',
            f'<rect width="{width}" height="{height}" fill="#fcfcfb"/>',
            '<text x="16" y="24" font-size="15" font-weight="600" fill="#0b0b0b">Benchmark on the Gazebo bag</text>',
            '<text x="16" y="42" font-size="12" fill="#52514e">ros2 bag play at 1x, the same map and initial pose. '
            'Median over the runs.</text>']
    with open(path, "w") as handle:
        handle.write("\n".join(head + out + ["</svg>"]) + "\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bench", required=True)
    parser.add_argument("--gt", required=True, help="ground truth (TUM, T_world_imu)")
    parser.add_argument("--settle", type=float, default=2.0, help="seconds after the first cloud excluded (default 2)")
    parser.add_argument("--stamp-tol", type=float, default=1e-3, help="pose to cloud stamp tolerance [s] (default 0.001)")
    parser.add_argument("--svg", help="also write a bar chart of latency and ATE")
    args = parser.parse_args(argv)

    gt = evaluate.read_tum(args.gt, "ground truth")
    methods = sorted((m for m in os.listdir(args.bench) if os.path.isdir(os.path.join(args.bench, m)) and m != "bag"),
                     key=lambda m: (ORDER.index(m) if m in ORDER else len(ORDER), m))
    report = {"settle_s": args.settle, "stamp_tol_s": args.stamp_tol, "alignment": "none", "methods": {}}
    summaries, num_runs = {}, {}
    for method in methods:
        mdir = os.path.join(args.bench, method)
        run_dirs = sorted(d for d in os.listdir(mdir) if d.startswith("run") and os.path.isfile(os.path.join(mdir, d, "resources.json")))
        if not run_dirs:
            continue
        runs = {d: run_metrics(os.path.join(mdir, d), gt, args.settle, int(args.stamp_tol * 1e9)) for d in run_dirs}
        summaries[method] = summarize(list(runs.values()))
        num_runs[method] = len(runs)
        report["methods"][method] = {"label": LABELS.get(method, method), "runs": runs, "summary": summaries[method]}
    if not summaries:
        print(f"compare_methods: no runs in {args.bench}", file=sys.stderr)
        return 2
    with open(os.path.join(args.bench, "benchmark.json"), "w") as handle:
        json.dump(report, handle, indent=2)
        handle.write("\n")
    table = format_table(summaries, num_runs)
    with open(os.path.join(args.bench, "benchmark.md"), "w") as handle:
        handle.write(table)
    if args.svg:
        plot_svg(summaries, args.svg)
    sys.stdout.write(table)
    return 0


if __name__ == "__main__":
    sys.exit(main())
