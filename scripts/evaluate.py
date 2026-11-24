#!/usr/bin/env python3
"""Evaluate a replay run: trajectory error against ground truth, and processing time.

Inputs are the outputs of glim_localization_replay (traj_localization.txt, frames.csv)
and a ground truth in TUM format (T_world_imu). Trajectories are NOT aligned: the
purpose is the absolute error in the map frame.

Exit codes: 0 ok, 1 an error threshold or the pose coverage check failed, 2 bad input
(including an incomplete run).

Coverage (needs --frames): the poses matched to ground truth in the evaluated window must be at
least --min-coverage of the frames in the window. The window starts --settle seconds after the first
frame that is not `dropped`, so frames before --init-stamp are excluded, while `skipped_no_imu`,
`diverged` and later `dropped` frames count as missing. frames.csv must belong to a complete run:
summary.json next to it must exist and its num_frames must equal the number of rows, unless
--allow-incomplete is given (the report then says "incomplete": true). Without --frames the window
starts at the first estimate and coverage is not checked; only the estimates without a ground
truth partner are bounded by --min-coverage.
"""
import argparse
import copy
import csv
import json
import os
import sys

import numpy as np

# Plots are optional and must work without a display.
import matplotlib

matplotlib.use("Agg")

from evo.core import metrics, sync  # noqa: E402
from evo.core.units import Unit  # noqa: E402
from evo.tools import file_interface  # noqa: E402


class InputError(Exception):
    pass


def read_tum(path, label):
    if not os.path.isfile(path):
        raise InputError(f"{label} not found: {path}")
    try:
        trajectory = file_interface.read_tum_trajectory_file(path)
    except Exception as error:  # evo raises its own exception types for malformed files
        raise InputError(f"cannot read {label} {path}: {error}") from error
    if trajectory.num_poses == 0:
        raise InputError(f"{label} has no poses: {path}")
    if not (np.isfinite(trajectory.positions_xyz).all() and np.isfinite(trajectory.orientations_quat_wxyz).all() and np.isfinite(trajectory.timestamps).all()):
        raise InputError(f"{label} contains non-finite values: {path}")
    return trajectory


def statistics(metric):
    return {name: float(value) for name, value in metric.get_all_statistics().items()}


def absolute_error(gt, est, relation):
    metric = metrics.APE(relation)
    metric.process_data((gt, est))
    return metric


def relative_error(gt, est, relation, delta_m):
    metric = metrics.RPE(relation, delta_m, Unit.meters, 0.1, all_pairs=False)
    metric.process_data((gt, est))
    return metric


FRAMES_COLUMNS = ("stamp_ns", "result", "t_load_ms", "t_preprocess_ms", "t_odometry_ms")


def read_frames(frames_path):
    """Rows of frames.csv, checked for the columns that are used."""
    try:
        with open(frames_path, newline="") as handle:
            reader = csv.DictReader(handle)
            missing = [c for c in FRAMES_COLUMNS if c not in (reader.fieldnames or [])]
            if missing:
                raise InputError(f"frames.csv lacks the columns {missing}: {frames_path}")
            rows = list(reader)
        for row in rows:
            int(row["stamp_ns"])
            for name in FRAMES_COLUMNS[2:]:
                if not np.isfinite(float(row[name])):
                    raise ValueError(f"non-finite {name}")
    except InputError:
        raise
    except (OSError, ValueError, TypeError, csv.Error) as error:
        raise InputError(f"cannot read frames.csv {frames_path}: {error}") from error
    if not rows:
        raise InputError(f"frames.csv has no rows: {frames_path}")
    return rows


# Stamps are written with 9 decimals; a pose exactly on the window start belongs to the window
WINDOW_EPS = 1e-9


def check_complete(frames_path, rows):
    """summary.json is written only by a complete run, and the tool removes an old one at the start."""
    summary_path = os.path.join(os.path.dirname(os.path.abspath(frames_path)), "summary.json")
    problem = None
    if not os.path.isfile(summary_path):
        problem = f"{summary_path} is missing"
    else:
        try:
            with open(summary_path) as handle:
                summary = json.load(handle)
            num_frames = summary["num_frames"]
        except (OSError, ValueError, KeyError, TypeError) as error:
            problem = f"cannot read num_frames from {summary_path}: {error}"
        else:
            if num_frames != len(rows):
                problem = f"summary.json says {num_frames} frames but frames.csv has {len(rows)} rows"
            else:
                # The replay tool writes summary.json before it judges the tracking ratio, then exits 1 on a shortfall
                ratio, minimum = summary.get("tracking_ratio"), summary.get("min_tracking_ratio")
                if isinstance(ratio, (int, float)) and isinstance(minimum, (int, float)) and minimum > 0 and ratio < minimum:
                    problem = f"the replay failed its tracking ratio gate (tracking_ratio {ratio:.4f} < min_tracking_ratio {minimum:.4f})"
    if problem:
        raise InputError(f"incomplete or aborted run ({problem}); pass --allow-incomplete to evaluate it anyway")


def processing_time(rows):
    """Statistics from the rows of frames.csv. Times are milliseconds of wall clock per frame."""
    counts = {}
    for row in rows:
        counts[row["result"]] = counts.get(row["result"], 0) + 1

    def column(name, only_estimated=False):
        return np.array([float(r[name]) for r in rows if not only_estimated or r["result"] == "estimated"])

    stamps = column("stamp_ns") * 1e-9
    total = column("t_load_ms") + column("t_preprocess_ms") + column("t_odometry_ms")
    period = float(np.median(np.diff(stamps))) if len(stamps) > 1 else None
    duration = float(stamps[-1] - stamps[0]) if len(stamps) > 1 else None

    def summarize(values):
        if values.size == 0:
            return None
        return {
            "mean": float(np.mean(values)),
            "median": float(np.median(values)),
            "p95": float(np.percentile(values, 95)),
            "p99": float(np.percentile(values, 99)),
            "max": float(np.max(values)),
        }

    result = {
        "frames_by_result": counts,
        "odometry_ms": summarize(column("t_odometry_ms", only_estimated=True)),
        "total_ms": summarize(total),
        "frame_period_s": period,
        "data_duration_s": duration,
    }
    # Total processing time divided by data length: below 1 means faster than real time.
    result["realtime_factor"] = float(total.sum() * 1e-3 / duration) if duration else None
    result["overrun_ratio"] = float(np.mean(total > period * 1e3)) if period else None
    return result


def format_text(report):
    lines = []
    trajectory = report["trajectory"]
    lines.append(
        f"poses: {trajectory['num_matched']} matched, {trajectory['num_evaluated']} evaluated "
        f"(estimated {trajectory['num_estimated']}, ground truth {trajectory['num_ground_truth']}, settle {report['settle_s']} s)"
    )
    if trajectory["coverage"] is None:
        lines.append(f"coverage: not checked, pass --frames (unmatched estimates: {trajectory['num_unmatched_estimated']})")
    else:
        lines.append(
            f"coverage: {trajectory['coverage']:.3f} of {trajectory['num_expected_in_window']} expected poses ({trajectory['expected_source']}), "
            f"unmatched estimates: {trajectory['num_unmatched_estimated']}"
        )
    if report.get("incomplete"):
        lines.append("WARNING: incomplete run (--allow-incomplete)")
    for name, label, unit in (
        ("ate_trans_m", "ATE translation", "m"),
        ("ate_rot_deg", "ATE rotation", "deg"),
        ("rpe_trans_m", "RPE translation", "m"),
        ("rpe_rot_deg", "RPE rotation", "deg"),
    ):
        stats = report["errors"][name]
        if stats is None:
            lines.append(f"{label}: not available")
        else:
            lines.append(
                f"{label} [{unit}]: rmse {stats['rmse']:.4f}, mean {stats['mean']:.4f}, "
                f"median {stats['median']:.4f}, max {stats['max']:.4f}"
            )
    timing = report.get("timing")
    if timing:
        lines.append(f"frames: {timing['frames_by_result']}")
        if timing["odometry_ms"]:
            o = timing["odometry_ms"]
            lines.append(
                f"odometry [ms]: mean {o['mean']:.1f}, median {o['median']:.1f}, p95 {o['p95']:.1f}, p99 {o['p99']:.1f}, max {o['max']:.1f}"
            )
        if timing["realtime_factor"] is not None:
            lines.append(
                f"processing time / data length: {timing['realtime_factor']:.3f}, "
                f"frames slower than the frame period: {100.0 * timing['overrun_ratio']:.1f} %"
            )
    for failure in report["failures"]:
        lines.append(f"FAIL: {failure}")
    lines.append("RESULT: " + ("FAIL" if report["failures"] else "PASS"))
    return "\n".join(lines) + "\n"


def plot(gt, est, ate_trans, out_dir):
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, 2, figsize=(12, 4))
    axes[0].plot(gt.positions_xyz[:, 0], gt.positions_xyz[:, 1], label="ground truth")
    axes[0].plot(est.positions_xyz[:, 0], est.positions_xyz[:, 1], "--", label="estimate")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].axis("equal")
    axes[0].legend()
    axes[1].plot(est.timestamps, ate_trans.error)
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("translation error [m]")
    figure.tight_layout()
    figure.savefig(os.path.join(out_dir, "eval.png"), dpi=120)
    plt.close(figure)


def evaluate(args):
    gt = read_tum(args.gt, "ground truth")
    est = read_tum(args.est, "estimate")

    try:
        gt_matched, est_matched = sync.associate_trajectories(gt, est, max_diff=args.max_time_diff)
    except Exception as error:
        raise InputError(f"cannot associate the trajectories: {error}") from error
    num_matched = est_matched.num_poses
    frame_rows = None
    if args.frames:
        if not os.path.isfile(args.frames):
            raise InputError(f"frames.csv not found: {args.frames}")
        frame_rows = read_frames(args.frames)
        if not args.allow_incomplete:
            check_complete(args.frames, frame_rows)
    if num_matched == 0:
        raise InputError(f"no pose pairs within {args.max_time_diff} s; check that the stamps share a time base")

    # Settling time counts from the first estimated pose, so a late --init-stamp is handled.
    if frame_rows is not None:
        usable = [int(r["stamp_ns"]) for r in frame_rows if r["result"] != "dropped"]
        if not usable:
            raise InputError("every frame in frames.csv is dropped: nothing was estimated")
        start = min(usable) * 1e-9 + args.settle
    else:
        start = float(est.timestamps[0]) + args.settle
    # Select by index so that both trajectories keep the same pairs (their stamps may differ slightly).
    keep = np.where(est_matched.timestamps >= start - WINDOW_EPS)[0]
    if keep.size < 2:
        raise InputError(f"fewer than 2 poses remain after excluding the first {args.settle} s")
    # Expected poses in the window, and how many estimates have no ground truth partner
    if frame_rows is not None:
        expected = sum(1 for r in frame_rows if int(r["stamp_ns"]) * 1e-9 >= start - WINDOW_EPS)
        expected_source = "frames.csv"
    else:
        expected = None
        expected_source = "none"
    estimates_in_window = int(np.sum(est.timestamps >= start - WINDOW_EPS))
    unmatched_estimates = max(estimates_in_window - int(keep.size), 0)
    coverage = (int(keep.size) / expected if expected > 0 else 0.0) if expected is not None else None
    gt_eval = copy.deepcopy(gt_matched)
    est_eval = copy.deepcopy(est_matched)
    gt_eval.reduce_to_ids(keep)
    est_eval.reduce_to_ids(keep)

    ate_trans = absolute_error(gt_eval, est_eval, metrics.PoseRelation.translation_part)
    ate_rot = absolute_error(gt_eval, est_eval, metrics.PoseRelation.rotation_angle_deg)

    errors = {"ate_trans_m": statistics(ate_trans), "ate_rot_deg": statistics(ate_rot)}
    for name, relation in (("rpe_trans_m", metrics.PoseRelation.translation_part), ("rpe_rot_deg", metrics.PoseRelation.rotation_angle_deg)):
        try:
            errors[name] = statistics(relative_error(gt_eval, est_eval, relation, args.rpe_delta))
        except Exception:  # For example, a trajectory shorter than the delta
            errors[name] = None

    report = {
        "settle_s": args.settle,
        "max_time_diff_s": args.max_time_diff,
        "rpe_delta_m": args.rpe_delta,
        "alignment": "none",
        "trajectory": {
            "num_ground_truth": gt.num_poses,
            "num_estimated": est.num_poses,
            "num_matched": num_matched,
            "num_evaluated": est_eval.num_poses,
            "num_expected_in_window": expected,
            "expected_source": expected_source,
            "coverage": coverage,
            "num_unmatched_estimated": unmatched_estimates,
        },
        "min_coverage": args.min_coverage,
        "incomplete": bool(args.allow_incomplete and frame_rows is not None),
        "errors": errors,
        "failures": [],
    }
    if frame_rows is not None:
        report["timing"] = processing_time(frame_rows)

    if coverage is not None and coverage < args.min_coverage:
        report["failures"].append(
            f"only {keep.size} of {expected} expected poses ({expected_source}) are matched in the window: coverage {coverage:.3f} < {args.min_coverage}"
        )
    if estimates_in_window > 0 and unmatched_estimates / estimates_in_window > 1.0 - args.min_coverage:
        report["failures"].append(
            f"{unmatched_estimates} of {estimates_in_window} estimated poses in the window have no ground truth within {args.max_time_diff} s"
        )

    # Thresholds apply to the worst pose, as in the localization acceptance test.
    if args.max_trans is not None and errors["ate_trans_m"]["max"] > args.max_trans:
        report["failures"].append(f"max translation error {errors['ate_trans_m']['max']:.4f} m exceeds {args.max_trans} m")
    if args.max_rot_deg is not None and errors["ate_rot_deg"]["max"] > args.max_rot_deg:
        report["failures"].append(f"max rotation error {errors['ate_rot_deg']['max']:.4f} deg exceeds {args.max_rot_deg} deg")

    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "eval.json"), "w") as handle:
        json.dump(report, handle, indent=2)
        handle.write("\n")
    text = format_text(report)
    with open(os.path.join(args.out, "eval.txt"), "w") as handle:
        handle.write(text)
    if args.plot:
        plot(gt_eval, est_eval, ate_trans, args.out)
    sys.stdout.write(text)
    return 1 if report["failures"] else 0


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--est", required=True, help="traj_localization.txt (TUM, T_world_imu)")
    parser.add_argument("--gt", required=True, help="ground truth (TUM, T_world_imu)")
    parser.add_argument("--frames", help="frames.csv, for the processing time statistics")
    parser.add_argument("--out", required=True, help="output directory for eval.json and eval.txt")
    parser.add_argument("--settle", type=float, default=0.0, help="exclude poses within this many seconds of the first estimate (default 0)")
    parser.add_argument("--max-time-diff", type=float, default=0.01, help="pairing tolerance in seconds (default 0.01)")
    parser.add_argument("--min-coverage", type=float, default=0.9, help="fail if fewer than this fraction of the expected poses are matched (default 0.9)")
    parser.add_argument("--allow-incomplete", action="store_true", help="evaluate a frames.csv without a matching summary.json (an aborted run)")
    parser.add_argument("--rpe-delta", type=float, default=1.0, help="RPE distance interval in meters (default 1)")
    parser.add_argument("--max-trans", type=float, help="fail if the maximum translation error [m] exceeds this")
    parser.add_argument("--max-rot-deg", type=float, help="fail if the maximum rotation error [deg] exceeds this")
    parser.add_argument("--plot", action="store_true", help="also write eval.png")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if not 0.0 <= args.min_coverage <= 1.0:
        sys.stderr.write("error: --min-coverage must be in [0, 1]\n")
        return 2
    try:
        return evaluate(args)
    except InputError as error:
        sys.stderr.write(f"error: {error}\n")
        return 2


if __name__ == "__main__":
    sys.exit(main())
