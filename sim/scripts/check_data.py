#!/usr/bin/env python3
"""Sanity checks of the simulated replay dataset (inside the development image, numpy and scipy).

1. Timing in simulation time: the LiDAR, IMU and ground truth counts match the simulated duration
   (LiDAR duration * rate + 1, IMU duration * rate + 1, ground truth duration * rate + 1), the first
   stamp of each is within one period of 0 (nothing lost while the bridge discovered the Gazebo topics),
   the periods match the sensor model with no gap, and every LiDAR frame is inside the ground truth span.
2. IMU against the ground truth: the mean specific force over the initial hold is gravity, gyro z is the
   ground truth yaw rate, and the horizontal specific force is the second derivative of the ground truth
   position in the IMU frame. A Gazebo IMU without linear acceleration fails here.
3. Scan-to-map consistency: every frame, moved to the world frame by the ground truth
   (T_world_imu * T_imu_lidar), lies on the sdf2map map. A median point-to-map distance near the
   LiDAR noise plus the map voxel shows that the scans, the ground truth and the map agree (a wrong
   extrinsic, a map in another frame, or scans rendered at another pose than their stamp would not).

Usage: check_data.py <dataset> <map.pcd> --out <json> --duration <s> --t-imu-lidar x y z
       [--lidar-rate 10] [--imu-rate 200] [--gt-rate 100] [--hold 2] [--stride 1] [--max-median 0.05]
Exit codes: 0 ok, 1 a check failed, 2 bad arguments or input.
"""
import argparse
import glob
import json
import os
import struct
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

BIN_HEADER = struct.Struct("<4sIqIIII")
GRAVITY = 9.80665


def read_pcd_xyz(path):
    with open(path, "rb") as f:
        header = {}
        while True:
            line = f.readline().decode("ascii").strip()
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(" ")
            header[key.upper()] = value.split()
            if key.upper() == "DATA":
                break
        fields = header["FIELDS"]
        sizes = [int(v) for v in header["SIZE"]]
        types = header["TYPE"]
        counts = [int(v) for v in header.get("COUNT", ["1"] * len(fields))]
        n = int(header["POINTS"][0])
        if header["DATA"][0] == "ascii":
            data = np.loadtxt(f, ndmin=2)
            cols = np.cumsum([0] + counts)
            return np.stack([data[:, cols[fields.index(a)]] for a in "xyz"], axis=1)
        if header["DATA"][0] != "binary":
            raise ValueError(f"unsupported PCD DATA {header['DATA'][0]}")
        dtype = np.dtype([(name, f"<{t.lower()}{s}", (c,)) if c > 1 else (name, f"<{t.lower()}{s}")
                          for name, s, t, c in zip(fields, sizes, types, counts)])
        arr = np.frombuffer(f.read(n * dtype.itemsize), dtype=dtype, count=n)
        return np.stack([arr[a].astype(float) for a in "xyz"], axis=1)


def read_frame(path):
    with open(path, "rb") as f:
        magic, _, stamp_ns, n, _, step, _ = BIN_HEADER.unpack(f.read(BIN_HEADER.size))
        assert magic == b"GLRP" and step == 20
        body = np.frombuffer(f.read(n * step), dtype="<f4").reshape(n, 5)
    return stamp_ns, body[:, :3].astype(float)


def check_stream(stamps_s, rate, duration, tolerance, name):
    stamps_s = np.asarray(stamps_s, dtype=float)
    expected_period = 1.0 / rate
    expected_count = int(round(duration * rate)) + 1
    result = {"count": int(len(stamps_s)), "expected_count": expected_count, "expected_period_s": expected_period}
    if len(stamps_s) < 2:
        result.update(ok=False, reason="fewer than 2 stamps")
        print(f"{name}: {len(stamps_s)} stamps: FAIL")
        return result
    d = np.diff(stamps_s)
    result.update(first_s=float(stamps_s[0]), last_s=float(stamps_s[-1]), period_median_s=float(np.median(d)),
                  period_min_s=float(d.min()), period_max_s=float(d.max()))
    checks = {
        "count": abs(len(stamps_s) - expected_count) <= tolerance,
        "first_stamp": 0.0 <= stamps_s[0] <= expected_period + 1e-6,
        "period": abs(result["period_median_s"] - expected_period) < 1e-3 and result["period_max_s"] < 1.5 * expected_period
        and result["period_min_s"] > 0.5 * expected_period,
    }
    checks = {k: bool(v) for k, v in checks.items()}
    result["checks"] = checks
    result["ok"] = all(checks.values())
    failed = [k for k, v in checks.items() if not v]
    print(f"{name}: {len(stamps_s)} (expected {expected_count}), first {stamps_s[0]:.3f} s, last {stamps_s[-1]:.3f} s, "
          f"period median {result['period_median_s']:.4f} s (min {result['period_min_s']:.4f}, max "
          f"{result['period_max_s']:.4f}, expected {expected_period:.4f}): {'ok' if not failed else 'FAIL ' + ', '.join(failed)}")
    return result


def moving_average(x, n):
    if n <= 1:
        return x
    kernel = np.ones(n) / n
    return np.apply_along_axis(lambda c: np.convolve(c, kernel, mode="same"), 0, x)


def check_imu(imu, gt, hold, imu_rate, gt_rate):
    """IMU against the ground truth (planar or not: the full rotation is used)."""
    t_imu, acc, gyro = imu[:, 0], imu[:, 1:4], imu[:, 4:7]
    t_gt, pos = gt[:, 0], gt[:, 1:4]
    rot = Rotation.from_quat(gt[:, 4:8])
    result = {}

    # Gravity during the initial hold (skip the first 0.2 s)
    sel = (t_imu > 0.2) & (t_imu < hold - 0.2)
    mean_acc = acc[sel].mean(axis=0)
    result["hold"] = {"samples": int(sel.sum()), "mean_acc": mean_acc.tolist(),
                      "ok": bool(sel.sum() > 10 and abs(mean_acc[2] - GRAVITY) < 0.05 and np.hypot(*mean_acc[:2]) < 0.05)}
    print(f"imu hold: mean acc ({mean_acc[0]:.3f}, {mean_acc[1]:.3f}, {mean_acc[2]:.3f}) m/s^2 over {sel.sum()} samples "
          f"(expected (0, 0, {GRAVITY:.3f})): {'ok' if result['hold']['ok'] else 'FAIL'}")

    # Ground truth body rates and specific force, smoothed over 0.1 s like the IMU, at the IMU stamps
    inside = (t_imu >= t_gt[0] + 0.05) & (t_imu <= t_gt[-1] - 0.05)
    rotvec_rate = np.gradient(np.unwrap(rot.as_euler("ZYX")[:, 0]), t_gt)  # yaw rate (world z)
    vel = np.gradient(pos, t_gt, axis=0)
    acc_world = np.gradient(vel, t_gt, axis=0)
    force_body = rot.inv().apply(acc_world + np.array([0.0, 0.0, GRAVITY]))
    n_gt = max(1, int(round(0.1 * gt_rate)))
    n_imu = max(1, int(round(0.1 * imu_rate)))
    yaw_rate_gt = np.interp(t_imu[inside], t_gt, moving_average(rotvec_rate, n_gt))
    force_gt = np.stack([np.interp(t_imu[inside], t_gt, moving_average(force_body[:, i], n_gt)) for i in range(2)], axis=1)
    gyro_z = moving_average(gyro[:, 2], n_imu)[inside]
    acc_h = moving_average(acc[:, :2], n_imu)[inside]

    gyro_rmse = float(np.sqrt(np.mean((gyro_z - yaw_rate_gt) ** 2)))
    result["gyro_z"] = {"rmse_rad_s": gyro_rmse, "gt_max_abs_rad_s": float(np.abs(yaw_rate_gt).max()),
                        "ok": bool(gyro_rmse < 0.02 and np.abs(yaw_rate_gt).max() > 0.3)}
    print(f"imu gyro z vs ground truth yaw rate: rmse {gyro_rmse:.4f} rad/s (|rate| up to "
          f"{result['gyro_z']['gt_max_abs_rad_s']:.2f} rad/s): {'ok' if result['gyro_z']['ok'] else 'FAIL'}")

    acc_rmse = float(np.sqrt(np.mean(np.sum((acc_h - force_gt) ** 2, axis=1))))
    gt_max = float(np.linalg.norm(force_gt, axis=1).max())
    imu_std = float(acc_h.std(axis=0).max())
    no_linear_acc = imu_std < 1e-3 and gt_max > 0.1
    result["horizontal_acc"] = {"rmse_m_s2": acc_rmse, "gt_max_m_s2": gt_max, "imu_std_m_s2": imu_std,
                                "no_linear_acceleration": bool(no_linear_acc),
                                "ok": bool(acc_rmse < 0.1 and gt_max > 0.3 and not no_linear_acc)}
    if no_linear_acc:
        print("imu horizontal acc: FAIL, the IMU reports no linear acceleration while the ground truth accelerates")
    print(f"imu horizontal acc vs ground truth (second derivative, IMU frame): rmse {acc_rmse:.4f} m/s^2 (|acc| up to "
          f"{gt_max:.2f} m/s^2): {'ok' if result['horizontal_acc']['ok'] else 'FAIL'}")
    result["ok"] = all(result[k]["ok"] for k in ("hold", "gyro_z", "horizontal_acc"))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("dataset")
    parser.add_argument("map_pcd")
    parser.add_argument("--out", required=True)
    parser.add_argument("--duration", type=float, required=True, help="simulated duration [s] (record.json sim_duration_s)")
    parser.add_argument("--t-imu-lidar", type=float, nargs=3, required=True, help="LiDAR position in the IMU frame (same axes)")
    parser.add_argument("--lidar-rate", type=float, default=10.0)
    parser.add_argument("--imu-rate", type=float, default=200.0)
    parser.add_argument("--gt-rate", type=float, default=100.0)
    parser.add_argument("--hold", type=float, default=2.0, help="the rig is still for this long at the start [s]")
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--max-median", type=float, default=0.05, help="fail if a frame's median distance exceeds this [m]")
    args = parser.parse_args(argv)

    frames = sorted(glob.glob(os.path.join(args.dataset, "points", "*.bin")))
    if not frames:
        print(f"FAIL: no frames in {args.dataset}/points", file=sys.stderr)
        return 2
    imu = np.loadtxt(os.path.join(args.dataset, "imu.csv"), delimiter=",", skiprows=1, ndmin=2)
    imu[:, 0] *= 1e-9
    gt = np.loadtxt(os.path.join(args.dataset, "traj_gt.txt"), comments="#", ndmin=2)
    if len(gt) < 2 or len(imu) < 2:
        print("FAIL: fewer than 2 ground truth poses or IMU samples", file=sys.stderr)
        return 2
    lidar = [read_frame(p) for p in frames]
    lidar_t = np.array([s for s, _ in lidar]) * 1e-9

    report = {
        "lidar": check_stream(lidar_t, args.lidar_rate, args.duration, 1, "lidar"),
        "imu": check_stream(imu[:, 0], args.imu_rate, args.duration, 2, "imu"),
        "ground_truth": check_stream(gt[:, 0], args.gt_rate, args.duration, 1, "ground truth"),
    }
    eps = 1e-6  # stamps from integer nanoseconds and from TUM text
    outside = lidar_t[(lidar_t < gt[0, 0] - eps) | (lidar_t > gt[-1, 0] + eps)]
    report["lidar_in_gt_span"] = {"outside": outside.tolist(), "ok": bool(len(outside) == 0)}
    print(f"lidar frames inside the ground truth span [{gt[0, 0]:.3f}, {gt[-1, 0]:.3f}] s: "
          + ("ok" if not len(outside) else f"FAIL, {len(outside)} outside: {outside[:5].tolist()}"))

    report["imu_vs_gt"] = check_imu(imu, gt, args.hold, args.imu_rate, args.gt_rate)

    tree = cKDTree(read_pcd_xyz(args.map_pcd))
    slerp = Slerp(gt[:, 0], Rotation.from_quat(gt[:, 4:8]))
    t_imu_lidar = np.asarray(args.t_imu_lidar)
    medians = []
    for (stamp_ns, xyz), t in list(zip(lidar, lidar_t))[:: args.stride]:
        t = min(max(t, gt[0, 0]), gt[-1, 0]) if gt[0, 0] - eps <= t <= gt[-1, 0] + eps else t
        if t < gt[0, 0] or t > gt[-1, 0] or len(xyz) == 0:
            continue
        rot = slerp([t])[0]
        pos = np.array([np.interp(t, gt[:, 0], gt[:, i]) for i in (1, 2, 3)])
        dist, _ = tree.query(rot.apply(xyz + t_imu_lidar) + pos)
        medians.append((t, float(np.median(dist)), float(np.percentile(dist, 90))))
    expected = len(lidar[:: args.stride])
    if not medians:
        report["scan_to_map"] = {"frames": 0, "expected_frames": expected, "ok": False}
        print("scan to map: FAIL, no frame inside the ground truth span has points")
    else:
        med = np.array(medians)
        s = report["scan_to_map"] = {
            "frames": len(medians),
            "expected_frames": expected,
            "median_of_medians_m": float(np.median(med[:, 1])),
            "max_median_m": float(med[:, 1].max()),
            "worst_frame_s": float(med[np.argmax(med[:, 1]), 0]),
            "median_p90_m": float(np.median(med[:, 2])),
        }
        s["ok"] = bool(s["max_median_m"] <= args.max_median and s["frames"] == expected)
        print(f"scan to map: {s['frames']} of {expected} frames, median distance {s['median_of_medians_m']:.4f} m "
              f"(worst frame {s['max_median_m']:.4f} m at {s['worst_frame_s']:.1f} s, p90 {s['median_p90_m']:.4f} m): "
              f"{'ok' if s['ok'] else 'FAIL'}")
    with open(args.out, "w") as f:
        json.dump(report, f, indent=2)
    return 0 if all(v["ok"] for v in report.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
