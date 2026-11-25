#!/usr/bin/env python3
"""Sanity checks of the simulated replay dataset (inside the development image, numpy and scipy).

1. Rates in simulation time: the LiDAR frame and IMU sample periods match the sensor model (10 Hz,
   200 Hz) with no gap, however slow the simulation ran in wall time.
2. Scan-to-map consistency: every frame, moved to the world frame by the ground truth
   (T_world_imu * T_imu_lidar), lies on the sdf2map map. A median point-to-map distance near the
   LiDAR noise shows that the scans, the ground truth stamps and the map agree (no render lag).

Usage: check_data.py <dataset> <map.pcd> --out <json> [--lidar-rate 10] [--imu-rate 200]
       [--t-imu-lidar x y z] [--stride 1]
Exit codes: 0 ok, 1 a check failed, 2 bad arguments.
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


def check_period(stamps_ns, rate, name):
    d = np.diff(np.asarray(stamps_ns, dtype=np.int64)) * 1e-9
    expected = 1.0 / rate
    result = {
        "count": len(stamps_ns),
        "span_s": (stamps_ns[-1] - stamps_ns[0]) * 1e-9,
        "period_median_s": float(np.median(d)),
        "period_max_s": float(d.max()),
        "period_min_s": float(d.min()),
        "expected_period_s": expected,
    }
    result["ok"] = bool(abs(result["period_median_s"] - expected) < 1e-3 and result["period_max_s"] < 1.5 * expected
                        and result["period_min_s"] > 0.5 * expected)
    print(f"{name}: {result['count']} over {result['span_s']:.3f} s, period median {result['period_median_s']:.4f} s "
          f"(min {result['period_min_s']:.4f}, max {result['period_max_s']:.4f}, expected {expected:.4f}): "
          f"{'ok' if result['ok'] else 'FAIL'}")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("dataset")
    parser.add_argument("map_pcd")
    parser.add_argument("--out", required=True)
    parser.add_argument("--lidar-rate", type=float, default=10.0)
    parser.add_argument("--imu-rate", type=float, default=200.0)
    parser.add_argument("--t-imu-lidar", type=float, nargs=3, default=[0.10, 0.0, 0.20])
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--max-median", type=float, default=0.05, help="fail if a frame's median distance exceeds this [m]")
    args = parser.parse_args(argv)

    frames = sorted(glob.glob(os.path.join(args.dataset, "points", "*.bin")))
    if not frames:
        print(f"no frames in {args.dataset}/points", file=sys.stderr)
        return 2
    imu_stamps = np.loadtxt(os.path.join(args.dataset, "imu.csv"), delimiter=",", skiprows=1, usecols=0, dtype=np.int64)
    gt = np.loadtxt(os.path.join(args.dataset, "traj_gt.txt"), comments="#", ndmin=2)

    report = {"imu": check_period(imu_stamps, args.imu_rate, "imu")}
    lidar = [read_frame(p) for p in frames]
    report["lidar"] = check_period([s for s, _ in lidar], args.lidar_rate, "lidar")

    tree = cKDTree(read_pcd_xyz(args.map_pcd))
    slerp = Slerp(gt[:, 0], Rotation.from_quat(gt[:, 4:8]))
    t_imu_lidar = np.asarray(args.t_imu_lidar)
    medians = []
    for stamp_ns, xyz in lidar[:: args.stride]:
        t = stamp_ns * 1e-9
        if t < gt[0, 0] or t > gt[-1, 0] or len(xyz) == 0:
            continue
        rot = slerp([t])[0]
        pos = np.array([np.interp(t, gt[:, 0], gt[:, i]) for i in (1, 2, 3)])
        world = rot.apply(xyz + t_imu_lidar) + pos
        dist, _ = tree.query(world)
        medians.append((t, float(np.median(dist)), float(np.percentile(dist, 90))))
    med = np.array(medians)
    report["scan_to_map"] = {
        "frames": len(medians),
        "median_of_medians_m": float(np.median(med[:, 1])),
        "max_median_m": float(med[:, 1].max()),
        "worst_frame_s": float(med[np.argmax(med[:, 1]), 0]),
        "median_p90_m": float(np.median(med[:, 2])),
    }
    report["scan_to_map"]["ok"] = bool(report["scan_to_map"]["max_median_m"] <= args.max_median)
    s = report["scan_to_map"]
    print(f"scan to map: {s['frames']} frames, median distance {s['median_of_medians_m']:.4f} m "
          f"(worst frame {s['max_median_m']:.4f} m at {s['worst_frame_s']:.1f} s, p90 {s['median_p90_m']:.4f} m): "
          f"{'ok' if s['ok'] else 'FAIL'}")
    with open(args.out, "w") as f:
        json.dump(report, f, indent=2)
    return 0 if all(report[k]["ok"] for k in ("imu", "lidar", "scan_to_map")) else 1


if __name__ == "__main__":
    sys.exit(main())
