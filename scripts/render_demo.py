#!/usr/bin/env python3
"""Render a top-down animation of a replay run (GIF or WebP), offscreen.

Each animation frame shows the map points, the current scan transformed by the estimated pose, the
estimated trajectory so far (and the ground truth, when given), and the localization state and
inlier ratio from frames.csv.

Inputs:
  --data  a replay dataset (points/*.bin, optional traj_gt.txt)
  --run   the output directory of glim_localization_replay (traj_localization.txt, frames.csv)
  --map   the map as a PCD file (ascii or binary; x, y, z as float or double)

Only frames with an estimated pose are drawn. Scans are in the LiDAR frame and are moved to the map
frame with T_world_imu * inv(T_lidar_imu); pass --t-lidar-imu with the value of config_sensors.json
when it is not the identity.

Exit codes: 0 ok, 2 bad input.
"""
import argparse
import csv
import os
import struct
import sys

import numpy as np

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
from PIL import Image  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402


class InputError(Exception):
    pass


STATE_COLORS = {
    "TRACKING": "#2e7d32",
    "DEGRADED": "#ef6c00",
    "INITIALIZING": "#1565c0",
    "LOST": "#c62828",
}


def read_pcd_xyz(path):
    """x, y, z of a PCD file (DATA ascii or binary), as an (N, 3) float64 array."""
    try:
        with open(path, "rb") as handle:
            header = {}
            while True:
                line = handle.readline()
                if not line:
                    raise InputError(f"PCD header without DATA: {path}")
                text = line.decode("ascii", errors="replace").strip()
                if not text or text.startswith("#"):
                    continue
                key, _, value = text.partition(" ")
                header[key.upper()] = value.split()
                if key.upper() == "DATA":
                    break
            body = handle.read()
    except OSError as error:
        raise InputError(f"cannot read the map {path}: {error}") from error

    try:
        fields = header["FIELDS"]
        sizes = [int(v) for v in header["SIZE"]]
        types = header["TYPE"]
        counts = [int(v) for v in header.get("COUNT", ["1"] * len(fields))]
        num_points = int(header["POINTS"][0])
    except (KeyError, ValueError, IndexError) as error:
        raise InputError(f"incomplete PCD header in {path}: {error}") from error
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise InputError(f"inconsistent PCD header in {path}")
    if any(axis not in fields for axis in "xyz"):
        raise InputError(f"the PCD has no x, y, z fields: {path}")
    encoding = header["DATA"][0].lower()

    if encoding == "ascii":
        columns = []
        for name, count in zip(fields, counts):
            columns.extend([name] + [f"{name}_{k}" for k in range(1, count)])
        try:
            values = np.loadtxt(body.decode("ascii").splitlines(), ndmin=2, dtype=np.float64)
        except ValueError as error:
            raise InputError(f"cannot parse the PCD data in {path}: {error}") from error
        if values.size == 0:
            return np.zeros((0, 3))
        if values.shape[1] != len(columns):
            raise InputError(f"PCD rows have {values.shape[1]} values, the header says {len(columns)}: {path}")
        return values[:, [columns.index(axis) for axis in "xyz"]]

    if encoding != "binary":
        raise InputError(f"PCD DATA {encoding} is not supported (ascii or binary): {path}")
    kinds = {"F": "f", "I": "i", "U": "u"}
    dtype = []
    for name, size, kind, count in zip(fields, sizes, types, counts):
        if kind not in kinds:
            raise InputError(f"unknown PCD type {kind}: {path}")
        dtype.append((name, f"<{kinds[kind]}{size}", (count,) if count > 1 else ()))
    try:
        dtype = np.dtype(dtype)
    except TypeError as error:
        raise InputError(f"bad PCD field layout in {path}: {error}") from error
    if len(body) < num_points * dtype.itemsize:
        raise InputError(f"truncated PCD data in {path}")
    data = np.frombuffer(body, dtype=dtype, count=num_points)
    return np.stack([data[axis].astype(np.float64) for axis in "xyz"], axis=1)


SCAN_HEADER = struct.Struct("<4sIqIIII")


def read_scan(path):
    """x, y, z of a replay points/*.bin file (DESIGN 6.5), as an (N, 3) float64 array."""
    try:
        with open(path, "rb") as handle:
            raw = handle.read()
    except OSError as error:
        raise InputError(f"cannot read the scan {path}: {error}") from error
    if len(raw) < SCAN_HEADER.size:
        raise InputError(f"truncated scan header: {path}")
    magic, version, _, num_points, _, step, _ = SCAN_HEADER.unpack_from(raw)
    if magic != b"GLRP" or version != 1 or step != 20:
        raise InputError(f"not a version 1 replay scan: {path}")
    if len(raw) < SCAN_HEADER.size + num_points * step:
        raise InputError(f"truncated scan data: {path}")
    points = np.frombuffer(raw, dtype="<f4", count=num_points * 5, offset=SCAN_HEADER.size).reshape(-1, 5)
    return points[:, :3].astype(np.float64)


def read_tum(path):
    """(stamps [s], poses as (N, 4, 4)) of a TUM file."""
    try:
        values = np.loadtxt(path, ndmin=2, comments="#")
    except (OSError, ValueError) as error:
        raise InputError(f"cannot read the trajectory {path}: {error}") from error
    if values.size == 0:
        return np.zeros(0), np.zeros((0, 4, 4))
    if values.shape[1] != 8 or not np.isfinite(values).all():
        raise InputError(f"not a finite TUM trajectory (t x y z qx qy qz qw): {path}")
    poses = np.tile(np.eye(4), (len(values), 1, 1))
    poses[:, :3, :3] = Rotation.from_quat(values[:, 4:8]).as_matrix()
    poses[:, :3, 3] = values[:, 1:4]
    return values[:, 0], poses


def read_frames(path):
    try:
        with open(path, newline="") as handle:
            reader = csv.DictReader(handle)
            missing = [c for c in ("stamp_ns", "result", "state", "inlier_ratio") if c not in (reader.fieldnames or [])]
            if missing:
                raise InputError(f"frames.csv lacks the columns {missing}: {path}")
            return list(reader)
    except (OSError, csv.Error) as error:
        raise InputError(f"cannot read frames.csv {path}: {error}") from error


def pose_matrix(values):
    """A 4x4 matrix from x y z qx qy qz qw."""
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(values[3:7]).as_matrix()
    matrix[:3, 3] = values[:3]
    return matrix


def transform(matrix, points):
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def z_filter(points, z_min, z_max):
    keep = np.ones(len(points), dtype=bool)
    if z_min is not None:
        keep &= points[:, 2] >= z_min
    if z_max is not None:
        keep &= points[:, 2] < z_max
    return points[keep]


def subsample(points, limit, rng):
    if limit > 0 and len(points) > limit:
        return points[rng.choice(len(points), limit, replace=False)]
    return points


def collect_frames(args):
    """The frames to draw: (stamp [s], T_world_imu, state, inlier ratio, scan path), in stamp order."""
    stamps, poses = read_tum(os.path.join(args.run, "traj_localization.txt"))
    if len(stamps) == 0:
        raise InputError(f"no estimated poses in {args.run}/traj_localization.txt")
    rows = read_frames(os.path.join(args.run, "frames.csv"))
    frames = []
    for row in rows:
        if row["result"] != "estimated":
            continue
        stamp_ns = int(row["stamp_ns"])
        index = int(np.argmin(np.abs(stamps - stamp_ns * 1e-9)))
        if abs(stamps[index] - stamp_ns * 1e-9) > 1e-6:
            raise InputError(f"no pose in traj_localization.txt for the estimated frame at {stamp_ns} ns")
        scan = os.path.join(args.data, "points", f"{stamp_ns:019d}.bin")
        try:
            inlier = float(row["inlier_ratio"])
        except ValueError:
            inlier = float("nan")
        frames.append((stamp_ns * 1e-9, poses[index], row["state"], inlier, scan))
    if not frames:
        raise InputError(f"frames.csv in {args.run} has no estimated frames")
    frames = frames[:: args.stride]
    if args.max_frames > 0:
        frames = frames[: args.max_frames]
    return frames


def render(args):
    if os.path.splitext(args.out)[1].lower() not in (".gif", ".webp"):
        raise InputError(f"the output must end in .gif or .webp: {args.out}")
    rng = np.random.default_rng(0)
    frames = collect_frames(args)
    T_imu_lidar = np.linalg.inv(pose_matrix(args.t_lidar_imu))

    gt_path = None if args.no_gt else (args.gt or os.path.join(args.data, "traj_gt.txt"))
    gt_xy = None
    if gt_path and (args.gt or os.path.isfile(gt_path)):
        _, gt_poses = read_tum(gt_path)
        gt_xy = gt_poses[:, :2, 3]

    map_points = z_filter(read_pcd_xyz(args.map), args.z_min, args.z_max)
    if len(map_points) == 0:
        raise InputError(f"no map points left after the height filter: {args.map}")
    est_xy = np.array([f[1][:2, 3] for f in frames])

    # The view: the trajectory with a margin, clipped to the map
    low = est_xy.min(axis=0) - args.margin
    high = est_xy.max(axis=0) + args.margin
    low = np.maximum(low, map_points[:, :2].min(axis=0) - 1.0)
    high = np.minimum(high, map_points[:, :2].max(axis=0) + 1.0)
    in_view = np.all((map_points[:, :2] >= low) & (map_points[:, :2] <= high), axis=1)
    map_xy = subsample(map_points[in_view, :2], args.max_map_points, rng)

    # The height follows the aspect of the view, below a fixed header band for the text
    header_px, pad_px = 56, 8
    width_px = args.width
    plot_w = width_px - 2 * pad_px
    plot_h = int(np.clip(plot_w * (high[1] - low[1]) / (high[0] - low[0]), 120, 2 * width_px))
    height_px = header_px + plot_h + pad_px
    height_px += height_px % 2
    figure = plt.figure(figsize=(width_px / 100, height_px / 100), dpi=100)
    axes = figure.add_axes([pad_px / width_px, pad_px / height_px, plot_w / width_px, plot_h / height_px])
    axes.set_aspect("equal")
    axes.set_xlim(low[0], high[0])
    axes.set_ylim(low[1], high[1])
    axes.set_xticks([])
    axes.set_yticks([])
    for spine in axes.spines.values():
        spine.set_color("#bbbbbb")
    axes.scatter(map_xy[:, 0], map_xy[:, 1], s=0.6, c="#8a8a8a", linewidths=0, rasterized=True)
    if gt_xy is not None:
        axes.plot(gt_xy[:, 0], gt_xy[:, 1], color="#424242", linewidth=1.0, linestyle=(0, (3, 2)), label="ground truth")
    (est_line,) = axes.plot([], [], color="#1565c0", linewidth=1.8, label="estimate")
    scan_artist = axes.scatter([], [], s=1.0, c="#e53935", linewidths=0, label="scan")
    (pose_marker,) = axes.plot([], [], marker="o", markersize=6, color="#1565c0", markeredgecolor="white")
    heading = axes.annotate("", xy=(0, 0), xytext=(0, 0), arrowprops={"arrowstyle": "-|>", "color": "#1565c0", "linewidth": 1.5})
    axes.legend(loc="lower right", fontsize=7, markerscale=4, framealpha=0.8)
    def header_y(px):
        return 1.0 - px / height_px

    figure.text(pad_px / width_px, header_y(16), args.title, fontsize=11, weight="bold", va="center")
    state_text = figure.text(pad_px / width_px, header_y(40), "", fontsize=10, weight="bold", va="center")
    info_text = figure.text(1.0 - pad_px / width_px, header_y(40), "", fontsize=9, va="center", ha="right", family="monospace")
    arrow_length = 0.04 * float(np.max(high - low))

    images = []
    t0 = frames[0][0]
    for i, (stamp, T_world_imu, state, inlier, scan_path) in enumerate(frames):
        scan = transform(T_world_imu @ T_imu_lidar, read_scan(scan_path))
        scan = subsample(z_filter(scan, args.z_min, args.z_max), args.max_scan_points, rng)
        scan_artist.set_offsets(scan[:, :2] if len(scan) else np.zeros((0, 2)))
        est_line.set_data(est_xy[: i + 1, 0], est_xy[: i + 1, 1])
        position = T_world_imu[:2, 3]
        forward = T_world_imu[:2, 0]
        norm = np.linalg.norm(forward)
        forward = forward / norm if norm > 1e-9 else np.array([1.0, 0.0])
        pose_marker.set_data([position[0]], [position[1]])
        heading.xy = tuple(position + arrow_length * forward)
        heading.set_position(tuple(position))
        state_text.set_text(state)
        state_text.set_color(STATE_COLORS.get(state, "#424242"))
        inlier_text = f"{inlier:.2f}" if np.isfinite(inlier) else "-"
        info_text.set_text(f"t {stamp - t0:5.1f} s   inlier ratio {inlier_text}")
        figure.canvas.draw()
        rgba = np.asarray(figure.canvas.buffer_rgba())
        images.append(Image.fromarray(rgba[:, :, :3].copy()))
    plt.close(figure)

    duration_ms = int(round(1000.0 / args.fps))
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    if args.out.lower().endswith(".gif"):
        # One adaptive palette for all frames keeps the colors stable and the file small
        palette = images[len(images) // 2].quantize(colors=64, method=Image.Quantize.MEDIANCUT)
        frames_p = [image.quantize(palette=palette, dither=Image.Dither.NONE) for image in images]
        frames_p[0].save(args.out, save_all=True, append_images=frames_p[1:], duration=duration_ms, loop=0, optimize=True)
    else:
        images[0].save(args.out, save_all=True, append_images=images[1:], duration=duration_ms, loop=0, quality=80, method=4)
    size = os.path.getsize(args.out)
    print(f"wrote {args.out}: {len(images)} frames, {width_px}x{height_px} px, {size / 1e6:.2f} MB")
    return 0


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", required=True, help="replay dataset directory (points/, optional traj_gt.txt)")
    parser.add_argument("--run", required=True, help="glim_localization_replay output directory")
    parser.add_argument("--map", required=True, help="map as a PCD file")
    parser.add_argument("--out", required=True, help="output animation (.gif or .webp)")
    parser.add_argument("--gt", help="ground truth (TUM); default <data>/traj_gt.txt when it exists")
    parser.add_argument("--no-gt", action="store_true", help="do not draw the ground truth")
    parser.add_argument("--t-lidar-imu", type=float, nargs=7, default=[0, 0, 0, 0, 0, 0, 1], metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
                        help="T_lidar_imu of config_sensors.json (default identity)")
    parser.add_argument("--z-min", type=float, help="draw only points at or above this map-frame height (for example, to hide the floor)")
    parser.add_argument("--z-max", type=float, help="draw only points below this map-frame height (for example, to hide a ceiling)")
    parser.add_argument("--margin", type=float, default=8.0, help="view margin around the trajectory in meters (default 8)")
    parser.add_argument("--width", type=int, default=720, help="image width in pixels; the height follows the view (default 720)")
    parser.add_argument("--fps", type=float, default=10.0, help="animation frame rate (default 10)")
    parser.add_argument("--stride", type=int, default=1, help="draw every n-th estimated frame (default 1)")
    parser.add_argument("--max-frames", type=int, default=0, help="stop after this many animation frames (default 0: all)")
    parser.add_argument("--max-map-points", type=int, default=60000, help="random subset of the map points drawn (default 60000, 0: all)")
    parser.add_argument("--max-scan-points", type=int, default=4000, help="random subset of each scan drawn (default 4000, 0: all)")
    parser.add_argument("--title", default="glim_localization: replay on a prebuilt map", help="title line")
    args = parser.parse_args(argv)
    if args.stride < 1 or args.fps <= 0 or args.width < 160:
        parser.error("--stride must be >= 1, --fps > 0 and --width >= 160")
    return args


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        return render(args)
    except InputError as error:
        sys.stderr.write(f"error: {error}\n")
        return 2


if __name__ == "__main__":
    sys.exit(main())
