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
from matplotlib import font_manager  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from PIL import Image  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402


class InputError(Exception):
    pass


ESTIMATE_COLOR = "#1565c0"
SCAN_COLOR = "#e53935"
GT_COLOR = "#757575"
# Near-black: apart from the blue estimate, the red scan, the gray map and ground truth, and the state
# colors, and kept by the GIF palette (the title text is the same tone)
POSE_COLOR = "#222222"

MAP_COLOR = "#8a8a8a"
FRAME_COLOR = "#bbbbbb"
LEGEND_EDGE_COLOR = "#cccccc"
TEXT_COLOR = "#000000"
FALLBACK_FONT = "DejaVu Sans"  # shipped with matplotlib

STATE_COLORS = {
    "TRACKING": "#2e7d32",
    "DEGRADED": "#ef6c00",
    "INITIALIZING": "#1565c0",
    "LOST": "#c62828",
}


def finite(points):
    """The rows of an (N, 3) array whose x, y and z are all finite."""
    return points[np.isfinite(points).all(axis=1)]


def read_pcd_xyz(path):
    """Finite x, y, z of a PCD file (DATA ascii or binary), as an (N, 3) float64 array."""
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
        return finite(values[:, [columns.index(axis) for axis in "xyz"]])

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
    return finite(np.stack([data[axis].astype(np.float64) for axis in "xyz"], axis=1))


SCAN_HEADER = struct.Struct("<4sIqIIII")


def read_scan(path):
    """Finite x, y, z of a replay points/*.bin file (DESIGN 6.5), as an (N, 3) float64 array."""
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
    return finite(points[:, :3].astype(np.float64))


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
            rows = list(reader)
    except (OSError, csv.Error) as error:
        raise InputError(f"cannot read frames.csv {path}: {error}") from error
    for line, row in enumerate(rows, start=2):
        if None in row.values():
            raise InputError(f"short row on line {line} of {path}")
        try:
            int(row["stamp_ns"])
        except ValueError as error:
            raise InputError(f"stamp_ns is not an integer on line {line} of {path}: {row['stamp_ns']!r}") from error
    return rows


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


def select_font(family):
    """family if matplotlib finds it, otherwise DejaVu Sans with a warning on stderr."""
    try:
        font_manager.findfont(font_manager.FontProperties(family=family), fallback_to_default=False)
        return family
    except ValueError:
        sys.stderr.write(f"warning: font {family!r} not found, using {FALLBACK_FONT}\n")
        return FALLBACK_FONT


def fixed_colors():
    """The colors that gif_palette puts first, in this order."""
    return [TEXT_COLOR, "#ffffff", MAP_COLOR, FRAME_COLOR, LEGEND_EDGE_COLOR, ESTIMATE_COLOR, SCAN_COLOR, GT_COLOR, POSE_COLOR,
            *STATE_COLORS.values()]


def gif_palette(images, colors):
    """One palette for all frames: the drawing colors exactly, plus adaptive colors from sample frames.

    The fixed entries keep black, white, the text and state colors and the plot colors exact in every
    frame, whether they appear in the sampled frames or not; the adaptive ones, from the first, middle
    and last frames stacked, cover the antialiased edges. Frames are mapped without dithering, so text
    edges are not speckled.
    """
    samples = [images[0], images[len(images) // 2], images[-1]]
    image = Image.new("RGB", (samples[0].width, sum(sample.height for sample in samples)))
    for k, sample in enumerate(samples):
        image.paste(sample.convert("RGB"), (0, k * samples[0].height))
    entries = [tuple(int(c[k:k + 2], 16) for k in (1, 3, 5)) for c in fixed_colors()]
    adaptive = image.quantize(colors=max(colors - len(entries), 2), method=Image.Quantize.MEDIANCUT)
    values = adaptive.getpalette()[: 3 * len(adaptive.getcolors())]
    for k in range(0, len(values) - 2, 3):
        entry = tuple(values[k:k + 3])
        if entry not in entries and len(entries) < 256:
            entries.append(entry)
    flat = [v for entry in entries for v in entry]
    flat += flat[-3:] * (256 - len(entries))
    palette = Image.new("P", (1, 1))
    palette.putpalette(flat)
    return palette


def to_palette(image, palette):
    """image mapped to the nearest palette entry, without dithering.

    Done here rather than by Image.quantize(palette=...), whose lookup works on reduced-precision
    colors and can map an exact drawing color (for example white) to a near neighbor.
    """
    entries = np.array(palette.getpalette()[:768], dtype=np.int32).reshape(-1, 3)
    rgb = np.asarray(image.convert("RGB"), dtype=np.uint32)
    # One integer per pixel, so np.unique sorts a flat array instead of rows (same colors, same order).
    keys, inverse = np.unique(((rgb[..., 0] << 16) | (rgb[..., 1] << 8) | rgb[..., 2]).reshape(-1), return_inverse=True)
    colors = np.stack([keys >> 16, (keys >> 8) & 0xFF, keys & 0xFF], axis=1).astype(np.int32)
    nearest = np.argmin(((colors[:, None, :] - entries[None, :, :]) ** 2).sum(axis=2), axis=1).astype(np.uint8)
    mapped = Image.fromarray(nearest[inverse.reshape(-1)].reshape(rgb.shape[:2]), mode="P")
    mapped.putpalette(palette.getpalette()[:768])
    return mapped


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

    # Layout in logical pixels (at --scale 1); the height follows the aspect of the view, below a
    # fixed header band for the text. Font sizes are in points: 1 pt = 100 / 72 logical px.
    matplotlib.rcParams["font.family"] = select_font(args.font)
    header_px, pad_px = 64, 8
    width_px = args.width
    plot_w = width_px - 2 * pad_px
    plot_h = int(np.clip(plot_w * (high[1] - low[1]) / (high[0] - low[0]), 120, 2 * width_px))
    height_px = header_px + plot_h + pad_px
    height_px += height_px % 2
    figure = plt.figure(figsize=(width_px / 100, height_px / 100), dpi=100 * args.scale)
    axes = figure.add_axes([pad_px / width_px, pad_px / height_px, plot_w / width_px, plot_h / height_px])
    axes.set_aspect("equal")
    axes.set_xlim(low[0], high[0])
    axes.set_ylim(low[1], high[1])
    axes.set_xticks([])
    axes.set_yticks([])
    for spine in axes.spines.values():
        spine.set_color(FRAME_COLOR)
    axes.scatter(map_xy[:, 0], map_xy[:, 1], s=1.0, c=MAP_COLOR, linewidths=0, antialiased=False, rasterized=True)
    gt_line = None
    if gt_xy is not None:
        (gt_line,) = axes.plot(gt_xy[:, 0], gt_xy[:, 1], color=GT_COLOR, linewidth=1.0, linestyle=(0, (3, 2)), label="ground truth")
    (est_line,) = axes.plot([], [], color=ESTIMATE_COLOR, linewidth=1.8, label="estimate")
    scan_artist = axes.scatter([], [], s=2.0, c=SCAN_COLOR, linewidths=0, antialiased=False)
    (pose_marker,) = axes.plot([], [], marker="o", markersize=7, color=POSE_COLOR, markeredgecolor="white", linestyle="none", zorder=5, label="pose")
    heading = axes.annotate(
        "", xy=(0, 0), xytext=(0, 0), arrowprops={"arrowstyle": "-|>", "color": POSE_COLOR, "linewidth": 1.8}, zorder=5)
    # Proxy handles: the scan dots are tiny on the plot but need a visible legend marker
    handles = [pose_marker, est_line] + ([gt_line] if gt_line is not None else []) + [
        Line2D([], [], marker="o", markersize=4, color=SCAN_COLOR, linestyle="none", label="scan")]
    legend = axes.legend(handles=handles, loc="lower right", fontsize=8.6, framealpha=1.0, fancybox=False)
    legend.get_frame().set_facecolor("white")
    legend.get_frame().set_edgecolor(LEGEND_EDGE_COLOR)

    def text(x_px, y_px, value="", ha="left", **kwargs):
        x = x_px / width_px if x_px >= 0 else 1.0 + x_px / width_px
        return figure.text(x, 1.0 - y_px / height_px, value, ha=ha, va="center", color=TEXT_COLOR, **kwargs)

    # Labels and values are separate texts at fixed positions; the values are right-aligned, and
    # Noto Sans and DejaVu Sans digits have one width, so the numbers do not move between frames.
    text(pad_px, 18, args.title, fontsize=11.5, weight="bold")
    state_text = text(pad_px, 46, fontsize=10, weight="bold")
    text(-(pad_px + 205), 46, "t", ha="right", fontsize=9.4)
    time_text = text(-(pad_px + 150), 46, ha="right", fontsize=9.4)
    text(-(pad_px + 40), 46, "inlier ratio", ha="right", fontsize=9.4)
    inlier_text = text(-pad_px, 46, ha="right", fontsize=9.4)
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
        time_text.set_text(f"{stamp - t0:.1f} s")
        inlier_text.set_text(f"{inlier:.2f}" if np.isfinite(inlier) else "-")
        figure.canvas.draw()
        rgba = np.asarray(figure.canvas.buffer_rgba())
        images.append(Image.fromarray(rgba[:, :, :3].copy()))
    plt.close(figure)

    duration_ms = int(round(1000.0 / args.fps))
    try:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        if args.out.lower().endswith(".gif"):
            palette = gif_palette(images, args.colors)
            frames_p = [to_palette(image, palette) for image in images]
            frames_p[0].save(args.out, save_all=True, append_images=frames_p[1:], duration=duration_ms, loop=0, optimize=True)
        else:
            images[0].save(args.out, save_all=True, append_images=images[1:], duration=duration_ms, loop=0, quality=80, method=4)
    except OSError as error:
        raise InputError(f"cannot write {args.out}: {error}") from error
    size = os.path.getsize(args.out)
    print(f"wrote {args.out}: {len(images)} frames, {images[0].width}x{images[0].height} px, {size / 1e6:.2f} MB")
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
    parser.add_argument("--z-min", type=float, help="draw only map and scan points at or above this map-frame height (for example, to hide the floor)")
    parser.add_argument("--z-max", type=float, help="draw only map and scan points below this map-frame height (with --z-min, a horizontal slice)")
    parser.add_argument("--margin", type=float, default=8.0, help="view margin around the trajectory in meters (default 8)")
    parser.add_argument("--width", type=int, default=720, help="layout width in pixels; the height follows the view (default 720)")
    parser.add_argument("--scale", type=float, default=2.0, help="output pixels per layout pixel, for HiDPI screens (default 2: 1440 px wide)")
    parser.add_argument("--colors", type=int, default=256, help="GIF palette size, including the fixed drawing colors (default 256, the maximum)")
    parser.add_argument("--fps", type=float, default=10.0, help="animation frame rate (default 10)")
    parser.add_argument("--stride", type=int, default=1, help="draw every n-th estimated frame (default 1)")
    parser.add_argument("--max-frames", type=int, default=0, help="stop after this many animation frames (default 0: all)")
    parser.add_argument("--max-map-points", type=int, default=60000, help="random subset of the map points drawn (default 60000, 0: all)")
    parser.add_argument("--max-scan-points", type=int, default=4000, help="random subset of each scan drawn (default 4000, 0: all)")
    parser.add_argument("--font", default="Noto Sans", help="font family for all text (default Noto Sans, in the image; DejaVu Sans if missing)")
    parser.add_argument("--title", default="glim_localization: replay on a prebuilt map", help="title line")
    args = parser.parse_args(argv)
    if args.stride < 1 or args.fps <= 0 or args.width < 160 or args.scale <= 0 or not 24 <= args.colors <= 256:
        parser.error("--stride must be >= 1, --fps > 0, --width >= 160, --scale > 0 and --colors in [24, 256]")
    if args.z_min is not None and args.z_max is not None and args.z_min >= args.z_max:
        parser.error("--z-min must be below --z-max")
    return args


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        return render(args)
    except (InputError, OSError) as error:
        sys.stderr.write(f"error: {error}\n")
        return 2


if __name__ == "__main__":
    sys.exit(main())
