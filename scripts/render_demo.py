#!/usr/bin/env python3
"""Render an animation of a replay run (GIF or WebP), offscreen.

Each animation frame shows the map points, the current scan transformed by the estimated pose, the
estimated trajectory so far (and the ground truth, when given), the current pose, and the localization
state and inlier ratio from frames.csv.

Views (--view):
  oblique  (default) a fixed perspective view of the whole run from above and beside; the map is
           colored by height and the pose is a 3D frame (heading arrow and up axis)
  chase    the same 3D drawing from a camera that follows the sensor
  top      a top-down view of the map points, optionally a height band (--z-min, --z-max)

The 3D views project the points with a pinhole camera and draw them far to near (painter's order).

Inputs:
  --data  a replay dataset (points/*.bin, optional traj_gt.txt)
  --run   the output directory of glim_localization_replay (traj_localization.txt, frames.csv; with
          --init-search also replay_args.json and summary.json)
  --map   the map as a PCD file (ascii or binary; x, y, z as float or double)

Only frames with an estimated pose are drawn. Scans are in the LiDAR frame and are moved to the map
frame with T_world_imu * inv(T_lidar_imu); pass --t-lidar-imu with the value of config_sensors.json
when it is not the identity.

Exit codes: 0 ok, 2 bad input.
"""
import argparse
import csv
import json
import os
import struct
import sys

import numpy as np

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import colormaps, font_manager  # noqa: E402
from matplotlib.colors import to_hex  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
from PIL import Image  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402


class InputError(Exception):
    pass


ESTIMATE_COLOR = "#1565c0"
SCAN_COLOR = "#e53935"
GT_COLOR = "#757575"
# Okabe-Ito orange: apart from the blue estimate also with color-vision deficiency (green would not be
# apart from the red scan). The dot has a thin dark edge, which the edgeless scan dots do not.
POSE_COLOR = "#e69f00"
POSE_EDGE_COLOR = "#222222"

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
# The --init-search intro: the given pose, then the pose the yaw search picked
INIT_GUESS_COLOR = "#6a1b9a"

# Map points of the 3D views. The ground (up to GROUND_CUT above the lowest map height) is a light gray,
# drawn smaller, so it recedes and the red scan keeps its contrast on it. An opaque light gray stands in
# for transparency: blended points would make colors that are not palette entries. The points above it
# are colored by height with viridis (perceptually uniform, no red) from dark purple to light green, in
# a few discrete levels so each level is one exact palette entry. The yellow top of viridis is left out,
# so no level is near the orange pose.
GROUND_COLOR = "#d9d9d9"
GROUND_CUT = 0.15
GROUND_SIZE = 0.75  # diameter relative to the other points
HEIGHT_LEVELS = 16
HEIGHT_COLORS = [to_hex(colormaps["viridis"](0.85 * k / (HEIGHT_LEVELS - 1))) for k in range(HEIGHT_LEVELS)]

VIEWS = ("oblique", "chase", "top")


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


def read_json(path):
    try:
        with open(path) as handle:
            return json.load(handle)
    except (OSError, ValueError) as error:
        raise InputError(f"cannot read {path}: {error}") from error


def read_init_search(run):
    """(the given initial T_world_imu, the yaw search record of summary.json) of a replay run."""
    args = read_json(os.path.join(run, "replay_args.json"))
    summary = read_json(os.path.join(run, "summary.json"))
    try:
        values = [float(v) for v in args["effective"]["init_T_world_imu_xyz_qxyzw"]]
    except (KeyError, TypeError, ValueError) as error:
        raise InputError(f"replay_args.json in {run} has no initial pose: {error}") from error
    if len(values) != 7 or not np.isfinite(values).all() or np.linalg.norm(values[3:]) < 1e-9:
        raise InputError(f"bad initial pose in {run}/replay_args.json: {values}")
    search = summary.get("init_search") if isinstance(summary, dict) else None
    if not isinstance(search, dict) or search.get("result") != "ACCEPTED":
        raise InputError(f"summary.json in {run} has no accepted yaw search (run with init_yaw_search: true)")
    try:
        search = {key: float(search[key]) for key in ("yaw_offset_deg", "score", "candidates", "seconds")}
    except (KeyError, TypeError, ValueError) as error:
        raise InputError(f"incomplete init_search in {run}/summary.json: {error}") from error
    return pose_matrix(values), search


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


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """R_cam_world of a camera at eye looking at target: rows right, down, forward (x right, y down in the image)."""
    forward = np.asarray(target, dtype=float) - np.asarray(eye, dtype=float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, up)
    if np.linalg.norm(right) < 1e-9:
        raise ValueError("the view direction is parallel to up")
    right /= np.linalg.norm(right)
    return np.stack([right, np.cross(forward, right), forward])


class Camera:
    """A pinhole camera: R_cam_world, the eye in the world, the focal length and principal point in pixels."""

    def __init__(self, R, eye, focal, center, near=0.3):
        self.R = np.asarray(R, dtype=float)
        self.eye = np.asarray(eye, dtype=float)
        self.focal = float(focal)
        self.center = np.asarray(center, dtype=float)
        self.near = near

    def project(self, points):
        """(pixel u v as (N, 2), depth (N,)) of world points; u v are NaN behind the near plane."""
        cam = (np.asarray(points, dtype=float).reshape(-1, 3) - self.eye) @ self.R.T
        depth = cam[:, 2]
        uv = np.full((len(cam), 2), np.nan)
        front = depth > self.near
        uv[front] = self.focal * cam[front, :2] / depth[front, None] + self.center
        return uv, depth


def fit_camera(R, eye, corners, width, height, margin):
    """The focal length and principal point that fit the corners (all in front) into the image with a margin."""
    cam = (np.asarray(corners, dtype=float) - eye) @ np.asarray(R).T
    if np.any(cam[:, 2] <= 0.0):
        raise ValueError("a corner is behind the camera")
    normalized = cam[:, :2] / cam[:, 2:3]
    low, high = normalized.min(axis=0), normalized.max(axis=0)
    span = np.maximum(high - low, 1e-9)
    focal = min((width - 2 * margin) / span[0], (height - 2 * margin) / span[1])
    center = np.array([width / 2.0, height / 2.0]) - focal * (low + high) / 2.0
    return focal, center


def painter_order(depth):
    """Indices that draw far points first, so that near points cover them (stable for equal depths)."""
    return np.argsort(-np.asarray(depth), kind="stable")


def height_levels(z, low, high):
    """The index into HEIGHT_COLORS of each height, from low to high."""
    scaled = (np.asarray(z) - low) / max(high - low, 1e-9) * HEIGHT_LEVELS
    return np.clip(np.floor(scaled), 0, HEIGHT_LEVELS - 1).astype(int)


def support_heights(map_points, positions, radius=0.35, clearance=0.05):
    """For each position, the highest map point below it (minus a clearance) within radius in x y.

    The surface the sensor is over: the floor, or the top of an obstacle it passes over. The lowest map
    height where nothing is below.
    """
    tree = cKDTree(map_points[:, :2])
    floor = float(map_points[:, 2].min())
    heights = []
    for position in positions:
        near = tree.query_ball_point(position[:2], radius)
        below = [map_points[k, 2] for k in near if map_points[k, 2] < position[2] - clearance]
        heights.append(max(below) if below else floor)
    return np.array(heights)


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
            POSE_EDGE_COLOR, *STATE_COLORS.values(), INIT_GUESS_COLOR, GROUND_COLOR, *HEIGHT_COLORS]


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


def rgba_of(color):
    """(r, g, b, 1) in [0, 1] of a #rrggbb color."""
    return np.array([int(color[k:k + 2], 16) / 255.0 for k in (1, 3, 5)] + [1.0])


def unit_xy(vector, fallback=(1.0, 0.0)):
    norm = np.linalg.norm(vector[:2])
    return vector[:2] / norm if norm > 1e-9 else np.asarray(fallback, dtype=float)


class TopView:
    """Top-down view: map points in gray, the pose as a dot and a heading arrow."""

    def __init__(self, figure, rect, low, high, map_points, gt_xyz, est_xyz, args, rng):
        axes = figure.add_axes(rect)
        axes.set_aspect("equal")
        axes.set_xlim(low[0], high[0])
        axes.set_ylim(low[1], high[1])
        axes.set_xticks([])
        axes.set_yticks([])
        for spine in axes.spines.values():
            spine.set_color(FRAME_COLOR)
        in_view = np.all((map_points[:, :2] >= low) & (map_points[:, :2] <= high), axis=1)
        map_xy = subsample(map_points[in_view, :2], args.max_map_points, rng)
        axes.scatter(map_xy[:, 0], map_xy[:, 1], s=1.0, c=MAP_COLOR, linewidths=0, antialiased=False, rasterized=True)
        self.gt_line = None
        if gt_xyz is not None:
            (self.gt_line,) = axes.plot(gt_xyz[:, 0], gt_xyz[:, 1], color=GT_COLOR, linewidth=1.0, linestyle=(0, (3, 2)), label="ground truth")
        (self.est_line,) = axes.plot([], [], color=ESTIMATE_COLOR, linewidth=1.8, label="estimate")
        self.scan_artist = axes.scatter([], [], s=2.0, c=SCAN_COLOR, linewidths=0, antialiased=False)
        (self.pose_marker,) = axes.plot([], [], marker="o", markersize=7, color=POSE_COLOR, markeredgecolor=POSE_EDGE_COLOR,
                                        markeredgewidth=0.8, linestyle="none", zorder=5, label="pose")
        self.heading = axes.annotate(
            "", xy=(0, 0), xytext=(0, 0), arrowprops={"arrowstyle": "-|>", "color": POSE_COLOR, "linewidth": 1.8}, zorder=5)
        self.arrow_length = 0.04 * float(np.max(high - low))
        self.est_xyz = est_xyz
        self.axes = axes

    def update(self, index, T_world_imu, scan):
        self.scan_artist.set_offsets(scan[:, :2] if len(scan) else np.zeros((0, 2)))
        self.est_line.set_data(self.est_xyz[: index + 1, 0], self.est_xyz[: index + 1, 1])
        position = T_world_imu[:2, 3]
        forward = unit_xy(T_world_imu[:3, 0])
        self.pose_marker.set_data([position[0]], [position[1]])
        self.heading.xy = tuple(position + self.arrow_length * forward)
        self.heading.set_position(tuple(position))


class PerspectiveView:
    """3D view: map points colored by height and the scan, far to near; the pose as a frame with a stem to the surface below."""

    SCAN_DEPTH_BIAS = 0.15  # [m] toward the camera, so a scan point on a map surface is drawn over it

    def __init__(self, figure, rect, size, map_points, gt_xyz, est_xyz, args, rng):
        self.width, self.height = size
        axes = figure.add_axes(rect)
        axes.set_xlim(0, self.width)
        axes.set_ylim(self.height, 0)
        axes.set_xticks([])
        axes.set_yticks([])
        for spine in axes.spines.values():
            spine.set_color(FRAME_COLOR)
        self.axes = axes
        self.args = args
        self.est_xyz = est_xyz
        self.gt_xyz = gt_xyz

        # The map around the run: the trajectory's x y range with a margin
        low = est_xyz[:, :2].min(axis=0) - args.margin
        high = est_xyz[:, :2].max(axis=0) + args.margin
        near_run = np.all((map_points[:, :2] >= low) & (map_points[:, :2] <= high), axis=1)
        if not np.any(near_run):
            raise InputError("no map points near the trajectory")
        cropped = map_points[near_run]
        # Scan points are drawn inside the same box, up to the top of the map
        self.box = (np.array([*cropped[:, :2].min(axis=0), -np.inf]), np.array([*cropped[:, :2].max(axis=0), cropped[:, 2].max() + 0.1]))
        self.map_points = subsample(cropped, args.max_map_points, rng)
        # The ground is gray up to GROUND_CUT above the floor (a low percentile, robust to stray points);
        # the height colors span from there to the top of the map
        z_floor, z_high = np.percentile(cropped[:, 2], [0.5, 99.5])
        cut = round(round(float(z_floor), 1) + GROUND_CUT, 2) + 0.0
        self.z_range = (cut, max(round(float(z_high), 1) + 0.0, cut + 0.1))
        palette = np.array([rgba_of(c) for c in HEIGHT_COLORS])
        self.map_ground = self.map_points[:, 2] < cut
        self.map_rgba = palette[height_levels(self.map_points[:, 2], *self.z_range)]
        self.map_rgba[self.map_ground] = rgba_of(GROUND_COLOR)
        self.scan_rgba = rgba_of(SCAN_COLOR)
        self.support = support_heights(cropped, est_xyz)
        travelled = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(est_xyz, axis=0), axis=1))])
        self.drop_indices = np.searchsorted(travelled, np.arange(0.0, travelled[-1] + 1e-9, 0.6))

        self.cloud = axes.scatter(np.zeros(0), np.zeros(0), s=1.0, linewidths=0, antialiased=False, marker="o")
        self.gt_line = None
        if gt_xyz is not None:
            (self.gt_line,) = axes.plot([], [], color=GT_COLOR, linewidth=1.0, linestyle=(0, (3, 2)), zorder=3, label="ground truth")
        (self.est_line,) = axes.plot([], [], color=ESTIMATE_COLOR, linewidth=1.8, zorder=3, label="estimate")
        # Drop lines from the estimate to the surface below it, every 0.6 m along the path: the height of the
        # path above the floor or an obstacle
        (self.drops,) = axes.plot([], [], color=ESTIMATE_COLOR, linewidth=0.7, zorder=3)
        (self.stem,) = axes.plot([], [], color=POSE_EDGE_COLOR, linewidth=0.8, zorder=4)
        (self.foot,) = axes.plot([], [], marker="o", markersize=3, color=POSE_EDGE_COLOR, linestyle="none", zorder=4)
        (self.up_axis,) = axes.plot([], [], color=POSE_EDGE_COLOR, linewidth=1.4, solid_capstyle="round", zorder=5)
        self.heading = axes.annotate(
            "", xy=(0, 0), xytext=(0, 0), arrowprops={"arrowstyle": "-|>", "color": POSE_COLOR, "linewidth": 2.0,
                                                      "mutation_scale": 12}, zorder=5, annotation_clip=False)
        (self.pose_marker,) = axes.plot([], [], marker="o", markersize=7, color=POSE_COLOR, markeredgecolor=POSE_EDGE_COLOR,
                                        markeredgewidth=0.8, linestyle="none", zorder=6, label="pose")

        self.view = args.view
        if self.view == "oblique":
            self.fixed_camera = self.oblique_camera(cropped)

    def oblique_camera(self, cropped):
        """A fixed camera above and behind the run that fits the map around it into the view."""
        low = np.array([*self.est_xyz[:, :2].min(axis=0) - self.args.margin, cropped[:, 2].min()])
        high = np.array([*self.est_xyz[:, :2].max(axis=0) + self.args.margin, np.percentile(cropped[:, 2], 99.5)])
        low[:2] = np.maximum(low[:2], cropped[:, :2].min(axis=0))
        high[:2] = np.minimum(high[:2], cropped[:, :2].max(axis=0))
        target = (low + high) / 2.0
        azimuth, elevation = np.radians(self.args.azimuth), np.radians(self.args.elevation)
        direction = np.array([np.cos(elevation) * np.cos(azimuth), np.cos(elevation) * np.sin(azimuth), np.sin(elevation)])
        eye = target + self.args.distance * float(np.linalg.norm(high - low)) * direction
        R = look_at(eye, target)
        corners = np.array([[x, y, z] for x in (low[0], high[0]) for y in (low[1], high[1]) for z in (low[2], high[2])])
        focal, center = fit_camera(R, eye, corners, self.width, self.height, margin=4)
        return Camera(R, eye, focal, center)

    def chase_camera(self, index):
        """A camera behind and above the sensor, along the direction of travel (smoothed over +-0.5 s)."""
        k = 5
        ahead = self.est_xyz[min(index + k, len(self.est_xyz) - 1)] - self.est_xyz[max(index - k, 0)]
        direction = unit_xy(ahead)
        position = self.est_xyz[index]
        eye = np.array([*(position[:2] - 6.0 * direction), position[2] + 4.0])
        target = np.array([*(position[:2] + 4.0 * direction), position[2] - 0.5])
        focal = (self.width / 2.0) / np.tan(np.radians(35.0))
        return Camera(look_at(eye, target), eye, focal, (self.width / 2.0, self.height / 2.0))

    def point_sizes(self, depth, reference):
        """Marker areas (pt^2): about 1.2 layout px across at the reference depth, larger when near."""
        diameter_px = np.clip(1.2 * reference / np.maximum(depth, 1e-3), 0.8, 2.6)
        return (diameter_px * 72.0 / 100.0) ** 2

    def line(self, camera, points):
        uv, _ = camera.project(points)
        return uv[:, 0], uv[:, 1]

    def update(self, index, T_world_imu, scan):
        camera = self.fixed_camera if self.view == "oblique" else self.chase_camera(index)
        map_uv, map_depth = camera.project(self.map_points)
        scan = scan[np.all((scan >= self.box[0]) & (scan <= self.box[1]), axis=1)]
        scan_uv, scan_depth = camera.project(scan)
        uv = np.concatenate([map_uv, scan_uv])
        depth = np.concatenate([map_depth, scan_depth - self.SCAN_DEPTH_BIAS])
        rgba = np.concatenate([self.map_rgba, np.tile(self.scan_rgba, (len(scan), 1))])
        diameter = np.concatenate([np.where(self.map_ground, GROUND_SIZE, 1.0), np.ones(len(scan))])
        visible = np.isfinite(uv).all(axis=1)
        order = painter_order(depth[visible])
        reference = float(np.median(map_depth[np.isfinite(map_uv).all(axis=1)])) if np.any(np.isfinite(map_uv)) else 1.0
        self.cloud.set_offsets(uv[visible][order])
        self.cloud.set_facecolors(rgba[visible][order])
        self.cloud.set_sizes(self.point_sizes(depth[visible][order], reference) * diameter[visible][order] ** 2)

        if self.gt_line is not None:
            self.gt_line.set_data(*self.line(camera, self.gt_xyz))
        self.est_line.set_data(*self.line(camera, self.est_xyz[: index + 1]))
        past = self.drop_indices[self.drop_indices <= index]
        tops = self.est_xyz[past]
        bottoms = np.column_stack([tops[:, :2], self.support[past]])
        gaps = np.full_like(tops, np.nan)
        self.drops.set_data(*self.line(camera, np.stack([tops, bottoms, gaps], axis=1).reshape(-1, 3)))
        origin = T_world_imu[:3, 3]
        ground = np.array([origin[0], origin[1], self.support[index] if index < len(self.support) else self.support[-1]])
        self.stem.set_data(*self.line(camera, np.stack([origin, ground])))
        self.foot.set_data(*self.line(camera, ground[None]))
        self.up_axis.set_data(*self.line(camera, np.stack([origin, origin + 0.8 * T_world_imu[:3, 2]])))
        (tip_u, tip_v), (base_u, base_v) = camera.project(np.stack([origin + 2.2 * T_world_imu[:3, 0], origin]))[0]
        self.heading.xy = (tip_u, tip_v)
        self.heading.set_position((base_u, base_v))
        self.heading.set_visible(bool(np.isfinite([tip_u, tip_v, base_u, base_v]).all()))
        self.pose_marker.set_data([base_u], [base_v])


def render(args):
    if os.path.splitext(args.out)[1].lower() not in (".gif", ".webp"):
        raise InputError(f"the output must end in .gif or .webp: {args.out}")
    rng = np.random.default_rng(0)
    frames = collect_frames(args)
    T_imu_lidar = np.linalg.inv(pose_matrix(args.t_lidar_imu))
    intro = read_init_search(args.run) if args.init_search else None

    gt_path = None if args.no_gt else (args.gt or os.path.join(args.data, "traj_gt.txt"))
    gt_xyz = None
    if gt_path and (args.gt or os.path.isfile(gt_path)):
        _, gt_poses = read_tum(gt_path)
        gt_xyz = gt_poses[:, :3, 3]

    map_points = z_filter(read_pcd_xyz(args.map), args.z_min, args.z_max)
    if len(map_points) == 0:
        raise InputError(f"no map points left after the height filter: {args.map}")
    est_xyz = np.array([f[1][:3, 3] for f in frames])

    # Layout in logical pixels (at --scale 1), below a fixed header band for the text. Font sizes are
    # in points: 1 pt = 100 / 72 logical px.
    matplotlib.rcParams["font.family"] = select_font(args.font)
    header_px, pad_px = 64, 8
    width_px = args.width
    plot_w = width_px - 2 * pad_px
    if args.view == "top":
        # The view: the trajectory with a margin, clipped to the map; the height follows its aspect
        low = est_xyz[:, :2].min(axis=0) - args.margin
        high = est_xyz[:, :2].max(axis=0) + args.margin
        low = np.maximum(low, map_points[:, :2].min(axis=0) - 1.0)
        high = np.minimum(high, map_points[:, :2].max(axis=0) + 1.0)
        plot_h = int(np.clip(plot_w * (high[1] - low[1]) / (high[0] - low[0]), 120, 2 * width_px))
    else:
        plot_h = int(round(plot_w * args.aspect))
    height_px = header_px + plot_h + pad_px
    height_px += height_px % 2
    figure = plt.figure(figsize=(width_px / 100, height_px / 100), dpi=100 * args.scale)
    rect = [pad_px / width_px, pad_px / height_px, plot_w / width_px, plot_h / height_px]
    if args.view == "top":
        view = TopView(figure, rect, low, high, map_points, gt_xyz, est_xyz, args, rng)
    else:
        view = PerspectiveView(figure, rect, (plot_w, plot_h), map_points, gt_xyz, est_xyz, args, rng)

    # Proxy handles: the scan dots are tiny on the plot but need a visible legend marker
    handles = [view.pose_marker, view.est_line] + ([view.gt_line] if view.gt_line is not None else []) + [
        Line2D([], [], marker="o", markersize=4, color=SCAN_COLOR, linestyle="none", label="scan")]
    if isinstance(view, PerspectiveView):
        handles.append(Line2D([], [], marker="s", markersize=5, color=GROUND_COLOR, linestyle="none", label="floor"))
    legend = view.axes.legend(handles=handles, loc="lower right", fontsize=8.6, framealpha=1.0, fancybox=False)
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
    if isinstance(view, PerspectiveView):
        # Height key on the title line: one swatch per level, from the ground cut to z_high (the floor
        # below the cut is gray, in the legend)
        z_low, z_high = view.z_range
        bar_right, bar_w, bar_h = width_px - pad_px - 30, 96, 8
        text(-(pad_px + 30 + bar_w + 4), 18, f"map height  {z_low:g} m", ha="right", fontsize=9.4)
        text(-pad_px, 18, f"{z_high:g} m", ha="right", fontsize=9.4)
        step = bar_w / HEIGHT_LEVELS
        for k, color in enumerate(HEIGHT_COLORS):
            x0 = bar_right - bar_w + k * step
            figure.patches.append(Rectangle((x0 / width_px, 1.0 - (18 + bar_h / 2) / height_px), step / width_px, bar_h / height_px,
                                            transform=figure.transFigure, facecolor=color, edgecolor="none", antialiased=False))

    def capture():
        figure.canvas.draw()
        rgba = np.asarray(figure.canvas.buffer_rgba())
        return Image.fromarray(rgba[:, :, :3].copy())

    def scan_in_map(T_world_imu, scan_path):
        scan = transform(T_world_imu @ T_imu_lidar, read_scan(scan_path))
        return subsample(z_filter(scan, args.z_min, args.z_max), args.max_scan_points, rng)

    images = []
    t0 = frames[0][0]
    if intro is not None:
        # The given pose with the first scan placed by it, then the pose the yaw search picked
        T_given, search = intro
        first_scan = frames[0][4]
        given = (T_given, f"INITIAL GUESS: yaw off by {-search['yaw_offset_deg']:+.0f} deg", scan_in_map(T_given, first_scan), "-")
        found = (frames[0][1], f"YAW SEARCH: {search['yaw_offset_deg']:+.0f} deg, {search['candidates']:.0f} candidates",
                 scan_in_map(frames[0][1], first_scan), f"{search['score']:.2f}")
        time_text.set_text(f"{0.0:.1f} s")
        for T_world_imu, label, scan, score in (given, found):
            view.update(0, T_world_imu, scan)
            state_text.set_text(label)
            state_text.set_color(INIT_GUESS_COLOR)
            inlier_text.set_text(score)
            image = capture()
            images.extend([image] * args.intro_frames)
    for i, (stamp, T_world_imu, state, inlier, scan_path) in enumerate(frames):
        view.update(i, T_world_imu, scan_in_map(T_world_imu, scan_path))
        state_text.set_text(state)
        state_text.set_color(STATE_COLORS.get(state, "#424242"))
        time_text.set_text(f"{stamp - t0:.1f} s")
        inlier_text.set_text(f"{inlier:.2f}" if np.isfinite(inlier) else "-")
        images.append(capture())
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
    parser.add_argument("--view", choices=VIEWS, default="oblique", help="oblique (default), chase or top; see above")
    parser.add_argument("--gt", help="ground truth (TUM); default <data>/traj_gt.txt when it exists")
    parser.add_argument("--no-gt", action="store_true", help="do not draw the ground truth")
    parser.add_argument("--t-lidar-imu", type=float, nargs=7, default=[0, 0, 0, 0, 0, 0, 1], metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
                        help="T_lidar_imu of config_sensors.json (default identity)")
    parser.add_argument("--z-min", type=float, help="draw only map and scan points at or above this map-frame height (for example, to hide the floor)")
    parser.add_argument("--z-max", type=float, help="draw only map and scan points below this map-frame height (with --z-min, a horizontal slice)")
    parser.add_argument("--margin", type=float, help="map margin around the trajectory in meters (default 8 for top, 6 for the 3D views)")
    parser.add_argument("--azimuth", type=float, default=-110.0, help="oblique view: direction from the run to the camera, degrees from +x toward +y (default -110)")
    parser.add_argument("--elevation", type=float, default=42.0, help="oblique view: camera elevation above the horizon in degrees (default 42)")
    parser.add_argument("--distance", type=float, default=0.9, help="oblique view: camera distance as a multiple of the map box diagonal (default 0.9)")
    parser.add_argument("--aspect", type=float, default=0.5, help="3D views: plot height / width (default 0.5)")
    parser.add_argument("--init-search", action="store_true",
                        help="start with the given initial pose and the pose the yaw search picked (needs init_search in <run>/summary.json)")
    parser.add_argument("--intro-frames", type=int, default=15, help="with --init-search, animation frames for each of the two intro images (default 15)")
    parser.add_argument("--width", type=int, default=720, help="layout width in pixels; the height follows the view (default 720)")
    parser.add_argument("--scale", type=float, default=2.0, help="output pixels per layout pixel, for HiDPI screens (default 2: 1440 px wide)")
    parser.add_argument("--colors", type=int, default=256, help="GIF palette size, including the fixed drawing colors (default 256, the maximum)")
    parser.add_argument("--fps", type=float, default=10.0, help="animation frame rate (default 10)")
    parser.add_argument("--stride", type=int, default=1, help="draw every n-th estimated frame (default 1)")
    parser.add_argument("--max-frames", type=int, default=0, help="stop after this many animation frames (default 0: all)")
    parser.add_argument("--max-map-points", type=int, default=60000, help="random subset of the map points drawn (default 60000, 0: all)")
    parser.add_argument("--max-scan-points", type=int, help="random subset of each scan drawn (default 4000 for top, 1000 for the 3D views; 0: all)")
    parser.add_argument("--font", default="Noto Sans", help="font family for all text (default Noto Sans, in the image; DejaVu Sans if missing)")
    parser.add_argument("--title", default="glim_localization: replay on a prebuilt map", help="title line")
    args = parser.parse_args(argv)
    if args.stride < 1 or args.fps <= 0 or args.width < 160 or args.scale <= 0 or not 24 <= args.colors <= 256:
        parser.error("--stride must be >= 1, --fps > 0, --width >= 160, --scale > 0 and --colors in [24, 256]")
    if args.z_min is not None and args.z_max is not None and args.z_min >= args.z_max:
        parser.error("--z-min must be below --z-max")
    if not (0.1 <= args.aspect <= 2.0 and 5.0 <= args.elevation <= 85.0 and args.distance > 0 and args.intro_frames >= 1):
        parser.error("--aspect must be in [0.1, 2], --elevation in [5, 85], --distance > 0 and --intro-frames >= 1")
    if args.margin is None:
        args.margin = 8.0 if args.view == "top" else 6.0
    if args.max_scan_points is None:
        args.max_scan_points = 4000 if args.view == "top" else 1000
    if args.margin < 0:
        parser.error("--margin must be >= 0")
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
