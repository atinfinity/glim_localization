#!/usr/bin/env python3
# Derived from koide3/glim include/glim/util/ros_cloud_converter.hpp and src/glim/util/time_keeper.cpp (per-point time rules) at commit 1a197c999e4865f5cab7c9cd25474128413ebbca (MIT License, Copyright (c) 2024 koide3)
"""Convert a ROS 2 bag (sqlite3 or mcap) into a glim_localization replay dataset.

The dataset format is described in docs/DESIGN.md. The conversion normalizes the frame
stamp (the acquisition time of the first point) and the per-point times (seconds since
the frame stamp, in [0, 1)) once, so that the replay needs no heuristics.

Per-point time field (decided from the first frame, then fixed):
  field names  t, time, time_stamp, timestamp
  UINT32       nanoseconds
  FLOAT32/64   as is
  other types  error
  max < 1.0    relative to the frame stamp; negative values shift the frame stamp
  max >= 1.0   absolute; the frame stamp becomes the first point time
  min > 1e16   nanoseconds (for example a Livox PointCloud2 with FLOAT64 nanoseconds)
Without a time field the frames have no per-point time and all t are 0. Livox CustomMsg
is not supported.

Exit codes: 0 ok, 1 conversion failed, 2 bad arguments.
"""
import argparse
import json
import math
import os
import shutil
import struct
import sys

import numpy as np

FORMAT_NAME = "glim_localization_replay"
IMU_HEADER = "stamp_ns,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z"
TIME_FIELD_NAMES = ("t", "time", "time_stamp", "timestamp")
BIN_HEADER = struct.Struct("<4sIqIIII")  # magic, version, stamp_ns, num_points, flags, point_step, reserved
FLAG_POINT_TIME = 1 << 0
FLAG_INTENSITY = 1 << 1

# sensor_msgs/PointField datatypes
POINT_FIELD_DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
UINT32, FLOAT32, FLOAT64 = 6, 7, 8
POINTCLOUD2 = "sensor_msgs/msg/PointCloud2"
IMU = "sensor_msgs/msg/Imu"


# In absolute mode, the first point time must be near the header stamp. Drivers differ by a fraction of a scan.
ABSOLUTE_STAMP_TOLERANCE_NS = 1_000_000_000


class ConversionError(Exception):
    pass


class TimeSettings:
    """How to read the per-point time of one bag. Decided from the first frame."""

    def __init__(self, field, datatype, mode, scale):
        self.field = field
        self.datatype = datatype
        self.mode = mode  # "relative" or "absolute"
        self.scale = scale

    def describe(self):
        return {"field": self.field, "datatype": POINT_FIELD_DTYPES[self.datatype], "mode": self.mode, "scale": self.scale}


def stamp_to_ns(stamp):
    ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    if ns < 0:
        # The replay format stores stamps as unsigned file names and an unset header (0) is not a time base
        raise ConversionError(f"negative message stamp {ns * 1e-9:.9f} s; the bag clock is not usable")
    return ns


def find_time_field(fields, override):
    names = (override,) if override else TIME_FIELD_NAMES
    found = None
    for field in fields:  # As upstream, the last matching field wins
        if field.name in names:
            found = field
    if found is None and override:
        raise ConversionError(f"the point cloud has no field named '{override}'")
    return found


def decode_cloud(msg, time_field_override, intensity_field):
    """Return xyz (N, 3), raw time values or None, intensity or None, and the time field."""
    num_points = int(msg.width) * int(msg.height)
    fields = {f.name: f for f in msg.fields}
    for axis in ("x", "y", "z"):
        if axis not in fields:
            raise ConversionError(f"the point cloud has no '{axis}' field")
    if not (fields["x"].datatype == fields["y"].datatype == fields["z"].datatype) or fields["x"].datatype not in (FLOAT32, FLOAT64):
        raise ConversionError("x, y and z must all be FLOAT32 or all be FLOAT64")

    time_field = find_time_field(msg.fields, time_field_override)
    if time_field is not None and time_field.datatype not in (UINT32, FLOAT32, FLOAT64):
        raise ConversionError(
            f"unsupported datatype {time_field.datatype} of the time field '{time_field.name}' (supported: UINT32 as ns, FLOAT32, FLOAT64)"
        )
    intensity = fields.get(intensity_field) if intensity_field else None
    if intensity is not None and intensity.datatype not in POINT_FIELD_DTYPES:
        raise ConversionError(f"unsupported datatype {intensity.datatype} of the intensity field '{intensity_field}'")

    prefix = ">" if msg.is_bigendian else "<"
    wanted = {"x": fields["x"], "y": fields["y"], "z": fields["z"]}
    if time_field is not None:
        wanted["time"] = time_field
    if intensity is not None:
        wanted["intensity"] = intensity
    dtype = np.dtype(
        {
            "names": list(wanted),
            "formats": [prefix + POINT_FIELD_DTYPES[f.datatype] for f in wanted.values()],
            "offsets": [int(f.offset) for f in wanted.values()],
            "itemsize": int(msg.point_step),
        }
    )
    data = np.asarray(msg.data, dtype=np.uint8).tobytes()
    if len(data) < num_points * int(msg.point_step):
        raise ConversionError("the point cloud data is shorter than width * height * point_step")
    records = np.frombuffer(data, dtype=dtype, count=num_points)

    # The dataset stores float32 coordinates and intensity; narrow here so that overflow to inf is caught by the finite check
    with np.errstate(over="ignore"):
        xyz = np.stack([records["x"], records["y"], records["z"]], axis=1).astype(np.float32)
        values = records["intensity"].astype(np.float32) if intensity is not None else None
    time_raw = records["time"].astype(np.float64) if time_field is not None else None
    return xyz, time_raw, values, time_field


def detect_time_settings(time_field, time_values, mode_override, scale_override):
    """Same rules as GLIM's TimeKeeper autoconf, applied once to the first frame.

    time_values are in the field's units (UINT32 already converted from ns to seconds).
    """
    base_scale = 1e-9 if time_field.datatype == UINT32 else 1.0
    values = time_values * base_scale
    low, high = float(np.min(values)), float(np.max(values))

    mode = mode_override
    scale = base_scale
    if mode is None:
        if high < 1.0:
            mode = "relative"
        else:
            mode = "absolute"
            if low > 1e16 and time_field.datatype != UINT32:
                scale = 1e-9
    elif mode == "absolute" and low > 1e16 and time_field.datatype != UINT32:
        scale = 1e-9
    if scale_override is not None:
        scale = scale_override
    return TimeSettings(time_field.name, time_field.datatype, mode, scale)


def normalize_times(stamp_ns, time_raw, settings, frame_index):
    """Return (stamp_ns, t) with t in seconds since the frame stamp and min(t) == 0."""
    values = time_raw * settings.scale
    low, high = float(np.min(values)), float(np.max(values))
    hint = "decided from the first frame; override with --time-mode/--time-scale"
    if settings.mode == "relative":
        if high >= 1.0:
            raise ConversionError(
                f"frame {frame_index}: the largest per-point time is {high:.6f} s, which contradicts the relative interpretation "
                f"(scale {settings.scale}) {hint}"
            )
        # Shifting by the minimum keeps every absolute point time unchanged.
        stamp_ns += int(round(low * 1e9))
    else:
        first_point_ns = int(round(low * 1e9))
        if abs(first_point_ns - stamp_ns) > ABSOLUTE_STAMP_TOLERANCE_NS:
            if high < 1.0:
                raise ConversionError(
                    f"frame {frame_index}: the per-point times look relative (largest {high:.6f} s), which contradicts the absolute "
                    f"interpretation (scale {settings.scale}) {hint}"
                )
            raise ConversionError(
                f"frame {frame_index}: the first point time {first_point_ns * 1e-9:.6f} s is more than "
                f"{ABSOLUTE_STAMP_TOLERANCE_NS * 1e-9:g} s from the header stamp {stamp_ns * 1e-9:.6f} s. The per-point times and the "
                f"header use different clocks (an offset between the sensor clock and the host clock); --time-mode/--time-scale cannot fix "
                f"this, and the IMU stamps must share the clock of the points, so fix the clock offset in the bag first"
            )
        if first_point_ns < 0:
            raise ConversionError(f"frame {frame_index}: negative first point time {first_point_ns * 1e-9:.9f} s")
        stamp_ns = first_point_ns
    t = (values - low).astype(np.float32)  # The reader requires float32 t in [0, 1) after narrowing
    if float(np.max(t)) >= 1.0:
        raise ConversionError(
            f"frame {frame_index}: per-point times span {float(np.max(t)):.9f} s as float32, which is not below 1 s "
            f"(scale {settings.scale}); override with --time-mode/--time-scale"
        )
    return stamp_ns, t


def write_bin(path, stamp_ns, xyz, t, intensity):
    flags = (FLAG_POINT_TIME if t is not None else 0) | (FLAG_INTENSITY if intensity is not None else 0)
    n = xyz.shape[0]
    body = np.zeros((n, 5), dtype="<f4")
    body[:, 0:3] = xyz
    if t is not None:
        body[:, 3] = t
    if intensity is not None:
        body[:, 4] = intensity
    with open(path, "wb") as handle:
        handle.write(BIN_HEADER.pack(b"GLRP", 1, stamp_ns, n, flags, 20, 0))
        handle.write(body.tobytes())


class Converter:
    def __init__(self, args):
        self.args = args
        self.settings = None
        self.has_time_field = None
        self.frame_stamps = set()
        self.imu_rows = {}
        self.counts = {
            "frames_written": 0,
            "frames_empty": 0,
            "frames_duplicate_stamp": 0,
            "points_non_finite": 0,
            "imu_non_finite": 0,
            "imu_duplicate_stamp": 0,
        }
        self.frame_index = 0

    def add_frame(self, msg):
        index = self.frame_index
        self.frame_index += 1
        xyz, time_raw, intensity, time_field = decode_cloud(msg, self.args.time_field, self.args.intensity_field)

        if self.has_time_field is None:
            self.has_time_field = time_field is not None
        elif self.has_time_field != (time_field is not None):
            raise ConversionError(f"frame {index}: the time field appears or disappears between frames")

        keep = np.isfinite(xyz).all(axis=1)
        if time_raw is not None:
            keep &= np.isfinite(time_raw)
        if intensity is not None:
            keep &= np.isfinite(intensity)
        self.counts["points_non_finite"] += int(keep.size - keep.sum())
        if not keep.any():
            self.counts["frames_empty"] += 1
            return
        xyz = xyz[keep]
        time_raw = time_raw[keep] if time_raw is not None else None
        intensity = intensity[keep] if intensity is not None else None

        stamp_ns = stamp_to_ns(msg.header.stamp)
        t = None
        if time_raw is not None:
            if self.settings is None:
                self.settings = detect_time_settings(time_field, time_raw, self.args.time_mode, self.args.time_scale)
            elif time_field.name != self.settings.field or time_field.datatype != self.settings.datatype:
                raise ConversionError(f"frame {index}: the time field changed from '{self.settings.field}'")
            stamp_ns, t = normalize_times(stamp_ns, time_raw, self.settings, index)

        if stamp_ns in self.frame_stamps:
            self.counts["frames_duplicate_stamp"] += 1
            return
        self.frame_stamps.add(stamp_ns)
        write_bin(os.path.join(self.args.out, "points", f"{stamp_ns:019d}.bin"), stamp_ns, xyz, t, intensity)
        self.counts["frames_written"] += 1

    def add_imu(self, msg):
        stamp_ns = stamp_to_ns(msg.header.stamp)
        values = np.array(
            [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ],
            dtype=np.float64,
        )
        if not np.isfinite(values).all():
            self.counts["imu_non_finite"] += 1
            return
        if stamp_ns in self.imu_rows:
            self.counts["imu_duplicate_stamp"] += 1
            return
        self.imu_rows[stamp_ns] = values

    def write_imu(self):
        if not self.imu_rows:
            raise ConversionError("no usable IMU messages")
        stamps = sorted(self.imu_rows)
        acc = np.array([self.imu_rows[s][:3] for s in stamps])
        median_norm = float(np.median(np.linalg.norm(acc, axis=1)))
        if self.args.acc_scale is None:
            if median_norm < 5.0:
                raise ConversionError(
                    f"the median acceleration norm is {median_norm:.3f}, so the unit may be g rather than m/s^2; "
                    "pass --acc-scale 9.80665 (or --acc-scale 1 to keep the values)"
                )
            acc_scale = 1.0
        else:
            acc_scale = self.args.acc_scale
        with open(os.path.join(self.args.out, "imu.csv"), "w") as handle:
            handle.write(IMU_HEADER + "\n")
            for stamp, row in zip(stamps, (self.imu_rows[s] for s in stamps)):
                a = row[:3] * acc_scale
                handle.write(f"{stamp},{a[0]:.17g},{a[1]:.17g},{a[2]:.17g},{row[3]:.17g},{row[4]:.17g},{row[5]:.17g}\n")
        return acc_scale, len(stamps), median_norm


def select_topics(connections, args):
    clouds = [c for c in connections if c.msgtype == POINTCLOUD2]
    imus = [c for c in connections if c.msgtype == IMU]

    def choose(candidates, wanted, kind):
        if wanted:
            found = [c for c in connections if c.topic == wanted]
            if not found:
                raise ConversionError(f"the bag has no topic {wanted}")
            return found[0]
        if len(candidates) == 1:
            return candidates[0]
        listing = ", ".join(c.topic for c in candidates) or "none"
        raise ConversionError(f"cannot choose the {kind} topic automatically (candidates: {listing}); pass it explicitly")

    points = choose(clouds, args.points_topic, "point cloud")
    imu = choose(imus, args.imu_topic, "IMU")
    for connection, expected in ((points, POINTCLOUD2), (imu, IMU)):
        if connection.msgtype == "livox_ros_driver2/msg/CustomMsg" or connection.msgtype == "livox_ros_driver/msg/CustomMsg":
            raise ConversionError("Livox CustomMsg is not supported; use the PointCloud2 output of the driver")
        if connection.msgtype != expected:
            raise ConversionError(f"topic {connection.topic} has type {connection.msgtype}, expected {expected}")
    return points, imu


def convert(args):
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore

    out = args.out
    if os.path.exists(out) and os.listdir(out):
        raise ConversionError(f"output directory is not empty: {out}")
    if not os.path.isdir(args.bag):
        raise ConversionError(f"the bag is not a rosbag2 directory: {args.bag}")
    os.makedirs(os.path.join(out, "points"), exist_ok=True)

    typestore = get_typestore(Stores.ROS2_JAZZY)
    converter = Converter(args)
    try:
        with Reader(args.bag) as reader:
            points_connection, imu_connection = select_topics(reader.connections, args)
            for connection, _, raw in reader.messages(connections=[points_connection, imu_connection]):
                msg = typestore.deserialize_cdr(raw, connection.msgtype)
                if connection is points_connection:
                    converter.add_frame(msg)
                else:
                    converter.add_imu(msg)
    except ConversionError:
        raise
    except Exception as error:  # rosbags raises its own exception types for unreadable bags
        raise ConversionError(f"cannot read the bag {args.bag}: {error}") from error

    if converter.counts["frames_written"] == 0:
        raise ConversionError("no usable point cloud frames")
    acc_scale, num_imu, median_acc = converter.write_imu()

    if args.gt:
        if not os.path.isfile(args.gt):
            raise ConversionError(f"ground truth file not found: {args.gt}")
        shutil.copyfile(args.gt, os.path.join(out, "traj_gt.txt"))

    conversion = {
        "bag": os.path.basename(os.path.normpath(args.bag)),
        "points_topic": points_connection.topic,
        "imu_topic": imu_connection.topic,
        "time_field": converter.settings.describe() if converter.settings else None,
        "acc_scale": acc_scale,
        "imu_median_acc_norm_before_scale": median_acc,
        "excluded": converter.counts,
    }
    meta = {
        "format": FORMAT_NAME,
        "version": 1,
        "stamp_definition": "first_point",
        "per_point_time": bool(converter.has_time_field),
        "num_frames": converter.counts["frames_written"],
        "num_imu": num_imu,
        "conversion": conversion,
    }
    with open(os.path.join(out, "meta.json"), "w") as handle:
        json.dump(meta, handle, indent=2)
        handle.write("\n")
    return meta


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", help="rosbag2 directory (sqlite3 or mcap)")
    parser.add_argument("--out", required=True, help="output dataset directory (must be empty or absent)")
    parser.add_argument("--points-topic", help="sensor_msgs/PointCloud2 topic (default: the only one)")
    parser.add_argument("--imu-topic", help="sensor_msgs/Imu topic (default: the only one)")
    parser.add_argument("--acc-scale", type=float, help="multiply accelerations by this (9.80665 for g units). Required if they look like g")
    parser.add_argument("--time-field", help="per-point time field name (default: t, time, time_stamp, timestamp)")
    parser.add_argument("--time-mode", choices=("relative", "absolute"), help="override the detected per-point time mode")
    parser.add_argument("--time-scale", type=float, help="override the factor that converts field values to seconds")
    parser.add_argument("--intensity-field", default="intensity", help="intensity field name (default: intensity)")
    parser.add_argument("--gt", help="ground truth in TUM format (T_world_imu) to copy into the dataset as traj_gt.txt")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.acc_scale is not None and not (math.isfinite(args.acc_scale) and args.acc_scale > 0):
        sys.stderr.write("error: --acc-scale must be positive\n")
        return 2
    try:
        meta = convert(args)
    except ConversionError as error:
        sys.stderr.write(f"error: {error}\n")
        return 1
    conversion = meta["conversion"]
    print(f"wrote {meta['num_frames']} frames and {meta['num_imu']} IMU samples to {args.out}")
    print(f"per-point time: {conversion['time_field']}; excluded: {conversion['excluded']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
