#!/usr/bin/env python3
"""Convert a glim_localization replay dataset into a ROS 2 bag (the inverse of bag_to_replay.py, for tests).

Writes /points (sensor_msgs/PointCloud2, Ouster layout: x y z intensity t, with t as UINT32 nanoseconds
relative to the frame stamp; the intensity is not kept, it is the constant 7.0) and /imu (sensor_msgs/Imu), and optionally /initialpose messages taken from the
ground truth (traj_gt.txt) so that glim_rosbag can start the localization without a live publisher.

--initial-pose T[:dx,dy,dz,yaw_deg]  may be repeated. The pose is the ground truth of the first frame at or
    after T seconds from the first frame, optionally offset (translation in the map frame, then a yaw about z).
    The message is stamped 1 ms before that frame (not before 0) and recorded at the same time.

--drop-imu START:DURATION  may be repeated. Leaves out the IMU samples stamped in [START, START + DURATION) seconds
    from the first frame, to make a bag with an IMU gap (the point clouds are kept).

--wheel-odom  also writes /wheel_odom (nav_msgs/Odometry, child_frame_id base_link) at the ground-truth stamps, with
    the twist of the IMU frame taken from traj_gt.txt by central differences (one-sided at the ends). It is the
    input of a wheel odometry config with the identity wheel_odom_T_imu_base. The pose is the ground truth.

The bag writer is write_bag from scripts/tests/test_bag_to_replay.py, which the bag_to_replay tests use.
Only synthetic datasets are meant to go through this tool.

Exit codes: 0 ok, 1 conversion failed, 2 bad arguments.
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "tests"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bag_to_replay  # noqa: E402
import test_bag_to_replay as bag_tools  # noqa: E402  (write_bag, make_cloud, make_imu)

from rosbags.rosbag2 import StoragePlugin  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

POSE_TYPE = "geometry_msgs/msg/PoseWithCovarianceStamped"
ODOM_TYPE = "nav_msgs/msg/Odometry"
WHEEL_TOPIC = "/wheel_odom"
WHEEL_CHILD_FRAME = "base_link"
STAMP_LEAD_NS = 1_000_000  # an initial pose is stamped this long before its frame


class ConversionError(Exception):
    pass


def read_clouds(dataset):
    clouds = []
    files = sorted(os.listdir(os.path.join(dataset, "points")))
    if not files:
        raise ConversionError(f"{dataset}/points has no frames")
    for name in files:
        raw = open(os.path.join(dataset, "points", name), "rb").read()
        magic, version, stamp_ns, n, flags, step, _ = bag_to_replay.BIN_HEADER.unpack(raw[: bag_to_replay.BIN_HEADER.size])
        if magic != b"GLRP" or version != 1 or step != 20:
            raise ConversionError(f"{name}: not a version 1 replay cloud")
        pts = np.frombuffer(raw[bag_to_replay.BIN_HEADER.size :], dtype="<f4").reshape(n, 5)
        t_ns = np.clip(np.rint(pts[:, 3].astype(np.float64) * 1e9), 0, 2**32 - 1).astype("<u4")
        msg = bag_tools.make_cloud(stamp_ns, t_ns, "t", bag_tools.Field.UINT32, xyz=np.ascontiguousarray(pts[:, :3]))
        clouds.append((stamp_ns, msg))
    return clouds


def read_imu(dataset):
    imus = []
    with open(os.path.join(dataset, "imu.csv")) as f:
        for row in csv.DictReader(f):
            ns = int(row["stamp_ns"])
            acc = [float(row[k]) for k in ("acc_x", "acc_y", "acc_z")]
            gyro = [float(row[k]) for k in ("gyro_x", "gyro_y", "gyro_z")]
            imus.append((ns, bag_tools.make_imu(ns, acc=acc, gyro=gyro)))
    if not imus:
        raise ConversionError(f"{dataset}/imu.csv has no samples")
    return imus


def read_gt(dataset):
    path = os.path.join(dataset, "traj_gt.txt")
    if not os.path.exists(path):
        raise ConversionError(f"{path} is required for --initial-pose and --wheel-odom")
    rows = [line.split() for line in open(path) if line.strip() and not line.startswith("#")]
    return np.array([[float(v) for v in r[:8]] for r in rows])


def parse_initial_pose(text):
    head, _, tail = text.partition(":")
    try:
        offset = [float(v) for v in tail.split(",")] if tail else [0.0, 0.0, 0.0, 0.0]
        at = float(head)
    except ValueError as e:
        raise argparse.ArgumentTypeError(f"bad --initial-pose '{text}': {e}")
    if len(offset) != 4 or not all(np.isfinite(offset)) or not np.isfinite(at) or at < 0:
        raise argparse.ArgumentTypeError(f"bad --initial-pose '{text}': expected T[:dx,dy,dz,yaw_deg] with T >= 0")
    return at, offset


def parse_drop_imu(text):
    head, sep, tail = text.partition(":")
    try:
        start, duration = float(head), float(tail)
    except ValueError as e:
        raise argparse.ArgumentTypeError(f"bad --drop-imu '{text}': {e}")
    if not sep or not np.isfinite(start) or not np.isfinite(duration) or start < 0 or duration <= 0:
        raise argparse.ArgumentTypeError(f"bad --drop-imu '{text}': expected START:DURATION with START >= 0 and DURATION > 0")
    return start, duration


def drop_imu(imus, specs, first_frame_ns):
    """Return the IMU samples outside every [START, START + DURATION) window (seconds from the first frame)."""
    windows = [(first_frame_ns + int(round(s * 1e9)), first_frame_ns + int(round((s + d) * 1e9))) for s, d in specs]
    return [(ns, msg) for ns, msg in imus if not any(lo <= ns < hi for lo, hi in windows)]


def make_initial_poses(specs, clouds, gt):
    """Return (topic, type, record ns, message) tuples for the /initialpose messages."""
    Msg = bag_tools.TS.types[POSE_TYPE]
    Pose = bag_tools.TS.types["geometry_msgs/msg/PoseWithCovariance"]
    PoseBody = bag_tools.TS.types["geometry_msgs/msg/Pose"]
    Point = bag_tools.TS.types["geometry_msgs/msg/Point"]
    stamps = [ns for ns, _ in clouds]
    out = []
    for at, (dx, dy, dz, yaw_deg) in specs:
        frame_ns = next((ns for ns in stamps if ns >= stamps[0] + int(round(at * 1e9))), None)
        if frame_ns is None:
            raise ConversionError(f"--initial-pose {at}: no frame at or after {at} s")
        k = int(np.argmin(np.abs(gt[:, 0] - frame_ns * 1e-9)))
        if abs(gt[k, 0] - frame_ns * 1e-9) > 1e-3:
            raise ConversionError(f"--initial-pose {at}: no ground truth within 1 ms of the frame at {frame_ns} ns")
        p = gt[k, 1:4] + np.array([dx, dy, dz])
        q = (Rotation.from_euler("z", yaw_deg, degrees=True) * Rotation.from_quat(gt[k, 4:8])).as_quat()
        stamp_ns = max(frame_ns - STAMP_LEAD_NS, 0)  # a bag cannot start before 0; a 0 stamp means "the next frame"
        msg = Msg(
            header=bag_tools.Header(stamp=bag_tools.make_time(stamp_ns), frame_id="map"),
            pose=Pose(
                pose=PoseBody(
                    position=Point(x=float(p[0]), y=float(p[1]), z=float(p[2])),
                    orientation=bag_tools.Quat(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])),
                ),
                covariance=np.zeros(36),
            ),
        )
        out.append(("/initialpose", POSE_TYPE, stamp_ns, msg))
    return out


def gt_twists(gt):
    """Return (linear, angular) arrays, both in the body frame of each ground-truth row [m/s, rad/s]."""
    n = len(gt)
    if n < 2:
        raise ConversionError("--wheel-odom needs at least two ground-truth poses")
    stamps = gt[:, 0]
    if np.any(np.diff(stamps) <= 0):
        raise ConversionError("--wheel-odom needs strictly increasing ground-truth stamps")
    rotations = Rotation.from_quat(gt[:, 4:8])
    linear = np.zeros((n, 3))
    angular = np.zeros((n, 3))
    for i in range(n):
        lo, hi = max(i - 1, 0), min(i + 1, n - 1)
        dt = stamps[hi] - stamps[lo]
        linear[i] = rotations[i].inv().apply((gt[hi, 1:4] - gt[lo, 1:4]) / dt)
        angular[i] = (rotations[lo].inv() * rotations[hi]).as_rotvec() / dt
    return linear, angular


def make_wheel_odom(gt):
    """Return (topic, type, record ns, message) tuples for /wheel_odom, one per ground-truth pose."""
    Msg = bag_tools.TS.types[ODOM_TYPE]
    PoseCov = bag_tools.TS.types["geometry_msgs/msg/PoseWithCovariance"]
    PoseBody = bag_tools.TS.types["geometry_msgs/msg/Pose"]
    Point = bag_tools.TS.types["geometry_msgs/msg/Point"]
    TwistCov = bag_tools.TS.types["geometry_msgs/msg/TwistWithCovariance"]
    Twist = bag_tools.TS.types["geometry_msgs/msg/Twist"]
    Vector3 = bag_tools.TS.types["geometry_msgs/msg/Vector3"]
    linear, angular = gt_twists(gt)
    out = []
    for row, v, w in zip(gt, linear, angular):
        stamp_ns = int(round(row[0] * 1e9))
        msg = Msg(
            header=bag_tools.Header(stamp=bag_tools.make_time(stamp_ns), frame_id="odom"),
            child_frame_id=WHEEL_CHILD_FRAME,
            pose=PoseCov(
                pose=PoseBody(
                    position=Point(x=float(row[1]), y=float(row[2]), z=float(row[3])),
                    orientation=bag_tools.Quat(x=float(row[4]), y=float(row[5]), z=float(row[6]), w=float(row[7])),
                ),
                covariance=np.zeros(36),
            ),
            twist=TwistCov(
                twist=Twist(
                    linear=Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2])),
                    angular=Vector3(x=float(w[0]), y=float(w[1]), z=float(w[2])),
                ),
                covariance=np.zeros(36),
            ),
        )
        out.append((WHEEL_TOPIC, ODOM_TYPE, stamp_ns, msg))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("dataset", help="replay dataset directory")
    parser.add_argument("--out", required=True, help="output bag directory (must not exist)")
    parser.add_argument("--storage", choices=["sqlite3", "mcap"], default="sqlite3")
    parser.add_argument("--initial-pose", action="append", default=[], type=parse_initial_pose, metavar="T[:dx,dy,dz,yaw_deg]")
    parser.add_argument("--drop-imu", action="append", default=[], type=parse_drop_imu, metavar="START:DURATION")
    parser.add_argument("--wheel-odom", action="store_true", help=f"also write {WHEEL_TOPIC} from the ground truth")
    args = parser.parse_args(argv)

    if os.path.exists(args.out):
        print(f"error: {args.out} already exists", file=sys.stderr)
        return 2
    try:
        meta = json.load(open(os.path.join(args.dataset, "meta.json")))
        if meta.get("format") != bag_to_replay.FORMAT_NAME:
            raise ConversionError(f"{args.dataset}/meta.json is not a {bag_to_replay.FORMAT_NAME} dataset")
        clouds = read_clouds(args.dataset)
        imus = read_imu(args.dataset)
        num_imus = len(imus)
        imus = drop_imu(imus, args.drop_imu, clouds[0][0])
        if args.drop_imu and len(imus) == num_imus:
            raise ConversionError("--drop-imu removed no IMU sample")
        gt = read_gt(args.dataset) if args.initial_pose or args.wheel_odom else None
        extra = make_initial_poses(args.initial_pose, clouds, gt) if args.initial_pose else []
        wheel = make_wheel_odom(gt) if args.wheel_odom else []
    except (ConversionError, OSError, ValueError, KeyError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    plugin = StoragePlugin.SQLITE3 if args.storage == "sqlite3" else StoragePlugin.MCAP
    bag_tools.write_bag(args.out, clouds, imus, plugin, extra + wheel)
    print(
        f"wrote {len(clouds)} frames, {len(imus)} IMU samples ({num_imus - len(imus)} dropped), {len(extra)} initial poses"
        f" and {len(wheel)} wheel odometry messages to {args.out}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
