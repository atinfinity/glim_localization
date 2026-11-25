#!/usr/bin/env python3
"""Write a geometry_msgs/msg/PoseStamped topic of a rosbag2 as a TUM trajectory.

Runs in the simulation image (ROS 2 Jazzy, rosbag2_py). The stamp is the message header stamp
(simulation time), and the pose is written as is: for /sim_eval/imu_pose it is T_world_imu, the
ground truth that scripts/bag_to_replay.py --gt copies into the replay dataset.

With --stats, also writes a JSON file with the simulated span, the receive-time span of the same
messages and their ratio, the real-time factor of the recording (plus any --stats-extra key=value).

Exit codes: 0 ok, 1 no message or a bad stamp, 2 bad arguments.
"""
import argparse
import json
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

POSE_STAMPED = "geometry_msgs/msg/PoseStamped"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag", help="rosbag2 directory")
    parser.add_argument("--topic", required=True, help="geometry_msgs/msg/PoseStamped topic")
    parser.add_argument("--out", required=True, help="output TUM file")
    parser.add_argument("--stats", help="also write the real-time factor of the recording to this JSON file")
    parser.add_argument("--stats-extra", nargs="*", default=[], help="key=value pairs (numbers) added to --stats")
    args = parser.parse_args(argv)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if types.get(args.topic) != POSE_STAMPED:
        print(f"pose_to_tum: {args.topic} is not a {POSE_STAMPED} topic in {args.bag}", file=sys.stderr)
        return 2
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
    msg_type = get_message(POSE_STAMPED)

    rows = {}
    receive_ns = []
    while reader.has_next():
        _, data, received = reader.read_next()
        receive_ns.append(received)
        msg = deserialize_message(data, msg_type)
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        p, q = msg.pose.position, msg.pose.orientation
        rows[stamp_ns] = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)  # a repeated stamp keeps the last pose
    if not rows:
        print(f"pose_to_tum: no message on {args.topic}", file=sys.stderr)
        return 1

    with open(args.out, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw (T_world_imu, simulation time)\n")
        for stamp_ns in sorted(rows):
            values = " ".join(f"{v:.9f}" for v in rows[stamp_ns])
            f.write(f"{stamp_ns // 1_000_000_000}.{stamp_ns % 1_000_000_000:09d} {values}\n")
    print(f"pose_to_tum: {len(rows)} poses from {args.topic} to {args.out}")

    if args.stats:
        sim_span = (max(rows) - min(rows)) * 1e-9
        wall_span = (max(receive_ns) - min(receive_ns)) * 1e-9
        stats = {
            "poses": len(rows),
            "sim_span_s": sim_span,
            "receive_span_s": wall_span,
            "real_time_factor": sim_span / wall_span if wall_span > 0 else None,
        }
        for item in args.stats_extra:
            key, _, value = item.partition("=")
            stats[key] = float(value)
        with open(args.stats, "w") as f:
            json.dump(stats, f, indent=2)
        print(f"pose_to_tum: {sim_span:.1f} s simulated in {wall_span:.1f} s, real-time factor {stats['real_time_factor']:.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
