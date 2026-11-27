#!/usr/bin/env python3
"""Record what the benchmark measures of one localization node (scripts/sim_benchmark.sh, docs/benchmark.md).

  bench_recorder.py --pid <node pid> --cloud-topic <topic> --pose-topic <topic> --out <dir>

Runs next to the node, in the same container (the same /proc), until SIGINT or SIGTERM, then writes:
- clouds.csv: the header stamp and the receive time of every cloud;
- poses.csv: the header stamp, the receive time and the pose of every pose message;
- traj.txt: the poses in TUM format (stamp x y z qx qy qz qw);
- resources.json: utime + stime of the node process at the first and the last cloud, VmHWM at the end,
  and VmRSS sampled at 1 Hz.
Receive times are CLOCK_MONOTONIC in ns. The cloud subscription is raw and only the stamp is parsed from
the CDR bytes, so the recorder never deserializes the points. Latency and the other numbers are computed
from these files by scripts/compare_methods.py. rclpy only.
"""
import argparse
import json
import os
import signal
import struct
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2


def cdr_stamp_ns(data):
    """The header stamp of a serialized message whose first field is a std_msgs/Header."""
    # 4-byte encapsulation header (byte 1: 0 big endian, 1 little endian), then int32 sec, uint32 nanosec.
    order = "<" if data[1] & 1 else ">"
    sec, nanosec = struct.unpack_from(order + "iI", data, 4)
    return sec * 1000000000 + nanosec


def cpu_ticks(pid):
    with open(f"/proc/{pid}/stat") as handle:
        fields = handle.read().rsplit(")", 1)[1].split()
    return int(fields[11]) + int(fields[12])  # utime, stime (fields 14 and 15 of proc(5))


def status_kb(pid, key):
    with open(f"/proc/{pid}/status") as handle:
        for line in handle:
            if line.startswith(key + ":"):
                return int(line.split()[1])
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pid", type=int, required=True, help="the node process")
    parser.add_argument("--cloud-topic", required=True)
    parser.add_argument("--pose-topic", required=True, help="geometry_msgs/PoseWithCovarianceStamped")
    parser.add_argument("--out", required=True)
    parser.add_argument("--ready-file", help="created once the subscriptions exist")
    args = parser.parse_args()

    stop = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.append(True))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("bench_recorder")

    clouds = []  # (stamp_ns, recv_ns)
    poses = []  # (stamp_ns, recv_ns, x, y, z, qx, qy, qz, qw)
    cpu = {}
    rss = []

    def on_cloud(data):
        now = time.monotonic_ns()
        clouds.append((cdr_stamp_ns(data), now))
        try:
            ticks = cpu_ticks(args.pid)
        except OSError:
            return
        cpu.setdefault("first", (now, ticks))
        cpu["last"] = (now, ticks)

    def on_pose(msg):
        now = time.monotonic_ns()
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        poses.append((msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec, now,
                      p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    def sample_rss():
        try:
            rss.append((time.monotonic_ns(), status_kb(args.pid, "VmRSS")))
        except OSError:
            pass

    # Reliable (the player publishes reliable, sim/config/bench_play_qos.yaml) and deep queues, so that the
    # recorder sees every cloud of the bag: a cloud that the node's own best effort subscription drops then
    # counts as a scan without a pose.
    cloud_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.RELIABLE)
    pose_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1000, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(PointCloud2, args.cloud_topic, on_cloud, cloud_qos, raw=True)
    node.create_subscription(PoseWithCovarianceStamped, args.pose_topic, on_pose, pose_qos)
    node.create_timer(1.0, sample_rss)
    sample_rss()
    if args.ready_file:
        open(args.ready_file, "w").close()

    while not stop:
        rclpy.spin_once(node, timeout_sec=0.1)

    os.makedirs(args.out, exist_ok=True)
    hwm = None
    try:
        hwm = status_kb(args.pid, "VmHWM")
    except OSError:
        pass
    with open(os.path.join(args.out, "clouds.csv"), "w") as handle:
        handle.write("stamp_ns,recv_ns\n")
        handle.writelines(f"{s},{r}\n" for s, r in clouds)
    with open(os.path.join(args.out, "poses.csv"), "w") as handle:
        handle.write("stamp_ns,recv_ns,x,y,z,qx,qy,qz,qw\n")
        handle.writelines(",".join([str(p[0]), str(p[1])] + [repr(v) for v in p[2:]]) + "\n" for p in poses)
    with open(os.path.join(args.out, "traj.txt"), "w") as handle:
        handle.writelines(f"{p[0] * 1e-9:.9f} " + " ".join(f"{v:.9f}" for v in p[2:]) + "\n" for p in poses)
    resources = {
        "pid": args.pid,
        "clk_tck": os.sysconf("SC_CLK_TCK"),
        "cpu_first": cpu.get("first"),  # [recv_ns, utime + stime ticks] at the first cloud
        "cpu_last": cpu.get("last"),
        "vm_hwm_kb": hwm,
        "rss_kb": rss,  # [monotonic ns, VmRSS kB] at 1 Hz
    }
    with open(os.path.join(args.out, "resources.json"), "w") as handle:
        json.dump(resources, handle)
    node.destroy_node()
    rclpy.shutdown()
    print(f"bench_recorder: {len(clouds)} clouds, {len(poses)} poses, VmHWM {hwm} kB")


if __name__ == "__main__":
    main()
