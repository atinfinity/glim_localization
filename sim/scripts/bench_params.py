#!/usr/bin/env python3
"""Write the parameters of lidar_localization_ros2 for the Gazebo bag (sim/scripts/bench_run.sh).

  bench_params.py <default yaml> <out dir> <bag> <dest dir> [--no-imu]

<default yaml> is the shipped param/localization.yaml of lidar_localization_ros2. Writes, in <dest>:
- localization.yaml: the default file with the overrides below, under the same `/**` key;
- overrides.json: every override with the reason, and the frame of the clouds, for bench_run.sh and the
  records of docs/benchmark.md.
The overrides are the fewest that the bag needs: the map, the initial pose (the ground truth at the first
frame, <out>/traj_gt.txt), the base frame (the IMU frame, which the ground truth and glim_localization
use), simulation time, and the ones forced by the data (FORCED). --no-imu also turns the IMU
preintegration off. The map is converted to float32 (<dest>/map_f4.pcd, see to_float32_pcd). Runs in the
baseline image (python3-yaml, python3-numpy, rosbag2_py).
"""
import argparse
import json
import os

import numpy as np
import yaml

# Forced by the Gazebo data, not tuning. Each one is recorded in docs/benchmark.md.
FORCED = {}


def to_float32_pcd(src, dest):
    """Write the binary x y z PCD src (map_compiler --export-pcd writes float64) as float32.

    The baseline loads the map with pcl::loadPCDFile into PointXYZ, which does not match fields of SIZE 8
    ("Failed to find match for field 'x'") and leaves every point at zero. Returns the largest absolute
    coordinate, which bounds the rounding (float32 has a 24-bit mantissa).
    """
    header = {}
    with open(src, "rb") as handle:
        while True:
            raw = handle.readline()
            if not raw:
                raise RuntimeError(f"{src}: end of file before the DATA line")
            line = raw.decode("ascii").strip()
            if not line or line.startswith("#"):
                continue
            key, *values = line.split()
            header[key] = values
            if key == "DATA":
                break
        sizes = header.get("SIZE")
        if (header.get("FIELDS") != ["x", "y", "z"] or header.get("DATA") != ["binary"]
                or header.get("TYPE") != ["F"] * 3 or header.get("COUNT", ["1"] * 3) != ["1"] * 3
                or sizes not in (["4"] * 3, ["8"] * 3)):
            raise RuntimeError(f"{src}: expected binary float x y z fields of one size, got {header}")
        dtype = {"4": "<f4", "8": "<f8"}[header["SIZE"][0]]
        points = np.fromfile(handle, dtype=dtype, count=3 * int(header["POINTS"][0])).reshape(-1, 3)
    with open(dest, "wb") as handle:
        handle.write(("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
                      "TYPE F F F\nCOUNT 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\n"
                      "DATA binary\n").format(n=len(points)).encode("ascii"))
        handle.write(points.astype("<f4").tobytes())
    return float(np.abs(points).max())


def first_cloud_frame(bag, topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    while reader.has_next():
        _, data, _ = reader.read_next()
        return deserialize_message(data, PointCloud2).header.frame_id
    raise RuntimeError(f"no {topic} message in {bag}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("default_yaml")
    parser.add_argument("out", help="the sim_eval.sh output directory (traj_gt.txt, map_export.pcd)")
    parser.add_argument("bag", help="the restamped bag")
    parser.add_argument("dest")
    parser.add_argument("--map", help="the map PCD (default <out>/map_export.pcd)")
    parser.add_argument("--cloud-topic", default="/sim_eval/lidar/points")
    parser.add_argument("--no-imu", action="store_true")
    args = parser.parse_args()

    with open(os.path.join(args.out, "traj_gt.txt")) as handle:
        row = next(line for line in handle if line.strip() and not line.startswith("#")).split()
    x, y, z, qx, qy, qz, qw = (float(v) for v in row[1:8])
    os.makedirs(args.dest, exist_ok=True)
    source_map = args.map or os.path.join(args.out, "map_export.pcd")
    map_f4 = os.path.join(args.dest, "map_f4.pcd")
    max_abs = to_float32_pcd(source_map, map_f4)
    overrides = {
        "map_path": (map_f4, f"the map_compiler export of the same map ({source_map}), converted to float32: "
                     "pcl::loadPCDFile into PointXYZ does not read the float64 fields "
                     f"(largest coordinate {max_abs:.1f} m)"),
        "initial_pose_x": (x, "the ground truth at the first frame"),
        "initial_pose_y": (y, "the ground truth at the first frame"),
        "initial_pose_z": (z, "the ground truth at the first frame"),
        "initial_pose_qx": (qx, "the ground truth at the first frame"),
        "initial_pose_qy": (qy, "the ground truth at the first frame"),
        "initial_pose_qz": (qz, "the ground truth at the first frame"),
        "initial_pose_qw": (qw, "the ground truth at the first frame"),
        "base_frame_id": ("imu_link", "the pose is T_map_imu, as the ground truth (the IMU is the base frame)"),
        "use_sim_time": (True, "the bag is played with --clock"),
    }
    overrides.update(FORCED)
    if args.no_imu:
        overrides["use_imu_preintegration"] = (False, "the no-IMU row")

    with open(args.default_yaml) as handle:
        data = yaml.safe_load(handle)
    params = data["/**"]["ros__parameters"]
    for name, (value, _) in overrides.items():
        params[name] = value
    with open(os.path.join(args.dest, "localization.yaml"), "w") as handle:
        yaml.safe_dump(data, handle, sort_keys=False)
    record = {
        "default_yaml": args.default_yaml,
        "cloud_frame_id": first_cloud_frame(args.bag, args.cloud_topic),
        "overrides": {name: {"value": value, "reason": reason} for name, (value, reason) in overrides.items()},
    }
    with open(os.path.join(args.dest, "overrides.json"), "w") as handle:
        json.dump(record, handle, indent=2)
        handle.write("\n")
    print(record["cloud_frame_id"])


if __name__ == "__main__":
    main()
