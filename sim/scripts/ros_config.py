#!/usr/bin/env python3
"""Write the config of glim_localization_node (R2) for the Gazebo bag.

  ros_config.py <out dir> <dest dir> [--map-cloud-voxel V]

The config of sim/scripts/localize.sh (<out>/config: the shipped CPU config with the rig extrinsic) with
the R2 config_ros.json, the sim topics and the initial pose, which is the ground truth at the first frame
(<out>/traj_gt.txt). The gpu_lidar clouds have no time field: a global shutter LiDAR. <dest> is replaced.
--map-cloud-voxel sets the ~/map voxel (the shipped value when not given). Used by sim/scripts/ros_localize.sh
(the RViz2 capture) and sim/scripts/bench_run.sh (the benchmark). Standard library only.
"""
import argparse
import os
import re
import shutil


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("out", help="the sim_eval.sh output directory (config, traj_gt.txt)")
    parser.add_argument("dest", help="the config directory to write (replaced)")
    parser.add_argument("--map-cloud-voxel", type=float, help="~/map voxel [m]")
    args = parser.parse_args()

    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    shutil.rmtree(args.dest, ignore_errors=True)
    shutil.copytree(os.path.join(args.out, "config"), args.dest)
    shutil.copy(os.path.join(repo_root, "glim_localization_ros2", "config", "r2", "config_ros.json"), args.dest)

    with open(os.path.join(args.out, "traj_gt.txt")) as handle:
        row = next(line for line in handle if line.strip() and not line.startswith("#")).split()
    edits = {
        "config_ros.json": [(r'"imu_topic": "[^"]*"', '"imu_topic": "/sim_eval/imu"'),
                            (r'"points_topic": "[^"]*"', '"points_topic": "/sim_eval/lidar/points"')],
        "config_sensors.json": [(r'"global_shutter_lidar": \w+', '"global_shutter_lidar": true')],
        "config_odometry_localization_cpu.json": [
            (r'"map_path": "[^"]*",', '"map_path": "",\n    "init_T_world_imu": [' + ", ".join(row[1:8]) + "],")],
    }
    if args.map_cloud_voxel is not None:
        edits["config_ros.json"].append((r'"map_cloud_voxel": [0-9.]+', f'"map_cloud_voxel": {args.map_cloud_voxel}'))
    for name, subs in edits.items():
        path = os.path.join(args.dest, name)
        with open(path) as handle:
            text = handle.read()
        for pattern, repl in subs:
            text, n = re.subn(pattern, repl, text, count=1)
            assert n == 1, (name, pattern)
        with open(path, "w") as handle:
            handle.write(text)


if __name__ == "__main__":
    main()
