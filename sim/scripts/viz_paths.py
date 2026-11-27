#!/usr/bin/env python3
"""Paths for the RViz2 capture of scripts/sim_viz.sh.

  viz_paths.py <traj_gt.txt>

Publishes /sim_viz/gt_path once (latched), the ground truth (TUM, T_world_imu) in the map frame, and
/sim_viz/est_path, the poses of /glim_localization/pose so far, at every pose. The map frame is the
world frame of the simulation. Runs with use_sim_time.
"""
import sys

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def read_gt(path, step=5):
    rows = [line.split() for line in open(path) if line.strip() and not line.startswith('#')]
    poses = []
    for row in rows[::step]:
        t, x, y, z, qx, qy, qz, qw = (float(v) for v in row[:8])
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp.sec = int(t)
        p.header.stamp.nanosec = int(round((t - int(t)) * 1e9)) % 1000000000
        p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z
        o = p.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        poses.append(p)
    return poses


class VizPaths(Node):

    def __init__(self, gt_file):
        super().__init__('sim_viz_paths', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.gt_pub = self.create_publisher(Path, '/sim_viz/gt_path', latched)
        self.est_pub = self.create_publisher(Path, '/sim_viz/est_path', 10)
        gt = Path()
        gt.header.frame_id = 'map'
        gt.poses = read_gt(gt_file)
        self.gt_pub.publish(gt)
        self.est = Path()
        self.est.header.frame_id = 'map'
        self.create_subscription(PoseWithCovarianceStamped, '/glim_localization/pose', self.on_pose, 10)

    def on_pose(self, msg):
        p = PoseStamped()
        p.header = msg.header
        p.pose = msg.pose.pose
        self.est.poses.append(p)
        self.est.header.stamp = msg.header.stamp
        self.est_pub.publish(self.est)


def main():
    rclpy.init()
    rclpy.spin(VizPaths(sys.argv[1]))


if __name__ == '__main__':
    main()
