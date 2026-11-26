# Copyright (c) 2026 atinfinity
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
R2 with wheel odometry enabled: the node subscribes to it, converts it and reports it.

Headless; no display.

Inputs come from scripts/test_colcon.sh (see there), through environment variables:
  GLIM_R1_BAG   the bag made by scripts/replay_to_bag.py; only /imu and /points are played
  GLIM_R1_DATA  the replay dataset the bag was made from (traj_gt.txt is the ground truth)
  GLIM_R1_MAP   the map dump of the same scene

The synthetic data has no wheel odometry, so this checks the plumbing, not the estimate. The config
enables wheel_odom_enabled and sets localization_ros.wheel_odom_topic. Before the bag plays, the
test publishes NUM_VALID twists stamped after the end of the bag, one with another child_frame_id
and one without a stamp. The node must take the valid ones (none dropped by the odometry: their
stamps increase) and reject the other two, which its stop line reports on deactivate. No sample
covers a frame interval, so every frame counts as wheel_missing, and no frame gets a wheel factor.
"""
import json
import os
import sys
import unittest

from glim_localization_msgs.srv import Initialize
import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from lifecycle_msgs.msg import State, Transition
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import r2_helpers  # noqa: E402, I100

WHEEL_TOPIC = '/wheel_odom'
BASE_FRAME = 'base_link'
NUM_VALID = 5
INIT_AT_S = 2.0  # from the first frame (see test_r2_imu_gap.py for why not the bag's pose)
MIN_POSE_RATIO = 0.7
STOP_TIMEOUT_S = 30.0


def make_wheel_config_dir():
    """Copy the config (r2_helpers.make_config_dir) and enable wheel odometry in it."""
    config_dir = r2_helpers.make_config_dir()
    ros_path = os.path.join(config_dir, 'config_ros.json')
    config = r2_helpers.strip_comments(ros_path)
    config['localization_ros'].update({
        'wheel_odom_topic': WHEEL_TOPIC,
        'wheel_odom_child_frame_id': BASE_FRAME,
        'wheel_odom_qos': {'profile': 'default', 'reliability': 'reliable', 'depth': 100},
    })
    with open(ros_path, 'w') as f:
        json.dump(config, f)
    odom_path = os.path.join(
        os.path.dirname(config_dir), 'r1', 'config_odometry_localization_cpu.json')
    config = r2_helpers.strip_comments(odom_path)
    config['odometry_estimation']['wheel_odom_enabled'] = True
    with open(odom_path, 'w') as f:
        json.dump(config, f)
    return config_dir


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    r2_helpers.require_env('GLIM_R1_BAG')
    r2_helpers.require_env('GLIM_R1_DATA')
    proc = r2_helpers.node_process(make_wheel_config_dir(), r2_helpers.require_env('GLIM_R1_MAP'))
    return launch.LaunchDescription([proc, launch_testing.actions.ReadyToTest()]), {'proc': proc}


class TestR2WheelOdom(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.client = r2_helpers.R2Client(
            'r2_wheel_test_client', r2_helpers.require_env('GLIM_R1_DATA'))

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy()
        rclpy.shutdown()

    def publish_wheel(self, c, proc_output):
        pub = c.node.create_publisher(
            Odometry, WHEEL_TOPIC, QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        # Volatile: a message sent before the node's reader matches this writer is lost. Wait on
        # both sides (the node logs its matched publishers; this is the only one on the topic)
        c.wait_until(
            lambda: pub.get_subscription_count() >= 1, 'no match on the wheel topic')
        r2_helpers.wait_for_node_matched(proc_output)([WHEEL_TOPIC])
        end = c.gt_stamps[-1] + 100.0

        def message(stamp, child_frame_id):
            msg = Odometry()
            msg.header.frame_id = 'odom'
            msg.header.stamp.sec = int(stamp)
            msg.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
            msg.child_frame_id = child_frame_id
            msg.twist.twist.linear.x = 1.0
            return msg

        for i in range(NUM_VALID):
            pub.publish(message(end + 0.1 * i, BASE_FRAME))
        pub.publish(message(end + 1.0, 'wheels'))  # rejected: child_frame_id
        pub.publish(message(0.0, BASE_FRAME))  # rejected: no stamp
        c.spin_for(1.0)
        return pub

    def test_wheel_odom_input(self, proc_info, proc_output, proc):
        c = self.client
        self.assertTrue(c.transition(Transition.TRANSITION_CONFIGURE))
        self.assertTrue(c.transition(Transition.TRANSITION_ACTIVATE))
        self.assertEqual(c.state(), State.PRIMARY_STATE_ACTIVE)
        c.wait_for_subscribers(['/imu', '/points', WHEEL_TOPIC])
        pub = self.publish_wheel(c, proc_output)

        init = next(row for row in c.gt if row[0] >= c.gt_stamps[0] + INIT_AT_S - 1e-6)
        request = Initialize.Request()
        request.pose.header.frame_id = 'map'
        request.pose.header.stamp.sec = int(init[0])
        request.pose.header.stamp.nanosec = int(round((init[0] - int(init[0])) * 1e9))
        p, q = request.pose.pose.pose.position, request.pose.pose.pose.orientation
        p.x, p.y, p.z = init[1:4]
        q.x, q.y, q.z, q.w = init[4:8]
        response = c.call(
            c.node.create_client(Initialize, f'{r2_helpers.NODE}/initialize'), request)
        self.assertTrue(response.success, response.message)
        c.play_and_wait(r2_helpers.require_env('GLIM_R1_BAG'), ['/imu', '/points'])

        num_frames = len(c.gt)
        self.assertGreaterEqual(
            len(c.poses), MIN_POSE_RATIO * num_frames,
            f'{len(c.poses)} poses for {num_frames} frames')
        self.assertTrue(c.statuses, 'no status')
        self.assertGreater(max(s.wheel_missing for s in c.statuses), 0)
        self.assertEqual(max(s.wheel_factors for s in c.statuses), 0)

        self.assertTrue(c.transition(Transition.TRANSITION_DEACTIVATE))
        proc_output.assertWaitFor(
            f'stopping: wheel odometry received {NUM_VALID + 2}, rejected 2, '
            'dropped by the odometry 0', process=proc, timeout=STOP_TIMEOUT_S)
        c.node.destroy_publisher(pub)
        print(
            f'R2 wheel odometry: {len(c.poses)}/{num_frames} poses, '
            f'max wheel_missing {max(s.wheel_missing for s in c.statuses)}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[launch_testing.asserts.EXIT_OK, -2, 130])
