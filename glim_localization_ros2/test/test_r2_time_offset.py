# Copyright (c) 2026 atinfinity
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
R2 with points_time_offset: pose stamps are frame time and are not shifted.

Headless; no display.

Inputs come from scripts/test_colcon.sh (see there), through environment variables:
  GLIM_R1_BAG   the bag made by scripts/replay_to_bag.py; only /imu and /points are played
  GLIM_R1_DATA  the replay dataset the bag was made from (traj_gt.txt is the ground truth)
  GLIM_R1_MAP   the map dump of the same scene

The node runs with imu_time_offset = points_time_offset = OFFSET_S, so every frame (and every
published stamp) is OFFSET_S ahead of the bag's header stamps (frame time = cloud header +
points_time_offset). The stamp of /initialpose and ~/initialize is frame time too, the time base of
~/pose, TF and RViz's now(), and the node compares it with the frames as it is. A node that added
the offset to the pose stamp would apply the first pose OFFSET_S late, and would accept a pose
stamped with the raw cloud header of the last frame. Assertions are on stamps and bounds, not on
timing; deadlines fail the test.
"""
import os
import sys
import unittest

from geometry_msgs.msg import PoseWithCovarianceStamped
from glim_localization_msgs.srv import Initialize
import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from lifecycle_msgs.msg import State, Transition
import pytest
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import r2_helpers  # noqa: E402, I100

OFFSET_S = 5.0
INIT_AT_S = 1.0  # from the first frame; the frames before it are not posed
SETTLE_S = 3.0
MAX_TRANS_M = 0.1
MAX_ROT_DEG = 1.0
STAMP_TOLERANCE_S = 1e-3


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    r2_helpers.require_env('GLIM_R1_BAG')
    r2_helpers.require_env('GLIM_R1_DATA')
    config_dir = r2_helpers.make_config_dir(
        {'imu_time_offset': OFFSET_S, 'points_time_offset': OFFSET_S})
    proc = r2_helpers.node_process(config_dir, r2_helpers.require_env('GLIM_R1_MAP'))
    return launch.LaunchDescription([proc, launch_testing.actions.ReadyToTest()]), {'proc': proc}


class TestR2TimeOffset(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.client = r2_helpers.R2Client(
            'r2_time_offset_test_client', r2_helpers.require_env('GLIM_R1_DATA'))
        cls.initialize = cls.client.node.create_client(
            Initialize, f'{r2_helpers.NODE}/initialize')

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy()
        rclpy.shutdown()

    def call_initialize(self, gt_row, stamp):
        """Call ~/initialize with a ground truth pose and the given stamp [s]."""
        request = Initialize.Request()
        request.pose = PoseWithCovarianceStamped()
        request.pose.header.frame_id = 'map'
        request.pose.header.stamp.sec = int(stamp)
        request.pose.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
        p, q = request.pose.pose.pose.position, request.pose.pose.pose.orientation
        p.x, p.y, p.z = gt_row[1:4]
        q.x, q.y, q.z, q.w = gt_row[4:8]
        return self.client.call(self.initialize, request)

    def test_pose_stamps_in_frame_time(self, proc_info, proc_output, proc):
        c = self.client
        self.assertTrue(c.transition(Transition.TRANSITION_CONFIGURE))
        self.assertTrue(c.transition(Transition.TRANSITION_ACTIVATE))
        self.assertEqual(c.state(), State.PRIMARY_STATE_ACTIVE)
        c.wait_for_subscribers(['/imu', '/points'])

        # Before the playback: the pose of the frame at INIT_AT_S, stamped with its frame time,
        # applies at that frame
        init = next(row for row in c.gt if row[0] >= c.gt_stamps[0] + INIT_AT_S - 1e-6)
        response = self.call_initialize(init, init[0] + OFFSET_S)
        self.assertTrue(response.success, response.message)
        c.play_and_wait(
            r2_helpers.require_env('GLIM_R1_BAG'), ['/imu', '/points'],
            node_matched=r2_helpers.wait_for_node_matched(proc_output), time_offset=OFFSET_S)

        # ~/pose is in frame time: the first one is the frame of the initial pose, + OFFSET_S
        self.assertTrue(c.poses, 'no pose')
        first = r2_helpers.stamp_s(c.poses[0].header.stamp)
        self.assertAlmostEqual(first, init[0] + OFFSET_S, delta=STAMP_TOLERANCE_S)
        worst_trans, worst_rot, checked = c.pose_errors(init[0] + SETTLE_S, OFFSET_S)
        self.assertGreater(checked, 0, 'no pose after the settling time')
        self.assertLessEqual(worst_trans, MAX_TRANS_M, f'translation error {worst_trans:.3f} m')
        self.assertLessEqual(worst_rot, MAX_ROT_DEG, f'rotation error {worst_rot:.3f} deg')

        # After the playback: the late-pose check compares frame times. A pose at an early frame
        # time is far too late, and so is one stamped with the raw cloud header of the last frame
        # (late by OFFSET_S); one at the last frame time is on time
        for row, stamp, what in (
                (c.gt[1], c.gt[1][0] + OFFSET_S, 'an early frame time'),
                (c.gt[-1], c.gt[-1][0], 'the raw cloud header of the last frame')):
            response = self.call_initialize(row, stamp)
            self.assertFalse(response.success, f'a ~/initialize at {what} was accepted')
            self.assertIn('too late', response.message)
        response = self.call_initialize(c.gt[-1], c.gt[-1][0] + OFFSET_S)
        self.assertTrue(response.success, response.message)
        print(
            f'R2 time offset: {len(c.poses)} poses, first at {first:.3f} '
            f'(initial pose {init[0]:.3f} + {OFFSET_S}), worst error {worst_trans:.3f} m '
            f'{worst_rot:.3f} deg over {checked} poses')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[launch_testing.asserts.EXIT_OK, -2, 130])
