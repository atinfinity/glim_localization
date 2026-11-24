"""
R2 over a bag with an IMU hole: the node recovers by a self-reset.

Headless; no display.

Inputs come from scripts/test_colcon.sh (see there), through environment variables:
  GLIM_R2_GAP_BAG  the R1 dataset as a bag with no /initialpose and no IMU sample in
                   [GAP_START_S, GAP_START_S + GAP_S) from the first frame
                   (scripts/replay_to_bag.py --drop-imu)
  GLIM_R1_DATA     the replay dataset the bag was made from (traj_gt.txt is the ground truth)
  GLIM_R1_MAP      the map dump of the same scene

Before playing, the test sets the exact pose of the frame at INIT_AT_S through ~/initialize,
stamped with that frame: it applies there, whatever the node received before. A pose in the bag at
its first frame was lost in some runs: the node received nothing from the first second or so of
playback, although the player's publishers were visible.

The hole (0.3 s) is longer than max_imu_gap and shorter than imu_gap_reset_max (config/r1), so the
odometry drops the frames that span it and resets itself from the last pose: reset_count goes from
1 (~/initialize) to 2, the reason is IMU_GAP, and tracking resumes. Assertions are on counts and
bounds, not on timing; deadlines fail the test.
"""
import os
import sys
import unittest

from glim_localization_msgs.msg import LocalizationStatus
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

GAP_START_S = 6.0  # must match scripts/test_colcon.sh
GAP_S = 0.3
FRAME_PERIOD_S = 0.1  # the synthetic LiDAR runs at 10 Hz
INIT_AT_S = 2.0  # from the first frame; the frames before it are not posed
# Besides those 20 frames, the frames that span the hole and the restart after it are not posed
MIN_POSE_RATIO = 0.7
SETTLE_S = 3.0
MAX_TRANS_M = 0.1
MAX_ROT_DEG = 1.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    r2_helpers.require_env('GLIM_R2_GAP_BAG')
    r2_helpers.require_env('GLIM_R1_DATA')
    proc = r2_helpers.node_process(
        r2_helpers.make_config_dir(), r2_helpers.require_env('GLIM_R1_MAP'))
    return launch.LaunchDescription([proc, launch_testing.actions.ReadyToTest()]), {'proc': proc}


class TestR2ImuGap(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.client = r2_helpers.R2Client(
            'r2_gap_test_client', r2_helpers.require_env('GLIM_R1_DATA'))

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy()
        rclpy.shutdown()

    def test_imu_gap_recovery(self, proc_info, proc_output, proc):
        c = self.client
        self.assertTrue(c.transition(Transition.TRANSITION_CONFIGURE))
        self.assertTrue(c.transition(Transition.TRANSITION_ACTIVATE))
        self.assertEqual(c.state(), State.PRIMARY_STATE_ACTIVE)
        c.wait_for_subscribers(['/imu', '/points'])
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
        c.play_and_wait(r2_helpers.require_env('GLIM_R2_GAP_BAG'), ['/imu', '/points'])

        num_frames = len(c.gt)
        self.assertGreaterEqual(
            len(c.poses), MIN_POSE_RATIO * num_frames,
            f'{len(c.poses)} poses for {num_frames} frames')

        # One self-reset for the hole, after the ~/initialize
        self.assertTrue(c.statuses, 'no status')
        gap_resets = [
            s for s in c.statuses if s.last_reset_reason == LocalizationStatus.RESET_IMU_GAP]
        self.assertTrue(gap_resets, 'no IMU_GAP reset')
        hole_start = c.gt_stamps[0] + GAP_START_S
        # The reset status is stamped with the last published frame (DESIGN 6, "Stamps"): the frame
        # before the hole or later
        self.assertGreaterEqual(
            r2_helpers.stamp_s(gap_resets[0].header.stamp), hole_start - FRAME_PERIOD_S - 1e-6)
        final = c.statuses[-1]
        self.assertEqual(final.reset_count, 2)
        self.assertEqual(final.last_reset_reason, LocalizationStatus.RESET_IMU_GAP)
        self.assertEqual(final.state, LocalizationStatus.TRACKING)

        # Back on track: the poses after the hole and a settling time are within the R1 bounds
        worst_trans, worst_rot, checked = c.pose_errors(hole_start + GAP_S + SETTLE_S)
        self.assertGreater(checked, 0, 'no pose after the recovery')
        self.assertLessEqual(worst_trans, MAX_TRANS_M, f'translation error {worst_trans:.3f} m')
        self.assertLessEqual(worst_rot, MAX_ROT_DEG, f'rotation error {worst_rot:.3f} deg')
        print(
            f'R2 IMU gap: {len(c.poses)}/{num_frames} poses, reset_count {final.reset_count}, '
            f'worst error after recovery {worst_trans:.3f} m {worst_rot:.3f} deg '
            f'over {checked} poses'
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[launch_testing.asserts.EXIT_OK, -2, 130])
