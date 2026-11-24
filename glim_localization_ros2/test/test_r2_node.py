"""R2 end to end: glim_localization_node driven through its lifecycle, over a synthetic bag. Headless; no display.

Inputs come from scripts/test_colcon.sh (see there), through environment variables:
  GLIM_R1_BAG   the bag made by scripts/replay_to_bag.py (with two /initialpose messages); R2 uses the R1 bag
  GLIM_R1_DATA  the replay dataset the bag was made from (traj_gt.txt is the ground truth)
  GLIM_R1_MAP   the map dump of the same scene

The node starts with a bad map_path: configure must fail and leave it unconfigured. With the right map_path it is
configured and activated, and `ros2 bag play --clock` feeds it (use_sim_time). The bounds are those of R1.
Assertions are on counts and bounds, not on timing; deadlines fail the test.
"""
import os
import sys
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from glim_localization_msgs.msg import LocalizationStatus
from glim_localization_msgs.srv import Initialize
from lifecycle_msgs.msg import State, Transition
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import r2_helpers  # noqa: E402

# Acceptance bounds for the ROS 2 interface (as in test_r1_extension.py)
MIN_POSE_RATIO = 0.9
SETTLE_S = 3.0
MAX_TRANS_M = 0.1
MAX_ROT_DEG = 1.0
# ~/initialize, the bag's /initialpose at 0 s if it is applied separately, the bag's /initialpose at 5 s, ~/relocalize
EXPECTED_RESETS = (3, 4)
BAG_POSE_AT_S = 5.0  # the second /initialpose of the bag (scripts/test_colcon.sh)
FRAME_PERIOD_S = 0.1  # the synthetic LiDAR rate
RELOCALIZE_AT_S = 8.0  # ~/relocalize at the first TRACKING status from here
BAD_MAP = "/nonexistent/glim_localization_map"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    r2_helpers.require_env("GLIM_R1_BAG")
    r2_helpers.require_env("GLIM_R1_DATA")
    r2_helpers.require_env("GLIM_R1_MAP")
    proc = r2_helpers.node_process(r2_helpers.make_config_dir(), BAD_MAP)
    return launch.LaunchDescription([proc, launch_testing.actions.ReadyToTest()]), {"proc": proc}


class TestR2Node(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.client = r2_helpers.R2Client("r2_test_client", r2_helpers.require_env("GLIM_R1_DATA"))
        cls.initialize = cls.client.node.create_client(Initialize, f"{r2_helpers.NODE}/initialize")
        cls.relocalize = cls.client.node.create_client(Trigger, f"{r2_helpers.NODE}/relocalize")

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy()
        rclpy.shutdown()

    def call_initialize(self, stamp_s, global_search=False):
        """~/initialize with the ground truth pose of the first frame (stamp 0 means the next frame)."""
        request = Initialize.Request()
        request.pose = PoseWithCovarianceStamped()
        request.pose.header.frame_id = "map"
        request.pose.header.stamp.sec = int(stamp_s)
        request.pose.header.stamp.nanosec = int(round((stamp_s - int(stamp_s)) * 1e9))
        gt = self.client.gt[0]
        p, q = request.pose.pose.pose.position, request.pose.pose.pose.orientation
        p.x, p.y, p.z = gt[1:4]
        q.x, q.y, q.z, q.w = gt[4:8]
        request.global_search = global_search
        return self.client.call(self.initialize, request)

    def output_counts(self):
        c = self.client
        return len(c.poses), len(c.statuses), len(c.transforms)

    def test_lifecycle_and_localization(self, proc_info, proc_output, proc):
        c = self.client
        bag = r2_helpers.require_env("GLIM_R1_BAG")

        # A bad map fails configure, and the node stays unconfigured
        self.assertEqual(c.state(), State.PRIMARY_STATE_UNCONFIGURED)
        self.assertFalse(c.transition(Transition.TRANSITION_CONFIGURE))
        self.assertEqual(c.state(), State.PRIMARY_STATE_UNCONFIGURED)
        proc_output.assertWaitFor("failed to load the map", process=proc, timeout=10)

        # The right map: configure and activate
        c.set_string("map_path", r2_helpers.require_env("GLIM_R1_MAP"))
        self.assertTrue(c.transition(Transition.TRANSITION_CONFIGURE))
        self.assertEqual(c.state(), State.PRIMARY_STATE_INACTIVE)

        # While inactive the services answer, and refuse
        self.assertFalse(c.call(self.relocalize, Trigger.Request()).success)
        self.assertFalse(self.call_initialize(0.0).success)

        # First activation: sensor data without any pose. Every frame is dropped, but the late-pose check still knows
        # how far the odometry has got, so a pose stamped at the second frame is now far too late (not the first
        # frame: the synthetic data starts at stamp 0, which means "unstamped")
        self.assertTrue(c.transition(Transition.TRANSITION_ACTIVATE))
        self.assertEqual(c.state(), State.PRIMARY_STATE_ACTIVE)
        c.wait_for_subscribers(["/imu", "/points", "/initialpose"])
        c.play_to_end(bag, ["/imu", "/points"], rate=5.0)
        self.assertEqual(len(c.poses), 0, "a pose without an initial pose")
        response = self.call_initialize(c.gt_stamps[1])
        self.assertFalse(response.success, "a stale ~/initialize was accepted before the first pose")
        self.assertIn("too late", response.message)
        # A pose stamped after the last frame is accepted and stays pending (no frame comes); ~/relocalize must not
        # replace it, and there is no estimate to relocalize from anyway
        self.assertTrue(self.call_initialize(c.gt_stamps[-1] + 1.0).success)
        response = c.call(self.relocalize, Trigger.Request())
        self.assertFalse(response.success)
        self.assertIn("pending", response.message)

        # Deactivate and activate again: a new run (new odometry, publisher and TimeKeeper)
        self.assertTrue(c.transition(Transition.TRANSITION_DEACTIVATE))
        self.assertTrue(c.transition(Transition.TRANSITION_ACTIVATE))
        self.assertEqual(c.state(), State.PRIMARY_STATE_ACTIVE)
        c.wait_for_subscribers(["/imu", "/points", "/initialpose"])

        # The first pose comes from ~/initialize before playing, so a lost early /initialpose of the bag cannot leave
        # the node without a pose (PROGRESS, startup loss). The bag's pose at 0 s (with an offset) then either
        # replaces it, applies one frame later, or is lost; its pose at 5 s is a later reset.
        self.assertTrue(self.call_initialize(0.0).success)
        relocalized = {}

        def relocalize_once_tracking():
            # ~/relocalize in the middle of the bag, while TRACKING: the frames after it must be posed and accurate
            if relocalized or not c.statuses or c.statuses[-1].state != LocalizationStatus.TRACKING:
                return
            if r2_helpers.stamp_s(c.statuses[-1].header.stamp) < RELOCALIZE_AT_S:
                return
            reset_count = c.statuses[-1].reset_count
            response = c.call(self.relocalize, Trigger.Request())
            self.assertTrue(response.success, response.message)
            relocalized.update(reset_count=reset_count, stamp=r2_helpers.stamp_s(c.statuses[-1].header.stamp))

        c.play_and_wait(bag, r2_helpers.BAG_TOPICS, during=relocalize_once_tracking)

        num_frames = len(c.gt)
        # Pose coverage
        self.assertGreaterEqual(len(c.poses), MIN_POSE_RATIO * num_frames, f"{len(c.poses)} poses for {num_frames} frames")
        for pose in c.poses:
            self.assertEqual(pose.header.frame_id, "map")

        # Accuracy against the ground truth after the settling time
        worst_trans, worst_rot, checked = c.pose_errors(c.gt_stamps[0] + SETTLE_S)
        self.assertGreaterEqual(checked, MIN_POSE_RATIO * (num_frames - SETTLE_S * 10), "too few poses after the settling time")
        detail = c.describe_errors(c.gt_stamps[0] + SETTLE_S, MAX_TRANS_M, MAX_ROT_DEG)
        self.assertLessEqual(worst_trans, MAX_TRANS_M, f"translation error {worst_trans:.3f} m; {detail}")
        self.assertLessEqual(worst_rot, MAX_ROT_DEG, f"rotation error {worst_rot:.3f} deg; {detail}")

        # TF: map->odom and odom->base_link at (nearly) every frame, composing to the published pose
        pairs = c.tf_pairs()
        self.assertGreaterEqual(len(pairs), MIN_POSE_RATIO * num_frames, f"{len(pairs)} stamps with both map->odom and odom->base_link")
        for pose in c.poses[-5:]:
            self.assertTrue(c.tf_composes_to_pose(pairs, pose), "TF does not compose to the pose")

        # Resets, all external: ~/initialize (possibly replaced by the bag's pose at 0 s, or followed by it), the bag's
        # pose at 5 s, and ~/relocalize
        self.assertTrue(c.statuses, "no status")
        resets = [(r2_helpers.stamp_s(s.header.stamp), s.reset_count) for prev, s in zip(c.statuses, c.statuses[1:]) if s.reset_count != prev.reset_count]
        # A reset status carries the stamp of the last frame published before it (localization_publisher.cpp): 4.9 s
        # or 5.0 s here, depending on whether the 4.999 s pose arrived before or after the 5.0 s frame was taken
        self.assertTrue(
            any(BAG_POSE_AT_S - FRAME_PERIOD_S - 0.05 < stamp < BAG_POSE_AT_S + 0.05 for stamp, _ in resets),
            f"the bag's pose at {BAG_POSE_AT_S} s was not applied: {resets}",
        )
        self.assertTrue(relocalized, f"~/relocalize was not called: no TRACKING status after {RELOCALIZE_AT_S} s")
        final = c.statuses[-1]
        self.assertIn(final.reset_count, EXPECTED_RESETS)
        self.assertEqual(final.reset_count, relocalized["reset_count"] + 1, "~/relocalize was not applied")
        self.assertEqual(final.last_reset_reason, LocalizationStatus.RESET_EXTERNAL)
        self.assertEqual(final.state, LocalizationStatus.TRACKING)
        after_relocalize = [p for p in c.poses if r2_helpers.stamp_s(p.header.stamp) > relocalized["stamp"] + 1.0]
        self.assertGreaterEqual(len(after_relocalize), 10, "too few poses after ~/relocalize")
        print(
            f"R2: {len(c.poses)}/{num_frames} poses, worst error {worst_trans:.3f} m {worst_rot:.3f} deg, "
            f"{len(pairs)} TF pairs, reset_count {final.reset_count}, relocalized at {relocalized['stamp']:.2f} s"
        )

        # ~/initialize: global search is not implemented; a stale pose is too late
        response = self.call_initialize(c.gt_stamps[-1], global_search=True)
        self.assertFalse(response.success)
        self.assertIn("not implemented", response.message)
        response = self.call_initialize(c.gt_stamps[1])
        self.assertFalse(response.success)
        self.assertIn("max_init_pose_delay", response.message)

        # Deactivate: the inputs and outputs go away, and nothing is published any more
        self.assertTrue(c.transition(Transition.TRANSITION_DEACTIVATE))
        self.assertEqual(c.state(), State.PRIMARY_STATE_INACTIVE)
        n = c.node
        c.wait_until(lambda: all(n.count_subscribers(t) == 0 for t in r2_helpers.BAG_TOPICS), "the node still subscribes to its inputs")
        outputs = [f"{r2_helpers.NODE}/pose", f"{r2_helpers.NODE}/status", "/tf"]
        c.wait_until(lambda: all(n.count_publishers(t) == 0 for t in outputs), "the node still advertises its outputs")
        counts = self.output_counts()
        c.spin_for(1.0)
        self.assertEqual(self.output_counts(), counts, "output after deactivate")
        self.assertFalse(c.call(self.relocalize, Trigger.Request()).success)

        # Clean up: the services go away; configure again brings them back
        self.assertTrue(c.transition(Transition.TRANSITION_CLEANUP))
        self.assertEqual(c.state(), State.PRIMARY_STATE_UNCONFIGURED)
        c.wait_until(lambda: not self.initialize.service_is_ready() and not self.relocalize.service_is_ready(), "the services still exist after cleanup")
        self.assertTrue(c.transition(Transition.TRANSITION_CONFIGURE))
        self.assertEqual(c.state(), State.PRIMARY_STATE_INACTIVE)
        response = self.call_initialize(0.0)
        self.assertFalse(response.success)
        self.assertIn("not active", response.message)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # The test ends the node with SIGINT; any other failure is an error
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[launch_testing.asserts.EXIT_OK, -2, 130])
