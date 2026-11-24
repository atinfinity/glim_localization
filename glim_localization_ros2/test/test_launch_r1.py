# Copyright (c) 2026 atinfinity
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Test the installed R1 launch file (launch/localization_r1.launch.py) over the synthetic bag.

Headless (rviz:=false).

Inputs come from scripts/test_colcon.sh (GLIM_R1_BAG, GLIM_R1_DATA, GLIM_R1_MAP; see
test_r1_extension.py). The launch file runs glim_rosbag with the installed config/r1, filling in
the map path through its map_path argument, and a params_file of this test (auto_quit off, so that
the node and its latched ~/map stay up). Accuracy is covered by test_r1_extension.py; this test
checks that the launch file starts the extension and that ~/pose, TF and ~/map come out.
"""
import atexit
import os
import shutil
import sys
import tempfile
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import launch_file_helpers  # noqa: E402, I100
import r2_helpers  # noqa: E402  (its R2Client records ~/pose and /tf of a given node)

NODE = '/glim_ros'


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    bag = r2_helpers.require_env('GLIM_R1_BAG')
    r2_helpers.require_env('GLIM_R1_DATA')
    work_dir = tempfile.mkdtemp(prefix='glim_launch_r1_')
    atexit.register(shutil.rmtree, work_dir, ignore_errors=True)
    params_file = os.path.join(work_dir, 'params.yaml')
    with open(params_file, 'w') as f:
        # delay: the bag is read 2 s after the node is up, so that the test's subscribers are
        # connected first. dump_path: a path of this run, so that parallel tests share no output
        # path.
        f.write(
            '/**:\n  ros__parameters:\n    auto_quit: false\n    delay: 2.0\n'
            f"    dump_path: {os.path.join(work_dir, 'dump')}\n")
    include = launch_file_helpers.include_launch(
        'localization_r1.launch.py',
        {
            'bag': bag,
            'map_path': r2_helpers.require_env('GLIM_R1_MAP'),
            'params_file': params_file,
        })
    return launch.LaunchDescription([include, launch_testing.actions.ReadyToTest()]), {}


class TestLaunchR1(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.client = r2_helpers.R2Client(
            'launch_r1_test_client', r2_helpers.require_env('GLIM_R1_DATA'), target=NODE)

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy()
        rclpy.shutdown()

    def test_pose_tf_and_map(self):
        client = self.client
        last = client.gt_stamps[-1]
        client.wait_until(
            lambda: (
                client.poses and r2_helpers.stamp_s(client.poses[-1].header.stamp) >= last - 0.25),
            f'the last pose (stamp {last:.2f}) did not arrive', timeout=r2_helpers.DEADLINE_S)
        client.spin_for(1.0)

        # ~/pose and ~/map come only from the extension, so they also show that the launch file
        # loaded it
        num_frames = len(client.gt)
        self.assertGreaterEqual(
            len(client.poses), launch_file_helpers.MIN_POSE_RATIO * num_frames,
            f'{len(client.poses)} poses for {num_frames} frames')
        self.assertTrue(all(p.header.frame_id == 'map' for p in client.poses))
        pairs = client.tf_pairs()
        self.assertGreaterEqual(
            len(pairs), launch_file_helpers.MIN_POSE_RATIO * num_frames,
            f'{len(pairs)} stamps with map->odom and odom->base_link')
        self.assertTrue(
            client.tf_composes_to_pose(pairs, client.poses[-1]),
            'TF does not compose to the last pose')

        # ~/map was published when the extension was loaded, long before this subscription
        launch_file_helpers.check_map(
            self, launch_file_helpers.wait_for_latched_map(client.node, f'{NODE}/map'))


@launch_testing.post_shutdown_test()
class TestLaunchR1Shutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # The launch shutdown ends glim_rosbag with SIGINT; any other failure is an error
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[launch_testing.asserts.EXIT_OK, -2, 130])
