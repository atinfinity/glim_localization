"""The installed R2 launch file (launch/localization.launch.py) as a process (use_composition:=false). Headless.

Inputs come from scripts/test_colcon.sh (GLIM_R1_BAG, GLIM_R1_DATA, GLIM_R1_MAP; see test_r2_node.py). The launch
arguments set config_path (the test config with reliable inputs), map_path, use_sim_time and autostart; the node must
reach active by itself, publish the latched ~/map, and give ~/pose and TF while `ros2 bag play --clock` feeds it.
Accuracy is covered by test_r2_node.py. Deadlines fail the test.
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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import launch_file_helpers  # noqa: E402
import r2_helpers  # noqa: E402


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([launch_file_helpers.r2_launch(use_composition=False), launch_testing.actions.ReadyToTest()]), {}


class TestLaunchR2(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.client = r2_helpers.R2Client("launch_r2_test_client", r2_helpers.require_env("GLIM_R1_DATA"))

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy()
        rclpy.shutdown()

    def test_autostart_pose_tf_and_map(self):
        launch_file_helpers.check_r2_launch(self, self.client)


@launch_testing.post_shutdown_test()
class TestLaunchR2Shutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # The launch shutdown ends the node with SIGINT; any other failure is an error
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[launch_testing.asserts.EXIT_OK, -2, 130])
