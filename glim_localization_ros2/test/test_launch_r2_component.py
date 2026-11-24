"""The installed R2 launch file (launch/localization.launch.py) with the node as a component (use_composition:=true).

Headless. As test_launch_r2.py, but the node is loaded into a component container that the launch file creates, so
autostart has to work through the component interface (no main of our own).
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
    return launch.LaunchDescription([launch_file_helpers.r2_launch(use_composition=True), launch_testing.actions.ReadyToTest()]), {}


class TestLaunchR2Component(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.client = r2_helpers.R2Client('launch_r2_component_test_client', r2_helpers.require_env('GLIM_R1_DATA'))

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy()
        rclpy.shutdown()

    def test_autostart_pose_tf_and_map(self, proc_output):
        launch_file_helpers.check_r2_launch(self, self.client, proc_output)


@launch_testing.post_shutdown_test()
class TestLaunchR2ComponentShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # The launch shutdown ends the container with SIGINT; any other failure is an error
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[launch_testing.asserts.EXIT_OK, -2, 130])
