"""
Test the installed R1 launch file without bag:= (glim_rosnode) and with map_path empty.

Headless (rviz:=false).

The map path comes from the config instead: a copy of the installed config/r1 with
odometry_estimation.map_path filled in (by launch_utils.config_with_map, as the launch file does
for map_path:=). No sensor data is played; the test checks that glim_rosnode loads the extension
with that map and latches ~/map.
"""
import atexit
import os
import shutil
import sys
import tempfile
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import launch_file_helpers  # noqa: E402, I100
import r2_helpers  # noqa: E402

NODE = '/glim_ros'


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    work_dir = tempfile.mkdtemp(prefix='glim_launch_r1_live_')
    atexit.register(shutil.rmtree, work_dir, ignore_errors=True)
    config_dir = r2_helpers.load_launch_utils().config_with_map(
        os.path.join(get_package_share_directory('glim_localization_ros2'), 'config', 'r1'),
        r2_helpers.require_env('GLIM_R1_MAP'), work_dir)
    params_file = os.path.join(work_dir, 'params.yaml')
    with open(params_file, 'w') as f:
        # A quoted path that must reach GlimROS as a string, and a glim_ros section on top of /**
        f.write(
            f"/**:\n  ros__parameters:\n    dump_path: '{os.path.join(work_dir, 'dump')}'\n"
            'glim_ros:\n  ros__parameters:\n    debug: false\n')
    include = launch_file_helpers.include_launch(
        'localization_r1.launch.py',
        {'config_path': config_dir, 'map_path': '', 'params_file': params_file})
    return launch.LaunchDescription([include, launch_testing.actions.ReadyToTest()]), {}


class TestLaunchR1Live(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('launch_r1_live_test_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_map_from_the_config(self):
        launch_file_helpers.check_map(
            self, launch_file_helpers.wait_for_latched_map(self.node, f'{NODE}/map'))


@launch_testing.post_shutdown_test()
class TestLaunchR1LiveShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # The launch shutdown ends glim_rosnode with SIGINT; any other failure is an error
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[launch_testing.asserts.EXIT_OK, -2, 130])
