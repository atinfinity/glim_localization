# Copyright (c) 2026 atinfinity
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
One localization per process: a component container refuses a second R2 node.

Headless; no map and no bag. The node is only constructed (not configured), which is where it
takes the process claim. A second LoadNode into the same container must fail with the claim
message; after the first node is unloaded, a load must succeed again (the destructor releases the
claim).
"""
import time
import unittest

from composition_interfaces.srv import LoadNode, UnloadNode
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy

CONTAINER = '/single_instance_container'
PLUGIN = 'glim_localization_ros2::LocalizationNode'
CLAIM_MESSAGE = 'one localization per process: already held by R2 node /first_localization'
TIMEOUT_S = 30.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    container = launch_ros.actions.Node(
        package='rclcpp_components', executable='component_container',
        name='single_instance_container', output='screen')
    return launch.LaunchDescription([container, launch_testing.actions.ReadyToTest()]), {}


class TestR2SingleInstance(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('r2_single_instance_test_client')
        cls.load = cls.node.create_client(LoadNode, f'{CONTAINER}/_container/load_node')
        cls.unload = cls.node.create_client(UnloadNode, f'{CONTAINER}/_container/unload_node')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def call(self, client, request):
        self.assertTrue(client.wait_for_service(timeout_sec=TIMEOUT_S),
                        f'{client.srv_name} is not available')
        future = client.call_async(request)
        deadline = time.monotonic() + TIMEOUT_S
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.1)
        self.assertTrue(future.done(), f'{client.srv_name} did not answer')
        return future.result()

    def load_node(self, name):
        request = LoadNode.Request()
        request.package_name = 'glim_localization_ros2'
        request.plugin_name = PLUGIN
        request.node_name = name
        return self.call(self.load, request)

    def test_second_load_refused_until_unload(self, proc_output):
        first = self.load_node('first_localization')
        self.assertTrue(first.success, first.error_message)

        second = self.load_node('second_localization')
        self.assertFalse(second.success)
        self.assertIn(CLAIM_MESSAGE, second.error_message)

        request = UnloadNode.Request()
        request.unique_id = first.unique_id
        unloaded = self.call(self.unload, request)
        self.assertTrue(unloaded.success, unloaded.error_message)

        third = self.load_node('third_localization')
        self.assertTrue(third.success, third.error_message)
        self.assertEqual(third.full_node_name, '/third_localization')


@launch_testing.post_shutdown_test()
class TestR2SingleInstanceShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # The launch shutdown ends the container with SIGINT
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[launch_testing.asserts.EXIT_OK, -2, 130])
