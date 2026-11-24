"""
Shared parts of the tests of the installed launch files (test_launch_*.py).

Not a test module itself.
"""
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import r2_helpers  # the test files put this directory on sys.path
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

MAP_TIMEOUT_S = 60.0  # fails the test; not a success path
MIN_POSE_RATIO = 0.9  # as in the R1 and R2 tests
# Points of ~/map for the synthetic fixture map of scripts/test_colcon.sh (5 tiles) at the shipped
# map_cloud_voxel 0.5: 9245 on arm64, 9246 on amd64. The fixture floor (z = 0) and walls
# (y = +-5 m) lie on 0.5 m voxel planes with 1 cm noise, so a few points sit within float rounding
# of a voxel boundary, and the FMA contraction GCC does on aarch64 (not on baseline x86-64) moves
# one of them across. Hence a band: 1% is far below the change from another voxel size (0.4 m gives
# about 1.5 times the voxels on these surfaces) or from the raw map, and still catches another map.
EXPECTED_MAP_POINTS = 9245
MAP_POINTS_TOLERANCE = 0.01


def include_launch(file_name, arguments):
    """
    Include the installed launch/<file_name> of glim_localization_ros2, headless.

    `arguments` is {name: str}.
    """
    path = os.path.join(get_package_share_directory('glim_localization_ros2'), 'launch', file_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=dict(arguments, rviz='false').items())


def wait_for_latched_map(node, topic, timeout=MAP_TIMEOUT_S):
    """
    Subscribe to `topic` as RViz does (reliable, transient local) and return the first cloud.

    Raises on timeout. Called after the map was published: only a latched publisher delivers it.
    """
    received = []
    qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = node.create_subscription(PointCloud2, topic, received.append, qos)
    try:
        end = time.monotonic() + timeout
        while not received:
            if time.monotonic() > end:
                raise AssertionError(f'no latched map on {topic} within {timeout} s')
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(sub)
    return received[0]


def check_map(test, cloud):
    """Check the latched ~/map of the fixture map at the shipped map_cloud_voxel (0.5 m)."""
    test.assertEqual(cloud.header.frame_id, 'map')
    test.assertAlmostEqual(
        cloud.width * cloud.height, EXPECTED_MAP_POINTS,
        delta=EXPECTED_MAP_POINTS * MAP_POINTS_TOLERANCE,
        msg='not the fixture map at map_cloud_voxel 0.5')
    test.assertEqual([f.name for f in cloud.fields], ['x', 'y', 'z'])
    test.assertEqual(len(cloud.data), cloud.row_step * cloud.height)


def r2_launch(use_composition):
    """
    Include the installed R2 launch file.

    With the test config (reliable inputs), the fixture map, /clock and autostart.
    """
    r2_helpers.require_env('GLIM_R1_BAG')
    r2_helpers.require_env('GLIM_R1_DATA')
    arguments = {
        'config_path': r2_helpers.make_config_dir(),
        'map_path': r2_helpers.require_env('GLIM_R1_MAP'),
        'use_sim_time': 'true',
        'autostart': 'true',
        'use_composition': 'true' if use_composition else 'false',
    }
    return include_launch('localization.launch.py', arguments)


def check_r2_launch(test, client, proc_output):
    """
    Check that autostart reaches active and publishes ~/map, and the bag gives ~/pose and TF.

    The TF composes to the pose.

    The bag is resumed only after the node has logged a matched publisher on each bag topic: the
    first pose comes from the bag's /initialpose at 0 s, and when it was lost the node localized
    only from the next one at 5 s (issue #20).
    """
    client.wait_until(
        lambda: client.state() == r2_helpers.State.PRIMARY_STATE_ACTIVE,
        'the node did not become active (autostart)')
    # Published at configure, before this subscription
    check_map(test, wait_for_latched_map(client.node, f'{r2_helpers.NODE}/map'))

    client.wait_for_subscribers(['/imu', '/points', '/initialpose'])
    client.play_and_wait(
        r2_helpers.require_env('GLIM_R1_BAG'), r2_helpers.BAG_TOPICS,
        node_matched=r2_helpers.wait_for_node_matched(proc_output))
    num_frames = len(client.gt)
    print(f'{len(client.poses)} poses for {num_frames} frames', flush=True)
    test.assertGreaterEqual(
        len(client.poses), MIN_POSE_RATIO * num_frames,
        f'{len(client.poses)} poses for {num_frames} frames; {client.coverage_report()}')
    test.assertTrue(all(p.header.frame_id == 'map' for p in client.poses))
    pairs = client.tf_pairs()
    test.assertGreaterEqual(
        len(pairs), MIN_POSE_RATIO * num_frames,
        f'{len(pairs)} stamps with map->odom and odom->base_link')
    test.assertTrue(
        client.tf_composes_to_pose(pairs, client.poses[-1]),
        'TF does not compose to the last pose')
