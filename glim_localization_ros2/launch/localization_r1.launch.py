# Copyright (c) 2026 atinfinity
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
R1: glim_ros2's glim_rosnode (live topics) or glim_rosbag (a bag) with the localization extension.

    ros2 launch glim_localization_ros2 localization_r1.launch.py map_path:=<map>
    ros2 launch glim_localization_ros2 localization_r1.launch.py map_path:=<map> bag:=<bag>
    ros2 launch glim_localization_ros2 localization_r1.launch.py map_path:=<map> rviz:=true

The node is GLIM's (node name glim_ros), so the outputs are /glim_ros/pose, /glim_ros/map and so
on. This launch file needs glim_ros (glim_ros2) installed.

Arguments (`ros2 launch glim_localization_ros2 localization_r1.launch.py --show-args` lists them):
  bag           a bag to read with glim_rosbag; empty runs glim_rosnode on live topics (default
                empty)
  config_path   GLIM config directory with the extension enabled (default: config/r1 of this
                package). A relative path is taken from the current directory
  map_path      map directory; empty keeps odometry_estimation.map_path of the config. Otherwise
                the config is copied to a temporary directory with the map path filled in, in the
                odometry config that config.json names (global.config_odometry). The copy is
                removed when the launch ends normally
  params_file   ROS parameters of the GLIM node (default: config/localization_r1.param.yaml of
                this package). The entries under /** and then glim_ros are passed as typed
                `-p name:=value` (see launch_utils.py)
  use_sim_time  use /clock (default false)
  rviz          start rviz2 with rviz/localization.rviz (default false; skipped with a warning if
                rviz2 is not installed)
"""
import atexit
import importlib.util
import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load(file_name, module_name):
    """Load a file next to this one as a module (launch files are not on sys.path)."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), file_name)
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_r2_launch = _load('localization.launch.py', 'glim_localization_r2_launch')
rviz_actions, to_bool = _r2_launch.rviz_actions, _r2_launch.to_bool
launch_utils = _load('launch_utils.py', 'glim_localization_launch_utils')

PACKAGE = 'glim_localization_ros2'
# The RViz config shows the R2 node's topics; GLIM's node is glim_ros
RVIZ_REMAPPINGS = [
    (f'/glim_localization/{topic}', f'/glim_ros/{topic}') for topic in ('map', 'pose', 'odometry')]


def launch_setup(context):
    config_path = os.path.abspath(LaunchConfiguration('config_path').perform(context))
    map_path = LaunchConfiguration('map_path').perform(context)
    if map_path:
        work_dir = tempfile.mkdtemp(prefix='glim_localization_r1_')
        # Removed when the launch process exits, after it has reaped GLIM (which may still read the
        # config while it shuts down). Not at the launch shutdown event: that fires when the
        # processes are only being signalled. A killed launch leaks it.
        atexit.register(shutil.rmtree, work_dir, ignore_errors=True)
        config_path = launch_utils.config_with_map(config_path, map_path, work_dir)
    bag = LaunchConfiguration('bag').perform(context)
    use_sim_time = to_bool(LaunchConfiguration('use_sim_time').perform(context))

    params = launch_utils.read_params_file(LaunchConfiguration('params_file').perform(context))
    # config_path last: GlimROS would take a relative path from glim's share directory
    params.update({'config_path': config_path, 'use_sim_time': use_sim_time})
    node = Node(
        package='glim_ros',
        executable='glim_rosbag' if bag else 'glim_rosnode',
        arguments=[bag] if bag else [],
        # Not `parameters`: launch_ros passes those as --params-file <a file>, and glim_rosbag
        # reads every argument that names an existing file as a bag. `-p name:=value` names no
        # file.
        ros_arguments=launch_utils.to_ros_arguments(params),
        output='screen',
    )
    rviz = to_bool(LaunchConfiguration('rviz').perform(context))
    return [node] + rviz_actions(rviz, use_sim_time, RVIZ_REMAPPINGS)


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag', default_value='', description='bag for glim_rosbag; empty runs glim_rosnode'),
        DeclareLaunchArgument(
            'config_path', default_value=os.path.join(share, 'config', 'r1'),
            description='GLIM config directory (a relative path is from the current directory)'),
        DeclareLaunchArgument(
            'map_path', default_value='', description="map directory; empty keeps the config's"),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share, 'config', 'localization_r1.param.yaml'),
            description='ROS parameters of the GLIM node'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='use /clock'),
        DeclareLaunchArgument(
            'rviz', default_value='false', description='start rviz2 (if installed)'),
        OpaqueFunction(function=launch_setup),
    ])
