"""R2: glim_localization_node (a lifecycle node), standalone or as a component in a container.

    ros2 launch glim_localization_ros2 localization.launch.py map_path:=/path/to/map
    ros2 launch glim_localization_ros2 localization.launch.py map_path:=/path/to/map use_composition:=true rviz:=true

Arguments (`ros2 launch glim_localization_ros2 localization.launch.py --show-args` lists them):
  params_file      ROS parameters of the node (default: config/localization.param.yaml of this package)
  config_path      GLIM config directory; empty keeps the value of params_file
  map_path         map directory; empty keeps the value of params_file
  use_sim_time     use /clock (default false)
  autostart        configure and activate the node on start (default true)
  use_composition  load the node into a component container instead of running it as a process (default false)
  container_name   the container to create when use_composition is true (default glim_localization_container)
  rviz             start rviz2 with rviz/localization.rviz (default false; skipped with a warning if rviz2 is not installed)
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

PACKAGE = 'glim_localization_ros2'
NODE_NAME = 'glim_localization'


def to_bool(value):
    if value.lower() in ('true', '1', 'yes', 'on'):
        return True
    if value.lower() in ('false', '0', 'no', 'off'):
        return False
    raise ValueError(f"not a boolean: '{value}'")


def rviz_actions(enabled, use_sim_time, remappings=()):
    """rviz2 with the installed config, only when asked for and installed (rviz2 is not a dependency of this package)."""
    if not enabled:
        return []
    try:
        get_package_share_directory('rviz2')
    except PackageNotFoundError:
        return [LogInfo(msg='[WARN] rviz:=true but rviz2 is not installed; not starting it')]
    config = os.path.join(get_package_share_directory(PACKAGE), 'rviz', 'localization.rviz')
    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', config],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=list(remappings),
            output='screen',
        )
    ]


def launch_setup(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = to_bool(LaunchConfiguration('use_sim_time').perform(context))
    overrides = {'use_sim_time': use_sim_time, 'autostart': to_bool(LaunchConfiguration('autostart').perform(context))}
    for name in ('config_path', 'map_path'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value
    parameters = [params_file, overrides]

    if to_bool(LaunchConfiguration('use_composition').perform(context)):
        node = ComposableNodeContainer(
            name=LaunchConfiguration('container_name').perform(context),
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(package=PACKAGE, plugin='glim_localization_ros2::LocalizationNode', name=NODE_NAME, parameters=parameters),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        )
    else:
        node = Node(package=PACKAGE, executable='glim_localization_node', name=NODE_NAME, parameters=parameters, output='screen')

    return [node] + rviz_actions(to_bool(LaunchConfiguration('rviz').perform(context)), use_sim_time)


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory(PACKAGE), 'config', 'localization.param.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params, description='ROS parameters of glim_localization_node'),
        DeclareLaunchArgument('config_path', default_value='', description='GLIM config directory; empty keeps the value of params_file'),
        DeclareLaunchArgument('map_path', default_value='', description='map directory; empty keeps the value of params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='use /clock'),
        DeclareLaunchArgument('autostart', default_value='true', description='configure and activate the node on start'),
        DeclareLaunchArgument('use_composition', default_value='false', description='load the node into a component container'),
        DeclareLaunchArgument('container_name', default_value='glim_localization_container', description='container created when use_composition is true'),
        DeclareLaunchArgument('rviz', default_value='false', description='start rviz2 (if installed)'),
        OpaqueFunction(function=launch_setup),
    ])
