"""R1: glim_ros2's glim_rosnode (live topics) or glim_rosbag (a bag) with the localization extension.

    ros2 launch glim_localization_ros2 localization_r1.launch.py map_path:=/path/to/map
    ros2 launch glim_localization_ros2 localization_r1.launch.py map_path:=/path/to/map bag:=/path/to/bag rviz:=true

The node is GLIM's (node name glim_ros), so the outputs are /glim_ros/pose, /glim_ros/map and so on.

Arguments (`ros2 launch glim_localization_ros2 localization_r1.launch.py --show-args` lists them):
  bag           a bag to read with glim_rosbag; empty runs glim_rosnode on live topics (default empty)
  config_path   GLIM config directory with the extension enabled (default: config/r1 of this package)
  map_path      map directory; empty keeps odometry_estimation.map_path of the config. Otherwise the config is copied
                to a temporary directory (removed on exit) with the map path filled in
  params_file   ROS parameters of the GLIM node (default: config/localization_r1.param.yaml of this package)
  use_sim_time  use /clock (default false)
  rviz          start rviz2 with rviz/localization.rviz (default false; skipped with a warning if rviz2 is not installed)
"""
import atexit
import importlib.util
import json
import os
import shutil
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_r2_launch():
    """localization.launch.py (next to this file) as a module, for its rviz_actions and to_bool."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "localization.launch.py")
    spec = importlib.util.spec_from_file_location("glim_localization_r2_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_r2_launch = _load_r2_launch()
rviz_actions, to_bool = _r2_launch.rviz_actions, _r2_launch.to_bool

PACKAGE = "glim_localization_ros2"
ODOMETRY_CONFIG = "config_odometry_localization_cpu.json"
# The RViz config shows the R2 node's topics; GLIM's node is glim_ros
RVIZ_REMAPPINGS = [(f"/glim_localization/{topic}", f"/glim_ros/{topic}") for topic in ("map", "pose", "odometry")]


def config_with_map(config_path, map_path):
    """A temporary copy of config_path whose odometry config loads map_path."""
    work_dir = tempfile.mkdtemp(prefix="glim_localization_r1_")
    atexit.register(shutil.rmtree, work_dir, ignore_errors=True)
    dst = os.path.join(work_dir, "config")
    shutil.copytree(config_path, dst)
    path = os.path.join(dst, ODOMETRY_CONFIG)
    with open(path) as f:
        # The shipped configs carry // comments, which GLIM's loader accepts and json does not. No value contains //.
        config = json.loads("\n".join(line.split("//")[0] for line in f))
    config["odometry_estimation"]["map_path"] = os.path.abspath(map_path)
    with open(path, "w") as f:
        json.dump(config, f, indent=2)
    return dst


def read_params_file(path):
    """The ros__parameters of params_file for GLIM's node: those under /** and then those under glim_ros."""
    with open(path) as f:
        content = yaml.safe_load(f) or {}
    params = {}
    for key in ("/**", "glim_ros", "/glim_ros"):
        params.update((content.get(key) or {}).get("ros__parameters") or {})
    return params


def to_param_value(value):
    """A value as `-p name:=value` reads it (YAML)."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return repr(value)
    if isinstance(value, str):
        return value
    raise ValueError(f"unsupported parameter value in params_file: {value!r}")


def launch_setup(context):
    config_path = os.path.abspath(LaunchConfiguration("config_path").perform(context))
    map_path = LaunchConfiguration("map_path").perform(context)
    if map_path:
        config_path = config_with_map(config_path, map_path)
    bag = LaunchConfiguration("bag").perform(context)
    use_sim_time = to_bool(LaunchConfiguration("use_sim_time").perform(context))

    params = read_params_file(LaunchConfiguration("params_file").perform(context))
    # config_path last: GlimROS would take a relative path from glim's share directory
    params.update({"config_path": config_path, "use_sim_time": use_sim_time})
    node = Node(
        package="glim_ros",
        executable="glim_rosbag" if bag else "glim_rosnode",
        arguments=[bag] if bag else [],
        # Not `parameters`: launch_ros passes those as --params-file <a file>, and glim_rosbag reads every argument
        # that names an existing file as a bag. `-p name:=value` names no file.
        ros_arguments=[arg for name, value in params.items() for arg in ("-p", f"{name}:={to_param_value(value)}")],
        output="screen",
    )
    return [node] + rviz_actions(to_bool(LaunchConfiguration("rviz").perform(context)), use_sim_time, RVIZ_REMAPPINGS)


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    return LaunchDescription([
        DeclareLaunchArgument("bag", default_value="", description="bag for glim_rosbag; empty runs glim_rosnode"),
        DeclareLaunchArgument("config_path", default_value=os.path.join(share, "config", "r1"), description="GLIM config directory"),
        DeclareLaunchArgument("map_path", default_value="", description="map directory; empty keeps the config's"),
        DeclareLaunchArgument(
            "params_file", default_value=os.path.join(share, "config", "localization_r1.param.yaml"), description="ROS parameters of the GLIM node"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="use /clock"),
        DeclareLaunchArgument("rviz", default_value="false", description="start rviz2 (if installed)"),
        OpaqueFunction(function=launch_setup),
    ])
