# ROS 2

Two integrations share the localization core: R1, a GLIM extension module loaded by glim_ros2's nodes (for evaluation), and R2, a dedicated lifecycle node (for deployment). Design details are in [DESIGN 6](DESIGN.md#6-ros-2-support).

The commands run inside the container after the build (see [Build and test](build.md#build-and-test)). Source the colcon install first: `source /ws/build/colcon/install/setup.bash`. Run one localization (R1 or R2) per process.

## Launch files

```bash
# R2 (deployment): the lifecycle node, configured and activated on start (autostart)
ros2 launch glim_localization_ros2 localization.launch.py map_path:=/ws/out/map
ros2 launch glim_localization_ros2 localization.launch.py map_path:=/ws/out/map use_composition:=true   # as a component
# R1 (evaluation): glim_rosnode, or glim_rosbag with bag:=
ros2 launch glim_localization_ros2 localization_r1.launch.py map_path:=/ws/out/map bag:=/ws/out/my_bag
```

| Argument | R2 `localization.launch.py` | R1 `localization_r1.launch.py` |
| --- | --- | --- |
| `params_file` | `config/localization.param.yaml` (every node parameter, with comments) | `config/localization_r1.param.yaml` (the GLIM node's parameters, with comments) |
| `config_path` | empty: the value in `params_file` (`config/r2`; relative: from the package share) | `share/glim_localization_ros2/config/r1` (relative: from the current directory) |
| `map_path` | empty: the value in `params_file`, then the JSON config | empty: the JSON config; otherwise a temporary copy of the config with this map in the odometry config that `config.json` names (removed when the launch ends normally) |
| `use_sim_time` | `false` | `false` |
| `rviz` | `false` | `false` (the RViz topics are remapped to `/glim_ros/...`) |
| others | `autostart` (`true`), `use_composition` (`false`), `container_name` | `bag` (empty: `glim_rosnode`) |

R1 passes the `params_file` entries (under `/**`, then `glim_ros`) to GLIM's node as typed `-p name:=value`, because `glim_rosbag` would read a `--params-file` path as a bag: strings stay strings, integers of the double parameters become doubles, nested mappings are rejected. In the R2 `params_file`, write doubles with a decimal point (`1.0`, not `1`).

`rviz:=true` starts `rviz2` with `rviz/localization.rviz` (map frame, `~/map`, `~/pose`, `~/odometry`, TF, and the 2D Pose Estimate tool on `/initialpose`). `rviz2` is not a dependency of this package: when it is not installed, the launch file prints a warning and starts without it.

For the GPU module (CUDA image), use the `config/r1_gpu` and `config/r2_gpu` configs; see [CUDA variant](build.md#cuda-variant).

## R1: GLIM extension (evaluation)

`libglim_localization_extension.so` is loaded by glim_ros2's `glim_rosnode` / `glim_rosbag`, using the config in `share/glim_localization_ros2/config/r1`. Copy it and set `odometry_estimation.map_path` in `config_odometry_localization_cpu.json` (and `T_lidar_imu` in `config_sensors.json`):

```bash
cp -r "$(ros2 pkg prefix glim_localization_ros2)/share/glim_localization_ros2/config/r1" /ws/out/r1_config
# edit /ws/out/r1_config/config_odometry_localization_cpu.json: "map_path": "/ws/out/map"
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/ws/out/r1_config
ros2 run glim_ros glim_rosbag <bag_dir> --ros-args -p config_path:=/ws/out/r1_config   # offline
```

Topics are on the GLIM node (`/glim_ros/pose`, ...). The initial pose comes from `/initialpose` (base pose in the map frame), or from `init_T_world_imu` (IMU pose in the map frame, `[x, y, z, qx, qy, qz, qw]`) added under `odometry_estimation` in `config_odometry_localization_cpu.json`; the shipped config does not set it. R1 has no services.

> [!WARNING]
> Do not add GLIM's `librviz_viewer.so` or `libstandard_viewer.so` to `extension_modules`: they publish a conflicting TF.

## R2: lifecycle node (deployment)

`glim_localization_node` (node name `glim_localization`, also a component `glim_localization_ros2::LocalizationNode`):

```bash
ros2 run glim_localization_ros2 glim_localization_node --ros-args -p map_path:=/ws/out/map   # or the launch file
ros2 lifecycle set /glim_localization configure   # reads the config, loads the map, creates the services
ros2 lifecycle set /glim_localization activate    # starts the odometry and the subscriptions
ros2 service call /glim_localization/initialize glim_localization_msgs/srv/Initialize \
  "{pose: {header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}}"
```

ROS parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `config_path` | `config/r2` | GLIM-style config directory; a relative path is taken from the package share |
| `map_path` | (empty) | GLIM dump or compiled map; overrides `odometry_estimation.map_path` |
| `max_init_pose_delay` | 0.2 | Largest accepted delay [s] of a stamped initial pose behind the last input frame |
| `autostart` | false (`true` in the launch file) | Configure and activate once the node spins; a failed step is logged and leaves the node in the state it reached |

Everything else comes from the JSON files: input topics, QoS and time offsets from `glim_ros` in `config_ros.json` (default `/points` and `/imu`), the publisher settings from `localization_ros`, and the odometry settings from `config/r1/config_odometry_localization_cpu.json`. `config/r2/config.json` refers to the `r1` files, so copy the whole `config` directory to customize it.

Lifecycle: configure fails and leaves the node unconfigured on a missing config or an unreadable map. Input before activate is ignored. Deactivate stops processing; the next activate starts a new run. The services exist only while the node is configured and answer `success=false` while inactive.

## Topics, services and TF (R1 and R2)

| Name | Type | Notes |
| --- | --- | --- |
| points, imu (in) | `sensor_msgs/PointCloud2`, `sensor_msgs/Imu` | Topic names from `config_ros.json` |
| `/initialpose` (in) | `geometry_msgs/PoseWithCovarianceStamped` | Base pose in the map frame, applied as given (no yaw search) |
| `~/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Base pose in the map frame, LiDAR rate. Covariance is zero (not estimated) |
| `~/odometry` | `nav_msgs/Odometry` | Pose and twist of the base |
| `~/map` | `sensor_msgs/PointCloud2` | The loaded map in the map frame, published once, latched (reliable, transient local). One centroid per `map_cloud_voxel` (0.5 m) voxel; `publish_map: false` turns it off. Built once (R1: at node start, R2: at configure) at 12 bytes per point; with `map_cloud_voxel: 0` a map above 10 million points gets a warning, and a cloud above 1 GiB is not published |
| `~/status` | `glim_localization_msgs/LocalizationStatus` | State, inlier ratio, tiles, map factors, residual, processing time, reset count and reason |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | OK, WARN for DEGRADED, ERROR for LOST |
| `~/initialize` (R2) | `glim_localization_msgs/srv/Initialize` | Sets the pose. `success` means accepted, applied at the next frame. `global_search=true` returns `success=false` ([#3](https://github.com/atinfinity/glim_localization/issues/3)) |
| `~/relocalize` (R2) | `std_srvs/srv/Trigger` | Restarts from the newest estimate; refused unless TRACKING or DEGRADED with an estimate no older than `max_init_pose_delay` and no reset pending |

States: `INITIALIZING` (started or reset) -> `TRACKING` -> `DEGRADED` (low inlier ratio, map weight lowered) -> `LOST` (left only by an external initialization). In LOST, no pose, odometry or `map->odom` is published; `odom->base_link` continues in `internal_odom`.

`tf_mode` in `localization_ros`: `internal_odom` (default; publishes `map->odom` and `odom->base_link`), `external_odom` (publishes only `map->odom`, using `odom->base_link` from another source), `none` (no TF). Also set `base_frame_id` and `T_base_imu` there. Details are in [DESIGN 6](DESIGN.md#6-ros-2-support).
