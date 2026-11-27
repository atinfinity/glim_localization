# ROS 2

Two integrations share the localization core: R1, a GLIM extension module loaded by glim_ros2's nodes (for evaluation), and R2, a dedicated lifecycle node (for deployment). Design details are in [DESIGN 6](DESIGN.md#6-ros-2-support). [Core and ROS 2 boundary](DESIGN.md#core-and-ros-2-boundary) lists what the ROS-independent core does and what the ROS 2 packages add.

The commands run inside the container after the build (see [Build and test](build.md#build-and-test)). Source the colcon install first: `source /ws/build/colcon/install/setup.bash`. Run one localization (R1 or R2) per process: a second one in the same process (for example a second `LocalizationNode` loaded into a component container) is refused with `one localization per process: already held by <owner>`. An R2 component can be unloaded and loaded again; R1 holds the claim until its process exits ([DESIGN](DESIGN.md#r2-node-differences-from-the-plan)).

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

`rviz:=true` starts `rviz2` with `rviz/localization.rviz` (map frame, `~/map`, `~/aligned_points`, `~/pose`, `~/odometry`, TF, and the 2D Pose Estimate tool on `/initialpose`). `rviz2` is not a dependency of this package: when it is not installed, the launch file prints a warning and starts without it.

[sim_eval.md](sim_eval.md#rviz2-capture) shows the R2 node in RViz2 on the Gazebo bag, and `scripts/sim_viz.sh` records it.

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
| `max_init_pose_delay` | 0.2 | Largest accepted delay [s] of a stamped initial pose behind the last input frame. Both are frame time (cloud header + `glim_ros.points_time_offset`) |
| `autostart` | false (`true` in the launch file) | Configure and activate once the node spins; a failed step is logged and leaves the node in the state it reached |

Everything else comes from the JSON files: input topics, QoS and time offsets from `glim_ros` in `config_ros.json` (default `/points` and `/imu`), the publisher settings from `localization_ros`, and the odometry settings from `config/r1/config_odometry_localization_cpu.json`. `config/r2/config.json` refers to the `r1` files, so copy the whole `config` directory to customize it.

Lifecycle: configure fails and leaves the node unconfigured on a missing config or an unreadable map. Input before activate is ignored. Deactivate stops processing; the next activate starts a new run. The services exist only while the node is configured and answer `success=false` while inactive.

## Topics, services and TF (R1 and R2)

| Name | Type | Notes |
| --- | --- | --- |
| points, imu (in) | `sensor_msgs/PointCloud2`, `sensor_msgs/Imu` | Topic names from `config_ros.json` |
| `/initialpose` (in) | `geometry_msgs/PoseWithCovarianceStamped` | Base pose in the map frame. Applied as given, or refined by the [yaw search](#initial-pose-yaw-search) when it is enabled. The stamp is frame time (cloud header + `glim_ros.points_time_offset`), the time base of `~/pose`, TF and RViz `now()`; it is not shifted, so a client with a raw cloud header stamp adds the offset itself |
| `~/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Base pose in the map frame, LiDAR rate. Covariance is zero (not estimated) |
| `~/odometry` | `nav_msgs/Odometry` | Pose and twist of the base |
| `~/map` | `sensor_msgs/PointCloud2` | The loaded map in the map frame, published once, latched (reliable, transient local). One centroid per `map_cloud_voxel` (0.5 m) voxel; `publish_map: false` turns it off. Built once (R1: at node start, R2: at configure) at 12 bytes per point; with `map_cloud_voxel: 0` a map above 10 million points gets a warning, and a cloud above 1 GiB is not published |
| `~/aligned_points` | `sensor_msgs/PointCloud2` | The current scan (deskewed) in the map frame, stamped with the frame, for each published `~/pose`; nothing in LOST. Sensor data QoS. Built only while the topic has a subscriber. `aligned_points_voxel` (0: every point) keeps one centroid per voxel; `publish_aligned_points: false` turns it off |
| `~/status` | `glim_localization_msgs/LocalizationStatus` | State, inlier ratio, tiles, map factors, residual, processing time, reset count and reason, result of the initial pose search |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | OK, WARN for DEGRADED, ERROR for LOST (the message says why the yaw search rejected the given pose) |
| `~/initialize` (R2) | `glim_localization_msgs/srv/Initialize` | Sets the pose. `success` means accepted, applied at the first frame at or after its stamp (0: the next frame; the stamp is frame time, as for `/initialpose`); the yaw search result comes later on `~/status`. `global_search=true` returns `success=false` ([#3](https://github.com/atinfinity/glim_localization/issues/3)) |
| `~/relocalize` (R2) | `std_srvs/srv/Trigger` | Restarts from the newest estimate; refused unless TRACKING or DEGRADED with an estimate no older than `max_init_pose_delay` and no reset pending |

States: `INITIALIZING` (started or reset) -> `TRACKING` -> `DEGRADED` (low inlier ratio, map weight lowered) -> `LOST` (left only by an external initialization). In LOST, no pose, odometry or `map->odom` is published; `odom->base_link` continues in `internal_odom`. Wheel odometry does not change these states; raise `lost_frames` to tolerate poor matching for longer ([DESIGN 5.4](DESIGN.md#54-quality-monitoring)).

`tf_mode` in `localization_ros`: `internal_odom` (default; publishes `map->odom` and `odom->base_link`), `external_odom` (publishes only `map->odom`, using `odom->base_link` from another source), `none` (no TF). Also set `base_frame_id` and `T_base_imu` there. Details are in [DESIGN 6](DESIGN.md#6-ros-2-support).

## Initial pose yaw search

A pose given from outside can have a large yaw error, for example from a 2D Pose Estimate click, and scan-to-map registration alone does not recover from one: on the synthetic map a 75 deg error ends 69 deg off while the state still says TRACKING. With `init_yaw_search: true`, the first scan after the pose is registered to the map from several yaw candidates around it, and the estimation starts from the registered pose with the highest inlier ratio.

The search runs for the configured `init_T_world_imu`, `/initialpose`, `~/initialize` and the replay tool's `--init-pose`. It does not run for `~/relocalize` or for the self-reset after an IMU gap, which start from the estimate. Keys under `odometry_estimation` in `config_odometry_localization_cpu.json` (and the GPU config):

| Key | Default | Meaning |
| --- | --- | --- |
| `init_yaw_search` | `false` | Turn the search on |
| `init_yaw_search_range_deg` | `180.0` | Candidates from -range to +range around the given yaw (180 covers the full turn) |
| `init_yaw_search_step_deg` | `15.0` | Spacing of the candidates; 180 / 15 gives 24 candidates |
| `init_yaw_search_min_inlier_ratio` | `0.7` | The best candidate must reach this inlier ratio (finest map voxels, after registration) |
| `init_yaw_search_min_score_gap` | `0.05` | The best must beat the runner-up (the best candidate more than two steps, at most 90 deg, away in yaw) by more than this |
| `init_yaw_search_time_budget` | `10.0` | Seconds for the whole search; a search cut short by it is rejected |
| `init_yaw_search_max_iterations` | `10` | Levenberg-Marquardt iterations of each registration |
| `init_yaw_search_min_points` | `100` | Frames with fewer points are dropped while the search is pending |
| `init_yaw_search_max_translation` | `0.0` | Position uncertainty of the given pose [m] in xy; 0 is unbounded. Only registered poses within it are accepted, and a better match beyond it rejects the pose (OUT_OF_RANGE) |

The given pose is accepted only when all three hold (four with `init_yaw_search_max_translation`, below): every candidate was scored within the time budget, the best reaches the minimum inlier ratio, and the runner-up is more than the minimum score gap below it. Otherwise it is rejected: the state goes to LOST, frames are dropped and nothing is published until a new pose is given. A truncated search is rejected because the candidates not scored could have been better or equally good; a close runner-up means the scene looks alike from two yaws (for example a symmetric room), so the search cannot choose. With the search range set to 0 only the given yaw is scored, which turns the search into a check of the given pose. The gap check needs candidates more than two steps (at most 90 deg) apart, so with range 0 or a small range only the minimum inlier ratio applies: a pose that registers well at a wrong yaw is then not caught by it.

`init_yaw_search_max_translation` states how far off the given position may be. All candidates start at the given position, so a position several metres off can register into a wrong pose elsewhere that passes the checks above. With the bound, the best and the runner-up are taken only among registered poses within it, and the pose is rejected (OUT_OF_RANGE) when one beyond it reaches the minimum inlier ratio and scores at least as high. Choose it as the real uncertainty of the given pose: a pose further off than the bound is rejected even if the search would have found it, and a bound as large as the distance a wrong pose moves lets that pose back in (on the synthetic corridor 2.5 m works and 3.0 m does not).

`~/status` reports the result (`init_search`: NONE, ACCEPTED or REJECTED), the reason for a rejection (`init_search_reason`: NONE, TRUNCATED, LOW_SCORE, AMBIGUOUS, NO_CANDIDATE or OUT_OF_RANGE), the candidates scored and generated (`init_search_candidates`, `init_search_candidates_total`), the best inlier ratio, the yaw correction applied [rad], the runner-up's inlier ratio and yaw (for OUT_OF_RANGE, the match beyond the bound and the best within it), and the search time; `/diagnostics` (whose message gives the reason with the numbers that matter for it) and the replay `summary.json` (with `reason` and `truncated`) carry the same values.

> [!NOTE]
> The search runs on the first scan with enough points, in the estimation thread, so the first pose is published later by the search time. Scans that arrive meanwhile are queued. Measured on the synthetic data with 2 threads: about 30 ms per candidate on arm64 (24 candidates in about 0.8 s) and 70 to 80 ms per candidate on x86 (1.7 to 1.9 s), with the CPU module and with the GPU module on an RTX 2080 Ti (the search uses CPU factors in both). Real scans are larger and their timing is not measured; the 10 s default leaves room for that and for a loaded machine. If `init_search_candidates` is below `init_search_candidates_total`, raise the budget or narrow the range. Only the yaw is searched; position, roll and pitch come from the given pose and are refined by the registration.
