# Getting started

This guide takes you from a fresh clone to localization on your own map: build the Docker image, run the tests, replay a synthetic dataset, prepare a map, replay a real bag, and run live on ROS 2. Each step links to the page with the full reference.

> [!NOTE]
> Steps 1 to 4 need nothing but this repository. Steps 5 to 7 need your own data: a GLIM dump or a PCD map, and a rosbag2 or a live LiDAR with an IMU.

## 1. Prerequisites

- A host with Docker and Docker Compose. Ubuntu 24.04 on amd64 is the main target. macOS on Apple silicon (arm64) works with the CPU image.
- A GPU is optional. The default image is CPU only. The CUDA image needs a Linux amd64 host with an NVIDIA GPU, a driver that supports CUDA 13.1 or later, and the NVIDIA Container Toolkit (see [CUDA variant](build.md#cuda-variant)).
- For real data: a LiDAR with an IMU. LiDAR-only input is not supported.

Everything builds and runs inside an Ubuntu 24.04 + ROS 2 Jazzy image. You do not need ROS, GLIM or GTSAM on the host, and builds outside this image are not supported. The image is headless (no GLIM viewer, no RViz); visualize from another machine on the same ROS 2 network if needed (see [Networking](#networking) for what that takes).

## 2. Clone and build the image

```bash
git clone https://github.com/atinfinity/glim_localization.git
cd glim_localization
docker compose -f docker/compose.yaml build      # build the image (glim_localization:dev)
```

The image build installs GTSAM and gtsam_points and compiles GLIM and glim_ros2, all at pinned versions ([DEPENDENCIES.md](DEPENDENCIES.md)), so the first build takes a while. The compose file passes `JOBS` (default 4) to the build as the number of parallel jobs, for example `JOBS=8 docker compose -f docker/compose.yaml build`.

All later commands run from the repository root. `scripts/docker_run.sh <command>` runs a command in the container, and `scripts/docker_run.sh` with no arguments opens a shell there. In the container:

| Container path | What it is |
| --- | --- |
| `/ws/src/glim_localization` | This repository (the working directory) |
| `/ws/build` | Build outputs, in a Docker volume that persists between runs |
| `/ws/out` | The host directory `out/`: put your maps, bags and datasets here, and read the results here |

## 3. Build and run the tests

```bash
scripts/docker_run.sh scripts/ci.sh              # dependency check, build, all tests
```

`scripts/ci.sh` checks the dependencies, builds the core into `/ws/build/core` and all packages into `/ws/build/colcon/install`, and runs every test, including the synthetic replay of step 4. It is the same script that GitHub Actions runs. To build without the full test run, call the build scripts in this order:

```bash
scripts/docker_run.sh scripts/build_core.sh      # core with plain CMake and its tests
scripts/docker_run.sh scripts/build_colcon.sh    # all packages (reuses the core build)
```

The individual scripts, ccache and the CUDA variant are described in [Build and test](build.md).

## 4. Run the synthetic end-to-end replay

```bash
scripts/docker_run.sh scripts/test_replay_e2e.sh
```

The script generates a 12 s synthetic map and dataset, localizes from an offset initial pose on the GLIM dump and on a PCD map, and checks accuracy (0.1 m, 1 deg after 3 s), determinism and exit codes. It ends with `PASS: replay e2e`. In a CPU build it also prints `SKIPPED: GPU replay`, which is expected.

The results are in `out/phase4_e2e/` on the host. The main outputs:

| Path | Content |
| --- | --- |
| `input/data/` | The synthetic replay dataset, with the ground truth `traj_gt.txt` |
| `input/map/`, `input/map.pcd` | The synthetic map as a GLIM dump and as a PCD file |
| `run1/` | Replay on the GLIM dump: `traj_localization.txt` (TUM), `frames.csv`, `summary.json`, `replay_args.json` |
| `run1/eval.txt`, `run1/eval.json` | The evaluation: ATE and RPE (translation and rotation) and processing time |
| `pcd_map/`, `run_pcd/` | The PCD compiled by `map_compiler`, and the replay and evaluation on it |
| `run1.log`, `run_pcd.log`, ... | The replay output of each run (`pcd_map.log` is the `map_compiler` output) |

To see the trajectory error as a plot, run the evaluation again with `--plot` inside the container; it writes `eval.png` next to `eval.txt`:

```bash
alias py='env -u PYTHONPATH $GLIM_LOC_PYTHON'
py scripts/evaluate.py --est /ws/out/phase4_e2e/run1/traj_localization.txt --gt /ws/out/phase4_e2e/input/data/traj_gt.txt \
  --frames /ws/out/phase4_e2e/run1/frames.csv --out /ws/out/phase4_e2e/run1 --settle 3 --plot
```

The dataset format and the outputs are described in [Offline replay and evaluation](replay.md).

## 5. Prepare a map from your own data

> [!NOTE]
> This step needs your own map data. Copy it under `out/` on the host so that the container sees it under `/ws/out`.

Localization takes either a GLIM dump directory (from a GLIM mapping run) or a map compiled by `map_compiler`. A GLIM dump can be used as it is, or compiled; a PCD file must be compiled. Inside the container:

```bash
# From a GLIM dump (or a compiled map)
/ws/build/core/tools/map_compiler /ws/out/glim_dump /ws/out/map

# From a PCD file (x, y, z in the map frame)
/ws/build/core/tools/map_compiler /ws/out/map.pcd /ws/out/map --grid-size 50 --voxel-size 0.1
```

PCD input reads `DATA ascii` and `binary` with x, y, z as F4 or F8. Localization uses up to `map_max_submaps` (default 5) tiles nearest to the robot, so for long-range LiDAR use `--grid-size` 50 to 100. The options are in [Map preparation](maps.md).

## 6. Replay a real bag and evaluate it

> [!NOTE]
> This step needs a rosbag2 (sqlite3 or mcap) with a `sensor_msgs/PointCloud2` and a `sensor_msgs/Imu` topic, recorded in the area of your map, and an initial pose in the map frame. A reference trajectory is needed only for the evaluation.

First copy the replay config and set `T_lidar_imu` in its `config_sensors.json` for your sensor; the shipped configs use identity:

```bash
cp -r glim_localization/config /ws/out/my_config
# edit /ws/out/my_config/config_sensors.json: "T_lidar_imu": [x, y, z, qx, qy, qz, qw]
```

The container runs as root, so on a Linux host the files it writes under `out/` are owned by root: edit them inside the container, or use `sudo` on the host.

Then convert, localize and evaluate inside the container:

```bash
alias py='env -u PYTHONPATH $GLIM_LOC_PYTHON'

# 1. Convert a rosbag2 (sqlite3 or mcap) to a replay dataset
py scripts/bag_to_replay.py <bag_dir> --out /ws/out/dataset [--points-topic /points] [--imu-topic /imu] [--acc-scale 9.80665] [--gt traj.txt]

# 2. Localize
/ws/build/core/tools/glim_localization_replay --data /ws/out/dataset --config /ws/out/my_config \
  --map /ws/out/map --out /ws/out/run --init-pose x y z qx qy qz qw [--init-stamp <s>]

# 3. Evaluate against a reference trajectory (TUM, T_world_imu); traj_gt.txt is written only by --gt in step 1
py scripts/evaluate.py --est /ws/out/run/traj_localization.txt --gt /ws/out/dataset/traj_gt.txt \
  --frames /ws/out/run/frames.csv --out /ws/out/run [--settle 3] [--max-trans 0.1] [--max-rot-deg 1] [--plot]
```

- `--init-pose` is `T_world_imu`, the IMU pose in the map frame. The initial pose is applied exactly, so it must be close to the true pose, unless the [yaw search](ros2.md#initial-pose-yaw-search) is enabled in the config (it corrects the yaw, not the position).
- `--acc-scale` is required when the accelerations look like units of g.
- The replay exits with 0 when it succeeds. 1 is a runtime failure (including LOST or a low tracking ratio), 2 bad arguments, config, map or initial pose, 3 an invalid dataset (including IMU gaps), and 4 no pose.
- Without a reference trajectory, skip step 3 and inspect `traj_localization.txt`, `frames.csv` and `summary.json`.

All options are in [Offline replay and evaluation](replay.md).

## 7. Run live on ROS 2

> [!NOTE]
> This step needs a LiDAR and an IMU publishing on ROS 2, or a recorded bag, and the map from step 5.

The R2 lifecycle node is the deployment path. By default it subscribes to `/points` and `/imu` and publishes the base frame `base_link`. To change the topics, frames, `T_base_imu` or `T_lidar_imu`, copy the whole `config` directory, because `config/r2` refers to the `r1` files:

```bash
source /ws/build/colcon/install/setup.bash
cp -r "$(ros2 pkg prefix glim_localization_ros2)/share/glim_localization_ros2/config" /ws/out/ros_config
# edit /ws/out/ros_config/r2/config_ros.json: imu_topic, points_topic and localization_ros (base_frame_id, T_base_imu, tf_mode)
# edit /ws/out/ros_config/r1/config_sensors.json: T_lidar_imu
```

Start the node with the launch file. It configures and activates the node on start (`autostart`):

```bash
ros2 launch glim_localization_ros2 localization.launch.py map_path:=/ws/out/map config_path:=/ws/out/ros_config/r2
```

Leave out `config_path` to use the shipped `config/r2`. To feed a recorded bag instead of a live sensor, also add `use_sim_time:=true` and follow [Play a recorded bag](#play-a-recorded-bag) below.

`scripts/docker_run.sh` starts a new container each time, and the container sources only ROS 2 and GLIM, not this repository's packages. In every additional shell, run `source /ws/build/colcon/install/setup.bash` before the commands below.

### Networking

Containers started by `scripts/docker_run.sh` share the project network that Compose creates (for example `glim_localization_default`, a bridge network, not host networking), so ROS 2 nodes in two such shells see each other. DDS discovery with other machines, such as a live sensor driver or RViz on another host, usually needs host networking (for example a compose override with `network_mode: host` on Linux) or a DDS setup with unicast peers or a discovery server.

### Set the initial pose

The node needs an initial pose of the base frame in the map frame; input frames that arrive before it are dropped. Either:

- call the `~/initialize` service, which replies whether the pose was accepted. A zero stamp (as below) applies the pose at the next frame:

  ```bash
  source /ws/build/colcon/install/setup.bash
  ros2 service call /glim_localization/initialize glim_localization_msgs/srv/Initialize \
    "{pose: {header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}}"
  ```

- or publish `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`), for example with the 2D Pose Estimate tool in RViz. `rviz:=true` starts `rviz2` with `rviz/localization.rviz` where `rviz2` is installed; the image has no RViz, so run RViz on another machine on the same ROS 2 network (see [Networking](#networking)).

With the shipped config the pose is applied exactly; enable the [yaw search](ros2.md#initial-pose-yaw-search) to correct a large yaw error. Right after the node becomes active, the first messages can be missed, so prefer `~/initialize`, or resend `/initialpose`.

### Play a recorded bag

Put the bag under `out/` on the host so that the container sees it under `/ws/out`. Because frames before the initial pose are dropped and a zero-stamp `~/initialize` applies at the next frame, use this order:

1. Launch the node with `use_sim_time:=true` (above).
2. Call `~/initialize` with the pose at the start of the bag.
3. In another `scripts/docker_run.sh` shell, play the bag with its clock:

   ```bash
   source /ws/build/colcon/install/setup.bash
   ros2 bag play /ws/out/<bag_dir> --clock
   ```

### Check the topics and TF

```bash
source /ws/build/colcon/install/setup.bash
ros2 lifecycle get /glim_localization           # active [3]
ros2 topic echo /glim_localization/status       # state: 0 INITIALIZING, 1 TRACKING, 2 DEGRADED, 3 LOST
ros2 topic hz /glim_localization/pose           # base pose in the map frame, at the LiDAR rate
ros2 run tf2_ros tf2_echo map base_link         # map->odom->base_link (tf_mode internal_odom)
```

The node also publishes `~/odometry`, the map once on `~/map` (latched) and `/diagnostics`. In `LOST`, no pose, odometry or `map->odom` is published until you initialize again. All topics, services, parameters and the R1 integration are in [ROS 2](ros2.md).

## 8. Where to go next

- [Build and test](build.md): the CI scripts, ccache and the CUDA image with the GPU odometry module.
- [Map preparation](maps.md): `map_compiler` options for PCD maps.
- [Offline replay and evaluation](replay.md): every replay option, the outputs and the evaluation mapper.
- [ROS 2](ros2.md): launch arguments, R1 (the GLIM extension for evaluation), R2 parameters and lifecycle, topics, services and TF.
- [Status and limitations](status.md): what is verified, what is not, and the open issues.
- [Design](DESIGN.md): the architecture and the design decisions.
- [Contributing](../CONTRIBUTING.md): how to report issues and open pull requests.
