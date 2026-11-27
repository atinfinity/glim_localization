# RViz2 capture of the ROS 2 node on the Gazebo bag

- **Change**: `scripts/sim_viz.sh` runs `glim_localization_node` (R2) on the bag of a finished `scripts/sim_eval.sh` run, renders RViz2 under Xvfb in the simulation image, and writes an animated WebP and a last-frame PNG (`out/<name>/viz/`). The picture shows the map, the scan on `~/aligned_points` in the map frame, the pose, and the estimated path over the ground truth. It shows the ROS 2 integration running end to end; the offline replay animation in the README does not. The steps are described in [sim_eval.md](../sim_eval.md#rviz2-capture).
- **Decisions**:
  - The bag is restamped. `record.sh` runs Gazebo faster than real time, so the log time of each message is the recorder's wall time, while the header stamps are simulation time from 0 to 60 s. `ros2 bag play --clock` publishes `/clock` from the log time, so without the copy, `use_sim_time` nodes would see a clock that disagrees with the stamps. `restamp_bag.py` writes a copy with the log time set to the header stamp. It does not change the recorded bag.
  - RViz2 and ffmpeg run in the simulation image (`xvfb` and `ffmpeg` added; `rviz2` comes with `ros-jazzy-ros-gz`), and the node runs in the development image. The development image stays headless, as before. The two containers share the compose network and find each other with `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`.
  - The capture is a script, not a CI job. It needs the Gazebo bag, which `sim_eval.sh` makes manually (about 6 GB image).
  - The node config is the one of `localize.sh`, with these changes: the R2 `config_ros.json`, the sim topics, `global_shutter_lidar: true` (the Gazebo clouds have no per-point time), and the first ground truth pose as `init_T_world_imu`. `map_cloud_voxel` is 0.2 m instead of 0.5 m, because the coarse map looked sparse at this size. The map is drawn in gray and the scan in orange: with the map colored by height, the floor hid the scan.
  - The WebP is three times as fast as recorded, 5 fps, 640 px wide and quality 30: 2.9 MB for 60 s. At 10 fps, 720 px and quality 50 it was 9.6 MB. The quality changes little: the size comes from the many small points that move in every frame, so the frame rate and the width are the settings that matter (quality 20 instead of 30 saved 0.5 MB at 6 fps).
- **Verification** (Docker, amd64): the R2 node on the restamped bag had an RMSE of 3.4 mm against the ground truth, the same as the replay on the same data. A full `scripts/sim_viz.sh` run took 82 s once the images and the ROS build existed. The checks were the frames and the final picture, with the scan on the map walls and the estimate on the ground truth. The GLIM warning that per-point timestamps are not given is expected for a global shutter LiDAR.
- **Review** (agent): no high findings. Fixed:
  - a 30 min hang when the capture never saw `/clock`;
  - three paths that reported success on a broken run: rviz2 or `viz_paths.py` exiting, and the node exiting;
  - the order of the cleanup trap;
  - signals to `ros_localize.sh` as PID 1.

  Left as they are, and documented:
  - only one capture at a time;
  - no handshake between the subscribers and the playback (`SIM_VIZ_SETTLE`);
  - `restamp_bag.py` holds the bag in memory (60 s bags) and drops the recorded QoS profiles; the sensor data subscribers accept the reliable default.

  After the fixes, the full run passed again (80 s).
- **Open**: the picture is from simulation; a capture on a real bag waits for [#5](https://github.com/atinfinity/glim_localization/issues/5).
