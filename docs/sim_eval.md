# Simulation evaluation (Gazebo)

A manual evaluation in a simulated world with exact ground truth, from [#9](https://github.com/atinfinity/glim_localization/issues/9) Step 1. It does not run in CI and adds nothing to the development image.

> [!NOTE]
> Results from simulation are optimistic. `gpu_lidar` scans are instantaneous (no motion distortion), the IMU noise model is simple, and the map is sampled from the same geometry that the LiDAR sees. Real sensors are covered by [#5](https://github.com/atinfinity/glim_localization/issues/5).

## Run

```bash
scripts/sim_eval.sh            # results in out/sim_eval/run/eval.json and eval.txt
scripts/sim_eval.sh my_run     # another directory under out/
```

The script builds the simulation image `glim_localization:sim` the first time (`docker/sim/Dockerfile`, about 6 GB), and uses the development image `glim_localization:dev` for the rest. The out directory is removed at the start of every run. `COMPOSE_PROJECT_NAME` selects the compose project (volumes and network); the default is `glim_localization_sim`. Clean up with `docker compose -f docker/sim/compose.yaml down -v`.

On the development Mac (Docker, arm64, CPU only) one run takes about 45 s after the images and the core build exist.

## Chain

| Step | Image | What |
| --- | --- | --- |
| 1. record | sim | `sim/scripts/record.sh`: Gazebo Harmonic runs `sim/worlds/sim_eval_room.sdf` headless for 60 s of simulation time. `ros_gz_bridge` and `ros2 bag record` (mcap) write the LiDAR points, the IMU and the IMU link pose to `bag/`. `sim/scripts/pose_to_tum.py` writes the pose topic as `traj_gt.txt` (TUM, `T_world_imu`, simulation time) and the real-time factor to `record.json`. |
| 2. map | sim | sdf2map samples the world (without the rig) into `map.pcd`, density 400 points/m², 0.05 m voxel, fixed seed. |
| 3. localize | dev | `sim/scripts/localize.sh`: `bag_to_replay.py --gt`, `sim/scripts/check_data.py`, `map_compiler` (PCD path, 50 m tiles, 0.1 m voxel), the replay from the first ground truth pose, and `evaluate.py`, which never aligns trajectories (the world frame is the map frame). |

`check_data.py` fails the run when the sensor periods in simulation time differ from the sensor model (IMU 200 Hz, LiDAR 10 Hz), or when a scan placed by the ground truth does not lie on the map (median point-to-map distance above 0.05 m). This catches a scan rendered at a different time than its stamp, a wrong extrinsic, or a map in another frame.

Outputs under `out/<name>/`: `bag/`, `traj_gt.txt`, `record.json`, `map.pcd`, `dataset/`, `check_data.json`, `map/`, `config/`, `run/` (`traj_localization.txt`, `frames.csv`, `eval.json`, `eval.txt`, plots, `status_sequence.json`) and `logs/`.

## World, rig and trajectory

- **World** (`sim/worlds/sim_eval_room.sdf`): an L-shaped room of 30 m x 20 m from primitives only. The walls are 2.5 m thick, thicker than the coarsest map voxel (2 m), so sdf2map's two faces of a wall never share a voxel. An island, pillars, a rotated box and a low box make the layout asymmetric. No ceiling. Shadows are off, the physics step is 1 ms, and `real_time_factor` 0 runs the simulation as fast as it can.
- **Rig** (`sim/models/sim_eval_rig`): `imu_link` with a 200 Hz IMU (gyro noise 0.002 rad/s, accelerometer noise 0.02 m/s²) and a 10 Hz `gpu_lidar` at (0.10, 0, 0.20) in the IMU frame with the same axes: 32 beams over ±15°, 1024 samples, 0.3 to 50 m, noise 0.01 m. The replay config uses `T_lidar_imu` = (-0.10, 0, -0.20). The link is driven through prismatic x and y joints and a revolute yaw joint, so it has no collisions and its height is fixed at 1 m.
- **Trajectory**: the `ScriptedMotion` system plugin (`sim/plugins/scripted_motion`, built when the recording starts) sets the joint velocities from a list of segments in the world file: hold 2 s, accelerate to 1 m/s, a slalom east, three 90° left turns with a slalom west in between, stop, a full turn in place and hold, 60 s in total. It also publishes the world pose of `imu_link` at 100 Hz, stamped with simulation time, as the ground truth.

## Rendering

Docker on a Mac has no GPU, so `gpu_lidar` renders with Mesa llvmpipe through EGL (`--headless-rendering`, ogre2). The image sets no display, and the sensor manager renders in lockstep with the simulation, so a slow renderer only slows the run; simulation time and the data do not change.

## Determinism

The trajectory is a function of simulation time only. With sensor noise set to 0, two recordings give bit-identical messages on all three topics. With the noise of the rig, the stamps, counts and ground truth are identical, but the noise samples differ between runs (the gz-sensors noise is not reproducible from `--seed`), so the errors change in the fourth decimal.
