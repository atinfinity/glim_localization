# Simulation evaluation (Gazebo)

A manual evaluation in a simulated world with exact ground truth, from [#9](https://github.com/atinfinity/glim_localization/issues/9) Step 1. It does not run in CI and adds nothing to the development image.

> [!NOTE]
> Results from simulation are optimistic, and much better than on real data:
> - `gpu_lidar` scans are instantaneous (no motion distortion).
> - The motion is planar: the height is fixed at 1 m, with no roll or pitch.
> - The IMU noise is white only, with no bias and no bias random walk.
> - The initial pose is the exact ground truth, so the run measures tracking, not initialization.
> - The map and the LiDAR see the same primitive geometry (sdf2map samples the world the sensor renders).
>
> Real sensors are covered by [#5](https://github.com/atinfinity/glim_localization/issues/5).

## Run

```bash
scripts/sim_eval.sh            # results in out/sim_eval/run/eval.json and eval.txt
scripts/sim_eval.sh my_run     # another directory under out/
SIM_DURATION=20 SIM_SEED=3 scripts/sim_eval.sh short
```

The script builds the simulation image `glim_localization:sim` (`docker/sim/Dockerfile`, about 6 GB) on the first run and refreshes it from the build cache on later ones. The rest runs in the development image `glim_localization:dev`. The out directory is emptied at the start of every run; a directory that `sim_eval.sh` did not create (no `.sim_eval_out` marker) is refused. The outputs are deleted inside a container, because they belong to the container user.

| Variable | Default | Used by |
| --- | --- | --- |
| `SIM_DURATION` | 60 | Simulated seconds. The scripted profile is 60 s; a shorter run cuts it. |
| `SIM_SEED` | 1 | `gz sim --seed`. |
| `SIM_TIMEOUT` | 900 | Wall seconds allowed for `gz sim` before the run fails. |
| `SIM_EVAL_MAX_TRANS`, `SIM_EVAL_MAX_ROT_DEG` | 0.05, 1.0 | Loose accuracy gate: `evaluate.py --max-trans`, `--max-rot-deg`. |

> [!WARNING]
> The `loc` service has its own build volume, not the one of `docker/compose.yaml`, so the first run builds the core there (about 10 minutes). `COMPOSE_PROJECT_NAME` selects the compose project and so these volumes and the network (default `glim_localization_sim`). Do not set it to the project name of another compose setup, or the runs share and overwrite its volumes. Clean up with `docker compose -f docker/sim/compose.yaml down -v` under the same project name.

On the development Mac (Docker, arm64, CPU only) one run takes about 45 s once the images and the core build exist.

## Chain

| Step | Image | What |
| --- | --- | --- |
| 1. record | sim | `sim/scripts/record.sh`: builds the `ScriptedMotion` plugin, then Gazebo Harmonic runs `sim/worlds/sim_eval_room.sdf` headless for `SIM_DURATION` s of simulation time. `ros_gz_bridge` and `ros2 bag record` (mcap) write the LiDAR points, the IMU and the IMU link pose to `bag/`. `sim/scripts/pose_to_tum.py` writes the pose topic as `traj_gt.txt` (TUM, `T_world_imu`, simulation time), and the real-time factor and the Gazebo version to `record.json`. |
| 2. map | sim | sdf2map samples the world (without the rig) into `map.pcd`, density 400 points/m², 0.05 m voxel, fixed seed. |
| 3. localize | dev | `sim/scripts/localize.sh`: `bag_to_replay.py --gt`, `map_compiler` (PCD path, 50 m tiles, 0.1 m voxel), `sim/scripts/check_data.py`, the replay from the first ground truth pose, the status sequence, and `evaluate.py`, which never aligns trajectories (the world frame is the map frame). |

`check_data.py` fails the run before the replay when:
- a sensor or the ground truth has the wrong count for the duration, a first stamp more than one period after 0, or a gap;
- a LiDAR frame is outside the ground truth span;
- the IMU does not match the ground truth: gravity over the 2 s hold, gyro z against the yaw rate, and the horizontal acceleration against the second derivative of the position;
- a scan placed by the ground truth does not lie on the map (median point-to-map distance above 0.05 m). This catches a wrong extrinsic or a map in another frame. It is also consistent with scans rendered at their stamps, though a lag much shorter than a frame would not show.

The run fails after the replay on any LOST state, a replay error, or a pose error above the gate.

Outputs under `out/<name>/`: `bag/`, `traj_gt.txt`, `record.json`, `map.pcd`, `dataset/`, `map/`, `check_data.json`, `config/`, `run/` (`traj_localization.txt`, `frames.csv`, `eval.json`, `eval.txt`, plots, `status_sequence.json`) and `logs/`.

## World, rig and trajectory

- **World** (`sim/worlds/sim_eval_room.sdf`): an L-shaped room of 30 m x 20 m from primitives only. The walls are 2.5 m thick, thicker than the coarsest map voxel (2 m), so sdf2map's two faces of a wall never share a voxel. An island, pillars, a rotated box and a low box make the layout asymmetric. No ceiling. Shadows are off, the physics step is 1 ms, and `real_time_factor` 0 runs the simulation as fast as it can.
- **Rig** (`sim/models/sim_eval_rig`): `imu_link` with a 200 Hz IMU (white noise: gyro 0.002 rad/s, accelerometer 0.02 m/s²) and a 10 Hz `gpu_lidar` at (0.10, 0, 0.20) in the IMU frame with the same axes: 32 beams over ±15°, 1024 samples, 0.3 to 50 m, noise 0.01 m. `localize.sh` holds this extrinsic in one variable (`T_IMU_LIDAR`) for the config and the checks. The link is driven through prismatic x and y joints and a revolute yaw joint, so it has no collisions and its height is fixed at 1 m.
- **Trajectory**: the `ScriptedMotion` system plugin (`sim/plugins/scripted_motion`) sets the joint velocities from a list of segments in the world file: hold 2 s, accelerate to 1 m/s, a slalom east, three 90° left turns with a slalom west in between, stop, a full turn in place and hold, 60 s in total. It also publishes the world pose of `imu_link` at the first step and then at 100 Hz, stamped with simulation time, as the ground truth. A missing joint or link, a bad rate or segment, or a physics step that does not divide the pose period stops the server.

## Rendering

Docker on a Mac has no GPU, so `gpu_lidar` renders with Mesa llvmpipe through EGL (`--headless-rendering`, ogre2). The image sets no display. The sensors render in step with the simulation, so a slow renderer slows the run but does not change simulation time or the data. `gz sim` runs under `timeout` (`SIM_TIMEOUT`).

## Determinism

The trajectory is a function of simulation time only. With sensor noise set to 0, two recordings give bit-identical messages on all three topics. With the noise of the rig, the stamps, counts and ground truth are identical, but the noise samples differ between runs (the gz-sensors noise is not reproducible from `--seed`), so the errors change in the fourth decimal.
