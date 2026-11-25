# Simulation evaluation, Step 1 ([#9](https://github.com/atinfinity/glim_localization/issues/9), [#42](https://github.com/atinfinity/glim_localization/pull/42))

> [!NOTE]
> These are simulation results and are optimistic, much better than on real data:
> - `gpu_lidar` scans are instantaneous, with no motion distortion.
> - The motion is planar: the height is fixed at 1 m, with no roll or pitch.
> - The IMU noise is white only, with no bias and no bias random walk.
> - The initial pose is the exact ground truth.
> - The map is sampled from the same primitive geometry that the LiDAR sees.
>
> They do not replace [#5](https://github.com/atinfinity/glim_localization/issues/5) Step 2.

- **Change**:
  - `scripts/sim_eval.sh` runs the chain from the world to `eval.json` in a clean `out/<name>` with one command. How to run it: [sim_eval.md](../sim_eval.md).
  - A separate image, `glim_localization:sim` (`docker/sim/`), records the bag and samples the map. It contains:
    - ROS 2 Jazzy;
    - Gazebo Harmonic, gz sim 8.15.0, from `ros-jazzy-ros-gz`;
    - sdf2map, pinned at 66feb61.
  - The development image converts the bag, compiles the map, replays and evaluates.
  - Nothing is added to the development image or to `scripts/ci.sh`. No existing script changed:
    - `bag_to_replay.py` already drops non-finite points and copies `--gt`;
    - `evaluate.py` never aligns.
- **Decisions**:
  - **Scripted rig instead of a vehicle model.** A small system plugin, `ScriptedMotion`, moves the rig through prismatic x and y joints and a revolute yaw joint. The trajectory is a function of simulation time only. The same plugin publishes the ground truth: the world pose of `imu_link`, at 100 Hz in simulation time, from the physics state of the same step. It fails fast on:
    - a missing joint or link;
    - a pose rate ≤ 0;
    - a segment duration ≤ 0;
    - a physics step that does not divide the pose period.
  - **Walls thicker than the coarsest voxel.** The walls are 2.5 m thick, more than the coarsest voxel of the CPU config (2 m). This keeps out of Step 1 the map error from sdf2map sampling both faces of a wall (DESIGN 5.2).
  - **Exact initial pose.** The initial pose is the ground truth at the first frame, so Step 1 measures tracking only. The offset sweep is Step 2.
  - **Data checks before the replay.** `check_data.py` separates simulator or bridge problems from localization errors. It checks:
    - the counts against the duration;
    - that the first stamps are within one period of 0;
    - the periods in simulation time;
    - that every LiDAR frame lies in the ground truth span;
    - gravity over the 2 s hold;
    - gyro z against the ground truth yaw rate;
    - the horizontal acceleration against the second derivative of the ground truth position;
    - every scan placed by the ground truth against the map.
  - **Gate after the replay.** The run fails on any LOST state, a replay error, or an error above `evaluate.py --max-trans 0.05 --max-rot-deg 1`. The gate is loose: it catches a broken run, not a regression.
  - **Paused start.** The server starts paused and runs exactly `SIM_DURATION` / 1 ms steps through one WorldControl `multi_step` request. `gz sim --iterations` cannot be used, because it counts paused iterations. `gz sim` runs under `timeout` (`SIM_TIMEOUT`).
  - **Deletion inside a container.** `sim_eval.sh` deletes the outputs inside a container, because they belong to the container user.
  - **No PGM world.** The PGM option (map2sdf) was not used; the world is SDF primitives.
- **Verification** (Docker on the development Mac, arm64, no GPU):
  - **Rendering.** `gpu_lidar` renders with Mesa llvmpipe through EGL (`--headless-rendering`, ogre2). The real-time factor is about 12 (60 s simulated in about 5 s). The image is 6.0 GB.
  - **Run time.** A `sim_eval.sh` run takes about 45 s with the images and the core build in place. The first run also builds the core in the `loc` build volume, 157 s in total.
  - **Data** (one 60 s run, `check_data.json`):
    - 601 LiDAR frames, 12001 IMU samples and 6001 ground truth poses, with the first ones at 0.000 s, 0.001 s and 0.000 s.
    - Periods of 0.100 s, 0.005 s and 0.010 s in simulation time. The bag is 606 MiB (mcap).
    - IMU over the hold: mean specific force (0.000, -0.001, 9.808) m/s².
    - Gyro z against the ground truth yaw rate: rmse 0.0007 rad/s, with a peak rate of 1.57 rad/s.
    - Horizontal acceleration against the second derivative of the ground truth: rmse 0.0064 m/s², with a peak of 0.78 m/s².

    So Gazebo reports linear acceleration, including the centripetal part.
    - Scan to map at the ground truth: median point distance 0.030 m per frame (worst frame 0.030 m, p90 0.049 m).
    - sdf2map: 664,083 points.
  - **Results** (no alignment, 600 of 601 frames estimated; the frame at t = 0 has no IMU before it):
    - Status sequence: INITIALIZING x5, then TRACKING x596 to the end. No DEGRADED or LOST.
    - ATE translation rmse 0.0025 m (max 0.0072 m), rotation rmse 0.038° (max 0.084°).
    - RPE translation rmse 0.0021 m (max 0.0055 m), rotation rmse 0.026° (max 0.053°).
    - Odometry time per frame: mean 12.4 ms, median 7.9 ms, p95 10.1 ms, p99 215 ms, max 250 ms. 2.0 % of the frames take longer than the 100 ms period: the first frames and the submap insertions. Processing time over data length: 0.18.
  - **Determinism.** With the sensor noise set to 0, two recordings gave bit-identical messages on all three topics. With the noise of the rig, the stamps, counts, ground truth and map are identical between runs. The noise samples are not, because the gz-sensors noise does not follow `--seed`, so the errors differ in the fourth decimal.
  - **Risks hit:**
    - **Rendering on arm64 without a GPU.** This was the main risk. llvmpipe was fast enough, so no GPU host was used.
    - **Lost first messages.** A server started running published its first steps before the bridge had discovered its gz-transport publishers, and the first IMU sample was lost. The server now waits paused for discovery.
    - **Messages dropped during the run.** In most runs, a few IMU samples or poses were dropped in the middle of the run, and the new count checks caught it.
      - Capping the real-time factor at 2 did not help, and neither did deep bridge and recorder queues alone.
      - The cause is the ZeroMQ high-water mark of gz-transport, 1000 queued messages per socket by default. `record.sh` sets `GZ_TRANSPORT_RCVHWM=0` and `GZ_TRANSPORT_SNDHWM=0` (no limit) and keeps the deep queues (`sim/config/`).
    - **Signals.** The background bridge and recorder ignored SIGINT from the script, and the `ros2 run` wrapper did not pass on SIGTERM. The script now runs the bridge binary directly and stops both with SIGTERM, then SIGKILL after 20 s.
    - **`gpu_lidar` clouds:**
      - They are organized, with non-finite points for missing returns. `bag_to_replay.py` drops 2,982,729 of them over 601 frames, mostly the sky.
      - They have no time field, so all per-point times are 0. That is right for an instantaneous scan.
- **Open** (Step 2, [#9](https://github.com/atinfinity/glim_localization/issues/9)):
  - an initial pose offset sweep;
  - map changes;
  - IMU dropouts and frame drops;
  - calibration of `degraded_inlier_ratio`;
  - a second, larger world, with sdf2map `--density` and `--voxel` sensitivity.
