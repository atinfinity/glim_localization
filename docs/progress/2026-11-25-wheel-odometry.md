# Wheel odometry factor, core slice ([#14](https://github.com/atinfinity/glim_localization/issues/14), [PR #44](https://github.com/atinfinity/glim_localization/pull/44))

- **Change**: optional wheel odometry in the odometry factor graph, off by default (`wheel_odom_enabled: false`). The core API and the factor are in place and tested on synthetic data; the ROS input and the replay format come later (remaining items below).
- **Where the wheel input enters** (part of the change):
  - API: `OdometryEstimationLocalization::insert_wheel_odom(stamp, linear, angular)` takes the twist of the wheel base in the base frame. It is thread-safe (its own mutex), so a ROS callback may call it from any thread, as `insert_imu` is fed. It returns false when the feature is off or the sample is dropped (non-finite, or not newer than the last one).
  - Graph: `create_wheel_factors(current)` in the shared base class, called from `create_factors` of both backends (CPU: after the map factors, before the local model; GPU: after the GPU map factors). It adds `BetweenFactor<Pose3>(X(i-1), X(i))` between consecutive IMU poses, next to GLIM's IMU factor. GLIM is not patched.
  - ROS (next step): R2 holds the odometry module, so a `nav_msgs/Odometry` subscription can call `insert_wheel_odom` directly; R1 needs the module instance from the extension (the same way the estimate bridge reaches it). Samples must arrive within `wheel_odom_max_gap` after a frame's stamp before the frame is processed, or that frame gets no wheel factor.
- **Decisions**:
  - A relative pose factor rather than a velocity factor: the wheels measure displacement, and a between factor on the poses needs no new variables and no change to GLIM's IMU factors or keys.
  - The twist is linearly interpolated between samples (held constant up to `wheel_odom_max_gap` past the ends) and integrated over the frame interval with `Pose3::Expmap` at each segment's midpoint, which is exact for a constant twist (a circular arc). Then `T_imu = T_imu_base * T_base * T_imu_base^-1`, so a lever arm between the wheel base and the IMU is handled.
  - The samples must cover the interval: none further than `wheel_odom_max_gap` from either end, and no gap above it inside. Otherwise the frame has no wheel factor (counted in `LocalizationStatus::wheel_missing`).
  - Noise: diagonal, per axis `rotation_sigma_min + rotation_sigma_ratio * angle` and `translation_sigma_min + translation_sigma_ratio * distance`, with a Huber kernel (`wheel_odom_huber_threshold`, 0 disables). The rotation is weak by default (0.05 rad), as the gyro is better than wheel yaw.
  - Slip: the wheel motion is compared with the IMU prediction of the same interval (`frames[current]` still holds the prediction in `create_factors`). If the mean speed differs by more than `wheel_odom_gate_velocity` (1.0 m/s) or the angular speed by more than `wheel_odom_gate_angular_velocity` (0.5 rad/s), the factor is skipped (`wheel_gated`). A 0 disables a gate.
  - `wheel_odom_nonholonomic` zeroes the lateral and vertical velocity of each sample (a differential-drive or Ackermann base whose odometry reports them noisy).
  - With the feature off, `create_factors` returns the same graph as before (the CPU backend returns the map factors unchanged when there is no wheel factor), so results are bit-identical; a test checks this.
  - The buffer is trimmed at each frame and capped at 10000 samples, so wheel input without LiDAR does not grow it without bound. The counters in `LocalizationStatus` (`wheel_factors`, `wheel_gated`, `wheel_missing`) are cumulative and cleared on reset; they are not yet in the ROS status message.
  - The parameters are validated when enabled (`std::invalid_argument`): a rigid `T_imu_base`, a positive `max_gap` and sigma minimums, and non-negative ratios, threshold and gates.
  - Scenario choice: a feature-poor corridor (side walls and floor only) was tried first, but in the synthetic map each submap is a 15 m disc, and its cut edges pull the scan along x (the estimate ran at about 3.5 m/s instead of 2), which no odometry input can be expected to override at these weights. That is an artefact of the synthetic submaps, so the test uses a map 1 km away instead: no scan matches it, as with a dropped LiDAR, and only the IMU and the wheels observe the motion.
- **Config** (part of the change; `odometry_estimation` section, all four odometry configs, defaults):

| Key | Default | Meaning |
| --- | --- | --- |
| `wheel_odom_enabled` | `false` | Turn the wheel factor on |
| `wheel_odom_T_imu_base` | `[0, 0, 0, 0, 0, 0, 1]` | Pose of the wheel base in the IMU frame (x y z qx qy qz qw) |
| `wheel_odom_nonholonomic` | `false` | Drop the lateral and vertical velocity |
| `wheel_odom_max_gap` | `0.2` | Largest gap between samples, and between a frame and the nearest sample [s] |
| `wheel_odom_translation_sigma_min` / `_ratio` | `0.01` / `0.05` | Translation sigma [m] = min + ratio * distance |
| `wheel_odom_rotation_sigma_min` / `_ratio` | `0.05` / `0.2` | Rotation sigma [rad] = min + ratio * angle |
| `wheel_odom_huber_threshold` | `1.0` | Huber kernel on the whitened error (0: none) |
| `wheel_odom_gate_velocity` | `1.0` | Slip gate on the speed difference to the IMU prediction [m/s] (0: off) |
| `wheel_odom_gate_angular_velocity` | `0.5` | Slip gate on the angular speed difference [rad/s] (0: off) |

- **Verification**:
  - `scripts/ci.sh` in Docker on the development Mac (arm64) passed: core ctest 296/296 (14 new in `test_wheel_odometry`), colcon test 21/21 and 3/3 (177 results, all linters), interfaces, Python tests 240, replay e2e. The GPU backend change (one line, the same helper) is not compiled by CI (CUDA off).
  - `test_wheel_odometry` (run by `build_core.sh` through ctest, synthetic data only; the wheel twist is the exact base twist of the synthetic trajectory at 50 Hz with a base 0.3 m behind and 0.5 m below the IMU, turned 0.2 rad):
    - Buffer and factor: a constant twist integrates to the analytic arc (1e-9), linear interpolation, coverage and holes, rejected samples, trim and the cap, the non-holonomic option, the extrinsic (`T_imu_base * T_base * T_imu_base^-1`), the gate (stuck wheel, unseen turn, gate off), and parameter validation.
    - Off is identical: on the normal map (one thread), a run without wheel input, a disabled run fed samples, and an enabled run without samples give bit-identical poses, velocities and biases over 50 frames.
    - With the map: maximum translation error 0.0107 m without and with wheel input (120 wheel factors, none gated).
    - Without LiDAR constraints (map 1 km away, 12 s, about 20 m): IMU only 6.30 m maximum error; with wheel input 0.204 m (120 factors, none gated or missing).
    - Slip, without LiDAR constraints: the wheel stuck for 1.5 s at 2 m/s, then spinning at 1.8x for 1.5 s. The gate skips 30 frames, and the maximum error is 0.249 m (0.204 m with ideal wheels); without the gate it is 5.04 m.
  - `test_config`: `wheel_odom_enabled` is false in the CPU and GPU configs.
- **Open** (remaining for [#14](https://github.com/atinfinity/glim_localization/issues/14)): the ROS 2 `nav_msgs/Odometry` subscription (R1 and R2) and its topic and frame settings; wheel input in the replay dataset and `bag_to_replay.py`; per-sample covariance from the message; a time offset between the wheel and IMU clocks; using the wheel velocity across IMU gaps and in relocalization; the effect on the DEGRADED and LOST rules; the wheel counters in the ROS status message; evaluation in the Gazebo simulation ([#9](https://github.com/atinfinity/glim_localization/issues/9)) and on real data.
