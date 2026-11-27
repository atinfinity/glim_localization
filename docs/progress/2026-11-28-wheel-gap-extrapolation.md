# Wheel odometry for the IMU gap reset ([#14](https://github.com/atinfinity/glim_localization/issues/14))

- **Change**: the IMU gap self-reset (`handle_imu_gap`, `on_imu_gap: reset`) restarts from the last estimate moved at constant velocity, with the orientation held. A vehicle that brakes or turns during the gap therefore restarts off by the velocity change times the gap, plus the whole turn. With the new key `wheel_odom_reset_extrapolation` (default false; it has no effect unless `wheel_odom_enabled` is set), the reset uses the wheel motion over the gap instead:
  - Pose: `T_world_imu(last) * (T_imu_base * T_base * T_imu_base^-1)`, where `T_base` is integrated from `last.stamp` to the frame stamp, as for the wheel factor.
  - Velocity: `R_world_imu(new) * R_imu_base * (v_base + w_base x p_base_imu)`, with the twist at the frame stamp. The formula is in the free function `wheel_velocity_in_imu`.
  - The motion and the twists at both ends come from one call, `WheelOdometryBuffer::motion(t0, t1)`, which takes the lock once. A sample inserted from another thread, or a clock restart, cannot fall between the integration and the twists. `twist_at(t)` interpolates within coverage, holds the end samples for up to `wheel_odom_max_gap`, and is empty inside a hole.
  - A slip check runs before the wheel reset is used (see Decisions).
  - Fallback, with a warning, to the constant velocity extrapolation in three cases: the samples do not cover the interval or its ends, the result fails `is_valid_reset` (non-finite, or faster than 100 m/s), or the slip check fails. The status does not go LOST.
  - An IMU_GAP reset now trims the wheel buffer to the reset frame (`wheel_.trim(frame_stamp)`) instead of clearing it, with or without the key. The first interval after the reset then gets its wheel factor. Every other reset still clears the buffer, because after a bag loop the old samples may be stamped after the new ones.
  - `OdometryEstimationLocalization::last_reset_extrapolation()` reports how the last applied reset was extrapolated, as `ResetExtrapolation`: `WHEEL`, or why it fell back (`NO_COVERAGE`, `INVALID`, `SLIP_START`, `SLIP_ANGULAR`, `SLIP_GAP`), or `NONE` when the wheels were not tried or the reset was not an IMU gap. `last_reset_used_wheel()` is `== WHEEL`. Both are for tests and logs and are not thread-safe.
- **Decisions**:
  - The wheels can cover the gap because `wheel_.trim()` runs only in `create_wheel_factors`, and only up to the last estimated frame.
  - Why a slip check is needed:
    - A slipping or locked wheel would become the reset velocity.
    - That velocity is also the IMU prediction for the next frames.
    - So the wheel factor gate, which compares the wheels with that prediction, would agree with the bad wheel from then on.
  - The check uses the thresholds of that gate: `max(wheel_odom_gate_velocity, wheel_odom_gate_velocity_ratio * speed)`, where speed is the larger of the two velocities compared, and `wheel_odom_gate_angular_velocity`. It runs three comparisons in this order, and the first that fails names the fallback:
    1. `SLIP_START`, at the last estimate: the wheel velocity, rotated to the world frame, against the estimated velocity. This catches a slip already present at `last.stamp`.
    2. `SLIP_ANGULAR`, at the last estimate: the wheel angular velocity in the IMU frame (`R_imu_base * w_base`) against the mean of the gyro samples within `max_imu_gap` of `last.stamp` in the upstream queue, minus the gyro bias of the last estimate. It is skipped when the queue has no such sample. This catches a wrong wheel yaw rate whose linear velocity is right; the other two checks miss it, because a straight displacement matches the unturned end velocity.
    3. `SLIP_GAP`, over the gap: the wheel displacement against `0.5 * (v_last + v_reset) * dt`, the trapezoid of the estimated velocity at the start and the wheel velocity at the end, as a velocity (divided by dt). This catches a wheel that locks in the gap, or one that slips and recovers before the reset frame, when its velocity at both ends looks right.
  - A smooth brake or turn is close to linear over 0.5 s: in the braking and turning cases no check fires. A step in the real speed inside the gap fails the gap check, and then the old constant velocity reset is used. That is the behavior with the key off, so the check can only fall back to it.
  - Known blind spot: a scale slip that starts in the gap and lasts through the reset frame.
    - The wheel velocity at the reset frame is then wrong by the same factor k, so the displacement still matches the trapezoid. The gap check sees about `|k - 1| / 2 * v` against the gate `0.3 * max(1, k) * v`, and passes for 0.4 < k < 2.5.
    - Nothing after the gap gives another speed reference.
    - The reset then uses the wheels and is worse than the constant velocity one (turning, ×2 from 5.25 s: 0.866 m against 0.201 m). The map factors have to correct it.
    - `PersistentSlipFromTheGapIsNotCaught` pins this behavior.
  - No acceleration bound: it would need a vehicle-specific limit, a new key, and a default loose enough for real braking would not catch ×2 either.
  - `imu_gap_reset_max` (0.5 s) is unchanged. The wheels would make longer gaps safe in principle, but raising the limit is a separate decision.
  - The key is off by default. With the key off, the reset is computed exactly as before; the only change on that path is the trim of the wheel buffer above.
  - The reset reason stays `IMU_GAP`. internal_odom does not re-anchor, as before.
  - No status field for the wheel path; see Open.
- **Config**: `"wheel_odom_reset_extrapolation": false` was added to the four odometry configs (core CPU and GPU, R1 CPU and GPU). `test_config` checks the key, and the CPU and GPU key sets are still the same.
- **Verification** (Docker, amd64, `glim_localization:dev`):
  - New gtests in `test_wheel_odometry`:
    - `TwistAtInterpolatesWithinCoverage`;
    - `VelocityInImuAppliesTheLeverArm` (closed forms, and a finite-difference check against `integrate` through the extrinsic);
    - twelve `WheelGapResetTest` cases.
  - Setup of the `WheelGapResetTest` cases:
    - a synthetic 10 Hz trajectory with the IMU and the 50 Hz wheel data generated from it (zero bias, default IMU noise);
    - a map 1 km away, so no LiDAR constraint corrects the reset;
    - the IMU samples in [5.25, 5.55) s removed. The last estimate is at 5.2 s, four frames are dropped, and the reset applies at 5.7 s (a 0.5 s extrapolation).
  - Measured errors of the reset frame against the truth, and the largest translation error over the frames from the reset to 7 s:

    | case | key | extrapolation | translation | yaw | velocity | max after the reset |
    |---|---|---|---|---|---|---|
    | braking (3 m/s to 0, quintic over 4.5 to 6.5 s) | off | `NONE` | 0.329 m | 0.032 deg | 1.341 m/s | 2.094 m |
    | braking | on | `WHEEL` | 0.0036 m | 0.031 deg | 0.0028 m/s | 0.0038 m |
    | turning (2 m/s, 0.8 rad/s, R 2.5 m) | off | `NONE` | 0.201 m | 22.95 deg | 0.797 m/s | 1.160 m |
    | turning | on | `WHEEL` | 0.0031 m | 0.035 deg | 0.0033 m/s | 0.0049 m |
    | braking, wheel locked (0) from 5.25 s | off | `NONE` | 0.329 m | 0.032 deg | 1.341 m/s | 2.094 m |
    | braking, wheel locked | on | `SLIP_GAP` (0.92 m/s, gate 0.69) | 0.329 m | 0.032 deg | 1.341 m/s | 2.094 m |
    | turning, wheel ×2 in [5.25, 5.55) s | off | `NONE` | 0.201 m | 22.95 deg | 0.797 m/s | 1.160 m |
    | turning, wheel ×2 in the gap | on | `SLIP_GAP` (1.18 m/s, gate 0.60) | 0.201 m | 22.95 deg | 0.797 m/s | 1.160 m |
    | braking, wheel ×2 from 5 s | off | `NONE` | 0.437 m | 0.031 deg | 0.513 m/s | 0.653 m |
    | braking, wheel ×2 from 5 s | on | `SLIP_START` (2.18 m/s, gate 1.38) | 0.437 m | 0.031 deg | 0.513 m/s | 0.653 m |
    | turning, wheel angular velocity 0 from 5 s | off | `NONE` | 0.212 m | 22.96 deg | 0.809 m/s | 1.185 m |
    | turning, wheel angular velocity 0 from 5 s | on | `SLIP_ANGULAR` (0.80 rad/s, gate 0.50) | 0.212 m | 22.96 deg | 0.809 m/s | 1.185 m |
    | turning, wheel ×2 from 5.25 s to the end (blind spot) | off | `NONE` | 0.201 m | 22.95 deg | 0.796 m/s | 1.158 m |
    | turning, wheel ×2 from 5.25 s to the end (blind spot) | on | `WHEEL` | 0.866 m | 0.33 deg | 2.064 m/s | 3.126 m |

    - In the four slip cases that are caught, the key on falls back and gives the same estimates as the key off (single thread, errors equal to 1e-9). Each fallback test asserts the exact `ResetExtrapolation`.
    - With the checks disabled in a test build, the key on was worse than the key off at the reset frame in all four: 0.706, 0.579 and 0.866 m (linear checks), and 0.325 m (angular check only; the other two checks passed it).
    - With the key off and braking, 13 wheel factors after the reset are gated: the wrong restart velocity makes the IMU prediction disagree with the wheels.
    - The key-on braking run improved after the trim (velocity 0.011 to 0.0028 m/s, max after the reset 0.0048 to 0.0038 m), because the first interval after the reset now gets its wheel factor.
  - Other cases:
    - `KeyWithoutGapIsIdentical`: wheel odometry on and no IMU gap, so no reset. Key on and off give bit-identical estimates (single thread).
    - `KeyWithoutWheelOdometryIsIdentical`: wheel odometry off with a gap. Key on and off are bit-identical; this only shows that the key needs `wheel_odom_enabled`.
    - `KeepsTheWheelSamplesAcrossTheGapReset`: `wheel_odom_max_gap` is 0.03 s (1.5 wheel periods). After the reset, no wheel interval is missing, and all 13 frames after the reset frame get a factor (added or gated), with the key off and on. With `clear()` restored for a test build, one interval was missing (12 of 13).
    - Fallbacks:
      - With no wheel samples, or a wheel hole over [5.25, 5.5] s, the reset is the constant velocity one, `NO_COVERAGE`.
      - A wheel speed ×80 around the reset stamp fails `is_valid_reset` (`INVALID`); the reset falls back and the status does not go LOST.
    - The first frame after a fallback reset without wheel factors pulling on it is within 0.3 mrad and 5 mm/s of the constant velocity prior. It is not equal to it, because that frame is optimized with the IMU factors of its own interval.
  - While developing the change, the new `twist_at` returned empty at a sample stamp right before a hole. This was fixed before the first commit; it was never on main.
  - `scripts/docker_run.sh scripts/ci.sh` ends with `PASS: all checks`.
- **No result video**: the change affects only the reset frame after an IMU dropout. The replay datasets and the simulator have no IMU gaps on a braking or turning vehicle, so a replay GIF would look the same with the key on and off. The numbers above come from the synthetic gap tests.
- **Open**:
  - The slip check cannot catch a scale slip that starts in the gap and lasts through the reset frame (see Decisions); the map factors have to correct it.
  - Operators cannot see whether a reset used the wheels, or why it fell back: `LocalizationStatus` has no field for it, and only the log says so (`last_reset_extrapolation()` is a test hook).
  - Raising `imu_gap_reset_max` when the wheels cover the gap.
  - Checking the reset on a real bag with an IMU dropout ([#14](https://github.com/atinfinity/glim_localization/issues/14)).
