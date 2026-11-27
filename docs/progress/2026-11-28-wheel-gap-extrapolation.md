# Wheel odometry for the IMU gap reset ([#14](https://github.com/atinfinity/glim_localization/issues/14))

- **Change**: the IMU gap self-reset (`handle_imu_gap`, `on_imu_gap: reset`) restarts from the last estimate moved at constant velocity, with the orientation held. A vehicle that brakes or turns during the gap therefore restarts off by the velocity change times the gap, plus the whole turn. With the new key `wheel_odom_reset_extrapolation` (default false; it has no effect unless `wheel_odom_enabled` is set), the reset uses the wheel motion over the gap instead:
  - Pose: `T_world_imu(last) * (T_imu_base * T_base * T_imu_base^-1)`, where `T_base` is integrated from `last.stamp` to the frame stamp, as for the wheel factor.
  - Velocity: `R_world_imu(new) * R_imu_base * (v_base + w_base x p_base_imu)`, with the twist at the frame stamp. The formula is in the free function `wheel_velocity_in_imu`.
  - The motion and the twists at both ends come from one call, `WheelOdometryBuffer::motion(t0, t1)`, which takes the lock once. A sample inserted from another thread, or a clock restart, cannot fall between the integration and the twists. `twist_at(t)` interpolates within coverage, holds the end samples for up to `wheel_odom_max_gap`, and is empty inside a hole.
  - A slip check runs before the wheel reset is used (see Decisions).
  - Fallback, with a warning, to the constant velocity extrapolation in three cases: the samples do not cover the interval or its ends, the result fails `is_valid_reset` (non-finite, or faster than 100 m/s), or the slip check fails. The status does not go LOST.
  - An IMU_GAP reset now trims the wheel buffer to the reset frame (`wheel_.trim(frame_stamp)`) instead of clearing it, with or without the key. The first interval after the reset then gets its wheel factor. Every other reset still clears the buffer, because after a bag loop the old samples may be stamped after the new ones.
  - `OdometryEstimationLocalization::last_reset_used_wheel()` reports what the last applied reset used. It is for tests and logs and is not thread-safe.
- **Decisions**:
  - The wheels can cover the gap because `wheel_.trim()` runs only in `create_wheel_factors`, and only up to the last estimated frame.
  - Why a slip check is needed:
    - A slipping or locked wheel would become the reset velocity.
    - That velocity is also the IMU prediction for the next frames.
    - So the wheel factor gate, which compares the wheels with that prediction, would agree with the bad wheel from then on.
  - The check uses the thresholds of that gate, `max(wheel_odom_gate_velocity, wheel_odom_gate_velocity_ratio * speed)`, where speed is the larger of the two velocities compared. It does two comparisons:
    1. At the last estimate: the wheel velocity, rotated to the world frame, against the estimated velocity. This catches a wheel that is already slipping when the gap starts.
    2. Over the gap: the wheel displacement against `0.5 * (v_last + v_reset) * dt`, the trapezoid of the estimated velocity at the start and the wheel velocity at the end, as a velocity (divided by dt). This catches a wheel that slips or locks inside the gap, when its velocity at both ends looks right.
  - A smooth brake or turn is close to linear over 0.5 s: in the braking and turning cases the check passes without a warning. A step in the real speed inside the gap fails the check, and then the old constant velocity reset is used. That is the behavior with the key off, so the check can only fall back to it.
  - No acceleration bound: it would need a vehicle-specific limit, a new key. The trapezoid check already rejects the locked wheel.
  - `imu_gap_reset_max` (0.5 s) is unchanged. The wheels would make longer gaps safe in principle, but raising the limit is a separate decision.
  - The key is off by default. With the key off, the reset is computed exactly as before; the only change on that path is the trim of the wheel buffer above.
  - The reset reason stays `IMU_GAP`. internal_odom does not re-anchor, as before.
  - No status field for the wheel path; see Open.
- **Config**: `"wheel_odom_reset_extrapolation": false` was added to the four odometry configs (core CPU and GPU, R1 CPU and GPU). `test_config` checks the key, and the CPU and GPU key sets are still the same.
- **Verification** (Docker, amd64, `glim_localization:dev`):
  - New gtests in `test_wheel_odometry`:
    - `TwistAtInterpolatesWithinCoverage`;
    - `VelocityInImuAppliesTheLeverArm` (closed forms, and a finite-difference check against `integrate` through the extrinsic);
    - ten `WheelGapResetTest` cases.
  - Setup of the `WheelGapResetTest` cases:
    - a synthetic 10 Hz trajectory with the IMU and the 50 Hz wheel data generated from it (zero bias, default IMU noise);
    - a map 1 km away, so no LiDAR constraint corrects the reset;
    - the IMU samples in [5.25, 5.55) s removed. The last estimate is at 5.2 s, four frames are dropped, and the reset applies at 5.7 s (a 0.5 s extrapolation).
  - Measured errors of the reset frame against the truth, and the largest translation error over the frames from the reset to 7 s:

    | case | key | used the wheels | translation | yaw | velocity | max after the reset |
    |---|---|---|---|---|---|---|
    | braking (3 m/s to 0, quintic over 4.5 to 6.5 s) | off | no | 0.329 m | 0.032 deg | 1.341 m/s | 2.094 m |
    | braking | on | yes | 0.0036 m | 0.031 deg | 0.0028 m/s | 0.0038 m |
    | turning (2 m/s, 0.8 rad/s, R 2.5 m) | off | no | 0.201 m | 22.95 deg | 0.797 m/s | 1.160 m |
    | turning | on | yes | 0.0031 m | 0.035 deg | 0.0033 m/s | 0.0049 m |
    | braking, wheel locked (0) from 5.25 s | off | no | 0.329 m | 0.032 deg | 1.341 m/s | 2.094 m |
    | braking, wheel locked | on | no (gap check: 0.92 m/s, gate 0.69) | 0.329 m | 0.032 deg | 1.341 m/s | 2.094 m |
    | turning, wheel ×2 in [5.25, 5.55) s | off | no | 0.201 m | 22.95 deg | 0.797 m/s | 1.160 m |
    | turning, wheel ×2 in the gap | on | no (gap check: 1.18 m/s, gate 0.60) | 0.201 m | 22.95 deg | 0.797 m/s | 1.160 m |
    | braking, wheel ×2 from 5 s | off | no | 0.437 m | 0.031 deg | 0.513 m/s | 0.653 m |
    | braking, wheel ×2 from 5 s | on | no (start check: 2.18 m/s, gate 1.38) | 0.437 m | 0.031 deg | 0.513 m/s | 0.653 m |

    - In the three slip cases the key on falls back and gives the same estimates as the key off (single thread, errors equal to 1e-9). With the check disabled for a test build, the key on was worse than the key off at the reset frame in all three: 0.706, 0.579 and 0.866 m.
    - With the key off and braking, 13 wheel factors after the reset are gated: the wrong restart velocity makes the IMU prediction disagree with the wheels.
    - The key-on braking run improved after the trim (velocity 0.011 to 0.0028 m/s, max after the reset 0.0048 to 0.0038 m), because the first interval after the reset now gets its wheel factor.
  - Other cases:
    - `KeyWithoutGapIsIdentical`: wheel odometry on and no IMU gap, so no reset. Key on and off give bit-identical estimates (single thread).
    - `KeyWithoutWheelOdometryIsIdentical`: wheel odometry off with a gap. Key on and off are bit-identical; this only shows that the key needs `wheel_odom_enabled`.
    - `KeepsTheWheelSamplesAcrossTheGapReset`: `wheel_odom_max_gap` is 0.03 s (1.5 wheel periods). After the reset, no wheel interval is missing, and all 13 frames after the reset frame get a factor (added or gated), with the key off and on. With `clear()` restored for a test build, one interval was missing (12 of 13).
    - Fallbacks:
      - With no wheel samples, or a wheel hole over [5.25, 5.5] s, the reset is the constant velocity one and `last_reset_used_wheel()` is false.
      - A wheel speed ×80 around the reset stamp fails `is_valid_reset`; the reset falls back and the status does not go LOST.
    - The first frame after a fallback reset without wheel factors pulling on it is within 0.3 mrad and 5 mm/s of the constant velocity prior. It is not equal to it, because that frame is optimized with the IMU factors of its own interval.
  - While developing the change, the new `twist_at` returned empty at a sample stamp right before a hole. This was fixed before the first commit; it was never on main.
  - `scripts/docker_run.sh scripts/ci.sh` ends with `PASS: all checks`.
- **No result video**: the change affects only the reset frame after an IMU dropout. The replay datasets and the simulator have no IMU gaps on a braking or turning vehicle, so a replay GIF would look the same with the key on and off. The numbers above come from the synthetic gap tests.
- **Open**:
  - Operators cannot see whether a reset used the wheels: `LocalizationStatus` has no field for it, and only the log says so (`last_reset_used_wheel()` is a test hook).
  - Raising `imu_gap_reset_max` when the wheels cover the gap.
  - Checking the reset on a real bag with an IMU dropout ([#14](https://github.com/atinfinity/glim_localization/issues/14)).
