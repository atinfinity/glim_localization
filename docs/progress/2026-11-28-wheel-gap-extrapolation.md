# Wheel odometry for the IMU gap reset ([#14](https://github.com/atinfinity/glim_localization/issues/14))

- **Change**: the IMU gap self-reset (`handle_imu_gap`, `on_imu_gap: reset`) restarts from the last estimate moved at constant velocity, with the orientation held. A vehicle that brakes or turns during the gap therefore restarts off by the velocity change times the gap, plus the whole turn. With the new key `wheel_odom_reset_extrapolation` (default false; it has no effect unless `wheel_odom_enabled` is set), the reset uses the wheel motion over the gap instead:
  - Pose: `T_world_imu(last) * (T_imu_base * T_base * T_imu_base^-1)`, where `T_base = integrate(last.stamp, frame.stamp)` is the same integration the wheel factor uses.
  - Velocity: `R_world_imu(new) * R_imu_base * (v_base + w_base x p_base_imu)`. The twist comes from the new `WheelOdometryBuffer::twist_at(frame.stamp)`, which interpolates within coverage, holds the end samples for up to `wheel_odom_max_gap`, and is empty inside a hole. The formula is in the free function `wheel_velocity_in_imu`.
  - Fallback, with a warning: the constant velocity extrapolation is used when the samples do not cover the interval or the frame stamp, or when the result fails `is_valid_reset` (non-finite, faster than 100 m/s, not a rotation). The status does not go LOST in either case.
  - `OdometryEstimationLocalization::last_reset_used_wheel()` reports what the last applied reset used. It is for tests and logs and is not thread-safe.
- **Decisions**:
  - The wheels can cover the gap because `wheel_.trim()` runs only in `create_wheel_factors`, and only up to the last estimated frame. The buffer is cleared when the reset is applied, so the samples are still there when the reset is queued.
  - No slip gate. The wheel factor gate compares the wheels with the IMU prediction, but in a gap there is no usable prediction. A wild wheel state is still caught by `is_valid_reset`. A wheel that slips during the gap gives a wrong reset, just as a wrong constant velocity does, and the first frames after the reset correct it against the map.
  - `imu_gap_reset_max` (0.5 s) is unchanged. The wheels would make longer gaps safe in principle, but raising the limit is a separate decision.
  - The key is off by default. Off is exactly today's code path: the wheel branch runs only when both keys are true. `KeyWithoutWheelOdometryIsIdentical` checks that, with the wheels disabled, the estimates are bit-identical with the key on and off.
  - The reset reason stays `IMU_GAP`. internal_odom does not re-anchor, as before.
- **Config**: `"wheel_odom_reset_extrapolation": false` was added to the four odometry configs (core CPU and GPU, R1 CPU and GPU). `test_config` checks the key, and the CPU and GPU key sets are still the same.
- **Verification** (Docker, amd64, `glim_localization:dev`):
  - New gtests in `test_wheel_odometry`:
    - `TwistAtInterpolatesWithinCoverage`;
    - `VelocityInImuAppliesTheLeverArm` (closed forms, and a finite-difference check against `integrate` through the extrinsic);
    - five `WheelGapResetTest` cases.
  - Setup of the `WheelGapResetTest` cases:
    - a synthetic 10 Hz trajectory with the IMU and wheel data generated from it (zero bias, default IMU noise);
    - a map 1 km away, so no LiDAR constraint corrects the reset;
    - the IMU removed over (5.25, 5.55] s. The last estimate is at 5.2 s, four frames are dropped, and the reset applies at 5.7 s (a 0.5 s extrapolation).
  - Measured errors of the reset frame against the truth:

    | case | key | translation | yaw | velocity | max translation error after the reset |
    |---|---|---|---|---|---|
    | braking (3 m/s to 0, quintic over 4.5 to 6.5 s) | off | 0.329 m | 0.032 deg | 1.341 m/s | 2.094 m |
    | braking | on | 0.0036 m | 0.031 deg | 0.011 m/s | 0.0048 m |
    | turning (2 m/s, 0.8 rad/s, R 2.5 m) | off | 0.201 m | 22.95 deg | 0.797 m/s | 1.160 m |
    | turning | on | 0.0031 m | 0.035 deg | 0.0033 m/s | 0.0049 m |

    With the key off and braking, 13 wheel factors after the reset are gated: the wrong restart velocity makes the IMU prediction disagree with the wheels.
  - Fallbacks:
    - With no wheel samples, or a hole over (5.25, 5.5] s, the reset is the constant velocity one and `last_reset_used_wheel()` is false.
    - A wheel speed ×80 around the reset stamp fails `is_valid_reset`; the reset falls back and the status does not go LOST.
    - The first frame after a fallback reset is within 0.3 mrad and 5 mm/s of the constant velocity prior. It is not equal to it, because that frame is optimized with the IMU factors of its own interval.
  - Bug found by the new test and fixed: `twist_at` at a sample stamp right before a hole returned empty.
  - `scripts/docker_run.sh scripts/ci.sh` ends with `PASS: all checks`.
- **No result video**: the change affects only the reset frame after an IMU dropout. The replay datasets and the simulator have no IMU gaps on a braking or turning vehicle, so a replay GIF would look the same with the key on and off. The numbers above come from the synthetic gap tests.
- **Open**:
  - Raising `imu_gap_reset_max` when the wheels cover the gap.
  - Checking the reset on a real bag with an IMU dropout ([#14](https://github.com/atinfinity/glim_localization/issues/14)).
