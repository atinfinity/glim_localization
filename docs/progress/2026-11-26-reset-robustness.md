# Reset robustness: speed limit, bias carry-over, velocity without a pose ([#6](https://github.com/atinfinity/glim_localization/issues/6))

- **Change**: the three robustness items of [#6](https://github.com/atinfinity/glim_localization/issues/6).
  - **Speed limit.** `reset()` rejects a velocity above `OdometryEstimationLocalization::kMaxResetSpeed` (100 m/s). The check is in `is_valid_reset`, so relocalize() and the IMU gap self-reset drop such an estimate too. Before, a finite velocity around 1e50 passed the check, and the smoother then aborted the process with `IndeterminantLinearSystemException`.
  - **Bias carry-over.** The member `start_imu_bias_` holds the bias the current run started from. Each reset updates it from the newest frame of the run before, if there is one. Before, a second reset that came before any accepted frame fell back to the configured `imu_bias`, losing the estimate. A getter, `start_imu_bias()`, exposes the value for tests and logs.
  - **Velocity without a pose.** When `init_v_world_imu` is set without `init_T_world_imu`, the params set `init_v_without_pose` and the constructor throws `std::invalid_argument`. Upstream sets `estimate_init_state = !init_T && !init_v`, so this config used to start at the identity pose in the map. The constructor also runs the reset checks on a configured initial state: finite, rigid, and within the speed limit.
- **Decisions**:
  - **The limit is a constant, not a parameter.** 100 m/s (360 km/h) is far above any ground platform, and the point is only to keep the smoother away from values it cannot solve. A parameter would be one more thing to misconfigure.
  - **Reject rather than wait.** A config with a velocity but no pose is rejected instead of waiting for a pose. It is a config mistake, and waiting would hide it.
  - **The rigidity check reuses the reset tolerance** (`|R^T R - I| < 1e-3`). A configured pose with a poorly normalized quaternion beyond that is now rejected at startup, as a `reset()` with the same pose already was.
- **Verification**: `scripts/docker_run.sh scripts/ci.sh`, Docker, amd64. New and changed tests in `test_localization_odometry.cpp`:
  - `RejectsInvalidResetRequests` now covers 1e50 m/s and the limit + 1 m/s;
  - `AcceptsAResetAtTheSpeedLimit`;
  - `RejectsInvalidConfiguredInitialState` covers velocity without a pose, a huge configured velocity and a scaled rotation;
  - `SecondResetBeforeAnyFrameKeepsTheEstimatedBias`: two resets with no estimated frame between them keep the bias estimated in the first run, not the configured one.
- **Open**:
  - The rest of [#6](https://github.com/atinfinity/glim_localization/issues/6): the covariance callback slot after teardown, and the test and investigation items.
  - The parsing of `init_v_without_pose` from a config file is a one-line check. It has no test of its own, because the test suite shares a single global config.
