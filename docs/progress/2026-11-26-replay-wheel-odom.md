# Wheel odometry in the replay dataset ([#14](https://github.com/atinfinity/glim_localization/issues/14))

- **Change**: a replay dataset may carry `wheel_odom.csv` (`stamp_ns,vx,vy,vz,wx,wy,wz`, the twist of the wheel base), and `glim_localization_replay` feeds it to the odometry through `insert_wheel_odom`. The synthetic dataset writes it, so wheel odometry can be evaluated offline with and without it on the same data. Converting a real bag (`bag_to_replay.py`) is the next step.
- **Dataset** (`replay_dataset.hpp`): `ReplayWheelOdom`, `ReplayDatasetWriter::write_wheel_odom`, `ReplayDataset::wheel_odom()` / `has_wheel_odom()`. `meta.json` gets `num_wheel_odom` only when the file is written, so datasets without it are written and read as before. `imu.csv` and `wheel_odom.csv` share one stamped CSV reader with the same strictness. A `num_wheel_odom` without the file, a row count that disagrees, or a negative count is an invalid dataset (exit code 3).
- **Runner** (`ReplayRunner`):
  - `ReplayOptions::feed_wheel_odom` (default true; samples are fed only when the odometry has wheel odometry enabled and the dataset has samples).
  - Per frame, after the IMU and before `insert_frame`: every sample up to the frame stamp, plus one. The first frame starts 1 s before its stamp (the buffer would drop a long lead-in anyway). Refused samples are counted as `rejected`.
  - `frames.csv` gets `num_wheel_fed` and `wheel` (`none`, `added`, `gated`, `missing`, from the per-frame deltas of the `LocalizationStatus` counters, reset with `reset_count`). `summary.json` gets `wheel_odom` with `fed`, `rejected`, `added`, `gated`, `missing`.
- **CLI**: `--wheel-odom on|off` overrides `wheel_odom_enabled` of the config. `on` without wheel samples (no `wheel_odom.csv`, or only its header) is exit code 2 (a bad argument for this dataset). The config enabled without samples warns and runs IMU only. `replay_args.json` records `effective.wheel_odom`.
- **Synthetic data**: `make_synthetic_wheel` (in `synthetic_sensor`) gives the twist of a base frame at `T_imu_base` from the synthetic trajectory (`Pose3::Logmap` of central differences), with optional Gaussian noise. `write_synthetic_replay` writes it at 50 Hz with 0.01 m/s and 0.002 rad/s noise. `test_wheel_odometry` now builds its samples from it.
- **Decisions**:
  - The `wheel` column reports the factor, not the samples: `gated` and `missing` are what explain a drift in the plot.
  - The 1 s lead-in at the first frame keeps the feed independent of how far before the first frame the wheel log starts (tested with a run starting at 5 s).
  - `on` without samples is an error rather than a warning: asking for wheel odometry on a dataset without it is a mistake in the command, and a silent IMU-only run would be read as a wheel result.
- **Evaluation** (`scripts/test_replay_e2e.sh`, synthetic 12 s run, Docker, amd64):
  - Normal map with `--wheel-odom on`: passes the Phase 3 criteria (settle 3 s, 0.1 m, 1 deg).
  - Map shifted 1 km in x, so no scan matches and there is no map factor (state stays INITIALIZING; `lost_frames` raised so the run is not cut):

    | run | ATE max [m] | ATE rmse [m] | wheel factors |
    | --- | --- | --- | --- |
    | IMU only (`--wheel-odom off`) | 6.366 | 2.52 | 0 |
    | with wheel odometry (`--wheel-odom on`) | 0.214 | 0.096 | 120 added, 0 gated, 0 missing (602 samples fed) |

    The e2e asserts the flags, one added factor per estimated frame but the first, no gated or missing factor, and the with-wheel max below 0.4 m and below 0.2 of the IMU-only max.
  - A dataset without `wheel_odom.csv` with `--wheel-odom on` exits 2.
- **Tests**: `ReplayDatasetTest` (optional file, round trip, bad files), `ReplayRunnerTest` (feed up to the stamp plus one, the 1 s lead-in, fed only when enabled, accuracy, writer output), `ReplayToolTest.WheelOdometryOption`, `SyntheticReplayTest` (103 samples over 2.05 s, the body velocity), `SyntheticSensorTest.NoiselessWheelTwistIntegratesToTruth` (integrating the twist of an offset, rotated base over 1 s at 1 kHz: under 1 mm and 1e-4 rad).
- **Verification**: `scripts/docker_run.sh scripts/ci.sh`, Docker, amd64: all checks pass (345 core tests, 23 `glim_localization_ros2` tests including the linters, the Python tests, the replay robustness and e2e). The review fix (`on` with a header-only `wheel_odom.csv` exits 2) was rerun with the 24 `ReplayToolTest` tests.
- **Known limitation**: the samples are fed before `insert_frame`, and a reset applied inside it (`--init-stamp`, an IMU-gap reset) clears the wheel buffer, including the extra sample after the frame stamp. The next frame then starts its interval with the second sample after the reset frame: fine at 50 Hz (held for up to 40 ms), but a wheel log at 10 Hz or slower can make that frame `missing`. Re-feeding after a reset is left for later.
- **Open**: `bag_to_replay.py` options for a wheel odometry topic (topic, child frame, time offset), the twist covariance as the factor noise, and the DEGRADED/LOST rules with wheel odometry.
