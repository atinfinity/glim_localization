# Status and limitations

> **Status: under development. CPU by default; the GPU module (CUDA image) is tested on synthetic data only.** The core, the offline tools and both ROS 2 integrations work on synthetic data and pass CI. Real-data evaluation is limited (see [Limitations](#limitations)). The interfaces may still change.

Measurements and the history behind these items are in [PROGRESS.md](PROGRESS.md); the plan is in [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md).

## Progress

| Phase | Content | State |
| --- | --- | --- |
| 0 to 2 | Docker environment, repository skeleton, MapManager and map I/O | Done |
| 3 | CPU localization odometry, quality monitoring (INITIALIZING / TRACKING / DEGRADED / LOST) | Done |
| 4 | Offline replay, bag conversion, evaluation, evaluation mapper | Done |
| 5 | ROS 2: glim_ros2 layer, msgs, IMU gap policy, publisher and TF, R1 extension, R2 lifecycle node | Done |
| 5 | Launch files, YAML parameter files, RViz config, `~/map` | Done |
| 6 | `map_compiler` PCD input | Done |
| 6 | README and user docs | Done |
| 6 | GPU (CUDA) odometry module, selected by the odometry config (CUDA image) | Done; tile prefetch open ([#2](https://github.com/atinfinity/glim_localization/issues/2)) |

## Open work

Not done yet (open issues):

- GPU follow-ups ([#2](https://github.com/atinfinity/glim_localization/issues/2)): tile prefetch and a GPU memory budget, real-data timing on the GPU. The GPU module itself is in place ([CUDA variant](build.md#cuda-variant)).
- Relocalizer: multi-hypothesis refinement of `/initialpose` and global relocalization ([#3](https://github.com/atinfinity/glim_localization/issues/3)).
- `map_compiler` PLY input and 2D OccupancyGrid export ([#4](https://github.com/atinfinity/glim_localization/issues/4)); PCD follow-ups such as `binary_compressed` input ([#12](https://github.com/atinfinity/glim_localization/issues/12)).
- Evaluation on real data against an independent reference ([#5](https://github.com/atinfinity/glim_localization/issues/5)) and in simulation ([#9](https://github.com/atinfinity/glim_localization/issues/9)).
- Optional wheel odometry fusion ([#14](https://github.com/atinfinity/glim_localization/issues/14)).
- Follow-ups for robustness, the IMU gap policy, the publisher and the R2 node ([#6](https://github.com/atinfinity/glim_localization/issues/6), [#8](https://github.com/atinfinity/glim_localization/issues/8), [#10](https://github.com/atinfinity/glim_localization/issues/10), [#13](https://github.com/atinfinity/glim_localization/issues/13)).

## Limitations

- **GPU module tested on synthetic data only.** The GPU module (CUDA image) is verified on one Turing GPU with synthetic data ([PROGRESS](PROGRESS.md)); it uploads map tiles on first use, without prefetch ([#2](https://github.com/atinfinity/glim_localization/issues/2)). With the CPU module, on one 10 Hz OS1-128 sequence (Docker, arm64) the mean was about 93 ms per frame for a 100 ms period, with p99 spikes over 1 s, and the ratio of processing time to data length varied between runs (1.06 and 0.64); real-time capability is not established.
- **Not yet verified:** accuracy on real data against an independent reference (the only real-data run, one OS1-128 sequence, compares with GLIM's own trajectory: ATE translation RMSE about 0.023 to 0.025 m, a consistency figure, not accuracy); real robots and real-time sensor input; most measurements so far ran in the Docker image on arm64, and amd64 has less coverage; a multithreaded component container; PCD maps from a real mapping pipeline (for example sdf2map, [#9](https://github.com/atinfinity/glim_localization/issues/9)); integration with Nav2 or an external EKF.
- `~/aligned_points` ([#18](https://github.com/atinfinity/glim_localization/issues/18)) and an IMU-rate odometry are not published.
- No initial pose search: `/initialpose` and `~/initialize` are applied exactly. No global relocalization ([#3](https://github.com/atinfinity/glim_localization/issues/3)).
- Right after the node becomes active, the first messages can be missed (cause not confirmed; DDS discovery is suspected). In R2 set the first pose with `~/initialize`, which replies; otherwise publish `/initialpose` after the node is matched, or resend it.
- IMU gaps in ROS (`on_imu_gap: reset` in the shipped config): a gap up to `imu_gap_reset_max` (0.5 s) resets the estimate from its own prediction; a longer gap goes LOST until an external initialization. The replay tool defaults to `--on-imu-gap abort` (exit code 3).
- Degenerate directions are not detected, and pose covariance is not estimated. Objects thinner than a voxel can bias the estimate.
- Map input: no PLY, no compressed PCD, no 2D map export ([#4](https://github.com/atinfinity/glim_localization/issues/4), [#12](https://github.com/atinfinity/glim_localization/issues/12)).
