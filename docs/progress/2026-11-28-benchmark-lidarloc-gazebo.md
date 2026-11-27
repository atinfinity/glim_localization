# Benchmark against lidar_localization_ros2 on the Gazebo bag ([#80](https://github.com/atinfinity/glim_localization/issues/80))

- **Change**: step 1 of [#80](https://github.com/atinfinity/glim_localization/issues/80). A benchmark script and its first results, against rsasaki0109/lidar_localization_ros2 (NDT) on the Gazebo bag of `scripts/sim_eval.sh`. The page is [docs/benchmark.md](../benchmark.md).
  - `docker/baseline/Dockerfile`: lidar_localization_ros2 and ndt_omp_ros2 cloned at pinned commits on the pinned `ros:jazzy-ros-base`, `rosdep install`, Release colcon build. It is the `baseline` service of `docker/sim/compose.yaml`, not vendored and not in CI.
  - `sim/scripts/localize.sh` also writes `map_export.pcd` (`map_compiler --export-pcd`), the map of the baseline. The script removes the file first because `--export-pcd` refuses an existing file. `localize.sh` rerun on an existing `out/sim_eval` still passes (ATE RMSE 3.4 mm).
  - `sim/scripts/ros_config.py`: the R2 config for the Gazebo bag, factored out of `ros_localize.sh` and shared with the benchmark.
  - `sim/scripts/bench_run.sh` (one run of one method), `sim/scripts/bench_recorder.py` (rclpy: receive times, CPU and memory of the node), `sim/scripts/bench_params.py` (the baseline's parameters), `sim/config/bench_play_qos.yaml` (the player QoS).
  - `scripts/sim_benchmark.sh` (host side: images, one restamp, build, interleaved runs, compare) and `scripts/compare_methods.py` (the table, `benchmark.json`, an SVG chart), with `scripts/tests/test_compare_methods.py`.
  - `docs/media/benchmark_gazebo.svg`: the chart, plain SVG in the style of the GPU profile figures. Its XML was checked, but it was not rendered here (no SVG renderer on the host).
- **Decisions**:
  - **Baseline version:** v1.12.0 (`6f6cf9a348db2f2a9cf37acffd5e90199ba46248`) with ndt_omp_ros2 `17ce1e19831447a1f5bc5a68a4522fa8087f8933`. Both built at these commits on the first try, so the main-head fallback was not needed.
  - **Rows:** the baseline's documented default (IMU preintegration on) is the headline row, and NDT without the IMU is a second row.
  - **Parameters:** defaults only. The overrides are only what the data needs: map path, initial pose, base frame, sim time. Tuning is a follow-up. Each method keeps its own thread default (NDT 4, glim_localization 2), and the table compares the CPU time per scan.
  - **Runs:** 3 per method, one at a time, interleaved (run 1 of every method, then run 2, ...) so that a slow drift of the host does not bias one method.
  - **Restamping:** the bag is restamped once (`restamp_bag.py`, log time = header stamp) and played with `--clock` at 1x and a 2 s delay.
  - **Latency:** the recorder's receive time of the pose minus its receive time of the cloud with the same stamp, on the monotonic clock. The recorder parses only the stamp from the raw CDR bytes, so it does not deserialize clouds.
  - **QoS:** the player publishes reliable with deep queues. The recorder's cloud subscription is reliable, so it sees every cloud (601 of 601 in every run), and a cloud that a node drops counts as a scan without a pose. The nodes keep their own subscription QoS: both are best effort, sensor data.
    - With the default player QoS and a best-effort recorder, the recorder lost 7 to 26 clouds per run. The node gave no pose for any of the clouds the recorder lost, so the losses are in transport, not in the recorder. FastDDS `LARGE_DATA` and UDP-only transports did not remove them; a reliable reader did.
  - **Map:** the same points for both methods. NDT builds its target at 1.0 m from `map_export.pcd` (235193 points, the 0.1 m voxel map), converted to float32. glim_localization uses the same map in 50 m tiles, with its VGICP voxel maps at 0.5, 1.0 and 2.0 m.
  - **Accuracy:** ATE and RPE (over 1 m) without alignment, 2 s settle from the first cloud, using the functions of `evaluate.py`.
  - **Licenses:** lidar_localization_ros2 and ndt_omp_ros2 are both BSD-2-Clause (checked in the cloned `LICENSE` files). Nothing of them is shipped.
- **Overrides needed beyond the plan**: one. `map_compiler --export-pcd` writes F8 fields. lidar_localization_ros2 loads the map with `pcl::loadPCDFile` into `PointXYZ`, prints "Failed to find match for field 'x'" (and y, z), and gets 235193 points at the origin. Every scan was then rejected (fitness 120 to 250 against `score_threshold` 2.0), and the run gave 0 poses. `bench_params.py` writes a float32 copy (`params/map_f4.pcd`); the largest coordinate is 35 m, so the rounding is far below a millimeter. Non-finite points were not a problem: the Gazebo clouds go through both methods unchanged. `use_continuous_time_deskew` stays at its default (on): it falls back to the raw scan for clouds without per-point time.
- **Verification**: Docker, amd64, Intel i7-9800X (8 cores, 16 threads, `nproc` 16), CPU only. The host load average was below 3 during the runs.
  - Run: `COMPOSE_PROJECT_NAME=gloc80sim scripts/sim_benchmark.sh` on the `out/sim_eval` of `sim_eval.sh` (60 s, 601 clouds, seed 1). It finished in 864 s.
  - Images: `glim_localization:dev` `sha256:0b729f1cf6926390fefd5f78b9caf3c17a554c9094355e7644610b6b16df4381`, `glim_localization:baseline-lidarloc` `sha256:922db5dc25d5e8591745baabb7238643cad46dd3318c5759bfb8fa62ba5dd18b`.
  - Results (median over 3 runs, range in brackets when the runs differ):

    | Method | Runs | ATE trans RMSE [mm] | ATE rot RMSE [deg] | RPE trans RMSE [mm/m] | RPE rot RMSE [deg/m] | Latency median [ms] | Latency p95 [ms] | Latency max [ms] | CPU per scan [ms] | CPU [% of a core] | Peak RSS [MiB] | Scans with a pose |
    |---|---|---|---|---|---|---|---|---|---|---|---|---|
    | glim_localization | 3 | 3.5 | 0.038 | 2.0 | 0.027 (0.027 to 0.028) | 40.4 (40.2 to 40.5) | 304.0 (292.5 to 314.0) | 563.8 (535.4 to 574.2) | 137.6 (136.8 to 137.8) | 138 (137 to 138) | 805 (805 to 809) | 0.972 (0.968 to 0.980) |
    | lidar_localization_ros2 NDT (IMU, default) | 3 | 13.3 (13.2 to 13.3) | 0.069 (0.069 to 0.070) | 7.6 | 0.082 (0.076 to 0.086) | 19.3 (19.2 to 19.7) | 22.5 (22.0 to 22.8) | 25.4 (25.4 to 25.5) | 145.3 (145.2 to 146.2) | 145 (145 to 146) | 100 (99 to 100) | 0.995 (0.990 to 0.998) |
    | lidar_localization_ros2 NDT, no IMU | 3 | 13.1 (13.0 to 13.1) | 0.065 (0.064 to 0.067) | 6.8 (6.7 to 7.4) | 0.079 (0.068 to 0.080) | 18.2 (18.1 to 18.5) | 22.9 (22.0 to 23.1) | 27.9 (27.5 to 29.3) | 169.8 (168.9 to 169.9) | 170 (169 to 170) | 99 (99 to 100) | 0.992 (0.987 to 0.995) |

  - **glim_localization's latency tail:** the latency stays at 20 to 50 ms, except for a stall of about 0.5 s every 3 to 5 s (first at 10 s, then about every 5 s, and every 3.3 s after 40 s). After a stall the backlog drains over about 6 scans (for example 504, 419, 334, 250, 167, 81 ms), so the p95 and the max come from these stalls alone. It matches the CPU odometry's frame-time maximum of 507 ms in the [GPU profile](2026-11-28-gpu-profile.md). Its cause is not investigated here.
  - **Scans without a pose:** glim_localization 12, 17 and 19 of 601. NDT 6, 3 and 1 with the IMU, and 3, 5 and 8 without. Every method misses the scan at stamp 0. The other misses are spread over the run, sometimes several in a row (for example 405 to 414 in glim run 2), and are not at the stalls. The cause was not established.
  - **The pose stamp equals the cloud stamp** for both methods: no pose matched no cloud.
  - The Python tests (`scripts/test_python.sh`, which includes `test_compare_methods.py`) and the full CI pass; see the pull request.
- **Against the predictions** in [the issue comment](https://github.com/atinfinity/glim_localization/issues/80):
  - "Accuracy, easy conditions: about the same": **partly**. Both are in the millimeter range, but glim_localization is about 4 times better (3.5 against 13 mm ATE, 2.0 against 7 mm/m RPE), with the defaults of both. NDT at 1.0 m resolution may close the gap with tuning.
  - "Per-frame latency and CPU: heavier, perhaps several times": **latency yes, CPU no**. The median latency is 2 times higher, and the tail 13 times (p95). The CPU time per scan is about the same (138 against 145 to 170 ms).
  - "Memory: higher": **yes**, 8 times (805 against 100 MiB).
  - "Gazebo will not separate the methods much": **mostly yes**. The accuracy difference is millimeters, and the IMU does not help NDT here. What separates them on this bag is cost (latency tail, memory), not accuracy.
  - Not tested yet: hard conditions and convergence from a wrong initial pose.
- **Open**:
  - The glim_localization latency stalls of about 0.5 s every 3 to 5 s in CPU mode.
  - The scans without a pose, for both methods.
  - The rest of [#80](https://github.com/atinfinity/glim_localization/issues/80): other baselines, a real bag ([#5](https://github.com/atinfinity/glim_localization/issues/5)), hard cases, one tuned setting per method, throughput and the GPU mode.
