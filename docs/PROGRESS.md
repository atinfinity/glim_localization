# Progress log

For each phase, record what was done, problems and workarounds, and open items.

## Starting point (2026-11-23)

- The design (docs/DESIGN.md) and the implementation plan (docs/IMPLEMENTATION_PLAN.md) are written. There is no code yet.
- Decisions: GLIM itself is not modified, base commit `1a197c9`.
- Next: start from Phase 0.

## Phase 0: environment setup and repository creation (completed 2026-11-23)

### Environment

- Verified in Docker (linux/arm64, 8 CPUs, about 7.7 GB memory). Phases 1 and 2 were also verified on GitHub Actions (amd64); later phases were verified on arm64 only unless stated.
- Container: `ros:jazzy-ros-base` (Ubuntu 24.04), g++ 13.3.0, Boost 1.83. No GUI.

### Target platform

- The target is Ubuntu 24.04 + ROS 2 Jazzy, and development is done in a Docker container. Native macOS builds are not supported: the library versions available there (for example Eigen 5) do not match the upstream verification environment.

### What was done

- Added `docker/`, `scripts/docker_run.sh`, `scripts/check_deps.sh` and `scripts/check_deps/`.
- GTSAM 4.3.0 and gtsam_points 1.2.2 are debs from the koide3 PPA; glim is built from source at `1a197c9` without the viewer (`JOBS=4`, image build 98 seconds, no OOM).
- `scripts/docker_run.sh scripts/check_deps.sh` exits with code 0.
- deb SHA256 (arm64):
  - `libgtsam-notbb-dev_4.3.0`: `211bc1693478a3afbd9cc70bbc386a9ec4c9d06b9f87beb9c2192f1bbc7303cb`
  - `libgtsam-points-dev_1.2.2`: `312b4f74e6e2ee84bbb7b7f7fef12e508603cb173e7007fea75b969134d29795`

### Problems and workarounds

- The PPA's GTSAM is installed in `/usr/local/lib`, but the ldconfig cache was not updated, and `libgtsam.so.4` was not found at run time. Fixed by running `ldconfig` in the Dockerfile.

### Open items and carry-overs

- Whether the PPA debs are the same commits as GTSAM `2f3e56c` and gtsam_points `9d32e7d` cannot be confirmed from the debs.
  - Resolved later (2026-11-24): installed headers match `2f3e56c` (tag `4.3a1`) and `9d32e7d` (tag `v1.2.2`) byte for byte, apart from generated config headers. Decision: keep the PPA. The Dockerfile pins the PPA key fingerprint and fails the build on a mismatch (checked with the real and a wrong fingerprint). Further hardening follows in [#11](https://github.com/atinfinity/glim_localization/issues/11). Details in `docs/DEPENDENCIES.md`.
- The same check on amd64 was completed in the Phase 1 CI (below).
- Upstream `config_ros.json` loads the viewer modules, and `config.json` points to the GPU config. A dlopen failure is only a warning, so when our own config is added, add a test that checks it does not include the 3 viewer modules.
- In Phase 5, check with `ldd` that our extension does not link to Iridescence.
- Phase 1 and later of the implementation plan still contained text that assumed macOS. It was later rewritten to assume the Ubuntu 24.04 + Jazzy container.

## Phase 1: repository skeleton (completed 2026-11-23)

### What was done

- Added `glim_localization` (a shared library that links to glim / gtsam_points, and tests), and empty `glim_localization_msgs` / `glim_localization_ros2`.
- The core builds both with plain CMake and with colcon (ament). GoogleTest uses `libgtest-dev` in the image.
- `scripts/ci.sh` is shared by local runs and CI. CI builds the same Dockerfile on amd64 and runs it.

### Results

- arm64 (Docker): `scripts/docker_run.sh scripts/ci.sh` exits with code 0. 3 ctest tests passed, 3 colcon packages succeeded.
- amd64 (GitHub Actions): passed. 5 minutes 33 seconds for the first run without cache.
- deb SHA256 (amd64, values from the PPA index):
  - `libgtsam-notbb-dev_4.3.0`: `257cbe2edf232cbce64f3bac45060c6e56f1b17ead939b415ae7247557cb4e66`
  - `libgtsam-points-dev_1.2.2`: `e00218a58827e1e1215ce51b72401e89cf2572716eb656b79678dc21b3de6eca`

### Problems and workarounds

- `glim-config.cmake` overwrites the caller's `BUILD_WITH_CUDA`, so it is saved under another name before `find_package(glim)`.

### Open items and carry-overs

- Remaining tasks are managed as GitHub issues.

## Phase 2: MapManager and map I/O (2026-11-23)

### What was done

- Synthetic map generator (`test/synthetic/`). It writes the GLIM dump format with upstream `SubMap::save`.
- `map_io`: reading GLIM dumps and slim dumps, overwriting poses from `values.bin`, input validation. If even one item is invalid, the whole load fails.
- `MapManager`: multi-resolution `GaussianVoxelMapCPU`, radius + K-nearest search with a KD-tree. An immutable snapshot after loading.
- `map_compiler`: builds a slim dump and `map_index.json` from a GLIM dump. A non-empty output directory is rejected.

### Results

- arm64 (Docker): `scripts/docker_run.sh scripts/ci.sh` exits with code 0. 32 ctest tests passed, 3 colcon packages succeeded.
- amd64 (GitHub Actions): passed.
- A round trip of a synthetic dump keeps poses within 1e-9 and point coordinates within 1e-5, and the point counts and `query` results match.
- Linking libglim alone is enough to read `values.bin` (our own `GTSAM_VALUE_EXPORT` is not needed).

### Problems and workarounds

- The nlohmann json bundled with GLIM is not installed, so `nlohmann-json3-dev` was added to the Dockerfile.
- A search that stops at K items picks tiles at equal distance in an undefined order, so all tiles within the radius are taken, sorted by distance and then id, and then cut to K.

### Open items and carry-overs

- Not verified on a real GLIM dump (many frames, old dumps in the `points.bin` format, voxel generation time at real size). To be checked in [#5](https://github.com/atinfinity/glim_localization/issues/5).
- Voxels for all tiles are generated sequentially at `load`. Lazy generation, parallelization and LRU are [#2](https://github.com/atinfinity/glim_localization/issues/2).

## Phase 3: CPU localization odometry (2026-11-24)

### What was done

- A full set of configs, synthetic scans by ray casting, and a synthetic IMU.
- Unary map VGICP factors (overlap filter, weighting) and quality monitoring (INITIALIZING / TRACKING / DEGRADED / LOST).
- `OdometryEstimationLocalizationCPU`: initialization in map coordinates, `reset`, status notification, optional local model constraints (`use_keyframes`).
- `libodometry_localization_cpu.so` (MODULE). If creation fails, it logs an error and aborts.

### Results

- arm64 (Docker): `scripts/docker_run.sh scripts/ci.sh` exits with code 0. 72 ctest tests passed, colcon succeeded.
- In a 12-second synthetic run with the initial pose off by (0.3, -0.2, 0.1) m and about 3°, the maximum error after 3 seconds is 0.0107 m and 0.09° (criteria 0.1 m, 1°). Processing time is about 0.9 seconds.
- `use_keyframes: true`, `reset` with no initial pose, and `reset` during a run also meet the same criteria.
- Shifting the map by 1000 m gives LOST after the 10 frames set in `lost_frames`.
- `libglim_localization` and the module do not link to GUI or ROS libraries (checked in `build_core.sh`).

### Problems and workarounds

- The plain corridor scene is almost degenerate in x (smallest Hessian eigenvalue about 2e4, others about 3e6). The acceptance criteria were not loosened; staggered side walls were added to the synthetic scene instead.
- With 0.4 m thick side walls, voxels containing both faces were averaged into a plane in the middle, giving a bias of up to 0.29 m. Fixed by making the walls 0.05 m, well thinner than a voxel.
- Upstream `IMUIntegration` applies a zero-order hold of each sample over the preceding interval. At 100 Hz this does not match the ideal trajectory, so the noise-free check of the IMU generator runs at 1 kHz.

### Open items and carry-overs

- Not verified on real data. The bias on thin objects and the processing time at real size are checked in [#5](https://github.com/atinfinity/glim_localization/issues/5).
- Multi-hypothesis refinement of `/initialpose` and global relocalization are [#3](https://github.com/atinfinity/glim_localization/issues/3). The GPU version is [#2](https://github.com/atinfinity/glim_localization/issues/2).
- Detection of degenerate directions and covariance output are not implemented.

## Phase 4: offline replay (2026-11-24)

### What was done

- Reader/writer for the replay dataset format, a deterministic `ReplayRunner`, and `glim_localization_replay` (independent of ROS and GUI. ldd check in `build_core.sh`).
- Synthetic dataset generator (`synthetic_replay_dataset`).
- A Python environment with pinned versions in Docker (rosbags 0.11.10, evo 1.38.1, pytest 9.2.0). `/opt/glim_localization_venv`, referenced through `GLIM_LOC_PYTHON`.
- `scripts/evaluate.py`, `scripts/bag_to_replay.py` (db3 and mcap, tested with 5 kinds of time layouts).
- Added `scripts/test_python.sh` and `scripts/test_replay_e2e.sh` to `ci.sh`.

### Results

- Replaying 12 s of synthetic data (initial pose off by (0.3, -0.2, 0.1) m and about 3°), the ATE after 3 s is translation max 0.0107 m, rotation max 0.0905 deg (`evaluate.py --settle 3 --max-trans 0.1 --max-rot-deg 1` passes).
- Two runs with the same input give the same `frames.csv` (stamp_ns, result), and the pose difference is 0 m.
- With `--init-stamp 2.0`, the 20 frames before 2.0 s are `dropped` and the first pose is at 2.000 s. No initial pose gives exit code 2, and `--init-stamp` after the end gives 4.
- Processing time / data length is 0.110 (synthetic data).

### Follow-up fixes: IMU coverage, outputs and input validation

- IMU coverage is judged by gaps between consecutive samples, not only at both ends (`ReplayOptions::max_imu_gap`).
- `ReplayOutputWriter` deletes an old `summary.json` at start.
- `evaluate.py`: checks pose coverage (default 0.9) and estimates without GT, uses the results in frames.csv, and gives exit code 2 for invalid input. The existing test `test_pairs_only_within_tolerance` expected exit code 0 for input with estimates for only half of the GT, so it was changed to expect 1 by default and 0 with `--min-coverage 0.5`.
- `bag_to_replay.py`: detects mode contradictions in later frames, uses the last matching time field, checks the range after rounding to float32.
- Dataset reader: rejects negative stamps, `version: 1.9`, and whitespace and hexadecimal in imu.csv.
- Removed `|| true` from `test_replay_e2e.sh`.

### Follow-up fixes: IMU dropouts and divergence

- Missing IMU: upstream integration uses the interval from the previously inserted frame, so skipping a dropout after the first pose accumulates error and diverges (reproduced with a dropout at 4-6 s). If there is a gap longer than `max_imu_gap` after the first pose, the run stops with `InvalidDatasetError` (exit code 3). A dropout at the start is `skipped_no_imu` as before. A frame that ends at the last IMU sample can now be estimated.
- Changed the default of `max_imu_gap` from 0.1 s to 0.05 s. Measured whether a 25 Hz IMU meets the Phase 3 criteria: translation max 0.0104 m, rotation max 0.073 deg (`LowRateImuMeetsAccuracy`). The fallback (0.02 s, verified at 50 Hz) was not needed. The tool rejects 5 Hz and 10 Hz IMUs with exit code 3 before reading the map.
- Divergence: an estimate that is non-finite or has |v| > 1000 m/s is recorded as `diverged` and the run stops with exit code 1 (no TUM file containing nan with exit code 0).
- `summary.json` is deleted before argument validation.
- `evaluate.py`: aligns the evaluated interval to the first usable frame (if the first 90 frames are `skipped_no_imu`, coverage is 0.1 and the exit code is 1), and requires the `--frames` run to have completed (consistent with summary.json) (exit code 2 otherwise, lifted by `--allow-incomplete`). Without `--frames`, coverage is not checked.
- `bag_to_replay.py`: rejects negative stamps. The clock offset error message now matches what actually happens.
- Changed existing tests (`test_evaluate.py`, thresholds not loosened): the `frames_csv` helper and `test_processing_time_statistics` now write summary.json (it became required for `--frames`). `test_pairs_only_within_tolerance` and `test_low_coverage_fails` expected exit code 1 for low coverage without `--frames`, but the spec now does not check coverage without `--frames`, so they were moved to pass frames.csv. The final exit code 1 without `--frames` in `test_frames_csv_counts_dropped_frames_as_missing` was changed to 0. `SkipsFramesInsideAnImuDropout` tested the wrong behavior of skipping frames after a dropout, so it was replaced by `AbortsOnImuDropoutAfterInsertedFrame`.

### Carry-overs

- Phase 5: a policy for IMU dropouts shared by online and replay (recovery by reset, `--on-imu-gap reset`). Replay currently only stops at a dropout.
- An override option for real bags whose point times and header clock are offset ([#5](https://github.com/atinfinity/glim_localization/issues/5)).
- The |v| > 1000 guard only detects divergence and does not guarantee accuracy.

### Rolling shutter deskew test (one item of [#6](https://github.com/atinfinity/glim_localization/issues/6))

- Added `SyntheticLidarParams::scan_duration` to the synthetic sensor. Each point is measured on the true trajectory at its own observation time (the point is in the IMU frame at that time) and has its offset from the stamp in `times`. The default 0 behaves as before.
- `test_replay_deskew`: a 0.1 s sweep (moving about 0.2 m and a few degrees in 0.1 s). With per-point times: translation max 0.0111 m, rotation max 0.075 deg (after 3 s, criteria 0.1 m / 1 deg). Control with the times removed from the same points: 0.179 m, 2.53 deg, above the criteria.
- Deskew after the IMU is trimmed while waiting for the initial pose: 0.0104 m / 0.071 deg with the equivalent of `--init-stamp 5.0`; the control without times is 0.179 m / 2.29 deg. No defect was found in the core.
- The earlier `scan_span` variant (spreading times over a static point cloud) cannot verify deskew, so this test does not use it.

### Problems and workarounds

- The ROS environment's `PYTHONPATH` leaks into the venv and pytest fails. Call Python with `env -u PYTHONPATH`.
- `python3-venv` is not in `ros:jazzy-ros-base`, and `apt-get update` is needed.
- evo's `reduce_to_time_range` works destructively and returns None. The evaluated interval applies the same index selection to both trajectories.

### Open items and carry-overs

- Not verified on a real bag ([#5](https://github.com/atinfinity/glim_localization/issues/5)). bag_to_replay was checked only with synthetic bags.
- The increase in CI time has not been measured on amd64 GitHub Actions.

## Real-data smoke test (first step of [#5](https://github.com/atinfinity/glim_localization/issues/5), 2026-11-24)

A private Ouster OS1-128 recording (1153 frames, about 115 s, 10 Hz LiDAR) was converted with `bag_to_replay.py` and run with `scripts/smoke_os1_128.sh`. **The reference is GLIM's own trajectory, not ground truth. The numbers below are agreement with GLIM (consistency), not accuracy.** The bag and the map are not committed.

### What was done

- Added `glim_localization_mapper` (for evaluation, no ROS) and `config/mapping_cpu/`. Usage is in DESIGN 6.5. Synthetic tests: on a 12 s sequence, several submaps are made by cutting a submap every 3 keyframes; the dump can be read by `load_map`, there are no jumps between frames, and invalid arguments give exit code 2 / an empty range gives 1.
- `T_lidar_imu = [0.006253, -0.011775, 0.007645, 0, 0, 0, 1]`: the point cloud frame_id is `os_sensor`, and the translation of os_sensor -> os_imu in `/tf_static` (identity rotation) has the same direction as GLIM's `p_lidar = T_lidar_imu * p_imu`. The script also sets the localization config to the same value (the bundled config has 0).
- Trajectory composition: the first implementation used `T_world_origin * frame->T_world_imu`, which jumped by tens of m across submaps. The frames of a submap stay in the world frame of the submap's own optimization, and global mapping moves only `T_world_origin`. Fixed to `T_world_origin * inv(pose of origin_frame) * pose of the frame`, and a jump check was added to the synthetic test (confirmed to fail with the old formula). Because of this error, the first held-out run looked like an ATE of 177 m, but the cause was the trajectory composition, not localization.

### Results

| Item | Value |
| --- | --- |
| mapper (whole range) exit code | 0, wall 41.0 s, max RSS 3.65 GB (criterion below 6 GB) |
| Number of submaps / `traj_glim.txt` | 74 / 1122 poses (97.3 % of the 1153 inserted frames, 99.9 % of the frames after odometry initialization. The first 30 frames were used for initialization and have no poses) |
| `map_compiler` | Wrote 74 submaps (criterion 5 or more) |
| Revisit check | 30.1 s of the tail [85 s, end] is within 10 m of poses of the first part [0, 80 s] (criterion 15 s) → held-out is run |
| mapper on the first part only (`--end 80`) | 51 submaps, wall 28.6 s, max RSS 2.43 GB. Over the 770 poses overlapping the whole-range run, translation difference rms 0.009 m, max 0.019 m (the world frames match, so the initial pose can be carried over) |
| replay (held-out, last 302 frames on the first-part map) | exit code 0, all 302 frames `estimated` (INITIALIZING 4, TRACKING 298), `diverged` 0 |
| Agreement (ATE against the GLIM trajectory, no realignment) | translation rmse 0.0254 m (max 0.188 m), rotation rmse 0.283 deg (max 1.99 deg). RPE translation rmse 0.0386 m. Held-out, so reported only (the gate is only for self-localization) |
| Processing time | odometry mean 93.4 ms, median 54.6 ms, p95 98.3 ms, p99 1600 ms, max 2525 ms. Processing time / data length 1.059, 14.6 % of frames slower than the frame period (0.1 s). wall 31.9 s (data 30 s) |

### Interpretation and limits

- The reference is GLIM itself and uses the same sensor, the same preprocessing and the same extrinsics, so common errors (extrinsics, time offsets) do not show in the agreement. Accuracy is to be evaluated in a later step of [#5](https://github.com/atinfinity/glim_localization/issues/5) (an independent run and ground truth).
- The initial pose is the GLIM trajectory pose at the start of the tail, with velocity 0.
- Processing time with this CPU version (Docker, arm64) is slightly slower than real time (ratio 1.059). The spikes around 1.6 s at p99 have not been investigated (possibly tile loading). Real-time capability cannot be concluded from this measurement.
- Self-localization over the whole range was run by forcing the branch with `TAIL_START=1000` (ATE rmse 0.0171 m, below). There was a revisit, so the main result is held-out.
- CI: all 119 ctest tests passed (4 mapper tests, about 2.6 s). The increase in `ci.sh` is about 16 s for a clean build of the mapper and `test_mapper_tool`, and about 3 s for tests.
- Rerun after the fixes below (same real data, held-out): ATE translation rmse 0.0233 m (max 0.189 m), rotation rmse 0.274 deg, all 302 frames `estimated`, the first-part mapper has 50 submaps (asynchronous, so it varies by about 1 per run), translation difference over the overlap with the whole range rms 0.012 m (max 0.052 m). Processing time / data length is 0.638, 2.6 % of frames slower than the frame period (the first value of 1.059 changes a lot with load even for the same run, and cannot be used to conclude about real-time capability). Self-localization over the whole range (branch forced with `TAIL_START=1000`) with `--init-stamp`: ATE rmse 0.0171 m (provisional gate below 0.3 m; the first 30 frames have no reference and are `dropped`).
- When the IMU ended before the LiDAR, the mapper stalled (the odometry waits for IMU, and the inserting side waits for the load to drop). Frames not covered by the IMU are skipped as in replay and counted in `frames_skipped_no_imu` of `mapper_summary.json`. A run with the real IMU cut at 60 s skipped 552 frames and gave exit code 0 at the time (now exit code 3, because it is above 5 %).
- The script checks the mapper's `trajectory_coverage` and the fraction of `estimated` in replay after initialization (from the first frame that is not `dropped` to the end), both at 95 % or more (exactly 95 % passes), and fails if not met. The mapper gives exit code 3 when frames not covered by the IMU exceed 5 % of the selected frames (`failed: true` in `mapper_summary.json`, which the script also treats as a failure). `trajectory_coverage` is divided by the selected frames (inserted + skipped). Cutting the IMU at t0+107 s gives 1039/1153 = 0.901, and cutting it at 7.5 s skips 1078 frames and fails.
- The script normalizes the out directory with `realpath -m` and rejects `/`, `/ws`, `/ws/out`, the repository, HOME, the dataset and its ancestors, and anything inside the dataset. A non-empty directory is cleaned only when it has a marker made by this script (earlier run directories are rejected, so use a new directory). `RESULT: PASS` is printed once, only after all stages and gates have passed.
- The script exits non-zero when the mapper or evaluate.py fails, and deletes only its own outputs before running. Only self-localization enforces the ATE rmse 0.3 m gate; held-out is reported only.
- Increase in clean build time (two translation units, mapper and test_mapper_tool, -j4): about 16 s. 4 tests take about 2.6 s.
- In the first run, no diverged frames and all frames `estimated` held even though the trajectory was wrong. The error could not be noticed without looking at ATE.

## Phase 5 PR 1: Docker layer for glim_ros2 (2026-11-24)

### What was done

- glim_ros2 is pinned to `4d4ec524ccf1b02aa09b0af2af767ecc54343798` (2026-08-14, `glim_ros` 1.2.2) and built from source in a separate layer after GLIM and before Python (merge install into `/opt/glim`). The GLIM pin (1a197c9) is unchanged.
- `third_party/patches/glim_ros2/0001-drop-cv-bridge-depend.patch`: removes only `<depend>cv_bridge</depend>` from `package.xml`. It is applied with `git apply` at build time, and the build fails if it does not apply (having zero patches also fails the build). glim_ros2 itself is not modified.
- Reason: `ament_auto_find_build_dependencies()` makes `cv_bridge` and `image_transport` in `package.xml` required, but every use in the sources is inside `#ifdef` of `BUILD_WITH_CV_BRIDGE` (unused when OFF). Installing `ros-jazzy-cv-bridge` adds 260 packages and 775 MB (including Qt5, VTK, Mesa/libGL and X11), so it was rejected. `ros-jazzy-image-transport` is installed with apt.
- The `rviz_viewer` library has no option in the upstream CMake and is always built (`BUILD_WITH_VIEWER` turns off only offline_viewer and map_editor). `librviz_viewer.so` is deleted in the same `RUN`. `check_deps.sh` is not loosened.
- Added to `check_deps.sh`: the glim_ros2 SHA; that `glim_rosnode`, `glim_rosbag` and `libglim_ros.so` exist; that ldd of `libglim_ros.so` has no Iridescence, glfw, libGL, libX11, Qt, VTK or opencv; and that no opencv or cv-bridge apt packages are installed.

### Measurements

- 4d4ec52 compiled and linked against 1a197c9 (configure also succeeded after the patch). The only warnings were notices about header placement by ament_auto_package.
- Layer build time: colcon build about 29 s (`-j4`, arm64). About 50 s for the whole layer including apt and clone. The whole `docker compose build` (first build of this layer) about 2 minutes.
- Image size: from 2,661,925,625 B to 2,674,028,797 B (+12 MB).
- `scripts/ci.sh` exits 0 (including colcon build, python tests and replay e2e; all 119 ctest tests passed).
- amd64 not verified at this step (arm64 Docker only).
- Fix: deleting `librviz_viewer.so` left `rviz_viewer` in `ament_cmake_export_libraries-extras.cmake`, and `find_package(glim_ros)` failed. It is removed from that export in the same `RUN`, and the build fails if `rviz_viewer` remains in glim_ros's CMake files. Added a consumer check with a throwaway project to `check_deps.sh` (configure, compile `glim_ros.hpp`, link, run).
- Fix: the ldd check treats a failure of ldd itself and `not found` as failures (previously missed with `| grep || true`). The `libglim.so` check uses the same function. The patch loop is `git apply "$p" || exit 1`, so a failure of a patch in the middle also stops the build (the same for the GLIM loop).
- Note for PR 5: the standard GLIM config aborts when loading the GPU odometry with CUDA OFF (rc=134). `config_ros.json` also lists `librviz_viewer.so`. The PR 5 config uses the CPU odometry and does not include this viewer.

## Phase 5 PR 2: msgs (2026-11-24)

- `glim_localization_msgs`: `LocalizationStatus.msg` and `Initialize.srv` (English comments for all fields and constants). Added `scripts/test_interfaces.sh` to `ci.sh`; it checks the installed interfaces, a throwaway C++ consumer, and Python import and use.
- `bool global` is a Python reserved word; `rosidl_generator_py` passed the build with only a warning, and the generated Python had a syntax error (not detectable by the build alone; found only at the Python import). Changed to `bool global_search`. `global_search=true` returns `success=false` and does not change the estimate until [#3](https://github.com/atinfinity/glim_localization/issues/3). An empty request (zero quaternion) is rejected and does not initialize with the identity pose.
- `last_reset_reason` is `uint8` (`RESET_NONE=0`, `RESET_EXTERNAL=1`, `RESET_IMU_GAP=2`). `processing_time` and `last_reset_reason` are not in the core yet, and are 0 / `RESET_NONE` until the core fills them.
- The initial pose from the config is not counted in `reset_count`: the constructor sets `init_T_world_imu` directly as the initial state (`odometry_estimation_localization_cpu.cpp:124-127`). The count is incremented in one place only, `apply_pending_reset` (same file, `:242`).
- Updated the DESIGN 6 rows for `~/status` (number of tiles in use etc.) and `~/initialize` (`global_search`, empty request rejected).
- Follow-ups: PR 3 adds `processing_time` (wall time of `insert_frame` [s]) and `last_reset_reason` (an enum with the same values as the msg) to the core `LocalizationStatus`. PR 4 depends on 2 and 3 and adds a gtest that the msg constants match the core enum. PR 6 validates the quaternion norm and frame_id in `~/initialize` before the reset, and tests empty requests and `global_search=true`.
- Follow-up fixes: `test_interfaces.sh` confirms the install under test (`ros2 pkg prefix`, the Python module path, `cmp` of `.msg`/`.srv` against the sources), actually loads the generated libraries with Python `rclpy.serialization` and C++ `rclcpp::Serialization`/service type support, and pins the types and constants with `ros2 interface show --no-comments`. `success` of `Initialize` means "accepted", not that the reset was applied. The status when a reset is applied can have `stamp = min_stamp` (which may be 0 and is not a frame time). More follow-ups: PR 4 handles this status (does not use it as a pose time). PR 6 decides whether `~/initialize` waits for the reset to be applied.

## Faster local verification (2026-11-24)

- `scripts/ci_local.sh` (host side) wraps `scripts/docker_run.sh scripts/ci.sh`. A clean tree is verified from a git-synced snapshot of the commit tree (`<git-common-dir>/ci_local/src`), never from the worktree, with its own build volume `glim_localization_ci_<clone hash>_build` and one waiting host lock per clone. The first version had four false passes: stale objects in the volume shared by all worktrees (make trusts mtimes), two worktrees building at once, a branch switched during the run, and skip-worktree edits hidden from `git status`. The snapshot rewrites a file whenever its content changes, so make always sees it as newer; the tree is resolved once at start; skip-worktree and assume-unchanged files count as dirty.
- The volume is wiped on `--force`, on an image change, when files were deleted since the last build, when the container clock differs from the host by more than 2 s, and after an interrupted run. The snapshot is hashed before and after the run; a difference records nothing.
- A pass is recorded under the git-ignored `out/ci_stamps/`, keyed by the tree hash and the Docker image ID, and reused only on a clean tree. A dirty tree runs the working tree as before and never reuses or records. `--force` runs everything and drops the old pass first. Nothing is recorded if the run fails or is killed, or the image or snapshot changed during the run. Empty or malformed stamps are ignored. Outputs of a recorded run land in `<git-common-dir>/ci_local/src/out`.
- The volume is also wiped when a CMake file, `package.xml` or a build script differs from the last built tree, because a cached `option()` default would otherwise survive the change (a tree whose fresh build fails could record a pass). After a run the snapshot is `chown`ed back to the host user through a container, because the container runs as root and root-owned `out/` would break `git clean` on native Linux Docker (simulated only; native Linux is not verified).
- The image carries the repository files it bakes in (`docker/Dockerfile`, `docker/entrypoint.sh`, `scripts/requirements.txt`, `third_party/patches/glim/`, `third_party/patches/glim_ros2/`) under `/etc/glim_localization/inputs/`. A clean run is refused before the build when their sha256 listing differs from the tree's, so a recorded pass means the image matches this tree. A test fails if a Dockerfile `COPY` source is not in the list `image_inputs` of `scripts/ci_local.sh`. The image must be rebuilt once after this change.
- The dependency check is skipped only when a recorded pass matches the image ID and the hash of `check_deps.sh`, `requirements.in`, and `scripts/check_deps/`; the wrapper passes `ci.sh --skip-deps`. `ci.sh` run directly (GitHub Actions) always runs it.
- Docs-only changes (`is_docs_only.sh` on the diff from the merge base with `origin/main`, clean tree only) skip build and tests with a clear message and are never recorded as a full pass. An empty diff is not docs-only.
- ctest runs with `-j${CTEST_JOBS:-4}` (default changed to the CPU count on 2026-11-25; compose passes `CTEST_JOBS` through when set). C++ tests are not selected by changed path.
- The container itself runs from a private tag `glim_localization:ci-<pid>` made from the resolved image ID (`GLIM_LOC_IMAGE` in `docker/compose.yaml`, default `glim_localization:dev`), removed on exit by a trap, so a tag moved away and back during the run cannot record a pass for an image that never ran. `docker_run.sh` refuses a missing `GLIM_LOC_IMAGE` because `compose run` would build it from the current tree. A SIGKILL leaves the private tag behind (harmless; `docker rmi` it).
- Tests: `scripts/tests/test_ci_local.py` with stubbed docker and `docker_run.sh`.

## Phase 5 PR 3: core IMU gap policy and replay quality gate (2026-11-24)

- The gap is defined in one place, `check_imu_coverage` in `odometry/imu_gap.hpp` (shared by `ReplayRunner` and the odometry module). The module overrides `insert_imu` and keeps its own IMU stamp history (upstream `IMUIntegration` removes integrated samples from the queue, so the gap cannot be judged from the queue).
- Online: if the gap is at most `imu_gap_reset_max` (0.5 s), it resets itself (`reset_count` + 1, `last_reset_reason` = `IMU_GAP`); if longer, it goes LOST and gives no pose until an external initialization. Settings are `on_imu_gap` (default `reset`, `off` behaves as upstream), `max_imu_gap` and `imu_gap_reset_max`.
- replay: `--on-imu-gap abort` (default, exit code 3) / `reset` (`skipped_no_imu`, `imu_gap_resets` in `summary.json`, 3 above 0.5 s). A late initial pose beyond `max_init_pose_delay` (0.2 s) gives exit code 2. LOST gives exit code 1. The tool checks `--min-tracking-ratio` (default 0.9, 0 disables) and writes it to `summary.json`. `ReplayRunner` itself has no gate, and the library tests do not change.
- The core `LocalizationStatus` has `processing_time` and `last_reset_reason` (`ResetReason`, with the same values as `RESET_*` of the msg. The core does not include the msg). PR 4 pins the matching values with a gtest.
- Tests: gaps of 0.49 s / 0.51 s (both abort and reset modes, and the policy of the module alone), an initial pose 0.19 s / 0.21 s late and unspecified, a tracking ratio exactly at the threshold and slightly below, abort on LOST, exit codes. Confirmed that the corresponding tests fail when each check is temporarily disabled (gap: the runner and module limits; delay: the rejection; ratio: `passes`; LOST: the abort). 8 tests failed in total.
- Ambiguities decided: (1) The gap length is the missing interval (the difference between the samples on both sides); with fewer than 2 samples, frame stamp - stamp of the last pose. (2) The initial pose delay is measured to the first frame at or after `--init-stamp` that the IMU covers. A warning is given, even for a delay of 0.2 s or less, only when frames not covered by the IMU were skipped. (3) `max_init_pose_delay` is a replay option only (online is PR 6). (4) The exit code for an insufficient tracking ratio is 1. (5) In reset mode, the odometry drops the first frame where the IMU is clean again after the gap (because there is a gap between it and the last pose), and reinitializes at the next frame.
- Effect of the default gates: `scripts/test_replay_e2e.sh` is unchanged (PASS, all 121 frames `estimated`). `scripts/smoke_os1_128.sh` (the private OS1-128 recording above) also PASSes unchanged: there is a revisit and the held-out branch runs, all 302 frames `estimated`, `tracking_ratio` 1.0 (default 0.9), `imu_gap_resets` 0, no LOST. The self-localization branch that uses `--init-stamp` does not run on this data (not verified).

### Fixes: longest gap, reset priority and stamp order

- replay: judged by the longest gap in the window (a short gap does not hide a long one). Another gap right after recovery is not treated as a hole at the start (`had_pose`). An empty cloud (no points left after preprocessing) does not satisfy the initial pose delay check.
- odometry: IMU samples out of order are not added to the history (upstream discards dt <= 0). An external `reset()` always takes priority over the self reset (`queue_reset` checks and writes under the same lock). The IMU stamp history is trimmed also while LOST and during a gap.
- Tool: when config `on_imu_gap` is not `reset`, `--on-imu-gap reset` gives exit code 2. `evaluate.py` rejects runs that fell short of the ratio gate.
- Tests: added a regression test for each fix, and confirmed that each fails when the original behavior is restored one at a time. Mutants that the earlier tests missed (extrapolation by velocity, reset clears an IMU LOST, an IMU LOST persists, `processing_time`) now also fail.

### Fixes: gap window and IMU stamp gate

- The interval for the gap check is now the whole span from the previous pose (the recovered frame) to the current frame, rather than starting from the previous pose or covering only the window (both replay and the module). Even for a dropout with no LiDAR frame, exceeding `imu_gap_reset_max` gives exit code 3 in replay and LOST in the module. Even if a recovery frame comes during an episode, the longest gap so far is used. `had_pose` was removed because the start point at recovery made it unnecessary.
- The history repair above (reordering and discarding future stamps) was dropped and replaced by a gate. Even with the history repaired, abnormal stamps remained in the upstream queue, and the two disagreed: integration stopped or diverged, a late batch hid a real dropout, and a clock rewind made `v * dt` negative. `insert_imu` passes only finite, strictly increasing stamps; after 5 consecutive drops it clears the upstream queue and the history and goes LOST; a frame stamp not newer than the previous one also gives LOST (DESIGN 6.5). The earlier statement that "the history tolerates abnormal stamps" does not hold for the upstream queue.
- Each test was confirmed to fail when its fix was reverted one at a time.

### Fix: a hole before the restart sample after an IMU fault

- A fault (5 consecutive rejections) clears the queue and restarts from that sample. When a frame older than the restart stamp became the first pose, the next frame had no "sample at or before the previous pose", `check_imu_coverage` took sample 0 as the start and missed the hole, and upstream integrated while holding the first sample over the hole (0.09 s; 0.44 s with a 0.4 s stop; the same when the `min_stamp` of an external `reset()` is before the restart). When there is no sample at or before the previous pose, the distance to the first sample also counts as a gap (only when it exceeds `max_imu_gap`. A longer gap in the window wins). The first frame itself does not integrate an IMU interval, so it is not checked. In a normal start where the IMU begins right after the first frame, this is not a gap if it is within the limit.
- Tests: a frame before the restart becomes the first pose (2 cases: with no pose, and an external `reset()` during a run) x restart at 3.09 s / 3.44 s, controls within the limit (0.04 s) and with `min_stamp` after the restart, and rejection of an equal stamp (the difference between `<=` and `<`). Before the fix 2 tests failed, reverting the fix makes the same 2 fail, and changing `<=` to `<` makes the equal-stamp test fail.

### Fix: the trimmed head of the history

- Regression from the previous fix: it assumed the head of the history was "the first sample after the hole", but a dropped frame trims the history, so at the next frame the head is at the trimmed position, and the span from the previous pose to the head was measured as a hole. Even gaps within the limit (0.30 to 0.49 s) exceeded `imu_gap_reset_max` and gave LOST (exit code 1 in replay). The 0.49 s test missed it because the hole was on the frame grid and did not exceed exactly 0.5 s.
- Fix: remember the newest stamp removed by trimming (`imu_trimmed_stamp_`) and pass it to `check_imu_coverage` as the sample just before the head. The distance is measured between samples that actually arrived next to each other, and `imu_trimmed_stamp_` is cleared only when a fault clears the history (that restart sample has no predecessor). An external `reset()` does not touch the queue, so it leaves `imu_trimmed_stamp_` as is. The history stays a copy of the upstream queue as before.
- Tests: gaps of 0.30, 0.35, 0.40, 0.45, 0.49 s (within the limit) and 0.51 s (above the limit) are swept over 10 phases relative to the frames, with scan durations of 0 and 0.09 s, and the replay runner (exit code 3 or completion) and the module (LOST or self reset) must give the same decision, with no pose across a hole (about 25 s). The initial pose is placed at 3.8 s and only frames from 3.8 to 5.0 s are used to limit the time. A mutant that does not use the sample before the head (`*last_stamp`) gives 215 failures.

### Fix: an old first frame after an external reset

- The `predecessor` of the previous fix excludes from the check a hold over an interval that upstream has already consumed, when the previous pose is older than the trimmed position of the history. After an external `reset()`, when an old frame (at or after `min_stamp`, before the frames seen so far) became the first pose, the next frame held from the `reset` stamp to the head of the upstream queue and still gave a pose (age 0.10 s to 2.00 s gave a hold of 0.11 to 2.01 s. Reproduced in the real module).
- Decision: `check_imu_coverage` is unchanged. At the entry of `insert_frame`, a frame is dropped without a pose when `frames` is empty (right after a reset) and its stamp is not newer than the newest frame seen so far (`newest_frame_stamp_`). Invariant: within one IMU epoch (from the start, or from the last IMU fault), all stamps that reach the trimming and upstream consumption increase monotonically with the frame stamps. So the last pose is never older than consumed data, and `predecessor` is used only within a gap episode. The watermark (`newest_frame_stamp_`) is not cleared by `reset()`, only by an IMU fault (so `reset()` can still recover after both clocks went back). The self reset happens in `handle_imu_gap` before `frames` becomes empty, so that frame becomes the first pose.
- The tests use an oracle that reads the actual upstream queue (`ProbeOdometry::max_hold`: the hold from the previous pose to the head of the upstream queue, before giving a pose) and check `<= max_imu_gap`. Frames of age 0 (equal stamp) to 2.00 s, 100 seeded random streams, 2 controls (`reset()` into the past, both clocks go back 100 s), and named tests with gaps from 3.95 to 4.44 s. The sweep now uses the 5 even-numbered phases (of 10). Mutants that remove the gate, remove the clear on a fault, or change `<=` to `<` each fail.
- Follow-up tasks: recovery from a rewind of the LiDAR clock alone, and that replaying frames after `reset()` can give a false LOST.

### Fix: a monotonic frame watermark and a check of the upstream queue

- The gate of the previous fix updated `newest_frame_stamp_` unconditionally, so when an old frame (LOST, or dropped below `min_stamp`) passed while `frames` was not empty, the watermark went down, and after a later `reset(min_stamp = t)` the frame at `t` became the first pose and the hold came back (when the LiDAR clock goes back twice with a reset between. t = 4.8 / 4.4 / 3.0 s gave holds of 0.11 / 0.51 / 1.91 s, and through `min_stamp` 0.11 / 0.51 / 1.31 s). Reproduced in the real module before the fix.
- Fix 1: the watermark is never lowered within an IMU epoch (updated with `max`). Fix 2: a check against the actual upstream queue was added only at the coverage-OK `return false` of `handle_imu_gap` (`longest_imu_hold`: skip the samples at or before the previous pose, hold each sample until the next one and the last one until the frame stamp, and take the largest interval). If it exceeds `max_imu_gap`, it goes LOST as an internal inconsistency and counts `hold_check_trips()`. This cannot happen if the history is correct, so it is 0 on a valid stream (confirmed in all controls and sweeps).
- The test oracle was changed to the same definition (the largest interval from the previous pose to the frame, including the tail) and written separately from the production helper. Only integrations where the queue has a sample at or after `scan_end_time - 1e-6` (covered) are judged with `max_hold`; the others (uncovered) are counted and not judged. The seeded random streams have 154 uncovered integrations, up to 0.2 s, and the other tests check `uncovered_integrations == 0`.
- Known limitation (follow-up task): a frame with `INCOMPLETE` (the end of the queue is before the end of the scan) outside a gap episode is not checked and gives a pose, as before. The upstream async wrapper holds frames while `scan_end_time > last_imu_time`, so such frames reach the module in two ways: (a) the end_of_sequence flush, and (b) the `last_imu_time` recorded by the wrapper comes from a sample our gate dropped. The likely fix is "drop without a pose" rather than LOST. Making a frame LOST at the flush would change the normal shutdown behavior, so it is not handled here. `on_imu_gap: off` behaves the same.

## Phase 5 PR 4: localization_publisher (2026-11-24)

- `glim_localization_ros2/` now builds the `localization_publisher` library (ROS only; the core is untouched). It publishes `~/pose`, `~/odometry`, `~/status`, `/diagnostics` (plain `diagnostic_msgs`) and TF. The input is a plain `FrameEstimate` plus the core's `LocalizationStatus`, so R1 and R2 share it and the tests need no GLIM objects.
- `tf_mode`: `internal_odom` integrates the frame-to-frame motion into a continuous odom->base and puts the rest in map->odom. (Superseded by the anchor rule below.) `external_odom` looks up odom->base at the frame stamp and publishes map->odom = T_map_base * inv(T_odom_base); without it, pose and odometry are still published and no TF is. `none` creates no broadcaster.
- Diagnostics: OK for INITIALIZING and TRACKING, WARN for DEGRADED, ERROR for LOST. Published on a level change, otherwise at most every `diagnostics_period` (1 s).
- A reset status has `frame_id < 0` and `stamp = min_stamp`, which may be 0. It is published on `~/status` stamped with the last frame stamp (the node clock before the first frame), never produces a pose or TF, and makes the next frame re-anchor odom->base (external resets only, see below).
- `test_message_constants` pins the core `ResetReason` and `LocalizationState` values to the message constants.
- `scripts/test_colcon.sh` runs `colcon test` for `glim_localization_ros2` and is called from `ci.sh` after `build_colcon.sh`. The core tests still run only in `build_core.sh` (`build_colcon.sh` keeps `-DBUILD_TESTS=OFF`).
- Not done: pose covariance (left at zero, meaning not estimated) is not estimated yet; the IMU-rate prediction is not published.
- Publication fix (replaces the jump guard): a frame is published only with the status of the same frame (no one-frame lag); LOST frames give no `~/pose`, `~/odometry` or `map->odom`, and `odom->base` continues in `internal_odom`; `map->odom` is an anchor that changes only at the first frame after an EXTERNAL reset, so `odom->base` does not step at a reset after any pause. `jump_translation`, `dt` and velocity are gone from the TF path. Corrections without a reset enter `odom->base` by definition (DESIGN 6, "Publication rules"). `FrameEstimate::w_imu` was added so that the twist includes the lever arm and the angular velocity. Older frame stamps are dropped, external odom is looked up at the stamp floored to 1 us, a non-frame status is stamped with the last frame stamp, the gtests set `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, and `build_colcon.sh` forces `BUILD_TESTING=ON` while `test_colcon.sh` fails on a cached OFF, stale results, or a missing result file of an expected gtest.

## Phase 5 PR 5: R1 extension and synthetic bag fixture (2026-11-24)

- `libglim_localization_extension.so` (`glim_localization_ros2/src/localization_extension.cpp`) is a GLIM `ExtensionModuleROS2`. It turns `OdometryEstimationCallbacks::on_update_new_frame` into `FrameEstimate` and the core's `LocalizationCallbacks::on_status` into `~/status`, both through `LocalizationPublisher`, and forwards `/initialpose` (a base-frame pose in the map frame; `T_map_imu = T_map_base * T_base_imu`) to `LocalizationCallbacks::request_reset` with `min_stamp` = the message stamp (0 when unstamped). A pose in another frame, a non-finite pose or a zero quaternion is logged and dropped. The topics are on the GlimROS node, so `~/pose` is `/glim_ros/pose`.
- The callbacks stay in the static `CallbackSlot`s after the extension is destroyed (the slots are not safe to modify while the odometry thread runs), so they check an `active` flag under a mutex that the destructor takes.
- `config/r1/` is the CPU odometry config: `config_ros.json` lists only our extension, local and global mapping are off, and the sub and global mapping `so_name` are empty (`config_sub_mapping_none.json`, `config_global_mapping_none.json`). `librviz_viewer.so` is neither listed nor loaded. The extension reads its own `localization_ros` section of `config_ros.json` (`tf_mode`, `base_frame_id`, `T_base_imu`, `diagnostics_period`); map and odom frame names come from `glim_ros`. The config is installed to `share/glim_localization_ros2/config/r1`; `map_path` is empty and must be set.
- `scripts/replay_to_bag.py` writes a replay dataset as a bag (`/points` Ouster layout, `/imu`) and optional `/initialpose` messages taken from `traj_gt.txt` (`--initial-pose T[:dx,dy,dz,yaw_deg]`). It reuses `write_bag` from `scripts/tests/test_bag_to_replay.py`, which gained an optional `extra` argument for more topics.
- Launch test `test/test_r1_extension.py`, run by `scripts/test_colcon.sh` (which makes the 12 s synthetic map, dataset and bag first): `glim_rosbag` over the bag with `delay:=2.0` (so that the test's subscribers are connected before the bag is read) and `auto_quit:=false`; the test waits for the last pose with a 180 s deadline that fails the test. It checks the pose count (>= 90 % of the frames), the error against ground truth after 3 s (0.1 m, 1 deg), `map->odom` and `odom->base_link` at >= 90 % of the frames and that they compose to the pose (checked for the last 5 poses only), `reset_count` equal to the two bag `/initialpose` messages with reason EXTERNAL, state TRACKING at the end, the config, and `ldd` of the extension and the odometry module (no Iridescence, glfw, libGL, libX11, nothing unresolved).
- `GlimROS` finds both libraries through `LD_LIBRARY_PATH` after `source install/setup.bash`: ament puts each package's `lib` there, so no patch of GLIM or glim_ros2 is needed.
- After the publication fix: the extension keeps the last second of raw gyro (`GyroBuffer`, header only, tested without ROS by `test_gyro_buffer`) from `OdometryEstimationCallbacks::on_insert_imu` and sets `FrameEstimate::w_imu` = nearest gyro to the frame stamp minus `frame->imu_bias.tail<3>()` (a frame is not published, with a throttled warning, if the nearest gyro is more than 20 ms from the frame stamp, which also covers an empty window and an odometry that lags by more than the 1 s window). The IMU, frame and status callbacks run on the `AsyncOdometryEstimation` thread (`insert_imu`, `insert_frame` and the reset applied there); the mutex remains for `/initialpose` and the destructor. `jump_translation` is gone from the extension and `config/r1`. `scripts/test_colcon.sh` also requires the launch test's xunit file.
- Measured after the publication fix (4 runs of `scripts/test_colcon.sh`, CPU, Docker on arm64): 116 to 117 of 121 poses, worst error 0.016 m and 0.090 deg after 3 s, `reset_count` 2 (both bag `/initialpose` messages count). The launch test takes about 4.5 s; all four runs passed.

## Phase 5 PR 6: R2 lifecycle node (2026-11-24)

- `glim_localization_node` (`glim_localization_ros2/src/localization_node.cpp`, component `glim_localization_ros2::LocalizationNode`, node name `glim_localization`) is a lifecycle node. Input handling (IMU scale and offsets, point extraction, `TimeKeeper`, `CloudPreprocessor`, QoS from `config_ros.json`) is derived from glim_ros2 `glim_ros.cpp` and `ros_qos.cpp` at 4d4ec524ccf1b02aa09b0af2af767ecc54343798. The node holds `OdometryEstimationLocalizationCPU` directly (no dlopen) inside `AsyncOdometryEstimation`. What differs from the plan is in DESIGN 6, "R2 node: differences from the plan".
- `EstimateBridge` (`estimate_bridge.{hpp,cpp}`, in the `localization_publisher` library) is the part that R1 and R2 share: it connects the static GLIM and core callback slots once, keeps the gyro window, turns frames into `FrameEstimate` and statuses into `~/status`. The R1 extension now uses it.
- Late initial pose: `check_init_pose_delay` (`init_pose_delay.hpp`, header only, gtest `test_init_pose_delay`, 7 cases) compares the stamp with the core's `last_input_frame_stamp()`, the last frame the odometry thread took off its queue, estimated or dropped (frames still queued are not counted). More than `max_init_pose_delay` (0.2 s) late is rejected; a shorter delay is applied with a warning; stamp 0 is the next frame. A 1 us tolerance keeps a delay of exactly 0.2 s accepted at epoch-sized stamps.
- `config/r2/`: its own `config_ros.json` (topics `/imu` and `/points`, sensor_data QoS with IMU depth 1000) and `config.json` pointing at the `config/r1` files for everything else.
- `scripts/replay_to_bag.py --drop-imu START:DURATION` (repeatable) removes the IMU samples in `[first frame + START, + DURATION)`; removing nothing is an error. Tested in `scripts/tests/test_replay_to_bag.py`.
- Launch tests, run by `scripts/test_colcon.sh` (which now also requires their xunit files): `test_r2_node.py` starts the node with a missing map (configure fails, the node stays unconfigured), sets `map_path`, configures and activates, plays the R1 bag with `ros2 bag play --clock` and `use_sim_time`, checks the R1 bounds, TF, the resets and the services, then deactivates and cleans up (details in the fixes below). `test_r2_imu_gap.py` plays a bag with no IMU for 0.3 s from 6.0 s and checks that the node resets itself once (`IMU_GAP`, `reset_count` 2), is TRACKING at the end, and is within 0.1 m and 1 deg from 3 s after the hole. Both use reliable, deep input QoS in a copied config, so DDS does not drop input.
- Startup loss seen in the tests: the player starts paused and is resumed once its publishers are visible to the test plus 2 s. Even so, in the IMU gap test the node received nothing from about the first second of playback in 4 of 8 runs, two of them while two test runs overlapped by mistake (the first IMU logged about 1 s after resume), so a `/initialpose` at the first frame was lost and no pose came. The cause is not known (DDS matching of the new node is the guess). Confirmed in [#20](https://github.com/atinfinity/glim_localization/issues/20): see "R2 launch tests: lost start of the bag" below. The gap test therefore sets the pose of the frame at 2.0 s through `~/initialize` before playing, stamped with that frame; the gap bag has no `/initialpose`. `test_r2_node.py` did not show the loss in 8 runs; it now also sets its first pose through `~/initialize` and fails with a clear message (not on the deadline) when the bag ends and no pose came. A real robot should publish `/initialpose` after the node is active and matched, or use `~/initialize`.
- Measured (3 runs in a row of `scripts/test_colcon.sh`, CPU, Docker on arm64): R2 node 121/121 poses, worst error 0.016 m and 0.090 deg after 3 s, 121 frames with both `map->odom` and `odom->base_link`, `reset_count` 2; IMU gap 98/121 poses (20 before the initial pose, the rest around the hole), `reset_count` 2 with reason IMU_GAP, worst error 0.011 m and 0.060 deg over 28 poses after recovery. R1 in the same runs: 117 to 118 of 121 poses. All three runs passed; each R2 launch test takes about 16 s.
- Not verified: a multithreaded component container, the real-time CPU load, and real bags. glim_ros2 has no LICENSE file in the image; its `package.xml` says MIT.
- Follow-up fixes:
  - Late-pose check: the bridge saw only estimated frames, so before the first pose any stamp passed. The core now records `last_input_frame_stamp` at the top of `insert_frame`, before any drop, and the node checks against it. `test_r2_node.py` plays the bag once with no pose and then requires a stale `~/initialize` to fail as too late. Mutation check: passing no stamp to `check_init_pose_delay` makes the test fail with "a stale ~/initialize was accepted before the first pose".
  - Relocalize moved into the core as `relocalize(max_age)`, called with `max_init_pose_delay`. It is refused while a reset is pending (so it cannot drop an accepted `~/initialize`), unless the state is TRACKING or DEGRADED, and when the newest estimate is more than `max_age` older than the last input frame. When applied on the odometry thread, it takes the newest estimate, moves it to the frame by the velocity and turns it at the rate between the last two estimates. Without the move the first frame was 0.20 m off at about 2 m/s, and without the turn 4.3 deg off. New gtest `RelocalizesFromTheNewestEstimate`. The bridge no longer keeps an estimate.
  - `test_r2_node.py`: the first pose comes from `~/initialize` before play; the bag's `/initialpose` at 5 s is still checked as applied. `~/relocalize` is called at the first TRACKING status from 8 s and must be followed by at least 10 poses within the bounds. After deactivate, the test checks that no pose, status or TF arrives, and that the subscriptions and publishers are gone. It also covers activate, deactivate, activate, and configure again after cleanup.
  - DESIGN: the services exist only while configured, the relocalize rule, the late-pose rule, and early message loss.
  - Measured after the fixes, on the head merged with main (3 runs in a row of `scripts/test_colcon.sh`): R2 node 121/121 poses, worst error 0.016 m and 0.298 deg after 3 s, at the first frame after `~/relocalize` (8.1 s); 0.090 deg without it. 121 TF pairs, `reset_count` 3 in these runs; the test accepts 3 or 4, since the bag's 0 s `/initialpose` either replaces the pending `~/initialize` or is applied separately. Relocalized at 8.00 s; IMU gap unchanged (98/121, 0.011 m, 0.060 deg); R1 116 to 118 of 121. All three runs passed; the R2 node test now takes about 24 to 27 s.
  - Relocalize checked again when applied: a relocalize was checked only when requested. It is now checked again when applied, and dropped with a warning if the odometry went LOST in between or the next frame is more than `max_age` after the newest estimate. Before this fix, a stall had the turn scaled by the stall length, skipped the IMU gap policy, and could clear LOST. New gtests: `RelocalizeIsDroppedAfterAStall` (0.6 s stall, max_age 0.2 s; also checks the refusal while INITIALIZING) and `RelocalizeIsDroppedWhenLostBeforeItApplies` (an IMU fault from repeated stamps after the request; also checks the refusal while LOST). Mutation check: with both apply-time checks disabled, each test fails on its own check (the stall test sees id 0 and `reset_count` 1; the LOST test sees estimates at 6.0 s and later). The drop is not reported in `~/status`.

## Phase 6 PR 1: map_compiler PCD input (2026-11-24)

- `map_compiler <file.pcd> <output_dir> [--grid-size m] [--voxel-size m] [--k-neighbors n]` builds a compiled map from a point cloud. The core gets `map/pcd_map.{hpp,cpp}` (`read_pcd`, `build_pcd_submaps`; ROS-free, no PCL). `save_compiled_map` takes the `source` recorded in `map_index.json` (`"pcd"` here). `load_map` and localization are unchanged.
- Design choices (the issue left them open):
  - PCD reader: `DATA ascii` and `binary`; `binary_compressed` is rejected with an error. x, y, z must be single F4 or F8 fields; all other fields (intensity, rgb, normals, any COUNT) are skipped. Points with a non-finite coordinate (organized clouds) are dropped. VIEWPOINT is ignored. Little-endian host assumed.
  - A `.pcd` input is detected by its extension (case-insensitive); the options are rejected for dump input.
  - Covariances: `gtsam_points::estimate_covariances` with k = 10 (GLIM's `k_correspondences` default) and its default EIG regularization, computed over the whole cloud before tiling, so tile edges use neighbours across the edge. There is no special case for sparse points: every point takes its k nearest, wherever they are, and the regularization fixes the eigenvalues. A cloud with fewer than k points is rejected.
  - Tiling: a 2D grid along world x and y, 20 m by default, spanning all z. No overlap between tiles (MapManager already uses up to 5 nearby submaps). Each non-empty cell is a submap with identity rotation and its origin at the centroid of its points (keeps float storage precise far from the world origin). Ids are 0, 1, ... in (x, y) cell order. Small tiles are kept.
  - Downsampling: off by default (`--voxel-size 0`); otherwise `gtsam_points::voxelgrid_sampling` around the bounding-box centre before covariance estimation.
  - Covariance estimation uses all hardware threads in the CLI.
- Tests (synthetic only, no PCD committed): `test_pcd_map` (ascii and binary, F4 and F8, extra fields with COUNT, NaN points, CRLF, 13 broken or unsupported files, grid split invariants, flat-floor covariances, downsampling, bad parameters, round trip through `save_compiled_map` and `load_map`); `test_map_compiler` (CLI with binary and ascii PCD, options, bad options, broken PCD leaves no output); `test_replay_tool` `LocalizesOnAMapBuiltFromPcd` (3 s replay from a pose 0.3 m and 3 degrees off, every frame within 0.1 m and 1 degree after 1 s); `test_replay_e2e.sh` now also compiles `map.pcd` with `map_compiler` and passes `evaluate.py` (12 s, 0.1 m and 1 degree after 3 s).
- Not done: PLY input and the 2D OccupancyGrid export ([#4](https://github.com/atinfinity/glim_localization/issues/4)). Recompiling a compiled PCD map records `source: "glim_dump"`. Not tried on a real PCD map from another tool, such as one generated from a Gazebo world with sdf2map ([#9](https://github.com/atinfinity/glim_localization/issues/9)).
- Follow-up fixes:
  - The header bounds COUNT (2^20) and the point size (2^24 bytes and values), and does size arithmetic in size_t. POINTS is checked against the bytes left in the file before allocating, and WIDTH x HEIGHT is checked for overflow.
  - `--voxel-size` corrupted maps far from the origin: `voxelgrid_sampling` packs 21 bits per axis and merged a plane at (500000, 4000000) into 10 made-up points, with exit 0. Downsampling now runs around the bounding-box centre, and a cloud too wide for the voxel grid fails with an error. gtsam_points is not patched.
  - Lower bounds: grid >= 1 m, voxel 0 or >= 0.01 m, k >= 5 (k = 1 gave zero covariances), all finite, checked by `check_pcd_map_params` in the core and the CLI. Repeated field names are rejected, except PCL's `_` padding. A `static_assert` requires a little-endian host. A `.pcd` input is now detected by extension only, so a missing file is reported as "file not found".
  - New tests catch two mutants that the earlier tests missed: `SplitsCellsAcrossTheAxes` (truncation instead of `floor`) and `CovariancesUseNeighboursAcrossTileEdges` (covariances estimated per tile). Every new guard was checked by a mutation build that copies the sources (no git), and each mutant fails its test. The offset and ascii index checks in the reader are defence in depth and cannot be reached on their own.
  - The disjoint 20 m tiles with `map_max_submaps` 5 cover only about 40 x 40 m (DESIGN §5.1 and the usage text suggest `--grid-size` 50 to 100 m for long-range LiDAR). MapManager is unchanged.
  - Follow-ups: `binary_compressed` input; memory per point (about 300 bytes) and the `int` count in `estimate_covariances` for 100M-point clouds; an upstream issue for `voxelgrid_sampling` outside its range.

## Lighter GitHub Actions runs (2026-11-25)

- Prepares CI for the public repository, where the build image and the C++ builds dominate the run time.
- `scripts/image_inputs_hash.sh` owns the list of repository files the image bakes in. `--manifest` prints the sha256 listing that `scripts/ci_local.sh` compares with `/etc/glim_localization/inputs/` (behaviour unchanged); without it, it prints the SHA-256 of that listing, which CI uses as the image tag. The test that every Dockerfile `COPY` source is listed now reads this script.
- CI pulls `ghcr.io/<repository owner, lowercased>/glim_localization:<hash>` and builds the image (buildx, `type=gha` cache) only when the pull fails. The workflow defaults to `contents: read`; `build-and-test` adds `packages: read`. A separate `publish-image` job, the only one with `packages: write`, runs only on a push to `main` of a non-fork repository when the image was missing, after the checks passed, and pushes it with the same build arguments.
- ccache: installed in a late image layer, and used by `build_core.sh` and `build_colcon.sh` only when `CCACHE_DIR` is set. `docker_run.sh` mounts the host directory `GLIM_LOC_CCACHE_DIR` as `/ws/ccache` when set. CI restores it with `actions/cache/restore` on every run (key: image hash and a hash of the package sources; fallback: any cache for the same image) and saves it with `actions/cache/save` only on pushes to `main`, so PRs reuse main's cache without writing their own.
- Review follow-ups:
  - `publish-image` rebuilds the image from the buildx layer cache instead of pushing the one `build-and-test` ran; reusing it would need a multi-GB artifact or `packages: write` in the test job. If the `ros:jazzy-ros-base` tag moved or the cache was evicted in between, the pushed image is a fresh, untested build. The tag identifies the repository's image inputs, not the image content, so base image and apt updates reach the published image only when an input changes or the tag is deleted on GHCR. The base digest is not pinned yet.
  - CI sets `CCACHE_MAXSIZE=500M` (passed through by `docker_run.sh`) so that saved caches do not evict the layer cache, and falls back to any `ccache-` entry when the image hash changed.
  - The pull is retried 3 times with a short backoff. `publish-image` runs only when the registry reports the image absent (`docker manifest inspect`: "manifest unknown" for a missing tag, "denied" for a package that does not exist yet). Any other persistent error builds the image for the run but does not publish, so a transient failure cannot cause a rebuild and re-push of an existing tag. Checked locally against a missing tag, a missing package and an unreachable registry.
  - Without `CCACHE_DIR` the build scripts pass empty `CMAKE_{C,CXX}_COMPILER_LAUNCHER`, so a build directory configured once with ccache stops using it.
- Draft PRs are skipped (`pull_request` types opened, synchronize, reopened, ready_for_review). Docs-only detection, concurrency and paths-ignore are unchanged.
- Verified locally: actionlint 1.7.12 and shellcheck on the changed files, the `ci_local` tests including a new one for the hash script, a full `scripts/ci_local.sh` pass with the rebuilt image, and a ccache run in the container (rebuild of core and colcon after wiping the build directories: 84 of 86 compilations were cache hits).
- Not verified until the repository is public: the GHCR pull and push, the package visibility (a new package may need to be made public once so that fork PRs can pull it; otherwise they build the image), and the `actions/cache` restore and save across runs.

## Faster colcon build and test (2026-11-25)

- **colcon test runs in parallel.**
  - The ROS gtests (`test_message_constants`, `test_localization_publisher`) use `ament_add_ros_isolated_gtest`.
  - The three launch tests pass `run_test_isolated.py` from `ament_cmake_ros` as their `RUNNER`, since Jazzy has no isolated launch test helper.
  - Each test reserves a free `ROS_DOMAIN_ID` (1 to 100) for its run. `ROS_AUTOMATIC_DISCOVERY_RANGE` stays `LOCALHOST`.
  - `test_colcon.sh` unsets `ROS_DOMAIN_ID`, because the runner keeps one that is already set.
  - `test_colcon.sh` passes `--ctest-args -j${CTEST_JOBS:-$(nproc)}`.
  - The R1 launch test now gives `glim_rosbag` its own `dump_path` instead of `/tmp/dump`. The other output paths were already per-test temporary directories.
- **The core is compiled once.**
  - In the colcon build tree, `glim_localization` is a link to `/ws/build/core`, which uses the same compile options. colcon reconfigures it with `GLIM_LOCALIZATION_USE_AMENT=ON` and installs it, and make finds it up to date.
  - colcon's `cmake_args.last` is removed first, so the reconfigure always happens.
  - After the build, `build_colcon.sh` checks four things: the cache has the ament option, the ament config and index entry are installed, and `glim_localization_ros2` found the core through that config.
  - Rejected alternatives:
    - Skipping the core in colcon and overlaying the plain CMake install would lose the ament package check.
    - ccache is opt-in.
    - `--cmake-force-configure` would reconfigure every package.
- **colcon build uses `--parallel-workers 2`** and `MAKEFLAGS=-j${CMAKE_BUILD_PARALLEL_LEVEL:-4}`; a `MAKEFLAGS` set by the caller is kept.
  - There is no `-l`. A first version used `-jN -lN` (the body of the commit that shares the core build still says so); it was removed in the next commit. Right after the parallel core tests the load average stays high and held `glim_localization_ros2` to about one job (60 s instead of 30 s).
- **The core ctest defaults to the CPU count.** `CTEST_JOBS` still overrides it.
  - The single-test times below are from the `ci_local --force` runs of the timings table. The commit that splits the tests quotes 24 s and 19 s from an earlier incremental run under lighter load.
  - The gap phase sweep (29.0 s at -j4) is now 12 cases: one per scan span and gap, each over the same 5 phases.
  - The 100 seeded random streams (22.9 s at -j4) are now 10 cases, one per remainder of the seed mod 10.
  - The checks over the whole sweep now apply to each case: at least 2 streams on the two-old-frames path and 3 covered integrations per stream. This implies the former totals of 20 and 300.
  - The same 154 uncovered integrations appear, so the streams are unchanged.
  - Each case now states the bound it relies on: a `static_assert` checks that every case has at least 2 seeds that force two replays (seeds above 60).
  - The longest core test is now `AFirstFrameOlder...` (18.9 to 19.8 s at -j8). The longest test overall is the R2 node launch test (about 24 s), which bounds colcon test.
- Review follow-ups:
  - `build_core.sh` sets `CMAKE_INSTALL_PREFIX` to its own install directory, so the shared build directory does not keep colcon's prefix.
  - The R1 launch test puts its config copy and dump in one temporary directory and removes it when the test process exits.
  - GlimROS's `/tmp/glim_log.log` is written only with `debug:=true`, and no parameter moves it. The R1 test does not set debug; a comment there notes the file for future GlimROS-based launch tests.
- **Timings**, from `scripts/ci_local.sh --force` on the 8-CPU Docker VM of the development Mac:

| Step | Before | After (3 runs) |
| --- | --- | --- |
| Whole run | 260 s | 186, 188, 190 s |
| Core ctest | 62.1 s (-j4, 196 tests) | 49.5, 51.1, 50.7 s (-j8, 216 tests) |
| colcon build | 52.2 s (core 16.7 s) | 34.3, 34.8, 35.5 s (core 0.4 s) |
| colcon test | 55.5 s | 24.0, 24.0, 23.9 s |

> [!NOTE]
> GitHub runners have 4 CPUs, so the core ctest gains less there. All 6 runs on the final test setup passed: 3 before the `-l` fix and the 3 above.

## Phase 5: launch files, parameter YAML, RViz config and ~/map (2026-11-25, [#1](https://github.com/atinfinity/glim_localization/issues/1))

- **Launch files** (`glim_localization_ros2/launch/`):
  - `localization.launch.py` (R2) runs `glim_localization_node` standalone, or with `use_composition:=true` as a component in a new `component_container`. `config_path` and `map_path` override `params_file` only when not empty.
  - `localization_r1.launch.py` (R1) runs `glim_rosnode`, or `glim_rosbag <bag>` with `bag:=`. R1 reads the map path only from JSON, so `map_path:=` makes a temporary copy of the config with `odometry_estimation.map_path` set. The copy is removed when the launch exits.
  - glim_rosbag takes every argument that names an existing file as a bag, and launch_ros passes `parameters` as `--params-file <file>`. So the R1 launch file reads `params_file` itself and passes each value as `-p name:=value`. The first R1 launch test run failed this way: rosbag2 tried to open the params file as a bag.
  - `rviz` defaults to false. `rviz2` is not a package dependency: with `rviz:=true` and no rviz2 installed, the launch file logs a warning and starts without it (checked in the container, which has no rviz2).
- **Parameter YAML**: `config/localization.param.yaml` (every R2 node parameter, with comments) and `config/localization_r1.param.yaml` (the GlimROS and glim_rosbag parameters).
- **RViz config**: `rviz/localization.rviz` shows the fixed frame `map`, a grid, `~/map` (transient local), `~/pose`, `~/odometry` and TF, and has the 2D Pose Estimate tool on `/initialpose`. The R1 launch file remaps its topics to `/glim_ros/...`.
- **R2 `autostart` parameter** (default false, true in the launch file). A one-shot wall timer configures and activates the node once it spins, the same in a process and in a container. A failed step is logged and leaves the node where it stopped. The existing R2 tests keep driving the lifecycle by hand.
- **`~/map`** (`map_publisher`, shared by R1 and R2): the loaded map in the map frame as x/y/z float32, published once with reliable, transient local QoS.
  - Each voxel of `map_cloud_voxel` (default 0.5 m; 0 keeps every point) keeps its centroid. `publish_map` turns it off. Both keys are in `localization_ros` of `config_ros.json`.
  - R2 publishes the map at configure.
  - R1 publishes it when the extension is loaded. GlimROS creates the odometry module before the extensions, so the map is loaded by then.
  - The synthetic fixture map gives 9245 points from 5 tiles on arm64 and 9246 on amd64 (see the fix below).
- **Core API additions** (no GLIM patch):
  - `MapManager::tiles()`: every tile, sorted by id.
  - `LocalizationCallbacks::loaded_map()`: the map of the oldest live localization odometry instance, so the R1 extension reuses the map the dlopen'd module loaded instead of loading it twice.
  - `OdometryEstimationLocalizationCPU::map()`.
  - Each has a core test.
- **Deferred:** `~/aligned_points`. It needs a per-frame copy of the scan on the odometry callback and a rate and downsampling policy. Filed as [#18](https://github.com/atinfinity/glim_localization/issues/18).
- **Tests:**
  - `test_map_publisher` (gtest) checks the map-frame transform, the voxel centroids, the skipping of non-finite points, the rejection of a bad voxel size, and delivery of the latched map to a late subscriber.
  - Three launch tests include the installed launch files headless with the synthetic fixtures. Each checks `~/pose`, TF that composes to the last pose, and the latched `~/map` received after it was published:
    - `test_launch_r1`: R1 with `bag:=`.
    - `test_launch_r2`: R2 standalone.
    - `test_launch_r2_component`: R2 as a component.
  - The R2 tests also check that autostart reaches active.
- **Verification**: `scripts/ci.sh` in Docker passes: core ctest 218/218 and colcon test 11/11. Launch test times: R1 5.1 s, R2 16.1 s, R2 component 17.2 s.
- **Not done**: a manual run on a real bag with the launch files and RViz on a display (part of [#18](https://github.com/atinfinity/glim_localization/issues/18)).

### Review fixes for [#1](https://github.com/atinfinity/glim_localization/issues/1) (2026-11-25)

- **`~/map` size**: the cloud size is computed in `size_t`. A cloud above 1 GiB (`kMaxMapCloudBytes`, about 89 million points), or above the uint32 `row_step` of the message, is refused with a warning and not published; before, `row_step` could wrap and the copy wrote past the buffer. `check_map_cloud_size()` is tested at the uint32 boundary, and `make_map_cloud()` with a small cap, so no large allocation is needed.
- **`~/map` cost**: with `map_cloud_voxel: 0` the points go straight into the message (12 bytes per point, no per-point sums); with downsampling the build holds about 40 bytes per occupied voxel, and the voxel index is reserved. A map above 10 million points with voxel 0 gets a warning. The build stays where it was (R1 node start, R2 configure): it runs once, and moving it to a thread would need locking around the publisher for a small gain. The cost is documented in `config_ros.json`, the README and DESIGN §6.
- **R1 parameters** (`launch/launch_utils.py`, tested without ROS in `scripts/tests/test_launch_utils.py`): strings are passed single-quoted (so `""`, `"123"`, `"true"` and `"a: b"` stay strings), doubles keep a decimal point, integers of GlimROS's double parameters (`delay`, `start_offset`, `playback_duration`, `playback_until`) become doubles, lists of one type are passed as YAML flow, and nested mappings, empty lists and nulls are rejected with a clear error. R2 cannot coerce (launch_ros loads its `params_file`), so its YAML says to write doubles with a decimal point.
- **R1 `map_path:=`** edits the odometry config that `global.config_odometry` of `config.json` names, relative to the config directory, with a clear error when the key or the file is missing. Names that point outside the config directory (such as `../r1/...` in `config/r2`) are rewritten in the copy. The JSON comment stripping now skips strings and handles `/* */`; the R2 test helpers reuse it. The copy is removed when the launch process exits (`atexit`), after launch has reaped GLIM. An `OnShutdown` handler was dropped after the re-review: it fires while the processes are only being signalled, so it removed the config while GLIM could still read it (for example `GlobalConfig::dump` when global mapping is enabled). The params file now rejects a node section or `ros__parameters` that is not a mapping with a ValueError.
- **Other**: `autostart` skips and logs the state when the node already left Unconfigured; `package.xml` declares `ament_index_python` and `glim_ros` as exec dependencies (`glim_ros` moved from test_depend); the R2 test config copies are removed at exit.
- **Tests**: the launch tests now assert the `~/map` point count (9245 within 1%; see below). New `test_launch_r1_live` runs `glim_rosnode` (no `bag:=`) with `map_path` empty and the map in the config. The `rviz:=true` fallback stays a manual check: a launch test would start rviz2 where it is installed, and the warning goes to the launch log, not to a process output that launch_testing can assert.
- **Verification**: `scripts/ci.sh` in Docker passes: core ctest 218/218, colcon test 12/12 (`test_launch_r1_live` 1.1 s), Python tests 175.
- **`~/map` count across architectures**: GitHub Actions (amd64) got 9246 points where the local arm64 build gets 9245, so the four launch tests that asserted the exact count failed. Cause: the fixture floor (z = 0) and side walls (y = ±5 m) lie exactly on 0.5 m voxel planes with 1 cm noise. With about 40 points per metre of noise density at the plane and float coordinates near 15 m (ulp about 1e-6 m), a few of the tens of thousands of floor and wall points fall within rounding distance of a boundary. GCC contracts `a*b+c` into FMA on aarch64 by default but not on baseline x86-64, so `T_world_origin * p` differs by an ulp and `floor(p / voxel)` moves one point to the neighbouring voxel. Forcing identical rounding (`-ffp-contract=off` plus the same Eigen vectorization on both) would only hide the sensitivity of real maps, so the tests now accept 9245 within 1%. That is far below the change from another voxel size (0.4 m gives about 1.5 times the voxels on these surfaces) or from the raw map. Only arm64 can be checked locally; the amd64 result comes from the Actions run.

## CUDA image and GPU build mode (2026-11-25, part of [#2](https://github.com/atinfinity/glim_localization/issues/2))

First step of [#2](https://github.com/atinfinity/glim_localization/issues/2): a CUDA build environment. There is no GPU odometry module yet.

- **Image** `docker/Dockerfile.cuda` (tag `glim_localization:dev-cuda`, compose service `dev-cuda`, `scripts/docker_run.sh --cuda`):
  - `ros:jazzy-ros-base` plus NVIDIA's CUDA 13.1 apt toolkit (`cuda-nvcc`, `cuda-cudart-dev`, `cuda-cccl`).
  - `libgtsam-points-cuda13.1-dev=1.2.2` from the koide3 PPA.
  - GLIM and glim_ros2 built with `BUILD_WITH_CUDA=ON` and `CMAKE_CUDA_ARCHITECTURES` from the build argument `CUDA_ARCHITECTURES` (default 75).
  - Pins and key fingerprints are in docs/DEPENDENCIES.md; the design is in DESIGN §5.6.
  - The CPU image, its inputs hash and the CPU path of every script are unchanged.
- **Build mode**: `GLIM_LOC_BUILD_WITH_CUDA=ON` (set by the CUDA image) makes `build_core.sh` and `build_colcon.sh` pass `BUILD_WITH_CUDA=ON`. In CPU mode `build_core.sh` passes `OFF` to the core, and `build_colcon.sh` passes nothing, because colcon gives its CMake arguments to every package. `check_deps.sh` checks the CUDA package, `GTSAM_POINTS_USE_CUDA`, nvcc 13.1 and GLIM's GPU odometry module (built and loadable).
- **GPU test**: `test_gpu_smoke` lists the devices and builds a `GaussianVoxelMapGPU` from a synthetic cloud, comparing it with `GaussianVoxelMapCPU`. Without a visible GPU it skips; `GLIM_LOC_REQUIRE_GPU=1`, which `ci.sh` sets when `nvidia-smi` lists a GPU, makes the skip a failure.
- **GitHub Actions** stays CPU-only. A new job lints `docker/Dockerfile.cuda` with `docker build --check`. `scripts/tests/test_dockerfile_cuda.py` keeps the shared pins and lines of the two Dockerfiles in sync.
- **GLIM patch (CUDA image only)**: `third_party/patches/glim_cuda/0001-gtsam-points-1.2.2-has-index.patch`.
  - The first CUDA image build failed in GLIM's `odometry_estimation_gpu.cpp`. GLIM `1a197c9` calls `IndexedSlidingWindow::has_index`, which gtsam_points added after v1.2.2 (upstream commit `8f50081`, 2026-08-02). The CPU build never compiles that file.
  - The patch replaces the one call with the same test through the public API: an index is held when `size() - inner_size() <= index < size()`, which is how upstream defines `has_index`.
  - It lives in its own directory so the CPU image inputs stay unchanged. Drop it when the gtsam_points pin moves to a release that has `has_index`.
- **Verification on a maintainer's GPU machine** (NVIDIA GeForce RTX 2080 Ti, sm_75, 11 GiB; driver 595.99.02; Docker 28.3.3, Compose 2.39.1; image built with the default `CUDA_ARCHITECTURES=75`, 2.55 GB):
  - `scripts/docker_run.sh --cuda scripts/ci.sh` passes in 276 s: dependency check, core ctest 220/220 (including `GpuSmoke.ListsDevices` 0.23 s and `GpuSmoke.BuildsGaussianVoxelMapGPU` 0.84 s, run with `GLIM_LOC_REQUIRE_GPU=1`), colcon test 12/12, Python tests 182, replay e2e.
  - The same image without `--gpus`: both GPU tests are reported as skipped and ctest passes. With `GLIM_LOC_REQUIRE_GPU=1` both fail.
  - Two image builds failed early on transient DNS errors resolving github.com on that machine; a retry passed. The build time is not recorded because later builds reused cached layers.
- **CPU path**: `scripts/ci_local.sh --force` on the development Mac passes. One of three runs failed in `test_launch_r2_component` (71 poses for 121 frames, below the 90% bound); the next run on the same tree passed. This change does not touch that test or any ROS code, so the flake was there before; the cause is not known yet and is tracked in [#20](https://github.com/atinfinity/glim_localization/issues/20).
- **Remaining for [#2](https://github.com/atinfinity/glim_localization/issues/2)**: the GPU localization odometry module (`odometry_localization_gpu`), `GaussianVoxelMapGPU` tiles in MapManager, GPU memory management (LRU and prefetch), GPU replay evaluation and the CPU vs GPU comparison.

### Review fixes for the CUDA variant (2026-11-25)

- **Compose**: `dev-cuda` requests all GPUs with `deploy.resources.reservations.devices` (driver nvidia, count all, capabilities gpu) instead of `gpus: all`. Compose before v2.30 rejects the `gpus` key while validating the whole file, which would also break the CPU service. The image no longer sets `NVIDIA_VISIBLE_DEVICES`, so GPUs come only from a device request, also on a host whose default runtime is `nvidia`.
- **Stale CUDA image**: `docker compose run` never rebuilds an existing tag. The CUDA image now copies its repository inputs to `/etc/glim_localization/inputs/`:
  - `docker/Dockerfile.cuda`
  - `docker/entrypoint.sh`
  - `scripts/requirements.txt`
  - the `glim`, `glim_cuda` and `glim_ros2` patches

  In CUDA mode, `ci.sh` first runs `scripts/check_cuda_image_inputs.sh`, which fails when these files differ from the mounted tree. `GLIM_LOC_ALLOW_STALE_IMAGE=1` makes it warn instead.
- **Tests**:
  - The Dockerfile sync test skips each replaced CPU line only once, so the later `RUN apt-get update` lines are checked too.
  - New tests check the inputs list against the Dockerfile COPY lines and the result of the stale check.
  - The `ci.sh` tests in `test_ci_local.py` set the mode explicitly. In the CUDA image they had failed: the image sets `GLIM_LOC_BUILD_WITH_CUDA=ON`, and the new check was not stubbed.
  - `test_gpu_smoke` also checks the y range of the voxel means.
- **build_colcon.sh** passes `-DBUILD_WITH_CUDA=ON` only in CUDA mode, so its CPU command line is the same as before the CUDA mode.
- **Flaky `test_launch_r2_component`**: tracked in [#20](https://github.com/atinfinity/glim_localization/issues/20) (drop counters to log, single-threaded component container). Resolved: see "R2 launch tests: lost start of the bag" below.
- **Verification on the GPU machine** (same machine as above; default Docker runtime runc):
  - `ci.sh` with the image built before this fix fails at once on the stale check.
  - After a rebuild (2.55 GB), `scripts/docker_run.sh --cuda scripts/ci.sh` passes in 278 s. The image check matches, and the GPU is visible through the device reservation. Core ctest is 220/220, with the two GPU tests passing under `GLIM_LOC_REQUIRE_GPU=1`. colcon test is 12/12, the Python tests 186, and replay e2e passes.
  - `docker run` without `--gpus`: `NVIDIA_VISIBLE_DEVICES` is unset, both GPU tests are reported as skipped, and with `GLIM_LOC_REQUIRE_GPU=1` both fail.
  - Installed CUDA packages:
    - `cuda-nvcc-13-1`, `cuda-crt-13-1`, `cuda-culibos-dev-13-1` and `cuda-cccl-13-1`: 13.1.115-1
    - `cuda-cudart-13-1`, `cuda-cudart-dev-13-1`, `cuda-driver-dev-13-1` and `cuda-toolkit-13-1-config-common`: 13.1.80-1
    - `cuda-toolkit-13-config-common` and `cuda-toolkit-config-common`: 13.5.37-1

## GPU localization odometry module (2026-11-25, part of [#2](https://github.com/atinfinity/glim_localization/issues/2))

Second step of [#2](https://github.com/atinfinity/glim_localization/issues/2): the GPU odometry module and backend selection. Design in DESIGN §5.7.

- **Split**: the localization odometry is now a shared base `OdometryEstimationLocalization` (map factor bookkeeping, quality monitor and status, resets, fail-closed paths, IMU gap policy) with two backends. `OdometryEstimationLocalizationCPU` keeps its behavior and config; the CPU tests are unchanged apart from includes and a latency print. The replay runner, the publisher, the R1 extension and the R2 node use the base class.
- **GPU module** `OdometryEstimationLocalizationGPU` (`libodometry_localization_gpu.so`, `BUILD_WITH_CUDA=ON` only), derived from GLIM's `odometry_estimation_gpu.cpp/.hpp`:
  - Map factors: the same tile selection as the CPU module (`select_map_tiles`), then one `IntegratedVGICPFactorGPU` per tile and level against `GaussianVoxelMapGPU` tiles. A weight other than 1 (DEGRADED, LOST, `map_factor_weight`) uses `WeightedVGICPFactorGPU`, a subclass that scales error and linearization like `WeightedFactor` but stays a GPU factor, so the smoother still batches it.
  - GPU tiles: `GpuTileCache` uploads tiles on first use and keeps at most `map_gpu_max_tiles` (default 32, must be at least `map_max_submaps`) in LRU order. Tiles referenced by factors in the smoother window stay alive after eviction.
  - Keyframes (`use_keyframes`): upstream's scan-to-keyframe GPU factors and keyframe strategies with their upstream keys.
  - GLIM's GPU linearization hook is registered before the base constructor, because the smoother copies the hook list when it is created.
- **Backend selection**: the replay tool and the R2 node read `so_name` from `config_odometry`. `libodometry_localization_cpu.so` or `_gpu.so` select the backend; anything else, or the GPU module in a CPU build, is an error (replay exit 2, R2 configure fails). `replay_args.json` records the backend in `effective.backend`. R1 loads the module through GLIM as before.
- **Configs**: `config_odometry_localization_gpu.json` (common keys identical to the CPU file; a test checks this), `glim_localization/config/gpu`, and `config/r1_gpu`, `config/r2_gpu` in the ROS package. They share every other file with the CPU configs.
- **Tests**: `test_localization_odometry_gpu` (CUDA builds): convergence from a perturbed start, with keyframes, with map weight 0.5, waiting for the initial pose with two resets; the parameter checks; tile cache eviction; the weighted GPU factor against the unweighted one; the factory and the plugin load. They skip without a GPU, and fail instead under `GLIM_LOC_REQUIRE_GPU=1`. `test_localization_backend` runs in both builds. `test_replay_e2e.sh` adds a GPU replay with the same limits when the build is CUDA and a GPU is visible, and prints the accuracy and odometry time of both backends.
- **Verification on the GPU machine** (NVIDIA GeForce RTX 2080 Ti, sm_75, 11 GiB; driver 595.99.02; CUDA 13.1 image `glim_localization:dev-cuda`, unchanged inputs, so the stale check passes):
  - `scripts/docker_run.sh --cuda scripts/ci.sh` passes: core ctest 235/235 (one skip by design: `GpuNeedsACudaBuild` only applies to CPU builds), colcon test 12/12, Python tests 187, replay e2e including the GPU replay.
  - The first CUDA build failed: the GPU module used the smoother and `GaussianFactorGraph` through forward declarations only. Fixed by including their headers. The CPU build does not compile that file.
  - A rebuild of the CUDA image failed on a transient network error while cloning GLIM; the existing image had the same inputs and was used.
- **CPU vs GPU** (same machine, same synthetic sequence: 12 s, 121 frames at 10 Hz, initial pose off by 0.3 / -0.2 / 0.1 m and 3 degrees; errors after 3 s):

  | Run | Backend | ATE translation RMSE / max [m] | Max rotation [deg] | Odometry time per frame, mean / p95 [ms] |
  | --- | --- | --- | --- | --- |
  | Replay e2e (`frames.csv`, `t_odometry_ms`) | CPU | 0.0058 / 0.0119 | 0.080 | 19.2 / 17.4 |
  | Replay e2e | GPU | 0.0058 / 0.0119 | 0.080 | 3.5 / 3.7 |
  | Unit test (`status().processing_time`) | CPU | (passes, < 0.1 m) | | 15.6 / 13.1 |
  | Unit test | GPU | tail RMS 0.0043, max 0.0119 | 0.080 | 2.7 / 2.9 |
  | Unit test, `use_keyframes` | GPU | tail RMS 0.0036, max 0.0090 | 0.086 | 7.0 / 19.2 |

  - The CPU and GPU replay trajectories differ by at most 1.1e-6 m over 121 poses: the GPU path computes the same factors (residuals differ in the 7th digit) and converges to the same optimum.
  - CPU means are above their p95 because of the first frames (initialization and the first factor builds); the GPU first frame includes the tile uploads (22 ms in replay).
  - These are small synthetic scans (about 4300 points); real-data timing on the GPU is not measured yet.
- **CPU path**: `scripts/ci_local.sh --force` on the development Mac passes: core ctest 224/224, colcon test 12/12, Python tests 187, replay e2e (GPU replay reported as skipped).
- **Review fixes** (PR [#21](https://github.com/atinfinity/glim_localization/pull/21) review):
  - Fail closed when CUDA is unusable. gtsam_points 1.2.2 only prints a warning on a CUDA error and goes on, so the GPU module checks `cudaGetDeviceCount` before construction and `cudaGetLastError` after creating its streams, and `GpuTileCache` synchronizes and checks it after each upload. A failure throws `std::runtime_error` (replay exit 1, R2 activation fails). `FailsClosedWithoutAVisibleDevice` hides every device (`CUDA_VISIBLE_DEVICES=`) and expects the constructor to throw. On the GPU machine it passes both in ctest and in the CUDA image started without `--gpus`, where `cudaGetDeviceCount` reports an insufficient driver.
  - `MapFactorMatchesTheCpuFactor` compares the GPU map factor with the CPU `IntegratedVGICPFactor` on the same tile, source and pose: error, Hessian and gradient within 1e-3 (relative) on every level.
  - `test_replay_e2e.sh` asserts that the CPU and GPU replays agree: same stamps, position within 1e-3 m, rotation within 0.05 degrees. Before, it only printed the figures.
  - `build_core.sh` requires `libodometry_localization_gpu.so` in CUDA mode and checks its links. A CPU build no longer installs the GPU headers (`PATTERN "*gpu*" EXCLUDE`), and `build_core.sh` checks this.
  - Documented in DESIGN §5.7: the unknown `keyframe_update_strategy` throws (upstream only logs), and with `ENTROPY` the entropy also sees the map factors (upstream: IMU and keyframe factors only).
  - The plugin GPU test restores the config and removes its temp dir through a scope guard.
  - Re-verified on the GPU machine: core ctest 237/237 (one skip by design), colcon test 12/12, Python tests 187, replay e2e. The CPU and GPU replays differ by at most 1.1e-5 m and 3.4e-4 degrees over 121 poses (1.1e-6 m in the earlier run: the GPU sums are not deterministic). The first colcon run hit the [#20](https://github.com/atinfinity/glim_localization/issues/20) flake in `test_launch_r2` (71 of 121 poses) on that machine too; the rerun passed.
- **Behavior change for release notes**: the replay tool and the R2 node now read `so_name` and need exactly `libodometry_localization_cpu.so` or `libodometry_localization_gpu.so` (a file name). An empty or missing `so_name`, a full path, or another module is an error (replay exit 2, R2 configure fails). Before this change both ignored `so_name`. R1 is unchanged.
- **Remaining for [#23](https://github.com/atinfinity/glim_localization/issues/23)**: tile prefetch along the predicted path (uploads are synchronous today), a GPU memory budget in bytes instead of a tile count, streaming tiles from disk (the whole map stays in host memory), real-data accuracy and timing on the GPU, and GPU CI (GitHub-hosted runners have no GPU; verification stays manual).

## R2 launch tests: lost start of the bag (2026-11-25, [#20](https://github.com/atinfinity/glim_localization/issues/20))

`test_launch_r2` and `test_launch_r2_component` sometimes got 71 of 121 poses (bound: 90%).

- **Cause**: the node under test had not yet matched the bag player's publishers when playback resumed. With reliable, volatile QoS, messages that go out before the match are lost, and DDS discovery runs per participant: the test client could already see the player while the node could not.
  - In a failing run, the player resumed about 2 s after it started. The node's first IMU sample had bag stamp 1.790 and its first cloud 1.800.
  - The `/initialpose` at 0 s (the offset pose) never arrived. The node only logged the reset from the second pose at 5 s, and the core drops frames before the first pose. That leaves the frames from 5.0 s to 12.0 s: 71 poses.
  - No drop counter was non-zero: 0 frames without gyro, 0 unpaired, 0 stale, 0 TF failures, in every run. The coverage report showed frames 0.0 to 4.9 s missing, 72 statuses, the first at 5.00 s, and reset_count 1.
  - So this is not a drop in the product code. The publisher, the pairing, the executor and the QoS depth are ruled out, and the non-component launch is affected the same way. It is the same "unknown cause" startup loss recorded for the IMU gap test above, which `test_r2_node` and `test_r2_imu_gap` already avoid by calling `~/initialize`.
- **Diagnostics in the R2 node**:
  - The imu, points and `/initialpose` subscriptions log `matched publishers on <topic>: N` when a publisher matches or unmatches. If the RMW does not support the event, the node falls back to a plain subscription.
  - The node logs the stamp of its first point cloud (and of its first IMU sample, in the existing `acc_scale` line).
  - On stop (deactivate, shutdown) it logs how many IMU samples and clouds it received, the frames it dropped (without gyro, unpaired, stale) and the TF lookup failures. `EstimateBridge::drop_counts()` exposes these.
- **Test fix**: `check_r2_launch` starts the player paused, then waits for the node's own `matched publishers on <topic>: [1-9]` lines for `/points`, `/imu` and `/initialpose` (through `proc_output`) before the discovery margin and the resume. On failure, the pose-count assertion includes a coverage report: missing frames, status count and states, last reset_count.
- **Rates** (`scripts/test_colcon.sh` in a loop, Docker on the development Mac):
  - Before: 20 runs, 1 failed (5%): `test_launch_r2_component`, 71 of 121 poses.
  - After (on top of the GPU odometry merge): 40 runs, 0 failed. All 80 R2 launch tests got the first cloud at stamp 0.000. The node logged its last matched line 2 ms after the player's first log line (`Set rate`, written once its publishers exist; median, max 2.9 s). The max is above the old 2 s margin, which alone could be too short.
  - The evidence for the fix is mainly the first cloud at 0.000 in all 80 tests, where the failure needs a first cloud near 1.8 s. 0 failures in 40 runs alone is weak: at a 5% failure rate, 0 of 40 still happens about 13% of the time.
  - In both loops, a few R2 launch tests got 120 of 121 poses (before: 1 of 40; after: 2 of 80). Frame 0.0 is missing because the first `/points` callback ran about 3 ms before the `/initialpose` callback, so the core dropped that frame as before any pose. There is no ordering across topics, the cost is at most one frame, and it is within the bound.
- **Review fixes** (PR [#22](https://github.com/atinfinity/glim_localization/pull/22)): the stop line is logged after the odometry thread is joined, so frames still queued are counted; the counters read by `stop()` are atomic; the node logs when the RMW has no matched events (the tests then time out on the matched line); the wait for the three matched lines shares one deadline; the comment on `DISCOVERY_MARGIN_S` says why it stays (the player's writers must match the node's readers too); the coverage report is printed only on failure.
- **Docs**: the "Early messages can be lost" note in DESIGN ("R2 node: differences from the plan") now names the cause and the new log lines.

## Relocalizer: yaw search around a given initial pose (2026-11-25, part of [#3](https://github.com/atinfinity/glim_localization/issues/3))

A pose from outside (a 2D Pose Estimate click, `~/initialize`, the configured `init_T_world_imu`, the replay `--init-pose`) can be off in yaw by tens of degrees, and scan-to-map registration alone does not recover from it. This adds an opt-in yaw search on the first scan. Design in DESIGN §5.3, keys in the ROS 2 guide ([Initial pose yaw search](ros2.md#initial-pose-yaw-search)).

- **Core** (`glim_localization/relocalization/`, no ROS):
  - `PoseCandidateGenerator` (`generate(CandidateQuery{hint, frame, map})`) is the plug point; `YawCandidateGenerator` turns the hint about world z by 0, +s, -s, ... up to the range (180 deg and 15 deg give 24 candidates; -180 is dropped as the same yaw as +180). A global relocalizer will implement the same interface without a hint (`set_candidate_generator`).
  - `score_candidate` registers the scan from the candidate with the unary VGICP map factors of all levels (the odometry's `create_map_factors`) and Levenberg-Marquardt (10 iterations), then scores the registered pose by the finest-voxel inlier ratio, the same quantity as the status inlier ratio. No overlapping tile or a non-finite solve scores 0.
  - `search_candidates` scores in order until the time budget (the first candidate always), and `select_best_candidate` takes the highest score at or above the minimum (ties to the earlier, nearer candidate; NaN never passes). The best one is reported also when rejected.
- **Odometry**: the search runs for the configured pose and EXTERNAL resets that are not `from_current`; `relocalize()` and IMU gap self-resets are not searched. Accepted: start from the registered 6-DoF pose, velocity turned by the yaw. Rejected: LOST, and frames are dropped until the next reset. Parameters are validated at construction (also when the search is off). Status fields `init_search`, `init_search_candidates`, `init_search_score`, `init_search_yaw_offset`, `init_search_time`, cleared by each reset.
- **Decisions**:
  - Opt-in (`init_yaw_search: false`), so existing behaviour and tests are unchanged.
  - The reported yaw correction is the registered pose's, not the candidate's offset: in the first run neighbouring candidates (for example -45 and -60 deg for a 75 deg error) converged to the same correct pose and the earlier one won the tie, so the candidate offset was 30 deg off the applied correction.
  - Inlier ratio as the score, rather than the residual, so that `init_yaw_search_min_inlier_ratio` and the quality monitor's thresholds mean the same thing.
  - The search runs synchronously in the estimation thread on the first scan; it happens once per given pose.
- **ROS**: `LocalizationStatus.msg` gets `INIT_SEARCH_NONE/ACCEPTED/REJECTED`, `init_search`, `init_search_candidates`, `init_search_score`, `init_search_yaw_offset`, `init_search_time`; the publisher maps them and adds them to the diagnostics (a rejected search says so in the diagnostic message). `test_message_constants` pins the enum, `test_localization_publisher` checks the mapping. `Initialize.srv` notes that its `score` cannot carry the search result (the pose is applied at the next frame). The new fields change the message type, so nodes and recorded bags built with the old definition do not interoperate with the new one; `scripts/test_interfaces.sh` pins the new definition.
- **Replay**: `summary.json` gets `init_search` (result, candidates, score, yaw_offset_deg, seconds).
- **Verification** (synthetic map `replay_map_params()`, Docker on the development Mac, arm64; position error 0.3 / -0.2 / 0.1 m on top of the yaw error in the odometry tests):

  | Case | Result | Search | Error after 3 s (max) |
  | --- | --- | --- | --- |
  | Odometry, 75 deg, search off | LOST at 12 s | | 25.2 m, 87.6 deg (last) |
  | Odometry, configured pose, 75 deg | ACCEPTED, correction -74.99 deg, inlier ratio 0.997 | 24 candidates, 0.83 s | 0.011 m, 0.090 deg |
  | Odometry, configured pose, -90 deg | ACCEPTED, 90.01 deg, 0.997 | 24, 0.84 s | 0.011 m, 0.090 deg |
  | Odometry, `reset()` at 0.5 s, -80 deg | ACCEPTED, 80.01 deg, 0.998 | 24, 0.84 s | 0.011 m, 0.090 deg |
  | Odometry, pose 500 m off the map | REJECTED, LOST, no estimate | 24, < 0.1 ms (no tiles) | |
  | Odometry, new pose after the rejection, 60 deg | ACCEPTED, -59.99 deg | 24, 0.86 s | 0.006 m, 0.090 deg |
  | Scoring unit test, 60 / 75 / 90 / -80 deg | Correction within 1 deg of the error | 24, 0.60 to 0.63 s | 0.002 to 0.003 m, 0.03 to 0.04 deg |
  | Replay tool, 75 deg, search off | exit 0, all frames TRACKING | | 3.39 m, 69.1 deg (last pose) |
  | Replay tool, 75 deg, search on | exit 0, ACCEPTED, -75 deg | 24, 0.86 s | 0.006 m, 0.052 deg (last pose) |
  | Replay tool, pose 200 m above the map, search on | exit 1 at the first LOST frame, log says the pose was rejected | | |

  - The first estimated frame takes about 0.84 s with the search instead of about 4 ms; later frames are 5 to 7 ms, as before.
  - The replay run without the search passes the tracking-ratio gate with a 69 deg error: the quality monitor alone does not catch a wrong yaw on this map.
  - Under parallel ctest the search slowed 2.5 times and the default 2 s budget stopped at 11 of 24 candidates (still the right yaw, since the candidates are ordered by distance from the given yaw). The tests that count candidates now set a 30 s budget.
  - Not verified at first: the GPU module (verified with the review fixes below), real scans and real-scan timing.
- **Review fixes** (PR [#24](https://github.com/atinfinity/glim_localization/pull/24)):
  - **Acceptance rules**: a pose is accepted only when every candidate was scored, the best reaches the minimum inlier ratio, and the runner-up (best candidate more than two steps away in yaw from the best) is more than `init_yaw_search_min_score_gap` below it. Before, a search cut short by the budget was accepted from the candidates it had scored, and two near-equal yaws were decided by order. `search_candidates` returns an outcome (ACCEPTED, NO_CANDIDATE, TRUNCATED, LOW_SCORE, AMBIGUOUS) with the runner-up.
  - **Defaults**: `init_yaw_search_min_inlier_ratio` 0.5 to 0.7, `init_yaw_search_time_budget` 2 to 10 s; new `init_yaw_search_min_score_gap` (0.05) and `init_yaw_search_min_points` (100). 0.7: on the synthetic map the right yaw scores 0.997, the runner-up (the 180 deg flip) 0.865 to 0.891, and a wrong yaw (75 deg) registered alone 0.55 to 0.61, which passed the old 0.5. 10 s: see the timing below; real-scan timing is not measured, and a truncated search is now rejected, so the budget needs room.
  - **Budget inside the registration**: the deadline is passed to Levenberg-Marquardt as `termination_criteria`, so the budget is checked between iterations, not only between candidates. The first candidate is scored without the deadline (bounded by `max_iterations`) so that a cut-short search still reports a score. With and without a far deadline the scores and poses were identical, and the time the same.
  - **Tiles at the registered pose**: the score is computed on the tiles selected again at the registered pose, not those of the candidate pose.
  - **Frames while the search is pending**: frames with fewer than `init_yaw_search_min_points` points are dropped. Upstream `OdometryEstimationIMU::insert_frame` only warns on an empty frame and still takes its first-frame branch, so without this an empty first frame would have started the estimation without a search.
  - **Status and message**: `init_search_candidates_total`, `init_search_runner_up_score` and `init_search_runner_up_yaw_offset` in the core status, `LocalizationStatus.msg` (a breaking change of the message type), the diagnostics and `summary.json` (also `truncated`). The diagnostic message of a rejection says whether the budget ran out or gives the best and runner-up scores.
  - **Limits**: step at least 0.5 deg (at most 720 candidates).
  - **Tests**: truncated search rejected (budget 0); runner-up reported and a stricter gap rejected as ambiguous; a symmetric synthetic room (half-turn poses score 0.932 and 0.930) rejected as ambiguous; a wrong yaw on the map with range 0 rejected (0.61 < 0.7); empty and sparse frames keep the search pending; the velocity turned with the yaw (error 0.004 m/s, 2.04 m/s if not turned); an IMU gap self-reset does not search; a non-identity LiDAR-IMU extrinsic; the deadline check does not change convergence; the GPU module corrects a 75 deg error (skipped without a GPU); invalid parameters; both shipped configs carry every key.
  - **GPU module** (RTX 2080 Ti, CUDA 13.1 image), in two rounds:
    - At 2e52c10, before the review fixes: the CUDA `ci.sh` passed (core 253/253, colcon 12/12, GPU replay e2e). The search timings come from an ad-hoc copy of `test_init_yaw_search` run on the GPU module at that commit: 24 candidates in 1.65 to 1.73 s with the GPU module and 1.73 to 1.88 s with the CPU module, about 70 to 78 ms per candidate (synthetic data, 2 threads); on arm64 it is about 26 to 33 ms per candidate. The search uses CPU factors in both modules.
    - At 65e0e2e, with the review fixes: `GLIM_LOC_REQUIRE_GPU=1` CUDA `ci.sh` passed. Core ctest 264/264 (the only skip is `GpuNeedsACudaBuild`, as expected in a CUDA build); colcon test 12/12 (75 tests); interfaces passed; Python 187 passed. Replay e2e: CPU and GPU ATE both rmse 0.0058 m, max 0.0119 m, rotation max 0.080 deg; odometry mean 18.7 ms (CPU) and 3.4 ms (GPU); CPU vs GPU over 121 poses at most 1.124e-05 m and 3.352e-04 deg apart. `LocalizationOdometryGPUTest.YawSearchCorrectsALargeYawError` ran on the GPU (not skipped) and passed: 24 of 24 candidates in 1896.32 ms, score 0.996018, runner-up 0.889904; after the search max translation 0.0119 m, max rotation 0.0795 deg, inlier ratio 0.9995.
    - The later fixes (the rejection reason, the distinct-yaw cap, the deadline and budget nits) change shared code (`candidate_search.cpp`, the odometry base class, the ROS publisher) but no GPU-specific file, so the GPU run was not repeated for them; the Actions CPU build covers them.
  - **Re-review fixes**:
    - **Rejection reason**: `init_search_reason` (NONE, TRUNCATED, LOW_SCORE, AMBIGUOUS, NO_CANDIDATE) in the core status, `LocalizationStatus.msg` (breaking), the diagnostics and `summary.json` (`reason`). The diagnostics message gives the numbers for that reason only; before, NO_CANDIDATE printed "best 0.000, runner-up 0.000" because the text was inferred from the counts.
    - **Distinct yaw**: the runner-up must be more than `min(2 * step, 90 deg)` from the best. With 2 * step alone, a step of 90 deg or more left no candidate far enough away, so no runner-up and no ambiguity check. A test with a 90 deg step now reports the half-turn runner-up. The docs no longer say a bad pose is never accepted silently: with range 0 or a small range only the inlier threshold applies.
    - **Deadline**: a registration that converges on the iteration where the deadline passes counts as completed. A non-finite time budget is treated as 0. The sparse-frame warning is logged once per pending search.
    - **Tests**: the one-iteration wall-clock check is loosened to 2 s; the bounded and unbounded scores are compared to 1e-6; the frame swap in the extrinsic test is undone by a scope guard.
- **Remaining for [#3](https://github.com/atinfinity/glim_localization/issues/3)**: global relocalization (FPFH + RANSAC/GNC on accumulated scans, a `PoseCandidateGenerator` without a hint, and `global_search` in `~/initialize`); roll and pitch from gravity and height from the map; accumulating several scans before scoring; running the search off the estimation thread; real-scan timing and evaluation on real data.

## map_compiler: PLY input and 2D occupancy grid (2026-11-25, [#4](https://github.com/atinfinity/glim_localization/issues/4))

- **PLY input** (`map/ply_reader.hpp`): a small reader written here, since PCL is not in the image and gtsam_points has no PLY reader. `ascii` and `binary_little_endian` 1.0; vertex `x`, `y`, `z` as `float`/`float32` or `double`/`float64`; other properties, list properties and other elements are skipped. After reading, PLY goes through the PCD path unchanged (downsampling, covariance estimation, tiling), with `source: "ply"`.
  - Rejected with a clear error: big-endian (with a hint to convert), unknown formats, keywords and types, duplicate elements or properties, missing or non-float x, y, z, an element with a count but no properties, negative or wrong list lengths, and truncated data. The vertex count is checked against the remaining file size before allocating, as for PCD. The binary reader assumes a little-endian host (static_assert).
- **2D occupancy grid** (`map/occupancy_grid.hpp`, `--occupancy-grid <prefix>`): map_server PGM and YAML from any input, written by the ROS-free core.
  - Cell rule: occupied if a point has `z_min <= z < z_max`, free if it has only points below `z_min`, unknown otherwise; points at or above `z_max` are ignored and do not extend the grid. The band is required (no default suits every map).
  - Values: occupied 0, free 254, unknown 205, `mode: trinary`, `negate: 0`, `occupied_thresh: 0.65`, `free_thresh: 0.1`. With the common `free_thresh: 0.25`, map_server would read 205 (occupancy 0.196) as free; 0.1 keeps a margin on both sides.
  - Cell edges are multiples of the resolution, so `origin` is exact even with UTM-scale coordinates. Built in two passes (extent, then fill) with a 2^30 cell limit checked before allocating. All grid options are checked, and existing grid files refused, before any map is read or written.
  - Assumes roughly flat ground: the band is one absolute map-frame z range, and PCD and PLY input must be z-up.
  - Not done: no noise filter (a single point marks a cell occupied; voxel downsampling does not remove isolated outliers), no ray casting for free space, no yaw in the origin.
- **Verification** (Docker on the development Mac): core ctest 244/244 and Python tests 189 pass.
  - New C++ tests: `test_ply_reader` (ascii and binary with extra and list properties, float and double, CRLF headers, sized type names, non-finite points, identity with `read_pcd`, 20 header errors, 13 data errors), `test_occupancy_grid` (cell classification through a rotated submap, band bounds, UTM-scale origin, parameter errors, cell limit, PGM rows from the highest y, exact YAML, refusing existing files), and `test_map_compiler` (PLY and PCD give the same compiled map apart from `source`, ascii and binary; grids from PLY and from a GLIM dump; option errors).
  - New Python test `scripts/tests/test_map_compiler_occupancy.py`: runs `map_compiler` on a PLY and loads the result with the rules of nav2_map_server's map_io (required keys and types, trinary thresholds, the y flip); origin, size and every cell match. A running map_server is not tested.
- **Review fixes** (PR [#27](https://github.com/atinfinity/glim_localization/pull/27)):
  - The YAML `image` is a double-quoted string with `\` and `"` escaped. Prefixes with an empty file name (`out/`, `.`, `..`) or control characters are rejected before anything is read.
  - map_compiler refuses a non-empty output directory before reading the input, so no grid is written next to a map that was not. If the grid write fails after the map was written, the error says the map is complete.
  - The grid build uses one buffer (one byte per cell) instead of two; the 2^30 cell cap stays (1 GiB at most).
  - The PLY header is capped at 64 KiB and read byte by byte, so a file without newlines is not read as one huge line. Ascii values other than x, y, z must be numbers too.
  - New tests: the header cap, a binary PLY with a CRLF header, a non-numeric ascii color, points on negative cell edges and at x = 0, quoted image names (C++ and a Python YAML load), prefixes without a file name, and a non-empty output directory with a grid.
  - Docs: the flat-ground assumption, and that no noise filter is applied.
  - Re-verified in Docker on the development Mac: core ctest 247/247, Python tests 190.
- **Open**: covariance recomputation for GLIM dumps moved to [#31](https://github.com/atinfinity/glim_localization/issues/31).

## map_compiler: recompute covariances for GLIM dump input (2026-11-25, [#31](https://github.com/atinfinity/glim_localization/issues/31))

- **What**: `map_compiler --recompute-covariances [--k-neighbors n]` re-estimates the per-point covariances of a GLIM dump or a compiled map with the estimator of the PCD path (`gtsam_points::estimate_covariances`, k nearest neighbours). The core gets `recompute_covariances` in `map/pcd_map.hpp`. Off by default, so the output without the flag does not change.
- **Decisions**:
  - k is shared with `--k-neighbors` (5 to 1000, default 10) instead of a second option. For map directory input it is accepted only together with `--recompute-covariances`, so a k that would be ignored is an error.
  - Neighbours are searched per submap in its origin frame, not over the union of submaps: GLIM submaps overlap, and the union would mix slightly offset copies of the same surface. (GLIM itself estimates covariances per scan with `k_correspondences` = 10 and sub-mapping merges the transformed per-scan covariances; it does not run kNN on the submap cloud. The default k = 10 matches that per-scan default.) For PCD-compiled tiles this means no neighbours across tile edges; recompiling from the cloud is the documented way to change k there.
  - The clouds are copied, and all submaps are computed before any is replaced, so a failure (a submap with fewer than k points) leaves the input unchanged and writes nothing.
  - `--recompute-covariances` with `.pcd` or `.ply` input is rejected rather than ignored, since that path always estimates covariances. Re-downsampling a dump is not part of this change.
- **Tests** (synthetic data): `test_pcd_map` checks that the recomputed covariances of a submap with a rotated origin equal the PCD path's world-frame covariances rotated into that frame (k = 5, 10, 25; a jittered surface, so the neighbours have no distance ties), and the parameter and small-submap errors with the input unchanged. `test_map_compiler` checks that the default output copies the dump covariances exactly, that `--recompute-covariances --k-neighbors 20` on a dump and on a compiled map gives `estimate_covariances` of the stored points (and differs from the k = 10 dump), that a k above the smallest submap's point count fails with no output, and the option errors.
- **Verification**: the targeted `test_pcd_map` and `test_map_compiler` suites pass in Docker on the development Mac; the full gate is GitHub Actions on the PR.
