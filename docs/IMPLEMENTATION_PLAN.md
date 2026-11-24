# Implementation plan

Development, builds, and tests run in a headless Docker container with Ubuntu 24.04 + ROS 2 Jazzy (`docker/`). The container covers **the CPU core, offline evaluation, and the ROS 2 packages**. The GPU build (`BUILD_WITH_CUDA=ON`) and real-robot evaluation need a Linux machine with an NVIDIA GPU and come later.
When a phase meets its completion criteria, its results are recorded in `docs/PROGRESS.md`.

Status: Phases 0 to 4 are done. Phase 5 is done except the `~/aligned_points` topic ([#18](https://github.com/atinfinity/glim_localization/issues/18)). Phase 6 is partly done (PCD input for map_compiler, README).

## Phase 0: Environment and repository (done)

1. Check the environment: `uname -m`, `docker version`, and inside the container `cmake --version` and `g++ --version`.
2. Put the dependencies in a Docker image (`docker/Dockerfile`, Ubuntu 24.04 + ROS 2 Jazzy, headless). CPU builds outside this container are not supported; the GPU build environment is tracked in [#2](https://github.com/atinfinity/glim_localization/issues/2):
   - GTSAM / gtsam_points: `libgtsam-notbb-dev=4.3.0` and `libgtsam-points-dev=1.2.2` from the koide3 PPA
   - glim (commit `1a197c9`): build from source with `-DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_OPENCV=OFF -DBUILD_WITH_MARCH_NATIVE=OFF` and install to `/opt/glim`
   - Iridescence, GLFW, OpenCV, and rviz2 are not in the image. If a build needs a change to upstream code, save the patch in `third_party/patches/<pkg>/` and `git apply` it from the Dockerfile.
3. Done when: `scripts/docker_run.sh scripts/check_deps.sh` exits 0. It checks the pinned versions, that nothing depends on a GUI, that a minimal project using `find_package(glim)` and `find_package(gtsam_points)` builds and runs, and that the CPU odometry module loads with dlopen.

## Phase 1: Repository skeleton (done)

- Create the three packages from the package layout in `docs/DESIGN.md` §6 (`glim_localization`, `glim_localization_msgs`, `glim_localization_ros2`).
- The core does not depend on ament and also builds with plain CMake (it has a `package.xml`, but `ament_cmake` is optional).
- CMake options: `BUILD_WITH_CUDA` (default OFF), `BUILD_TESTS` (default ON), `BUILD_TOOLS` (default ON). Tests use GoogleTest (FetchContent is allowed).
- Add `.github/workflows/ci.yml` (Ubuntu 24.04 + ROS 2 Jazzy; GTSAM and gtsam_points from the koide3 PPA, glim built from source at the pinned commit, as in `docker/Dockerfile`) as a starting point.
- Done when: an empty library and a dummy test build and pass in the container (`scripts/docker_run.sh scripts/ci.sh`).

## Phase 2: MapManager and map I/O (done)

- `MapManager` (DESIGN §5.1): load a GLIM dump, override poses with `values.bin`, select submaps by radius plus K nearest, and build multi-resolution `GaussianVoxelMapCPU`.
- A minimal `map_compiler`: GLIM dump to a lightweight dump plus `map_index.json`. PCD/PLY input is Phase 6.
- A **synthetic map generator** for tests (`test/synthetic/`): build a point cloud from planes such as a floor, walls, pillars, and boxes, and write it in the GLIM dump format (`%06d/data.txt` and point clouds). Write `values.bin` with GTSAM's `serializeToBinaryFile(gtsam::Values)`.
- Done when: tests pass that save and reload a synthetic dump with matching poses and point counts, and that check `query()` returns the expected submaps.

## Phase 3: CPU localization odometry (done)

In the CPU version, `use_keyframes` means relative pose factors from the local model (DESIGN §5.2).

- Copy `odometry_estimation_cpu.{hpp,cpp}` and `_create.cpp` and turn them into `OdometryEstimationLocalizationCPU` (with the provenance comment at the top).
- The map factors from DESIGN §5.2, `use_keyframes`, a weaker initial prior, `reset()`, quality metrics (inlier ratio and residual), and state transitions (§5.4).
- The module is built with `add_library(odometry_localization_cpu MODULE ...)` and exports `extern "C" create_odometry_estimation_module()`. GLIM loads it by its `so_name` with dlopen, so the target sets `PREFIX "lib"` and `SUFFIX ".so"` explicitly. These are the Linux defaults for a `MODULE` library; setting them keeps the file name independent of platform defaults.
- Done when (synthetic data): along a trajectory through the synthetic map, generate **synthetic LiDAR scans (ray casting plus noise) and IMU data (differentiated trajectory plus noise and bias)**; a test passes in which, starting from a slightly offset initial pose, it converges to a position error < 0.1 m and an orientation error < 1°.

## Phase 4: Offline tools, no ROS (done)

- `glim_localization_replay`: read a simple data format (`points/<stamp>.bin` and `imu.csv`; PLY frames are not supported), run GLIM's preprocessing and then localization, and write `traj_localization.txt` (TUM format).
- A converter for real data, `scripts/bag_to_replay.py`: convert a ROS 2 bag (db3/mcap) to the simple format with the Python `rosbags` package (no ROS needed).
- An evaluation script, `scripts/evaluate.py`: ATE/RPE with evo, and processing time statistics.
- Done when: replay on synthetic data reaches the same accuracy as Phase 3. If real data is available, convert and run it, and record the results in PROGRESS.md.
- An evaluation mapper for real data (first step of [#5](https://github.com/atinfinity/glim_localization/issues/5)): `glim_localization_mapper` (no ROS) and `scripts/smoke_os1_128.sh`. Build a GLIM map and a reference trajectory from a replay dataset, run localization with the same preprocessing, and measure the agreement with GLIM. Done when: a synthetic test shows the dump can be read with `load_map`. Record real-data results in PROGRESS.md.

## Phase 5: ROS 2 packages (done except `~/aligned_points`)

1. Build and test with colcon in the container (ROS 2 Jazzy). Turn synthetic data into a bag and check the R1 and R2 nodes with it.
2. `glim_localization_msgs`: `LocalizationStatus.msg` and `Initialize.srv`.
3. `localization_publisher` (shared by R1 and R2): `~/pose`, `~/odometry`, `~/status`, `/diagnostics`, and TF (three `tf_mode` options, DESIGN §6). `~/map` (latched, downsampled; `map_publisher`) is done; `~/aligned_points` is deferred ([#18](https://github.com/atinfinity/glim_localization/issues/18)).
4. R1: `localization_extension` (`ExtensionModuleROS2`). `/initialpose` calls `reset()` through a shared queue in the core.
5. R2: `glim_localization_node` (`rclcpp_lifecycle::LifecycleNode`, registered as a component). Base it on the input handling in `GlimROS` (copied parts carry the provenance comment).
6. Launch files (R1 and R2, the R2 node standalone or as a component with `autostart`), parameter YAML files and an RViz config (done). Launch tests start each launch file headless and check `~/pose`, TF and `~/map`.
7. Done when: the build and unit tests pass in CI, and playing a bag publishes `~/pose` and TF.

## Phase 6: Finishing (optional, partly done)

- Relocalizer (multiple yaw hypotheses, FPFH + RANSAC/GNC): planned ([#3](https://github.com/atinfinity/glim_localization/issues/3)).
- PCD input for map_compiler: done. PLY input and 2D OccupancyGrid output: planned ([#4](https://github.com/atinfinity/glim_localization/issues/4)).
- Write the GPU version (`OdometryEstimationLocalizationGPU`); build it in a CUDA environment: done (CUDA image, `config/gpu`). Tile prefetch and a GPU memory budget: planned ([#2](https://github.com/atinfinity/glim_localization/issues/2)).
- README (build steps, usage, limitations): done.

## Out of scope for the default container

- Building and benchmarking the GPU version (requires CUDA and an NVIDIA GPU)
- Real-time evaluation with real sensors
- Checking integration with an external EKF
