# Build and test

How to build the Docker image, build the packages and run the tests, including the optional CUDA image. For a guided first run, see [Getting started](getting_started.md).

## Requirements

- Docker with Compose. Builds and tests run in an Ubuntu 24.04 + ROS 2 Jazzy image (`docker/Dockerfile`). Builds outside this image are not supported.
- The image pins GLIM (commit `1a197c9`), glim_ros2 (commit `4d4ec52`, with a build-only patch that drops the cv_bridge dependency), GTSAM 4.3.0 and gtsam_points 1.2.2, all built for CPU (`BUILD_WITH_CUDA=OFF`). See [DEPENDENCIES.md](DEPENDENCIES.md). An optional CUDA image for amd64 NVIDIA hosts is described in [CUDA variant](#cuda-variant).
- A LiDAR with an IMU. LiDAR-only input is not supported.

> [!NOTE]
> **Headless.** The image has no GUI: no GLIM viewer, no Iridescence, no OpenCV and no RViz. The build and tests check that the GLIM libraries and ours do not link GUI libraries. Visualize from another machine on the same ROS 2 network if needed; see [Networking](getting_started.md#networking) for what that takes.

## Build and test

From the repository root on the host:

```bash
docker compose -f docker/compose.yaml build      # build the image
scripts/docker_run.sh scripts/ci.sh              # dependency check, build, all tests
```

`scripts/docker_run.sh <command>` runs a command in the container. The repository is mounted at `/ws/src/glim_localization`, build outputs go to a Docker volume at `/ws/build`, and the host directory `out/` is mounted at `/ws/out` (use it to exchange data, maps and results). Without a command it opens a shell.

`scripts/ci.sh` runs, in order: `check_deps.sh`, `build_core.sh` (core with plain CMake and its tests, into `/ws/build/core`), `build_colcon.sh` (all packages, into `/ws/build/colcon/install`), `test_colcon.sh`, `test_interfaces.sh`, `test_python.sh` and `test_replay_e2e.sh`. Each script can also be run alone. `build_colcon.sh` reuses the core build directory of `build_core.sh`, so run that first, with the same `CCACHE_DIR` setting (a different compiler launcher recompiles the shared core). `build_core.sh` and `test_colcon.sh` run the tests in parallel with `CTEST_JOBS` jobs (default: the CPU count).

> [!NOTE]
> `test_colcon.sh` unsets `ROS_DOMAIN_ID`. Each ROS test reserves a free domain ID for its run, so parallel tests do not see each other's topics.

`scripts/ci_local.sh [--force]` is an optional host-side wrapper of the same checks that reuses a recorded pass for the same tree and image, and skips docs-only changes. It is not required: GitHub Actions is the CI.

GitHub Actions runs `scripts/ci.sh` on amd64 (`build-and-test`) and on arm64 (`build-and-test-arm64`, an `ubuntu-24.04-arm` runner), each in an image tagged with `scripts/image_inputs_hash.sh`, the hash of the repository files the image bakes in. It pulls `ghcr.io/<owner>/glim_localization:<hash>` (amd64) or `<hash>-arm64` (arm64) when it exists and builds it otherwise; pushes to `main` publish a missing image after the checks of that architecture pass. The published image is rebuilt from the layer cache, not copied from the tested run. The tag identifies the image inputs, not the image content, so base image and apt updates reach it only when an input changes (or after the tag is deleted on GHCR). The C++ builds use a ccache saved from `main` (at most 500 MB, one per architecture). The repository's 10 GB Actions cache holds the two buildx layer caches (scopes `image-amd64` and `image-arm64`) and the two ccaches. Draft pull requests are skipped until they are marked ready for review.

To use ccache locally, set `GLIM_LOC_CCACHE_DIR` to a host directory when calling `scripts/docker_run.sh`. It is mounted as given, so its files are owned by the container user (root), and the path must not contain `:`.

Unless prefixed with `scripts/docker_run.sh`, the commands in these docs run inside the container (`scripts/docker_run.sh` with no arguments opens a shell there) after `scripts/ci.sh` or the build scripts have run.

## CUDA variant

`docker/Dockerfile.cuda` is a second image for a Linux amd64 host with an NVIDIA GPU, a driver that supports CUDA 13.1 or later, and the NVIDIA Container Toolkit. It is the CPU image plus the CUDA 13.1 toolkit from NVIDIA's apt repository, the CUDA 13.1 build of gtsam_points 1.2.2 from the same PPA, and GLIM and glim_ros2 built with `BUILD_WITH_CUDA=ON`. The CPU image stays the default and is the only one built on arm64 and in GitHub Actions.

```bash
docker compose -f docker/compose.yaml build dev-cuda   # glim_localization:dev-cuda
scripts/docker_run.sh --cuda scripts/ci.sh             # the same checks in CUDA mode
```

`--cuda` selects the compose service `dev-cuda`: the image `glim_localization:dev-cuda` (or `GLIM_LOC_CUDA_IMAGE`), all GPUs, and its own build volume, so CPU and CUDA builds do not share `/ws/build`. In this image `scripts/ci.sh` runs in CUDA mode (`GLIM_LOC_BUILD_WITH_CUDA=ON`): our packages are configured with `BUILD_WITH_CUDA=ON`, `check_deps.sh` checks the CUDA packages and GLIM's GPU odometry module, and the GPU tests (`test_gpu_smoke`, `test_localization_odometry_gpu`) and a GPU replay in `test_replay_e2e.sh` run on the GPU. When `nvidia-smi` lists a GPU, `ci.sh` sets `GLIM_LOC_REQUIRE_GPU=1`: the GPU tests then fail instead of skipping, and the GPU replay runs and must agree with the CPU replay. Without it, ctest reports the GPU tests as skipped and `test_replay_e2e.sh` prints `SKIPPED: GPU replay`.

The GPUs come from a device reservation in compose (`deploy.resources.reservations.devices`, which any Compose v2 accepts). The image does not set `NVIDIA_VISIBLE_DEVICES`, so a container started without a device request (`docker run` without `--gpus`) sees no GPU, even on a host whose default Docker runtime is `nvidia`. On such a host, passing `-e NVIDIA_VISIBLE_DEVICES=...` exposes GPUs anyway.

> [!IMPORTANT]
> `docker compose run` does not rebuild an existing image. After you change `docker/Dockerfile.cuda`, `docker/entrypoint.sh`, `scripts/requirements.txt` or a patch under `third_party/patches/`, rebuild with `docker compose -f docker/compose.yaml build dev-cuda`. In CUDA mode `ci.sh` first runs `scripts/check_cuda_image_inputs.sh`, which fails when the image was built from other versions of these files. `GLIM_LOC_ALLOW_STALE_IMAGE=1` turns that failure into a warning.

The CUDA build adds the GPU odometry module `libodometry_localization_gpu.so`. A config selects it through `so_name` in its odometry config; the shipped GPU configs share every other file with the CPU ones:

| Use | CPU config | GPU config |
| --- | --- | --- |
| Replay (`--config`) | `glim_localization/config` | `glim_localization/config/gpu` |
| R1 (`config_path`) | `config/r1` | `config/r1_gpu` |
| R2 (`config_path`) | `config/r2` | `config/r2_gpu` |

The GPU odometry config (`config_odometry_localization_gpu.json`) has the same common keys as the CPU one, plus GLIM's GPU keyframe keys for `use_keyframes` and these map tile keys:

| Key | Default | Range | Meaning |
| --- | --- | --- | --- |
| `map_gpu_max_tiles` | 32 | >= `map_max_submaps` | Map tiles kept on the GPU; the least recently used are evicted |
| `map_gpu_memory_budget_mb` | 1024 | 1 to 1048576 | GPU memory for map tiles in MiB (estimated from each tile's points and voxels); the least recently used are evicted to stay under it. The tiles of the current frame are always kept, even above the budget, and a warning is logged at startup if the budget exceeds the device's memory |
| `map_gpu_prefetch` | true | | Upload the tiles along the predicted path ahead of use, on a background thread and CUDA stream |
| `map_gpu_prefetch_lookahead` | 1.0 | 0 to 10 s | How far ahead the path is predicted from the current velocity |
| `map_gpu_prefetch_max_tiles` | 10 | 1 to `map_gpu_max_tiles` | Tiles prefetched per frame, nearest to the path first |

Both limits apply: a tile is evicted when either is exceeded. Prefetching never evicts a tile of the current frame and stops when the budget is full, so with a small budget it uploads less. In a CPU build, the replay tool exits with code 2 and the R2 node fails to configure when the config names the GPU module. The GPU module also fails closed when CUDA is unusable: with no visible device it does not start (replay exits with code 1, the R2 node fails to activate), and a failed upload of a tile a frame needs throws (a failed prefetch is only logged and dropped, and the tile is uploaded again when a frame needs it). The replay tool turns that into exit code 1. In R1 and R2 the exception escapes GLIM's background odometry thread and aborts the whole node process.

> [!WARNING]
> Behavior change in this release: the replay tool and the R2 node now read `so_name` in `config_odometry` and need exactly `libodometry_localization_cpu.so` or `libodometry_localization_gpu.so` (a file name, not a path). An empty or missing `so_name`, a full path, or another module is an error (replay exit 2, R2 configure fails). Before, both ignored `so_name`. R1 loads the module through GLIM and is unchanged.

For R1, copy the whole `config` directory, because `r1_gpu` refers to `../r1`.

The build argument `CUDA_ARCHITECTURES` (default `75`, Turing; for example `"75;86;89"`) sets `CMAKE_CUDA_ARCHITECTURES` for any CUDA code compiled in the image. It is never `native`, because no GPU is visible during `docker build`. The prebuilt gtsam_points package already contains code for sm_75 and newer. Set it with `CUDA_ARCHITECTURES=86 docker compose -f docker/compose.yaml build dev-cuda`.
