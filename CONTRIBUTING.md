# Contributing

Thank you for your interest in glim_localization. Bug reports, feature requests and pull requests are welcome.

## Issues

Use the issue templates (bug report or feature request).

> [!WARNING]
> Do not attach bags or maps to public issues; describe the data instead.

> [!IMPORTANT]
> Report security vulnerabilities as described in [SECURITY.md](SECURITY.md), not in issues.

## Build and test

Builds and tests run in the Ubuntu 24.04 + ROS 2 Jazzy Docker image (`docker/Dockerfile`, headless).

> [!NOTE]
> Builds outside this image are not supported.

From the repository root:

```bash
docker compose -f docker/compose.yaml build   # build the image
scripts/docker_run.sh scripts/ci.sh           # dependency check, build, all tests
```

`scripts/ci.sh` is the same script that GitHub Actions runs. Each step (`scripts/build_core.sh`, `scripts/test_colcon.sh`, ...) can also be run alone with `scripts/docker_run.sh <script>`. See the [README](README.md#build-and-test) for details.

## Engineering rules

- **ROS-free core.** `glim_localization/` must not depend on ROS. Keep ROS dependencies in `glim_localization_ros2/` (and the message definitions in `glim_localization_msgs/`).
- **Upstream code is not modified.** GLIM, gtsam_points and glim_ros2 are used as pinned dependencies (see [docs/DEPENDENCIES.md](docs/DEPENDENCIES.md)). If an upstream fix is unavoidable, add a patch under `third_party/patches/` and explain why in the pull request and in `docs/PROGRESS.md`.
- **Derived-from header.** A file derived from GLIM starts with a line naming its source path and the pinned commit:

  ```cpp
  // Derived from koide3/glim <path> at commit 1a197c999e4865f5cab7c9cd25474128413ebbca (MIT License, Copyright (c) 2024 koide3)
  ```

  Files derived from glim_ros2 use the same form with `koide3/glim_ros2` and its pinned commit (`4d4ec524ccf1b02aa09b0af2af767ecc54343798`).
- **CUDA off by default.** The `BUILD_WITH_CUDA` CMake option defaults to `OFF`, and the Docker image and CI are CPU only. GPU code must not be required for the default build.
- **No bags or maps in commits.** Tests use synthetic data.
- **Small commits** with English messages in the [Conventional Commits](https://www.conventionalcommits.org/) form, with the area as the scope: `feat(odometry): ...`, `fix(map): ...`, `docs: ...`, `ci: ...`, `test(ros2): ...`. Pull request titles use the same form. Do not force push shared branches.

## Pull requests

- Open pull requests against `main` and fill in the pull request template.
- CI (`.github/workflows/ci.yml`, which runs `scripts/ci.sh` in the Docker image) must pass. Run `scripts/docker_run.sh scripts/ci.sh` locally before opening the pull request. CI does not run on draft pull requests; mark the pull request ready for review to run it.
- If you change a file the image bakes in (`docker/Dockerfile`, `docker/entrypoint.sh`, `scripts/requirements.txt`, `third_party/patches/glim/`, `third_party/patches/glim_ros2/`; the list is `image_inputs` in `scripts/image_inputs_hash.sh`), CI builds the image itself instead of pulling the prebuilt one, which takes longer.
- Add or update tests for behavior changes, and update the README or `docs/` when behavior or interfaces change.

By contributing, you agree that your contributions are licensed under the [MIT License](LICENSE).
