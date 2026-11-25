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

`scripts/ci.sh` is the same script that GitHub Actions runs. Each step (`scripts/build_core.sh`, `scripts/test_colcon.sh`, ...) can also be run alone with `scripts/docker_run.sh <script>`. See [Build and test](docs/build.md#build-and-test) for details.

## ROS 2 code style

The ROS 2 packages (`glim_localization_ros2/`, `glim_localization_msgs/`) follow the [ROS 2 code style](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html). `ament_lint_auto` with `ament_lint_common` adds a test per linter (uncrustify, cpplint, cppcheck, lint_cmake, xmllint, flake8, pep257, copyright), and `scripts/test_colcon.sh` fails on any finding. The core (`glim_localization/`) is not covered.

Run one linter on a package inside the container (the image sources ROS):

```bash
scripts/docker_run.sh bash -c 'cd glim_localization_ros2 && ament_flake8 .'
scripts/docker_run.sh bash -c 'cd glim_localization_ros2 && AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 ament_cppcheck .'
```

Fix the C++ formatting with uncrustify, then check the diff:

```bash
scripts/docker_run.sh bash -c 'cd glim_localization_ros2 && ament_uncrustify --reformat .'
```

The commit that reformatted the packages is listed in `.git-blame-ignore-revs`. GitHub's blame view skips it; for a local `git blame`, run `git config blame.ignoreRevsFile .git-blame-ignore-revs` once. Add a commit there only if it is purely mechanical.

Python uses single quotes, lines of at most 99 columns, imports in the google order, and docstrings whose one-line summary is on the line after the opening `"""`. Every source file starts with the short MIT header:

```cpp
// Copyright (c) 2026 atinfinity
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
```

Exceptions, all in `glim_localization_ros2/CMakeLists.txt` and `scripts/test_colcon.sh`:

- `src/localization_node.cpp` is derived from glim_ros2, and its provenance lines (see below) must come first and are longer than 100 columns. cpplint checks it in a test of its own (`cpplint_derived`) with a line length of 150 columns instead of 100, so the rest of the file still has a limit; the other linters check it as usual.
- `ament_cppcheck` skips cppcheck 2.x (Jazzy ships 2.13) unless `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1` is set, which `scripts/test_colcon.sh` does.
- Imports after a `sys.path.insert` in the launch tests carry `# noqa: E402` (and `I100` on the first one).

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

## Releases

Releases are automated with [release-please](https://github.com/googleapis/release-please) (`.github/workflows/release-please.yml`, config in `release-please-config.json`, current version in `.release-please-manifest.json`). The repository has one version, tagged `vX.Y.Z`.

- Each push to `main` opens or updates a release PR (`chore: release X.Y.Z`) when `main` has `feat`, `fix`, `perf` or `revert` commits since the last release. The release PR updates `CHANGELOG.md`, the manifest, the `<version>` in the three `package.xml` files, and the lines marked `x-release-please-version` (`project(... VERSION ...)` in `glim_localization/CMakeLists.txt` and `test_version.cpp`).
- `docs`, `ci`, `test`, `style`, `build`, `refactor` and `chore` commits are left out of the changelog and do not start a release on their own.
- Before 1.0.0, `feat` bumps the minor version and `fix` the patch version, and a breaking change (`feat!:` or a `BREAKING CHANGE:` footer) bumps the minor version.
- Merging the release PR creates the tag and the GitHub Release. Do not edit the versions or the changelog by hand; to force a version, add a commit with a `Release-As: X.Y.Z` footer.

By contributing, you agree that your contributions are licensed under the [MIT License](LICENSE).
