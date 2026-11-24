# CLAUDE.md — glim_localization

A LiDAR-IMU localization package for prebuilt 3D maps that uses GLIM as a library.
Design is in `docs/DESIGN.md`, the implementation plan in `docs/IMPLEMENTATION_PLAN.md`, history in `docs/PROGRESS.md`. **Read only the sections relevant to the current task.**

## Rules

- Do not modify GLIM itself (glim / gtsam_points / glim_ros2). If a fix is needed, put it in `third_party/patches/` and write the reason in `docs/PROGRESS.md`.
- Put this line at the top of every file derived from GLIM (if the base commit changes, update it here and in DESIGN too):
  `// Derived from koide3/glim <path> at commit 1a197c999e4865f5cab7c9cd25474128413ebbca (MIT License, Copyright (c) 2024 koide3)`
- The core (`glim_localization/`) is ROS-independent. Keep ROS dependencies inside `glim_localization_ros2/`.
- The GPU build is off by default (`BUILD_WITH_CUDA=OFF`).
- Do not commit bags or maps. Tests use synthetic data.
- Keep commits small and in English, using Conventional Commits with the area as scope (for example `feat(odometry): ...`, `fix(map): ...`, `docs: ...`, `ci: ...`). Do not force push.
- Record verification results and design decisions in `docs/PROGRESS.md`.

## Build

Build in Docker with Ubuntu 24.04 + ROS 2 Jazzy (headless, no GUI).

```bash
docker compose -f docker/compose.yaml build
scripts/docker_run.sh scripts/ci.sh  # build, tests, dependency checks
scripts/ci_local.sh [--force]        # optional, host side. Reuses a pass for the same tree and image; skips docs-only changes
```

CI is GitHub Actions: `build-and-test` (amd64) and `build-and-test-arm64` run `scripts/ci.sh` on every non-draft PR and on push to main. Running `scripts/ci_local.sh` before a PR is optional, not required.

Offline replay (inside the container; design in section 6.5 of `docs/DESIGN.md`):

```bash
scripts/docker_run.sh scripts/test_replay_e2e.sh   # replay and evaluation on synthetic data
# real bag: convert, replay, evaluate (call Python as env -u PYTHONPATH $GLIM_LOC_PYTHON)
env -u PYTHONPATH $GLIM_LOC_PYTHON scripts/bag_to_replay.py <bag_dir> --out <dataset>
/ws/build/core/tools/glim_localization_replay --data <dataset> --config <config> --map <map> --out <out> --init-pose x y z qx qy qz qw
env -u PYTHONPATH $GLIM_LOC_PYTHON scripts/evaluate.py --est <out>/traj_localization.txt --gt <dataset>/traj_gt.txt --frames <out>/frames.csv --out <out>
```
