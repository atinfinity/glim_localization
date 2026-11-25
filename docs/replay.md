# Offline replay and evaluation

No ROS is needed. The replay reads a simple dataset (`meta.json`, `imu.csv`, `points/*.bin`; format in [DESIGN 6.5](DESIGN.md#65-offline-replay-phase-4)). Python tools run from the venv in the image:

```bash
alias py='env -u PYTHONPATH $GLIM_LOC_PYTHON'
```

Unless prefixed with `scripts/docker_run.sh`, the commands below run inside the container after the build (see [Build and test](build.md#build-and-test)). A map comes from [Map preparation](maps.md).

## Synthetic end to end

```bash
scripts/docker_run.sh scripts/test_replay_e2e.sh
```

Generates a 12 s synthetic map and dataset, localizes from an offset initial pose on the GLIM dump and on a PCD map, and checks accuracy (0.1 m, 1 deg after 3 s), determinism and exit codes. Results are in `out/phase4_e2e/`.

## Demo animation

The animation in the README (`docs/media/replay_synthetic.gif`) is rendered offscreen from the synthetic end-to-end run above. To regenerate it after `test_replay_e2e.sh`:

```bash
scripts/docker_run.sh bash -c 'env -u PYTHONPATH $GLIM_LOC_PYTHON scripts/render_demo.py \
  --data /ws/out/phase4_e2e/input/data --run /ws/out/phase4_e2e/run1 --map /ws/out/phase4_e2e/input/map.pcd \
  --z-min 1.0 --z-max 3.0 --out docs/media/replay_synthetic.gif'
```

- `render_demo.py` draws, per estimated frame, the map points, the current scan moved to the map frame by the estimate, the estimated trajectory so far, the ground truth (`<data>/traj_gt.txt` when it exists; `--gt`, `--no-gt`), and the state and inlier ratio from `frames.csv`. Frames that are not `estimated` are skipped.
- The map is a PCD file (ascii or binary); non-finite points are dropped. `--z-min` and `--z-max` draw only the map and scan points in a map-frame height band. The README image is a horizontal slice from 1 m (the sensor height) to 3 m: it hides the floor, and the low boxes in the middle of the corridor, which the trajectory passes over and would otherwise seem to drive through. Pass `--t-lidar-imu` with the value of `config_sensors.json` when it is not the identity.
- The output is a GIF or WebP, chosen by the extension. Other options: `--width`, `--fps`, `--stride`, `--max-frames`, `--margin`, `--title`.
- Keep the committed file under about 3 MB, and commit only the image: never a bag, dataset or map.

## Real bag

```bash
# 1. Convert a rosbag2 (sqlite3 or mcap) to a replay dataset
py scripts/bag_to_replay.py <bag_dir> --out /ws/out/dataset [--points-topic /points] [--imu-topic /imu] [--acc-scale 9.80665] [--gt traj.txt]

# 2. Localize
/ws/build/core/tools/glim_localization_replay --data /ws/out/dataset --config glim_localization/config \
  --map /ws/out/map --out /ws/out/run --init-pose x y z qx qy qz qw [--init-stamp <s>]

# 3. Evaluate against a reference trajectory (TUM, T_world_imu); traj_gt.txt is written only by --gt in step 1
py scripts/evaluate.py --est /ws/out/run/traj_localization.txt --gt /ws/out/dataset/traj_gt.txt \
  --frames /ws/out/run/frames.csv --out /ws/out/run [--settle 3] [--max-trans 0.1] [--max-rot-deg 1] [--plot]
```

- `bag_to_replay.py`: per-point time is detected as GLIM does (override with `--time-field`, `--time-mode`, `--time-scale`). `--acc-scale` is required when the accelerations look like units of g. Livox `CustomMsg` is not supported.
- `glim_localization_replay`: `--init-pose` is `T_world_imu` in the map frame. Other options: `--init-vel`, `--start`, `--end`, `--max-frames`, `--max-imu-gap` (default 0.05 s), `--on-imu-gap abort|reset`, `--min-tracking-ratio` (default 0.9). Outputs: `traj_localization.txt` (TUM), `frames.csv`, `summary.json`, `replay_args.json`. Exit codes: 0 ok, 1 runtime failure (including LOST or a low tracking ratio), 2 bad arguments, config, map or initial pose, 3 invalid dataset (including IMU gaps), 4 no pose.
- `evaluate.py`: ATE and RPE with evo (no realignment) and processing time statistics; writes `eval.json` and `eval.txt`, and exits 1 when a threshold or the pose coverage (`--min-coverage`, default 0.9) is not met.
- Set `T_lidar_imu` in `config_sensors.json` of a copied config for your sensor; the shipped configs use identity. Then pass `--config` your copied config instead of `glim_localization/config`.

The GPU module is selected with `--config glim_localization/config/gpu` in the CUDA image; see [CUDA variant](build.md#cuda-variant).

## Evaluation mapper

`glim_localization_mapper` and `scripts/smoke_os1_128.sh` build a GLIM map and a reference trajectory from a replay dataset for a real-data consistency check. They are evaluation tools, not a supported mapper; build maps for real use with GLIM.
