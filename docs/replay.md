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
  --out docs/media/replay_synthetic.gif'
```

> [!NOTE]
> The default view is `--view oblique`, a fixed 3D camera above and beside the run; the README uses it. `--view top` draws the earlier top-down 2D view, and `--view chase` a camera that follows the pose.

- `render_demo.py` draws, per estimated frame, the map points, the current scan moved to the map frame by the estimate, the current pose (an orange dot with a thin dark edge, and a heading arrow), the estimated trajectory so far, the ground truth (`<data>/traj_gt.txt` when it exists; `--gt`, `--no-gt`), and the state and inlier ratio from `frames.csv`. Frames that are not `estimated` are skipped.
- 3D views (`oblique`, `chase`): the map is cropped to the trajectory plus `--margin` (default 6 m) and the scan is drawn in red. The floor (up to 0.15 m above the lowest map height) is drawn as smaller light gray points, so the scan keeps its contrast on it. The points above are colored by height in 16 levels of viridis without its yellow end. The key in the header starts at the floor cut, and the floor is in the legend. The pose also has a dark up axis (its tilt) and a stem to the surface below, and thin blue lines drop from the estimate to the surface below it every 0.6 m of path: they show the height of the path above the floor, and that it passes over the low boxes in the corridor. Points are projected by a pinhole camera in numpy and drawn far to near; points behind the camera are dropped. `--azimuth` (default -110 deg), `--elevation` (default 42 deg) and `--distance` (default 0.9 times the map box diagonal) place the oblique camera, and `--aspect` (default 0.5) sets the plot height to width.
- Top view (`--view top`): the map in gray from above. `--z-min` and `--z-max` draw only the map and scan points in a map-frame height band, for example `--z-min 1.0 --z-max 3.0` to hide the floor and the low boxes, which the path passes over and would otherwise seem to drive through. The band also applies to the 3D views.
- The map is a PCD file (ascii or binary); non-finite points are dropped. Pass `--t-lidar-imu` with the value of `config_sensors.json` when it is not the identity.
- The output is a GIF or WebP, chosen by the extension. The layout is `--width` px wide (default 720) and is rendered at `--scale` times that (default 2, so 1440 px); the README shows it at `width="720"`, so it stays sharp on high-density screens and in a column wider than 720 px. A GIF uses one palette of `--colors` entries (default 256) that holds black, white, the text, state, plot and height colors exactly, plus adaptive colors from the first, middle and last frames; frames are mapped to it without dithering. Other options: `--fps`, `--stride`, `--max-frames`, `--max-map-points`, `--max-scan-points` (default 4000 for top, 1000 for the 3D views), `--title`.
- Text uses `--font` (default `Noto Sans`). The image installs it (`fonts-noto-core`) and builds the matplotlib font cache after it, so the README GIF uses Noto Sans when rendered in the image. Where the font is missing, the script warns on stderr and uses DejaVu Sans, which ships with matplotlib.
- The published image therefore includes `fonts-noto-core` under the SIL Open Font License 1.1; the license text is in `/usr/share/doc/fonts-noto-core/copyright` in the image. No font file is in this repository.
- Keep each committed file at about 2 MB or less, and commit only the image: never a bag, dataset or map.

### Yaw search clip

`docs/media/replay_yaw_search.gif` shows the initial yaw search ([#3](https://github.com/atinfinity/glim_localization/issues/3)) on the same data: the replay starts from a pose with the yaw about 60 deg off, the search finds the right yaw on the first scan, and tracking starts. `--init-search` adds two held images before the run: the first scan placed by the given pose ("INITIAL GUESS"), then by the pose the search picked ("YAW SEARCH", with the yaw correction and the number of candidates). It reads `replay_args.json` and the `init_search` record of `summary.json`, so the run needs `init_yaw_search: true` in a copied config; without an accepted search the script exits 2.

```bash
scripts/docker_run.sh bash -c '
  rm -rf /ws/out/config_yaw /ws/out/run_yaw &&
  cp -r glim_localization/config /ws/out/config_yaw &&
  sed -i "s/\"init_yaw_search\": false/\"init_yaw_search\": true/" /ws/out/config_yaw/config_odometry_localization_cpu.json &&
  /ws/build/core/tools/glim_localization_replay --data /ws/out/phase4_e2e/input/data --config /ws/out/config_yaw \
    --map /ws/out/phase4_e2e/input/map --out /ws/out/run_yaw --init-pose 5.3 -0.2 1.1 0 0 0.5225 0.8526 &&
  env -u PYTHONPATH $GLIM_LOC_PYTHON scripts/render_demo.py \
    --data /ws/out/phase4_e2e/input/data --run /ws/out/run_yaw --map /ws/out/phase4_e2e/input/map.pcd \
    --init-search --max-frames 60 --out docs/media/replay_yaw_search.gif'
```

<img src="media/replay_yaw_search.gif" width="720" alt="Yaw search on the synthetic replay data: the first scan placed by an initial pose with the yaw about 60 deg off, then by the pose the search picked, then tracking">

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
