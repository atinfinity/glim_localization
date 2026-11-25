# Clock-offset override and robustness on synthetic data ([#5](https://github.com/atinfinity/glim_localization/issues/5))

These are the parts of [#5](https://github.com/atinfinity/glim_localization/issues/5) that do not need a new dataset.

- **Change**:
  - `bag_to_replay.py --point-clock-offset SECONDS|estimate` converts bags whose absolute per-point times use another clock than the header and IMU stamps. Such bags were always rejected before.
  - `scripts/replay_robustness.py` replays the synthetic data from perturbed initial poses and against changed maps. Its quick suite runs from `scripts/test_replay_e2e.sh`, so CI gates it. Usage is in [replay.md](../replay.md#robustness-on-synthetic-data).
- **Decisions**:
  - **Offset convention:** header clock = point clock + offset. It is applied only to absolute per-point times. Relative times follow the header stamp and need no offset, so the option is refused for them, for `--time-mode relative`, and for clouds without a time field.
  - **Default:** the rejection stays the default. The error message now names the option.
  - **`estimate`:** uses the first frame only (header stamp - first point time) and keeps the streaming conversion in one pass. The 1 s check still applies to every frame after the offset, so drift, clock jumps and a wrong given value fail the conversion instead of producing shifted frames. A median over all frames was rejected: it needs a second pass over the bag, and with a constant offset it gives the same value. `estimate` assumes that the header stamps the first point; a driver that stamps the scan end shifts the frames by up to one scan period. replay.md warns about this, and `meta.json` records the residual so the user can check it.
  - **Validation and logging:** the value must be finite and at most 1e10 s; otherwise exit 2 before any output. The offset is printed at the start (given) or when estimated, and again with the residual at the end. `meta.json` gets `conversion.point_clock_offset` (`source`, `offset_ns`, `offset_s`, `estimated_from_frame`, `definition`, and `residual_s` min, median and max), or `null` without the option.
  - **Robustness cases:**
    - Initial pose: the ground truth at the first frame, moved along the xy diagonal (1, -1)/sqrt(2) by 0 to 3 m and rotated about the world z axis by 0 to 180 deg. Each pose runs with `init_yaw_search` off and on, on the GLIM dump.
    - Map change: done on the map PCD and compiled with `map_compiler`, including the unchanged baseline, so the rows compare like with like. The scans are unchanged. The replay starts from the e2e offset (0.3, -0.2, 0.1) m and 3 deg.
    - Pass: replay exit 0 and `evaluate.py` within 0.1 m and 1 deg after 3 s, with the default `--min-tracking-ratio` 0.9.
  - **Quick suite (CI):** cases well inside or outside the measured boundary, so noise between amd64 and arm64 does not flip them:
    - 0.5 m and 10 deg without the search: pass;
    - 0 m and 90 deg without the search: fail;
    - 0.5 m and 90 deg with the search: pass;
    - 3 m with the search: fail (ambiguous rejection);
    - drop 50 %: pass;
    - the combined map change: pass.
- **Verification** (Docker, arm64, 8 cores, image `glim_localization:dev`):
  - **Converter tests:** `scripts/tests/test_bag_to_replay.py` passes, 35 tests. The new ones cover:
    - the default rejection, now with a hint;
    - a given offset and `estimate`, each written to `meta.json`;
    - a negative offset;
    - `null` without the option;
    - drift caught at frame 2;
    - a wrong given value;
    - invalid input (`abc`, `nan`, `inf`, `-inf`, `1e11`, empty), exit 2 with no output;
    - refusal for relative times, `--time-mode relative`, and clouds without a time field.
  - **Existing real bag:** the private OS1-128 recording of the smoke test, mounted read-only. Its per-point times are relative (`t`, f4), so it cannot exercise the offset itself. The default conversion is unchanged: 1153 frames and 11525 IMU samples, with `point_clock_offset` `null`. `--point-clock-offset estimate` is refused with exit 1 ("applies to absolute per-point times only, but they are relative"). Together with `--time-mode relative` it gives exit 2. No output was kept.
  - **Quick suite and e2e:** `scripts/test_replay_e2e.sh` passes with the quick suite, which takes about 6 s.
  - **Full suite:** about 3.4 min with `--jobs 3`. Two runs gave the same pass grid.
  - Initial pose without the yaw search: passes up to 30 deg at every translation up to 3 m. From 45 deg it fails at every translation, except a single pass at 3 m and 45 deg. 19 of the failures exit 0 but converge to a wrong pose: ATE max 3.1 to 36 m, with a tracking ratio up to 1.0. The tracking-ratio gate does not catch a wrong convergence; only the ground truth does.

    | translation \ yaw [deg] | 0-30 | 45 | 60 | 90 | 135 | 180 |
    | --- | --- | --- | --- | --- | --- | --- |
    | 0, 0.5, 1 m | pass | wrong pose | LOST | LOST | wrong pose | wrong pose |
    | 1.5 m | pass | LOST | LOST | wrong pose | wrong pose | wrong pose |
    | 2 m | pass | wrong pose | wrong pose | LOST | wrong pose | wrong pose |
    | 3 m | pass | pass | wrong pose | LOST | wrong pose | wrong pose |

  - Initial pose with the yaw search (range 180 deg, step 15 deg): passes at every yaw up to 1.5 m, and up to 2 m except 2 m and 10 deg. It fails at every yaw at 3 m. All 12 failures are safe rejections: the search found the right yaw (inlier ratio 0.997), but the corridor turned by 180 deg scored within `init_yaw_search_min_score_gap` 0.05 (0.95 to 0.96). The search rejects the pose as ambiguous, and the replay stops LOST with exit 1 instead of tracking a wrong pose. **The success boundary is about 2 m of translation for any yaw with the search, and below 45 deg of yaw for any translation up to 3 m without it.**
  - Map change:

    | map | points | pass | ATE rmse / max [m] | rot max [deg] | mean inlier ratio | tracking ratio |
    | --- | --- | --- | --- | --- | --- | --- |
    | unchanged (compiled PCD) | 44264 | yes | 0.0142 / 0.0293 | 0.061 | 0.926 | 1.00 |
    | 30 % removed at random | 31007 | yes | 0.0114 / 0.0291 | 0.055 | 0.921 | 1.00 |
    | 50 % removed | 22217 | yes | 0.0132 / 0.0340 | 0.073 | 0.897 | 1.00 |
    | 70 % removed | 13413 | yes | 0.0091 / 0.0217 | 0.076 | 0.822 | 1.00 |
    | 90 % removed | 4516 | yes | 0.0081 / 0.0186 | 0.123 | 0.495 | 1.00 |
    | 95 % removed | 2227 | no (tracking ratio) | 0.0155 / 0.0305 | 0.178 | 0.296 | 0.46 |
    | 98 % removed | 861 | no (LOST at 4.9 s) | 0.0633 / 0.1116 | 1.100 | 0.115 | - |
    | pillars and low boxes removed (3793 points) | 40471 | yes | 0.0191 / 0.0388 | 0.172 | 0.897 | 1.00 |
    | 12 boxes added beside the path (2726 points) | 46990 | yes | 0.0142 / 0.0333 | 0.064 | 0.931 | 1.00 |
    | objects removed, boxes added, then 30 % removed | 31039 | yes | 0.0241 / 0.0721 | 0.291 | 0.895 | 1.00 |

    Accuracy holds up to 90 % of the points removed. At 95 % the error is still 0.03 m, but the inlier ratio falls below `degraded_inlier_ratio` 0.3, so the state is DEGRADED and the tracking-ratio gate fails the run. At 98 % it goes LOST. Removed objects cost the most among the realistic changes (ATE max 0.039 m, 0.072 m combined), and every one stays within the limits.
- **Open**:
  - A wrong convergence from a large yaw error without the search is not detected online (exit 0, tracking ratio 1.0). A consistency check after initialization is a possible follow-up; until then, enable `init_yaw_search` when the yaw is uncertain.
  - Left in [#5](https://github.com/atinfinity/glim_localization/issues/5):
    - selecting the repeated-traversal dataset;
    - building the map from one traversal and localizing on another;
    - the full evo report (ATE, RPE, processing time, tracking loss) on that dataset;
    - verifying `bag_to_replay.py`, including `--point-clock-offset`, on bags from other LiDAR drivers (the only real bag at hand has relative per-point times).
  - The synthetic scene is a corridor. Its 180 deg ambiguity shapes the yaw-search boundary, so real environments will differ.
