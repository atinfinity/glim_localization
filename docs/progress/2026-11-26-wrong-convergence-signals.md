# The monitor's signals do not detect a wrong convergence ([#50](https://github.com/atinfinity/glim_localization/issues/50))

- **Change**: no code. This entry records a measurement made before designing an online check for a wrong convergence after initialization, and why that check was not built. [DESIGN.md](../DESIGN.md) sections 5.3, 5.4 and 7 and [replay.md](../replay.md#robustness-on-synthetic-data) now say that TRACKING does not certify the initial pose.
- **Method**:
  - `scripts/replay_robustness.py --suite full --jobs 6 --keep-runs` at `435a8c3`, Docker, amd64, 16 cores, image `glim_localization:dev`. The outcomes were 104 `pass`, 48 `inaccurate`, 22 `rejected_low_score`, 12 `rejected_ambiguous`, 12 `lost` and 10 `low_tracking`.
  - These counts differ from [the arm64 entry](2026-11-25-real-data-prep.md), which reports 45 wrong convergences without the search and wrong search results at "4 to 5 m". The current full grid has translations of 4 m and 6 m, but no 5 m (the 5 m row on arm64 came from a separate probe). This run has 38 wrong convergences without the search. The platform and the commit differ, and some cells changed outcome, for example 1.5 m / 90 deg and 3 m / 135 deg.
  - Every case that exits 0 (`pass` or `inaccurate`) was split into groups. For each, the per-frame `inlier_ratio` and `residual` of `frames.csv` were taken over the first N = 10, 30 and 50 estimated frames.
  - **Correction:** an online proxy for the innovation of the map factors. It is the pose at frame k relative to the first estimate carried forward by the true motion, `est(t0) * gt(t0)^-1 * gt(tk)`, at k = 5, 10, 20 and 30. The true motion stands in for the IMU motion.
    - Without the search, the first estimate is about the given pose, so this is the correction that the map applies to the given pose.
    - With the search, the first estimate is the search result, so the search's own jump is not included.
- **Results:** the inlier ratio and the residual are over the first 30 frames (3 s), and the correction is the value at k = 30. N = 10 and 50 give the same picture.

  | Group | Cases | min inlier ratio | mean inlier ratio | median residual | correction [m] | correction [deg] |
  | --- | --- | --- | --- | --- | --- | --- |
  | pass, map change | 8 | 0.49 to 0.88 | 0.51 to 0.90 | 0.07 to 0.13 | 0.05 to 0.10 | 0.1 to 0.2 |
  | pass, no search | 41 | 0.38 to 0.99 | 0.71 to 1.00 | 0.07 | 0.01 to 4.24 | 0.1 to 44.9 |
  | pass, search | 55 | 0.99 | 1.00 | 0.07 | 0.01 to 0.02 | 0.1 |
  | wrong, no search, yaw 180 deg | 9 | 0.28 to 0.87 | 0.69 to 0.94 | 0.07 to 0.09 | 0.09 to 5.87 | 0.1 to 2.6 |
  | wrong, no search, other | 29 | 0.28 to 0.59 | 0.33 to 0.86 | 0.07 to 0.09 | 1.46 to 6.08 | 0.6 to 45.4 |
  | wrong, search at 4 m | 10 | 0.93 | 0.95 | 0.08 | 0.01 | 0.2 |

  - Across the whole sweep, every signal overlaps between passing and wrong cases:
    - The 90 % point removal passes with a mean inlier ratio of 0.51. The search cases at 4 m are wrong with 0.95, and the 180 deg flips without the search are wrong with up to 0.94.
    - The residual is 0.07 to 0.09 in every group except the map changes. Only the 90 % removal reaches 0.13, and it passes.
    - At k = 30, the passes without the search at 3 to 6 m correct 1.25 to 4.24 m and up to 45 deg. The 180 deg flips at 0 to 2 m correct 0.09 to 3.71 m and 0.1 to 0.9 deg. The search cases at 4 m correct 0.01 m, because the search lands directly on the wrong pose.
  - **Within search mode alone** the groups do separate in this sweep:
    - passing runs have a minimum inlier ratio of 0.993 and a residual of 0.071;
    - wrong runs have 0.934 and 0.084.

    The ten wrong search cases are, however, one deterministic run, so this is a single sample. A threshold near 0.95 would also fail every map-change case (0.51 to 0.90). Real scans are unlikely to score 0.996 either, so this gap is specific to the scene.
  - **The alias.** In the flip cases, the six 135 deg cases without the search (grouped under "other") and the search cases, the wrong pose is the corridor turned by 180 deg. Its ATE max is 36.24 m.
    - At the first frame it scores an inlier ratio of 0.959, against 0.996 at the truth. It agrees with the IMU and with the map.
    - At the start, the truth is close but turned by 180 deg: 3.75 m away in the search cases at 4 m, and 0.05 to 1.5 m away in the flips without the search at 0 to 2 m. The search generates candidates only at the given position, so it never scores the truth. The error grows to 36 m only because the two trajectories move in opposite directions.
- **Decisions**:
  - **No verifier on the monitor's signals.** A verification window on the inlier ratio, the residual or the correction was designed and then dropped. Over the whole sweep, any threshold would either fail passing cases or miss wrong ones. The only split, within search mode, rests on one run and a scene-specific score.
  - **No hard-coded alias check.** A check that scores the scan at the corridor-symmetric pose was rejected: it encodes the test scene. A generic re-check that runs the existing yaw search again around the converged estimate, after a short window, was not tried. It could catch the flips at 0 to 2 m without the search, which start within 1.5 m of the truth. It cannot catch the search cases at 4 m, whose converged pose is 3.75 m from the truth. It is listed under Open.
  - **No new quick-suite case.** A case that requires `inaccurate` would pin the defect as expected behaviour. The existing case at 0 m and 90 deg without the search keeps accepting `lost` or `inaccurate`.
  - **Fix path with the search:** the candidate search ([#3](https://github.com/atinfinity/glim_localization/issues/3)).
    - For example, a translation grid behind `set_candidate_generator` would put a candidate within reach of the truth, which is 4 m from the given position and 3.75 m and 180 deg from the alias. It would score about 0.996 next to the alias at 0.959.
    - The gap of 0.037 is within `init_yaw_search_min_score_gap` (0.05), so the existing rule would reject the pose as `rejected_ambiguous` instead of accepting the alias.
    - That margin is only 0.013, and the candidate count grows with the grid against the 10 s budget, so both need measuring first.
- **Verification**: the measurement above. The analysis scripts were throwaway and are not committed. The tables reproduce from the `frames.csv` and `traj_localization.txt` of each run kept by `--keep-runs`, together with the dataset's `traj_gt.txt`.
- **Open**:
  - [#50](https://github.com/atinfinity/glim_localization/issues/50) stays open.
  - **Without the search:** try the yaw re-check around the converged estimate (above).
  - **With the search:** the translation candidates of [#3](https://github.com/atinfinity/glim_localization/issues/3).
  - Until then, keep the initial pose within about 2 m, and enable `init_yaw_search` when the yaw is uncertain.
