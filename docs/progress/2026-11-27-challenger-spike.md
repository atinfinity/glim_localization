# A challenger search around the estimate does not detect the 180 deg alias ([#50](https://github.com/atinfinity/glim_localization/issues/50))

- **Change**: no code. This entry records a spike for an online check of a wrong convergence. The check compares the converged estimate with pose candidates around it, and it was not built. It follows [the measurement of the monitor's signals](2026-11-26-wrong-convergence-signals.md).
- **Idea tested**: an asymmetric challenger test.
  - After initialization, score yaw and translation candidates around the converged estimate (the incumbent) on the current scan with `score_candidate`, the scorer of the yaw search.
  - The pose would fail if a distinct challenger scored more than a margin above the incumbent.
  - Unlike the symmetric score gap of the yaw search, a close but lower challenger would not fail a correct pose.
  - Go criterion: on the same frame, every passing run must have a margin below 0 and every wrong run a margin above 0, with a usable gap.
- **Method**:
  - Baseline: `scripts/replay_robustness.py --suite full --jobs 6 --keep-runs` at `b1b5e72`, Docker, amd64, 16 cores. The outcomes were 104 `pass`, 48 `inaccurate`, 22 `rejected_low_score`, 12 `rejected_ambiguous`, 12 `lost` and 10 `low_tracking`, the same counts as in [#51](https://github.com/atinfinity/glim_localization/pull/51).
  - For every run that exits 0, on the scan of frame k, as `run_init_search` builds it (IMU frame, preprocessed, covariances, not deskewed), three things were registered and scored:
    - the incumbent, which is the run's estimate at k;
    - the ground truth;
    - challengers: 12 yaws at 30 deg times an xy grid of radius 2 m with a 1 m step, 155 candidates in all.
  - A challenger is distinct if its registered pose is more than 20 deg or 1 m from the registered incumbent.
  - **Margin** is the best distinct challenger score minus the incumbent score. **Truth margin** is the truth score minus the incumbent score, the oracle bound for a search that reaches the truth.
  - The tool was throwaway and is not committed.
- **Results**: margin per group, min to max.

  | Group | Runs | Frame 0 | Frame 10 | Frame 30 |
  | --- | --- | --- | --- | --- |
  | pass, no search | 41 | -0.037 to +0.533 | -0.036 to +0.390 | -0.069 |
  | pass, search | 55 | -0.036 | -0.036 | -0.069 |
  | pass, map change | 8 | +0.005 to +0.046 | +0.006 to +0.045 | -0.094 to -0.002 |
  | wrong, no search | 38 | +0.037 to +0.616 | -0.073 to +0.633 | +0.007 to +0.582 |
  | wrong, search at 4 m | 10 | +0.037 | +0.037 | +0.070 |

  - At frames 0 and 10 the passes without the search are still converging, so a better pose nearby is real. The check could only run after the estimate settles.
  - At frame 30 the signs separate, but by 0.0097 only: the highest pass is -0.002 and the lowest wrong run is +0.007. The small wrong margins (+0.007 to +0.027) come from the 2 m grid. In those runs the truth is 4.9 to 5.5 m away and scores +0.18 above the incumbent.
  - **Correct estimates lose to the alias.** In the 8 map-change passes the incumbent is within 0.03 m of the truth. In most samples the best challenger is the corridor turned by 180 deg, 1.3 to 4.4 m away (about 1.9 m at frame 30); four samples have a translation-only challenger. Margin over the run (11 sampled frames, passes only):

    | Map | 5 | 15 | 20 | 25 | 30 | 35 | 40 | 50 | 60 | 80 | 100 | mean |
    | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
    | none | +0.006 | -0.005 | -0.011 | -0.039 | -0.084 | -0.060 | -0.111 | -0.057 | +0.015 | -0.027 | -0.069 | -0.040 |
    | drop90 | +0.010 | +0.001 | -0.015 | -0.032 | -0.075 | -0.041 | -0.069 | -0.056 | +0.026 | +0.014 | -0.088 | -0.030 |
    | objects_removed | +0.043 | +0.041 | +0.031 | +0.010 | -0.004 | -0.036 | +0.061 | -0.008 | +0.021 | +0.005 | -0.021 | +0.013 |
    | combined | +0.040 | +0.038 | +0.032 | +0.011 | -0.002 | -0.027 | +0.065 | -0.002 | +0.026 | +0.014 | -0.087 | +0.010 |

    `drop30`, `drop50`, `drop70` and `objects_added` behave like `none` (mean -0.032 to -0.041). The margin of a correct pose is positive on 2 of the 11 sampled frames on the unchanged map, and on 2 to 4 with points removed. With objects removed it is positive on 7 of 11, by up to +0.061 (+0.065 with `combined`), and it is positive on the mean of the sampled frames. Wrong runs were measured at frames 0, 10 and 30 only.
  - **Oracle bound** (truth margin at frame 30, wrong runs):
    - alias cases, meaning the search cases at 4 m and the cases without the search at 135 to 180 deg: +0.069 in 22 of 25. The other three (180 deg at 0.5 to 1.5 m without the search, converged 3.7 m off) are at +0.169, and the 2 m grid already finds that margin;
    - the other wrong runs without the search: +0.176 to +0.582. Their converged error is 2.9 to 9.6 m.
- **Decisions**:
  - **No challenger check, single-scan or windowed.**
    - On a single scan the go criterion fails: frames 0 and 10 overlap, frame 30 separates by 0.0097 only, and a correct pose on a changed map turns positive on 2 to 7 of 11 sampled frames. Which frame is checked would decide the result.
    - Averaging does not help, because the alias advantage with objects removed is a bias, not noise.
    - A challenger carried forward with the incumbent's motion measures the same score difference.
    - For 22 of the 25 alias cases, even a search that always reached the truth would see +0.069. That is within 0.004 of the +0.065 that correct runs show against the alias, so any threshold that catches them fails correct runs.
  - **Most alias cases are not observable by scan matching here.** The synthetic corridor is nearly symmetric under a 180 deg turn. The search cases at 4 m and most cases without the search at 135 to 180 deg converge about 1.9 m from the truth, turned by 180 deg. There the scans prefer the truth by no more than they prefer the alias in correct runs. Detecting them needs information from outside the scan match: motion through an asymmetric part of the map, a prior from another sensor, or place recognition.
  - **The other wrong runs could be separated with enough reach.** Their oracle margin is +0.176 or more, against at most +0.065 for correct runs. The runs the 2 m grid misses have the truth 4.7 to 5.5 m away, so the grid would need about 6 m: roughly 9 times the 155 candidates, about a minute per frame on this machine. That is not affordable online. A translation search at initialization ([#3](https://github.com/atinfinity/glim_localization/issues/3)) addresses the same cases at the start, where the search already runs.
  - **For [#3](https://github.com/atinfinity/glim_localization/issues/3):**
    - Two distinct yaw hypotheses within about 0.07 to 0.1 of each other must not be decided on one scan. The existing `rejected_ambiguous` path, or keeping both hypotheses over several frames, is the safe answer.
    - The sweep gives poses up to 8 m off, and the converged errors reach 9.6 m, so the translation reach has to be chosen against the search budget, not from these runs.
- **Verification**: the measurement above (Docker, amd64). The tables reproduce from the kept runs (`traj_localization.txt`, `replay_args.json`) and the dataset's `traj_gt.txt`, by scoring the scans as described.
- **Open**:
  - [#50](https://github.com/atinfinity/glim_localization/issues/50): the done criteria are revised on the issue. The alias cases are a known limitation of a symmetric scene.
  - [#3](https://github.com/atinfinity/glim_localization/issues/3): translation candidates and multi-hypothesis yaw at initialization, with the constraints above.
