# Clock-offset override and robustness on synthetic data ([#5](https://github.com/atinfinity/glim_localization/issues/5), [#48](https://github.com/atinfinity/glim_localization/pull/48))

These are the parts of [#5](https://github.com/atinfinity/glim_localization/issues/5) that do not need a new dataset.

- **Change**:
  - `bag_to_replay.py --point-clock-offset SECONDS|estimate` converts bags whose absolute per-point times use another clock than the header and IMU stamps. Such bags were always rejected before.
  - `scripts/replay_robustness.py` replays the synthetic data from perturbed initial poses and against changed maps. Its quick suite runs from `scripts/test_replay_e2e.sh`, so CI gates it. Usage is in [replay.md](../replay.md#robustness-on-synthetic-data).
- **Decisions**:
  - **Offset convention:** header clock = point clock + offset. It is applied only to absolute per-point times. Relative times follow the header stamp and need no offset, so the option is refused for them, for `--time-mode relative`, and for clouds without a time field.
  - **Default:** the rejection stays the default. The error message now names the option.
  - **`estimate`:** uses the first frame with usable per-point times (header stamp - first point time), so the conversion stays a single streaming pass. A median over all frames was rejected: it needs a second pass, and with a constant offset it gives the same value.
  - **Checks after the offset:** these are on the residual (first point after the offset - header).
    - Each frame must be within 1 s of its header. This catches a wrong offset and large jumps.
    - The residual may spread (max - min) over at most one scan period, the median header interval. This catches drift and jumps below 1 s, which the 1 s check alone let through. It runs at the end of the conversion.
    - Changes smaller than one scan period still pass.
  - **Known limit of `estimate`:** it assumes that the header stamps the first point. A driver that stamps the scan end gives a constant error of up to one scan period. The residual cannot show this, because it is near 0 by construction (the estimate comes from frame 0). Only a known offset avoids it. replay.md says so in a warning.
  - **Validation and logging:** the value must be finite and at most 1e9 s (about 32 years); otherwise exit 2 before any output. The offset is printed at the start (given) or when estimated, and again with the residual at the end. `meta.json` gets `conversion.point_clock_offset` (`source`, `offset_ns`, `offset_s`, `estimated_from_frame`, `definition`, and `residual_s` min, median and max), or `null` without the option.
  - **Robustness cases:**
    - Initial pose: the ground truth at the first frame, moved along the xy diagonal (1, -1)/sqrt(2) by 0 to 8 m and rotated about the world z axis by 0 to 180 deg. Each pose runs with `init_yaw_search` off and on, on the GLIM dump.
    - Map change: done on the map PCD and compiled with `map_compiler`, including the unchanged baseline, so the rows compare like with like. The scans are unchanged. The replay starts from the e2e offset (0.3, -0.2, 0.1) m and 3 deg.
    - Pass: replay exit 0 and `evaluate.py` within 0.1 m and 1 deg after 3 s, with the default `--min-tracking-ratio` 0.9.
    - Each case gets an outcome:
      - `pass`;
      - `inaccurate`: exit 0 but outside the limits;
      - `lost`;
      - `rejected_<reason>`, with the reasons of `candidate_search.cpp`;
      - `low_tracking`;
      - `timeout`: 120 s per process by default;
      - `error`: an exception in the script, kept per case;
      - `exit<N>`.
  - **Quick suite (CI):**
    - Each case expects an outcome, not just a failure, so a crash or a bad argument cannot pass as the expected failure.
    - Every case is far from the measured boundary:

      | Case | Expected outcome | Margin |
      | --- | --- | --- |
      | 0.5 m and 10 deg, no search | pass | 35 deg from the yaw limit |
      | 0 m and 90 deg, no search | lost or inaccurate | |
      | 0.5 m and 90 deg, with search | pass | |
      | 8 m, with search | `rejected_low_score` | best inlier ratio 0.43 against the minimum 0.7 |
      | 50 % of points removed | pass | ATE max 0.028 to 0.034 m |
      | 12 boxes added | pass | ATE max 0.033 m |

    - The first version gated the combined map change and the ambiguous rejection at 3 m. Review of [#48](https://github.com/atinfinity/glim_localization/pull/48) found both too close to the limits:
      - the combined change has an ATE max of 0.086 m on amd64 against the 0.1 m limit;
      - the ambiguous score gap of 0.038 is 0.012 from the 0.05 threshold.

      Both are now in the full suite only.
- **Verification** (Docker, arm64, 8 cores, image `glim_localization:dev`):
  - **Converter tests:** `scripts/tests/test_bag_to_replay.py` passes, 39 tests. The new ones cover:
    - the default rejection, now with a hint;
    - a given offset and `estimate`, each written to `meta.json`;
    - a negative offset;
    - `null` without the option;
    - drift above 1 s, caught at frame 2;
    - slow drift, 30 ms per 0.1 s frame: caught by the spread check, while 5 ms per frame passes with a recorded spread of 0.02 s;
    - a wrong given value;
    - `estimate` with a first frame of NaN times or no points: it uses frame 1;
    - invalid input (`abc`, `nan`, `inf`, `-inf`, `2e9`, `-2e9`, empty), exit 2 with no output;
    - refusal for relative times, `--time-mode relative`, and clouds without a time field.
  - **Existing real bag:** the private OS1-128 recording of the smoke test, mounted read-only. Its per-point times are relative (`t`, f4), so it cannot exercise the offset itself. The default conversion is unchanged: 1153 frames and 11525 IMU samples, with `point_clock_offset` `null`. `--point-clock-offset estimate` is refused with exit 1 ("applies to absolute per-point times only, but they are relative"). Together with `--time-mode relative` it gives exit 2. No output was kept.
  - **Script failure paths:**
    - A case without an initial pose records `error` (KeyError), and the other cases still run.
    - `--timeout 0.3` records `timeout` for every case, exits 1, and still writes `results.json` and `results.md`.
  - **Quick suite** (about 15 s; ATE max):

    | Case | arm64 | amd64 (CI, first version) |
    | --- | --- | --- |
    | 0.5 m and 10 deg, no search | 0.0107 m | 0.0119 m |
    | 0 m and 90 deg, no search | lost | lost |
    | 0.5 m and 90 deg, with search | 0.0107 m | 0.0119 m |
    | 8 m, with search | `rejected_low_score`, 0.431 | not in that version |
    | 50 % of points removed | 0.0340 m | 0.0279 m |
    | 12 boxes added | 0.0333 m | not in that version |
    | combined change | 0.0721 m | 0.0858 m |

    The numbers below are from arm64, and amd64 differs by up to about 0.015 m in ATE max.
  - **Full suite:** about 9.3 min with `--jobs 3`. Rows 0 to 3 m and the map table are identical over three runs.
  - **Initial pose without the yaw search** (`P` pass; `i` inaccurate; `L` lost; `D` low tracking ratio):

    | translation \ yaw [deg] | 0-20 | 30 | 45 | 60 | 90 | 135 | 180 |
    | --- | --- | --- | --- | --- | --- | --- | --- |
    | 0, 0.5, 1 m | P | P | i | L | L | i | i |
    | 1.5 m | P | P | L | D | i | i | i |
    | 2 m | P | P | i | i | D | i | i |
    | 3 m | P | P | P | i | D | i | i |
    | 4 m | i | i | P | L | D | i | i |
    | 6 m | i | i | P | i | D | i | i |
    | 8 m | i | i | i | i | D | D | i |

    It passes below 45 deg up to 3 m. From 4 m it fails even at 0 deg (inaccurate, ATE max 2.9 to 10.7 m). The passes at 4 and 6 m with 45 deg are chance alignments.
    45 cases exit 0 with a wrong pose (ATE max 2.9 to 36 m, tracking ratio 0.93 to 1.0). The replay's tracking-ratio gate does not catch them; only the ground truth does.
  - **Initial pose with the yaw search** (range ±180 deg, step 15 deg):

    | translation | every yaw from 0 to 180 deg |
    | --- | --- |
    | 0 to 1.5 m | pass |
    | 2 m | pass, except 10 deg: `rejected_ambiguous` |
    | 2.25 to 3 m | `rejected_ambiguous`: 0.997 at the right yaw against 0.959 for the corridor turned by 180 deg, inside the 0.05 gap |
    | 4 to 5 m | **accepts the corridor turned by 180 deg** (inlier ratio 0.958 to 0.959), exit 0, ATE max 36.2 m |
    | 6 to 8 m | `rejected_low_score`: best inlier ratio 0.43 to 0.60 against the minimum 0.7 |

    The 2.25 m and 5 m rows come from a separate probe with yaw 0 and 90 deg.
    **The success boundary is 2 m of translation for any yaw with the search, and below 45 deg of yaw up to 3 m without it.**
    Between 2.25 and 3 m the search rejects safely. At 4 to 5 m it accepts a wrong pose without any signal: the right yaw is no longer among the best candidates, so the turned corridor wins the gap.
  - **Map change** (outcome as in the script):

    | map | points | outcome | ATE rmse / max [m] | rot max [deg] | mean inlier ratio | tracking ratio |
    | --- | --- | --- | --- | --- | --- | --- |
    | unchanged (compiled PCD) | 44264 | pass | 0.0142 / 0.0293 | 0.061 | 0.926 | 1.00 |
    | 30 % removed at random | 31007 | pass | 0.0114 / 0.0291 | 0.055 | 0.921 | 1.00 |
    | 50 % removed | 22217 | pass | 0.0132 / 0.0340 | 0.073 | 0.897 | 1.00 |
    | 70 % removed | 13413 | pass | 0.0091 / 0.0217 | 0.076 | 0.822 | 1.00 |
    | 90 % removed | 4516 | pass | 0.0081 / 0.0186 | 0.123 | 0.495 | 1.00 |
    | 95 % removed | 2227 | low_tracking | 0.0155 / 0.0305 | 0.178 | 0.296 | 0.46 |
    | 98 % removed | 861 | lost (at 4.9 s) | 0.0633 / 0.1116 | 1.100 | 0.115 | - |
    | pillars and low boxes removed (3793 points) | 40471 | pass | 0.0191 / 0.0388 | 0.172 | 0.897 | 1.00 |
    | 12 boxes added beside the path (2726 points) | 46990 | pass | 0.0142 / 0.0333 | 0.064 | 0.931 | 1.00 |
    | objects removed, boxes added, then 30 % removed | 31039 | pass | 0.0241 / 0.0721 | 0.291 | 0.895 | 1.00 |

    Accuracy holds up to 90 % of the points removed. At 95 % the error is still 0.03 m, but the inlier ratio falls below `degraded_inlier_ratio` 0.3. The state is then DEGRADED, and the tracking-ratio gate fails the run. At 98 % it goes LOST. Removed objects cost the most among the realistic changes (ATE max 0.039 m, 0.072 m combined), and every one stays within the limits.
- **Open**:
  - A wrong convergence is not detected online. This happens without the search from 45 deg of yaw, or from 4 m at any yaw. It also happens with the search at 4 to 5 m, where the search accepts the turned corridor. In all of these the exit code is 0 and the tracking ratio is up to 1.0. A consistency check after initialization is a possible follow-up. Until then, give an initial pose within about 2 m and enable `init_yaw_search` when the yaw is uncertain.
  - Left in [#5](https://github.com/atinfinity/glim_localization/issues/5):
    - selecting the repeated-traversal dataset;
    - building the map from one traversal and localizing on another;
    - the full evo report (ATE, RPE, processing time, tracking loss) on that dataset;
    - verifying `bag_to_replay.py`, including `--point-clock-offset`, on bags from other LiDAR drivers (the only real bag at hand has relative per-point times).
  - The synthetic scene is a corridor. Its 180 deg ambiguity shapes the yaw-search boundary, so real environments will differ.
