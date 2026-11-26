# A translation bound for the initial yaw search ([#3](https://github.com/atinfinity/glim_localization/issues/3))

- **Change**: a new key `init_yaw_search_max_translation` of the odometry config, in metres. 0, the default, is off and keeps the earlier behaviour.
  - With a bound, the best candidate and the runner-up are chosen only among registered poses within that xy distance of the given pose. z is not bounded.
  - A registered pose beyond the bound that reaches `init_yaw_search_min_inlier_ratio` and scores at least as high as the best within it rejects the pose with the new reason OUT_OF_RANGE. One beyond it that scores lower is ignored.
  - The order of the checks is NO_CANDIDATE, TRUNCATED, OUT_OF_RANGE, LOW_SCORE, AMBIGUOUS. The decision over the scores is now a pure function, `decide_candidates`, which `search_candidates` calls after scoring.
  - Reporting:
    - `CandidateScore::translation`, the xy distance from the given pose;
    - `CandidateSearchResult::out_of_range`, the best match beyond the bound;
    - `InitSearchReason::OUT_OF_RANGE = 5` and `INIT_SEARCH_REASON_OUT_OF_RANGE = 5` in `LocalizationStatus.msg`;
    - for this reason, `init_search_score` and `init_search_yaw_offset` are those of the match beyond the bound, and `init_search_runner_up_*` those of the best within it, the one it outscored;
    - a LOW_SCORE message names a match beyond the bound when there is one (`beyond 2.50 m: 0.600 at 5.00 m`), so that the bound does not hide it;
    - the log and diagnostics text starts with `out of range:`, and `replay_robustness.py` classifies it as `rejected_out_of_range`.
  - A negative or non-finite bound is rejected when the module is built, even with the search off.
  - `replay_robustness.py --search-max-translation` sets the bound for the cases with the search, and `results.json` and `results.md` record it. The quick suite gains the case at 4 m and 0 deg with the search and a 2.5 m bound, which is expected to be `rejected_out_of_range`.
- **Why**: on the synthetic corridor, a given pose 4 m off with the search converged to the corridor turned by 180 deg, about 1.9 m from the truth, and was accepted (10 `inaccurate` runs). All candidates start at the given position, and the wrong pose registers well, so nothing else in the search compares it against the truth. The registered pose of those runs moved 2.98 m from the given pose, further than any accepted correct pose moved.
- **Decisions**:
  - **A bound on the distance moved, not a translation grid.** A throwaway tool (not committed) scored the 24 yaws on frame 0, both at the given position and on an xy grid (step 2 m, out to 4 m), for every search case of the sweep (99 cases). The grid scores were then put through the selection rules offline:
    - the rules of today on the given position alone reproduce the sweep exactly: 55 pass, 22 low score, 12 ambiguous, 10 wrong at 4 m;
    - the grid out to 2 m keeps the 55 passes (gap to the runner-up 0.093 to 0.109) and turns the 10 wrong runs into low score;
    - the grid out to 4 m makes every pass ambiguous: it reaches the alias of each correct pose;
    - the bound alone, with no grid, keeps the 55 passes and rejects the 10 wrong runs for any bound from 2.0 to 2.5 m. At 3.0 m the wrong runs come back.

    The bound alone gives what the 2 m grid gives, at no extra cost. Distance moved by the best registered pose:

    | Cases (search on) | Outcome before | Moved [m] |
    | --- | --- | --- |
    | 0 to 2 m off | pass | at most the offset (0 to 2.0) |
    | 3 m off | ambiguous | 3.0 |
    | 4 m off, wrong | inaccurate | 2.98 |
    | 4 m off, other | ambiguous | 3.83 |
    | 6 m off | low score | 1.7 to 2.94 |
    | 8 m off | low score | 1.78 |

  - **The bound states the position uncertainty, and is a contract.** A given pose further off than the bound is rejected even where the search would have found the truth; the 3 m cases, which were already rejected as ambiguous, are now rejected as OUT_OF_RANGE: there the truth scores 0.996, 3.0 m from the given pose, and the best within the bound 0.88 to 0.93. A bound at or beyond the distance the alias moves (3.0 m here) lets the wrong convergence back in. 2.5 m is the value for this corridor, not a default for real maps; the default stays 0.
  - **A lower match beyond the bound is ignored, and one below the minimum falls through.** Rejecting on any match beyond the bound would reject correct poses whose 180 deg alias registers further away. A far pose that scores low everywhere (6 and 8 m) keeps the reason LOW_SCORE, which is the more useful one there.
  - **An excluded pose is never the runner-up.** Otherwise a correct pose would be ambiguous against a match the bound has already ruled out.
  - **The cases without the search are out of scope.** The 38 wrong convergences without the search have no candidate set to bound.
  - **Deferred**: a translation grid, FPFH or other global features. The grid found nothing the bound does not on this scene, and both need an asymmetric scene to be measured; the corridor is nearly symmetric under a half turn ([spike](2026-11-27-challenger-spike.md)).
- **Verification** (Docker, amd64, 16 cores):
  - Unit tests with fabricated scores (`DecideCandidates.*`): bound 0 and a bound without a hint position keep the unbounded decision; a lower match beyond the bound is ignored and is not the runner-up; a match beyond it at or above the minimum and the best within it, a tie included, gives OUT_OF_RANGE; one below the minimum gives LOW_SCORE; ambiguity is decided among poses within the bound; the bound is inclusive and ignores z; truncation and no candidate come first.
  - Synthetic scans:
    - `SymmetricSceneTest.MatchBeyondTheBoundIsRejected`: the given pose 1 m off with a 0.5 m bound gives `out of range: inlier ratio 0.932 at a yaw correction of -20.0 deg lies 1.00 m from the given pose, beyond 0.50 m (best within 0.426; ...)`;
    - `InitYawSearchTest.OffsetPoseWithinTheBoundConverges`: 1.5 m and 90 deg off with a 2.5 m bound is accepted (score 0.996) and tracks within 0.006 m;
    - `InitYawSearchTest.OffsetPoseBeyondTheBoundIsOutOfRange`: the same pose with a 0.5 m bound is rejected as OUT_OF_RANGE and LOST, with the status score that of the match beyond the bound;
    - `InitYawSearchTest.InvalidParametersAreRejected`: a bound of -1, NaN or infinity throws.
  - `scripts/replay_robustness.py --suite full --jobs 4`, on the same input as the baseline of the [spike](2026-11-27-challenger-spike.md):

    | Bound | pass | inaccurate | rejected | lost | low_tracking |
    | --- | --- | --- | --- | --- | --- |
    | baseline (before) | 104 | 48 | 22 low score + 12 ambiguous | 12 | 10 |
    | 0 | 104 | 48 | 22 low score + 12 ambiguous | 12 | 10 |
    | 2.5 m | 104 | 38 | 22 low score + 22 out of range | 12 | 10 |

    - Bound 0 matches the baseline case by case.
    - At 2.5 m only search cases change: the 10 wrong runs at 4 m and the 12 ambiguous ones (3 m and 4 m) become `rejected_out_of_range`. For example, 4 m at 0 deg: `inlier ratio 0.959 at a yaw correction of 179.9 deg lies 2.98 m from the given pose, beyond 2.50 m (best within 0.630; 24 of 24 candidates in 2.050 s)`. No search case is `inaccurate`, every pass stays a pass, and no case without the search or with a map change changes.
  - The throwaway tool's timings were taken with the machine oversubscribed (6 jobs of 8 threads on 16 cores) and are not usable as a cost measurement. The bound adds no registration, so it does not change the cost of the search.
  - `scripts/ci.sh` (Docker, amd64): all checks pass, 355 core tests, the colcon tests, and the quick suite with `pose_t4_yaw0_search_mt2.5: rejected_out_of_range`.
- **Open**:
  - The cases without the search (38 wrong convergences) and the 180 deg alias in general: [#50](https://github.com/atinfinity/glim_localization/issues/50) (closed as a known limitation of the scene).
  - [#3](https://github.com/atinfinity/glim_localization/issues/3): the translation grid and global features wait for an asymmetric scene to measure them on.
