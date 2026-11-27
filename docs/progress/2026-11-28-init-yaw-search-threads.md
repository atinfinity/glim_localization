# Thread count of the yaw search, and the close-out of [#35](https://github.com/atinfinity/glim_localization/issues/35)

- **Change**: a config key `init_yaw_search_num_threads` (default 0) sets the thread count of the initial-pose yaw search only, in both the CPU and the GPU modules.
  - 0 uses `map_factors.num_threads`, which is copied from the odometry `num_threads`, so the shipped behaviour does not change.
  - A positive value replaces it in the scorer's `map_factors` (the registrations and the residual of `score_candidate`). The per-frame odometry keeps `num_threads`.
  - A negative value is a config error (`std::invalid_argument` from the constructor, also with the search off, like the other `init_yaw_search_*` keys).
  - `run_init_search` now builds its `CandidateSearchParams` with a free function, `init_search_params(params, hint_position)`, so that the mapping is tested without running a search.
  - The key is in the four odometry configs (`glim_localization/config`, its `gpu` copy, and the two R1 configs), in the configuration reference of `docs/ros2.md` and in §5.3 of `docs/DESIGN.md`.
- **Why**: the [GPU profile](2026-11-28-gpu-profile.md) found that the search uses CPU factors in both modules and scales with the thread count. On the synthetic replay data (24 candidates, 8-core i7-9800X) it took 1.84 to 1.88 s at 2 threads (shipped), 1.19 to 1.21 s at 4 and 0.76 to 0.77 s at 8 on the CPU module, and about the same on the GPU module. Those numbers were taken by changing the odometry `num_threads`, which also changes the per-frame odometry. The new key gets the same gain for the one-time search alone.
- **Decisions** (the close-out of #35, approved by the owner):
  - **No GPU yaw search.** The gain is about 1.3 to 1.6 s, once per given pose, on a feature that ships disabled (`init_yaw_search: false`). 8 CPU threads already give 0.77 s, so a GPU path would save only about 0.2 to 0.5 s more, and it would bring tile uploads for poses that are then dropped (the reason the search uses CPU factors, §5.3).
  - Where the remaining items of #35 go:
    - **Batched multi-candidate scoring kernel**: moves to global search ([#3](https://github.com/atinfinity/glim_localization/issues/3)), where the candidate count reaches hundreds. The profile numbers above (about 77 ms per candidate at 2 threads, CPU factors) are its baseline.
    - **CPU overlap in the GPU module** (about 0.94 ms per frame, `select_map_tiles` and `map_inlier_ratio`): a possible separate issue. It is a design change (CPU/GPU parity of the inlier ratio, upload of selected tiles only), not a kernel.
    - **Preprocessing** (about 2.5 ms per frame with covariance estimation): waits for a profile on real data, which has more points per frame. Raising the preprocessing `num_threads` already removes 0.8 ms.
    - **Fused linearization kernel** (at most about 0.6 ms per frame): dropped. Tracking at about 3.2 ms per frame is well under the frame period, and it would need our own factor class.
- **Verification**:
  - **Search time against the new key** (CPU module, x86_64 i7-9800X, Docker, host load average about 2.8). The protocol is that of the [GPU profile](2026-11-28-gpu-profile.md): the same 12 s synthetic replay data, the e2e initial pose (ground truth plus (0.3, -0.2, 0.1) m and 3 deg), a copy of `glim_localization/config` with `init_yaw_search: true`, and `init_search.seconds` from `summary.json`, 3 runs per case. The odometry `num_threads` stayed at 2 in every case.

    | `init_yaw_search_num_threads` | Search, 24 candidates | Odometry per frame (median, later frames) |
    | --- | --- | --- |
    | 0 (shipped, uses `num_threads` 2) | 1.86 to 1.93 s | 14.8 to 14.9 ms |
    | 2 | 1.84 to 1.89 s | 14.8 to 14.9 ms |
    | 8 | 0.75 to 0.77 s | 13.6 to 14.0 ms |

    - Every run accepted -2.99 deg with score 0.996 and runner-up 0.890, and the trajectories of 0 and 8 are byte-identical. So the key changes the time, not the result.
    - The 8-thread search matches the profile's 0.76 to 0.77 s at `num_threads` 8, now without raising the per-frame thread count. The per-frame odometry was 0.8 to 1.2 ms faster in the 8-thread runs even though it still ran on 2 threads. That difference was not investigated.
  - **CI**: `git ls-files -z | xargs -0 touch`, then `COMPOSE_PROJECT_NAME=gloc35 scripts/docker_run.sh scripts/ci.sh` (Docker, amd64, `BUILD_WITH_CUDA=OFF`): `PASS: all checks`, 394 of 394 core tests including the new `InitYawSearchTest.SearchThreadCountDefaultsToTheOdometryAndCanBeOverridden`.
  - **GPU module**: its source is unchanged, but it includes the changed header and shares `run_init_search`. In the CUDA image (`glim_localization:dev-cuda`, `scripts/docker_run.sh --cuda`, a separate build directory with `BUILD_WITH_CUDA=ON`), `odometry_localization_gpu` and `test_localization_odometry_gpu` built, and the test passed 16 of 17 on the RTX 2080 Ti (the skip is `FailsClosedWithoutAVisibleDevice`). Its yaw search scored 24 of 24 candidates in 1.92 s at the default 0. The full CUDA test suite was not run.
- **Open**: none for #35. The items above are tracked in #3, or left for a new issue and for real data.
