# GPU map tile streaming: byte budget and prefetch ([#23](https://github.com/atinfinity/glim_localization/issues/23))

- **Change**:
  - The GPU module streams map tiles to the GPU within a byte budget and prefetches the tiles along the predicted path on a background thread. Before, `GpuTileCache` uploaded a tile synchronously on first use and kept at most `map_gpu_max_tiles` tiles, whatever their size.
  - New odometry keys (GPU config only, all validated before any GPU call):
    - `map_gpu_memory_budget_mb` (default 1024, 1 to 1048576);
    - `map_gpu_prefetch` (default true);
    - `map_gpu_prefetch_lookahead` (default 1.0 s, 0 to 10);
    - `map_gpu_prefetch_max_tiles` (default 10, 1 to `map_gpu_max_tiles`).
  - A budget above the device's total memory logs a warning at startup.
  - The policy is GPU-independent and part of the CPU library, so it is unit tested in the CPU CI:
    - `TileResidency`: byte and count accounting and LRU eviction plans;
    - `estimate_tile_device_bytes`;
    - `select_prefetch_tiles`;
    - `TileStreamer`: the acquire and prefetch logic and the worker thread, with the upload injected as a function.
  - `GpuTileCache` is a `TileStreamer` with CUDA uploads.
  - The cudaGetLastError nit: `cudaGetLastError` is per thread and returns any error left by earlier work. Before each upload and before the module creates its streams, `clear_earlier_cuda_error` reads such an error and logs it as coming from earlier work. After the upload, both the `cudaStreamSynchronize` result and `cudaGetLastError` fail the upload.
  - When the module shuts down it logs the counters: uploads, prefetched, misses, prefetch waits, evictions, frames over budget, and peak and budget MB.
- **Decisions**:
  - **The budget uses an estimate, not the measured size.** The estimate comes from the tile's point attributes and, per voxel level, the CPU voxel count. It uses 52 bytes per voxel and 16 per hash bucket, following gtsam_points 1.2.2's bucket doubling from 16384. So the budget can be checked before anything is allocated. A GPU test keeps the estimate within 0.5x to 2x of `memory_usage_gpu()` after the upload.
  - **A frame's tiles are pinned and acquired together.**
    - Eviction never removes a tile the current frame uses.
    - If one frame's tiles alone exceed the budget, they are still uploaded, and the frame counts as `over_budget`. Failing the frame would lose tracking because of a configuration mistake.
    - The excess is trimmed after the frame.
    - Tiles that factors in the smoother window still reference stay alive after eviction, as before. So the memory actually in use can exceed the budget by the window's tiles.
  - **Prefetch uses a worker thread, not just a second stream.** `GaussianVoxelMapGPU::insert` synchronizes on the stream it is given. An upload on a second stream from the odometry thread would still block that thread.
    - The worker has its own non-blocking `CUDAStream`.
    - A tile whose prefetch is still queued is uploaded in the foreground instead, and counts as a miss.
    - A tile whose prefetch is running is waited for (`prefetch_waits`).
    - A failed background upload is dropped and logged, not thrown: the tile was only predicted. A frame that needs it uploads it in the foreground, and only that failure throws, so the module still fails closed for the tiles it uses (changed in the review, see below).
    - The worker thread selects the device that was current when the cache was constructed.
  - **Prefetch never evicts the current frame's tiles or tiles still being uploaded.** It stops at the first tile that does not fit, so a small budget prefetches less rather than thrashing.
  - **Which tiles are prefetched.** The path from the current position to `p + v * lookahead` is sampled every half search radius, and `MapManager::query` runs at each sample. This gives the tiles the map factors will ask for. They are sorted by distance to the path, then to the current position (rounded to mm), then by id, and cut at `map_gpu_prefetch_max_tiles`. Selection by query, rather than a box around the path, keeps prefetch to tiles that will actually be used.
  - **Both limits apply.** `map_gpu_max_tiles` stays, and a tile is evicted when either limit is exceeded.
- **Verification**:
  - **CPU** (Docker on the development Mac): `scripts/ci.sh` passes. The CPU build and the CPU part of the e2e are unchanged. The new unit tests are `TileResidency*`, `TilePrefetchTest*` (8-tile synthetic map) and `TileStreamer*` (17 tests with a fake device, including threading, failure, cancel and destruction).
  - **GPU** (RTX 2080 Ti, CUDA 13.1 image): the core and colcon builds pass, and 327 of 327 core tests pass. Among them:
    - `TileSizeEstimateIsCloseToTheUpload`;
    - `TileCacheStaysWithinTheByteBudget`;
    - `TileCachePrefetchesInTheBackground`;
    - `ConvergesWithASmallMemoryBudget` (1 MiB);
    - `RejectsInvalidTileStreamingParameters`.

    The image predates the font package added for the demo renderer, so these runs used `GLIM_LOC_ALLOW_STALE_IMAGE=1`. Because that variable was set, the two Python tests that check this switch failed; they are unrelated to this change. `test_replay_e2e.sh` passes on the GPU.
  - **CPU vs GPU accuracy on the default map**: at most 3.5e-7 m and 2.4e-6 deg over 121 poses.
  - **New e2e block on the synthetic corridor in 4 m tiles** (85 tiles, `map_search_radius` 12, `map_max_submaps` 12): the GPU matches the CPU module within 7.2e-5 m and 1.6e-4 deg.
    - With prefetch: 33 uploads, 26 of them prefetched, 7 misses, 0 prefetch waits.
    - With a 24 MB budget: 3 evictions and a peak of 23.5 MB. It matches the CPU within 4.2e-6 m and 1.2e-4 deg.
  - **Latency** (`scripts/bench_tile_streaming.sh`, 121 frames, median of 5 runs per frame): "before" is main at the merge of [#43](https://github.com/atinfinity/glim_localization/pull/43). Its build ignores the new keys, so all its GPU runs behave alike. A transition frame is one whose nearby tiles (by ground truth position) differ from the previous frame's. All times are per-frame odometry time in ms.

    | Map | Run | All frames: mean / p95 / max | Transition frames: mean / p95 / max |
    | --- | --- | --- | --- |
    | 4 m tiles (85; 20 transitions) | before (main) | 5.10 / 5.48 / 19.62 | 4.76 / 5.11 / 5.28 |
    | | after, prefetch | 5.08 / 5.43 / 20.03 | 4.72 / 5.04 / 5.35 |
    | | after, no prefetch | 5.09 / 5.40 / 19.21 | 4.76 / 5.10 / 5.35 |
    | | after, 8 MB budget | 5.12 / 5.38 / 20.05 | 4.83 / 5.26 / 5.38 |
    | 10 m tiles (19; 8 transitions) | before (main) | 3.23 / 3.56 / 16.33 | 3.17 / 3.65 / 3.70 |
    | | after, prefetch | 3.22 / 3.44 / 17.14 | 3.02 / 3.26 / 3.27 |
    | | after, no prefetch | 3.24 / 3.51 / 16.83 | 3.11 / 3.54 / 3.67 |
    | | after, 24 MB budget | 3.22 / 3.45 / 17.30 | 3.01 / 3.23 / 3.24 |
    | CPU module, for reference | 10 m tiles | 8.99 / 9.43 / 195.93 | 7.23 / 8.42 / 8.60 |

    - **The overall max** (16 to 20 ms) is the first frame in every run: the initial uploads and CUDA warm-up. It is not a transition.
    - **4 m tiles**: a tile is about 0.3 MB, and its upload is too short to show in the frame time. Before and after agree within run-to-run noise.
      - Prefetch moved 26 of 34 uploads off the odometry thread, without a single wait.
      - The 8 MB budget forced 36 evictions with a peak of 7.9 MB, and no frame went over the budget.
    - **10 m tiles** (about 1 MB each): prefetch lowered the transition frames' p95 from 3.65 to 3.26 ms and their max from 3.70 to 3.27 ms. Without prefetch the transition numbers match main.
    - Pose differences to the CPU run were at most 1.1e-5 m and 4.1e-4 deg in every run.
  - **What the synthetic data cannot show**: the synthetic map is small and sparse, so a miss costs well under 1 ms here. The benefit on real maps (tiles of tens of MB) is not measured yet.
- **Review follow-up** ([PR #47](https://github.com/atinfinity/glim_localization/pull/47)):
  - `select_prefetch_tiles` capped the sample count after converting it to `int`, which is undefined for a huge velocity (on x86 nothing was prefetched; on arm64 it took 1000 queries per frame). The count is now capped in `double` first, and the test checks that a 1e12 and a 1e300 m/s velocity still return the current tiles.
  - A failed prefetch used to be rethrown from `poll`, `prefetch` and `acquire`, so an out-of-memory upload of a merely predicted tile escaped the odometry thread and aborted the node. It is now dropped, counted in `prefetch_failures` and logged; `acquire` retries a tile it needs in the foreground and throws only if that fails.
  - Queued prefetches are never evicted, so when they filled the budget `acquire` could not make room for the frame's tiles and went over budget. `acquire` now first cancels the queued prefetches the frame does not need (`prefetch_cancels`).
  - The worker thread did not set the CUDA device, so on a multi-GPU host it uploaded to device 0. `GpuTileCache` reads the device in its constructor and the worker calls `cudaSetDevice` before each upload.
  - The shutdown log line adds `prefetch_failures` and `prefetch_cancels`.
  - New CPU tests: a failed prefetch seen by `prefetch()` and by `acquire()` (of another tile, and of the same tile while it runs) is dropped and reported, then uploaded in the foreground; a frame tile that fails in the foreground still throws; queued prefetches filling the budget are cancelled for the frame's tiles without going over budget; the same tile id from another map while its prefetch runs.
- **Open** ([#23](https://github.com/atinfinity/glim_localization/issues/23)):
  - Real-data GPU evaluation: latency at tile transitions, and the estimate against real tile sizes.
  - Streaming tiles from disk into host memory: the whole map still stays resident in host memory.
  - GPU CI (the GPU tests run only on a host with a GPU).
