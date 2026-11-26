# Map source guidance: GLIM dump or point cloud ([#61](https://github.com/atinfinity/glim_localization/issues/61))

- **Change**: a section [Which map to keep](../getting_started.md#which-map-to-keep) in step 5 of the getting started guide, and a one-line pointer to it at the top of [Map preparation](../maps.md). Documentation only; no code changes.
  - Localization runs the compiled map from `map_compiler`. The master copy is the GLIM dump when the map was built with GLIM, and the PCD or PLY file otherwise (another SLAM, a survey, sdf2map).
  - A table of what the point cloud path changes: covariances estimated over the whole cloud with `--k-neighbors`, disjoint x-y tiles of `--grid-size`, and the whole cloud in memory with at most 2^31 - 1 points.
  - The precision pitfall of large coordinates, the `source` field of `map_index.json`, and that the accuracy difference is not measured yet.
- **Decisions**:
  - The guidance sits in the getting started guide, where users first run `map_compiler`, and `maps.md` links to it instead of repeating it. The README is unchanged: it already links both pages.
  - Every statement was checked against the code and the existing docs:
    - covariances and tiling: `map/pcd_map.hpp` (`build_pcd_submaps`, `PcdMapParams`) and DESIGN 5.1;
    - lossless recompiling of a dump: `--recompute-covariances` is off by default, so points, poses (from `values.bin`) and covariances are copied; a compiled map has `map_index.json` and the submap directories, but no `graph.bin`, `values.bin` or trajectories (`save_compiled_map` in `map/map_io.cpp`);
    - `source`: `map/map_io.hpp` and the [map_compiler follow-ups](2026-11-27-map-compiler-followups.md) entry;
    - large coordinates: `read_pcd` reads x, y and z into doubles (`read_float` widens F4), and each tile stores its points relative to its centroid, so F8 input keeps its precision. The float32 steps in the text (1/32 m at 500 km, 0.25 m at 3,000 km) are the spacing of single floats at those values.
  - `map_compiler` cannot export a compiled map back to a point cloud (it has no such option), so the guide says so as a reason to keep the PCD or PLY file. Whether to add an export stays open in [#61](https://github.com/atinfinity/glim_localization/issues/61).
- **Verification**: `zensical build --clean --strict` (Python 3.12, `requirements-docs.txt`) after `scripts/docs_site_index.py`, so the new anchor `getting_started.md#which-map-to-keep` resolves.
- **Open** ([#61](https://github.com/atinfinity/glim_localization/issues/61)):
  - measuring the same scene compiled from a GLIM dump and from its PCD export (ATE, inlier ratio, yaw search scores);
  - deciding whether `map_compiler` should export a compiled map to PCD.
