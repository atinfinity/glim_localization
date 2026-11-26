# Map preparation

Localization takes either a GLIM dump directory (from a GLIM mapping run) or a compiled map. `map_compiler` writes a compiled map: a slimmed dump without frame data plus a `map_index.json` with the submap poses and bounding boxes. It is also the way to build a map from a PCD or PLY file, and it can export a 2D occupancy grid for Nav2.

Which input to keep as the master copy, a GLIM dump or a point cloud, and what each one costs, is in [Which map to keep](getting_started.md#which-map-to-keep).

The commands run inside the container (see [Build and test](build.md#build-and-test)). Put your input under the host directory `out/`, which the container sees as `/ws/out`.

```bash
# From a GLIM dump (or a compiled map)
/ws/build/core/tools/map_compiler /ws/out/glim_dump /ws/out/map

# From a PCD or PLY file (x, y, z in the map frame)
/ws/build/core/tools/map_compiler /ws/out/map.pcd /ws/out/map --grid-size 50 --voxel-size 0.1
/ws/build/core/tools/map_compiler /ws/out/map.ply /ws/out/map --grid-size 50 --voxel-size 0.1
```

## Point cloud options

Point cloud options (`.pcd` and `.ply`; rejected for dump input, except `--k-neighbors` with [`--recompute-covariances`](#recomputing-covariances-of-a-glim-dump)):

| Option | Meaning | Default |
| --- | --- | --- |
| `--grid-size <m>` | Tile edge along x and y, at least 1 | 20 |
| `--voxel-size <m>` | Voxel downsampling before tiling, 0 disables it, otherwise at least 0.01 | 0 |
| `--k-neighbors <n>` | Neighbours for covariance estimation, 5 to 1000 | 10 |

PCD input reads `DATA ascii`, `binary` and `binary_compressed` (PCL's LZF format) with x, y, z as F4 or F8; other fields are ignored. PLY input reads `format ascii` and `binary_little_endian` with vertex x, y, z as `float` or `double`. Other properties (intensity, normals, colors, lists) and other elements (faces) are ignored, and big-endian PLY is rejected. The same points give the same map from either format. Localization uses up to `map_max_submaps` (default 5) tiles nearest to the robot, so 20 m tiles cover only about 40 x 40 m. For long-range LiDAR use `--grid-size` 50 to 100, or raise `map_max_submaps`.

The whole cloud is held in memory (roughly 300 bytes per point), and a cloud with more than 2^31 - 1 points after downsampling is rejected; use `--voxel-size` or split the cloud ([#12](https://github.com/atinfinity/glim_localization/issues/12)). See [Status and limitations](status.md).

## Recomputing covariances of a GLIM dump

By default, the per-point covariances of a GLIM dump or a compiled map are copied as they are. `--recompute-covariances` estimates them again from the k nearest neighbours, the same way as for PCD and PLY input:

```bash
/ws/build/core/tools/map_compiler /ws/out/glim_dump /ws/out/map --recompute-covariances --k-neighbors 20
```

| Option | Meaning | Default |
| --- | --- | --- |
| `--recompute-covariances` | Estimate the covariances again (GLIM dump or compiled map input only) | off |
| `--k-neighbors <n>` | Neighbours for covariance estimation, 5 to 1000; for map directory input only with `--recompute-covariances` | 10 |

- **Per submap:** neighbours are searched within each submap, not across submaps. GLIM submaps overlap and each covers the sensor range, so a search across submaps would mix slightly offset copies of the same surface. The default k = 10 matches GLIM's per-scan default (`k_correspondences`); GLIM itself estimates covariances per scan and merges the transformed scan covariances into the submap, so recomputed covariances come from a different neighbourhood than the ones in the dump.
- **Tile edges:** for a map compiled from a PCD or PLY file, the tiles do not overlap, so points near a tile edge then see only the neighbours in their own tile. To change k for such a map, compile it again from the point cloud.
- **Errors:** `--recompute-covariances` with `.pcd` or `.ply` input, and `--k-neighbors` with map directory input but without `--recompute-covariances`, are rejected. A submap with fewer than k points fails the run, and nothing is written.
- Points, poses and submap ids are unchanged, and `map_index.json` keeps the input's `source` (`glim_dump` for a dump, `pcd` for a map compiled from a PCD file, and so on).

## 2D occupancy grid for Nav2

`--occupancy-grid <prefix>` also writes `<prefix>.pgm` and `<prefix>.yaml` in the `map_server` format, from any input. The obstacle band is required and is an absolute z in the map frame:

```bash
# A PLY map whose ground is at about z = 0: obstacles from 0.1 m to 2 m above it
/ws/build/core/tools/map_compiler /ws/out/map.ply /ws/out/map --voxel-size 0.1 \
  --occupancy-grid /ws/out/map_2d --occupancy-resolution 0.05 --occupancy-z-min 0.1 --occupancy-z-max 2.0
```

For a GLIM dump, z = 0 is the first sensor pose, not the ground: shift the band down by the sensor height (for example, with the LiDAR 1.8 m above the ground, `--occupancy-z-min -1.7 --occupancy-z-max 0.2`).

| Option | Meaning | Default |
| --- | --- | --- |
| `--occupancy-grid <prefix>` | Write `<prefix>.pgm` and `<prefix>.yaml`; refused if either exists or the prefix has no file name (`out/`) | off |
| `--occupancy-resolution <m>` | Cell edge, at least 0.01 | 0.05 |
| `--occupancy-z-min <m>` | Lower bound of the obstacle band (map-frame z) | required |
| `--occupancy-z-max <m>` | Upper bound of the obstacle band (map-frame z) | required |

- **Cells:** a cell is occupied if it holds a point with `z-min <= z < z-max`, free if it holds only points below `z-min` (the ground), and unknown otherwise. Points at or above `z-max` (ceilings, tree tops) are ignored.
- **No noise filter:** one point is enough for occupied. Voxel downsampling does not remove isolated outliers, so clean a noisy cloud first (for example with statistical outlier removal) and remove dynamic objects.
- **Flat ground:** the band is the same absolute z everywhere. It suits roughly flat sites with a gravity-aligned z axis; on slopes or several levels, one band cuts the ground in some places and misses obstacles in others. PCD and PLY input must be z-up.
- **Files:** the YAML uses `mode: trinary`, `negate: 0`, `occupied_thresh: 0.65` and `free_thresh: 0.1`, with `origin` at the lower-left corner of the grid and zero yaw. The grid is limited to 2^30 cells (about 1.6 x 1.6 km at 0.05 m).

Load it with `nav2_map_server` (`yaml_filename`). Details are in [DESIGN 5.1](DESIGN.md#51-mapmanager).
