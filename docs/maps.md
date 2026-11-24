# Map preparation

Localization takes either a GLIM dump directory (from a GLIM mapping run) or a compiled map. `map_compiler` writes a compiled map: a slimmed dump without frame data plus a `map_index.json` with the submap poses and bounding boxes. It is also the way to build a map from a PCD file.

The commands run inside the container (see [Build and test](build.md#build-and-test)). Put your input under the host directory `out/`, which the container sees as `/ws/out`.

```bash
# From a GLIM dump (or a compiled map)
/ws/build/core/tools/map_compiler /ws/out/glim_dump /ws/out/map

# From a PCD file (x, y, z in the map frame)
/ws/build/core/tools/map_compiler /ws/out/map.pcd /ws/out/map --grid-size 50 --voxel-size 0.1
```

## PCD options

PCD options (rejected for dump input):

| Option | Meaning | Default |
| --- | --- | --- |
| `--grid-size <m>` | Tile edge along x and y, at least 1 | 20 |
| `--voxel-size <m>` | Voxel downsampling before tiling, 0 disables it, otherwise at least 0.01 | 0 |
| `--k-neighbors <n>` | Neighbours for covariance estimation, 5 to 1000 | 10 |

PCD input reads `DATA ascii` and `binary` with x, y, z as F4 or F8; other fields are ignored. Localization uses up to `map_max_submaps` (default 5) tiles nearest to the robot, so 20 m tiles cover only about 40 x 40 m. For long-range LiDAR use `--grid-size` 50 to 100, or raise `map_max_submaps`.

Not supported yet: PLY input, compressed PCD and 2D map export ([#4](https://github.com/atinfinity/glim_localization/issues/4), [#12](https://github.com/atinfinity/glim_localization/issues/12)). See [Status and limitations](status.md).
