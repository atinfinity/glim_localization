# map_compiler follow-ups: compressed PCD, source on recompile, point limit ([#12](https://github.com/atinfinity/glim_localization/issues/12))

- **Change**: three of the six items of [#12](https://github.com/atinfinity/glim_localization/issues/12).
  - **`binary_compressed` PCD input.** `read_pcd` now reads PCL's `DATA binary_compressed`. The layout is two little-endian `uint32` (compressed and uncompressed size), then an LZF stream. The uncompressed data is field-major (all values of one field, then the next) and leaves out the `_` padding fields. The LZF decoder `lzf_decompress` (`map/pcd_map.hpp`) is written here, because neither PCL nor liblzf is in the image.
  - **`source` when a compiled map is compiled again.** `map_compiler` used to write `source: "glim_dump"` for any map directory, so a PCD map compiled again (for example with `--recompute-covariances`) lost its origin. `load_map` takes an optional `std::string* source` and reports `glim_dump` for a dump, or the value recorded in `map_index.json` (`unknown` if it is missing or not a string). `map_compiler` writes it back.
  - **Point limit.** `gtsam_points::estimate_covariances` takes the point count as an `int`. `build_pcd_submaps` and `recompute_covariances` now fail with a clear error above `INT_MAX` points instead of passing a wrapped count. Per-tile estimation is deferred (below).
- **Decisions**:
  - sdf2map at the pinned `66feb61` writes `savePCDFileBinary` or `savePCDFileASCII`, never `binary_compressed` ([#9](https://github.com/atinfinity/glim_localization/issues/9) did not need it). The format matters for PCL-based maps such as Autoware's.
  - Compressed sizes are untrusted, so both are checked before anything is allocated:
    - the uncompressed size must equal POINTS times the packed point size (without padding); that limits the data to 4 GiB, as in PCL;
    - the compressed size must fit in the rest of the file;
    - the uncompressed size must be at most 88 times the compressed size, the largest LZF expansion (a 3-byte back reference copies 264 bytes).
  - The decoder bounds-checks every literal run and back reference, and fails unless the output is exactly the expected size.
  - PCL leaves the `_` padding out of the compressed data (`writeBinaryCompressed` skips it), so field offsets in the compressed layout skip it too. Offsets that included the padding would read the wrong bytes, and the size check would reject the file.
  - The source is informational: `load_map` never changes how it loads a map because of it, and an odd value does not fail the load. `--recompute-covariances` keeps the source, because the points still come from the same input.
- **Deferred** (left open in [#12](https://github.com/atinfinity/glim_localization/issues/12)):
  - *Memory (about 300 bytes per point) and per-tile covariance for more than 2^31 points.* Both need the cloud to be read and tiled in a streaming way, with a neighbour margin at the tile edges so that edge covariances stay the same. That redesigns `read_pcd` and `build_pcd_submaps` and needs numeric checks against the whole-cloud estimate. The new error message points to `--voxel-size` or splitting the cloud meanwhile.
  - *Tile selection by bounding box in MapManager.* The code would be small (the distance from the pose to each tile's world AABB, sorted by that distance, then by centroid distance, then by id, because overlapping GLIM submaps often have distance 0). But it changes the tiles that every map factor and the GPU prefetch see. It needs its own PR with a replay comparison, and the tests in `test_map_manager` and `test_tile_prefetch` would get new expectations. The `--grid-size 50` to `100` advice stays.
  - *Upstream report.* Not filed. Filing is the maintainer's call. A draft for gtsam_points:

    > **`voxelgrid_sampling` silently merges points outside the 21-bit voxel range**
    >
    > `gtsam_points::voxelgrid_sampling` (CPU) packs each voxel coordinate into 21 bits. A point more than about 2^20 voxels from the origin (about ±105 km at a 0.1 m voxel, which is reached with UTM coordinates) wraps into another voxel, and its point is averaged into a voxel that it is not in. The output then holds points that are not near any input point. The only sign is a warning. Expected: an error or a return value that callers can check, or coordinates relative to the cloud's centre. Workaround in glim_localization: downsample in a frame centred on the bounding box, and reject clouds wider than about 2^21 voxels.
- **Tests** (synthetic data only):
  - `test_pcd_map`:
    - `LzfTest.DecompressesLiteralsAndBackReferences`: literals, an overlapping back reference, a long match (`len == 7` plus an extra byte), and a 13-bit offset.
    - `LzfTest.RejectsMalformedStreamsAndWrongSizes`: a short or long output, a run past the end of the input, a reference before the start, and missing length or offset bytes.
    - `PcdMapTest.ReadsAsciiAndBinaryWithExtraFields` also covers `BinaryCompressed` (F4 and F8; the test writer encodes literal runs only).
    - `PcdMapTest.ReadsBinaryCompressedWithPaddingAndBackReferences`: `_` padding left out, a U2 field after z, a back reference across points, and an empty cloud.
    - `PcdMapTest.RejectsBrokenBinaryCompressed`: a missing size header, a wrong uncompressed size, a truncated stream, a short output, a bad reference, and two sizes that must fail before allocating gigabytes.
    - `PcdMapTest.RejectsUnsupportedOrBrokenFiles` now rejects an unknown `DATA` value instead of `binary_compressed`.
  - `test_map_compiler`:
    - `MapCompilerTest.BinaryCompressedPcdGivesTheSameMapAsBinary`: the same `map_index.json`, points and covariances as binary PCD, with F4 and F8. Without `--voxel-size`: the multi-threaded `voxelgrid_sampling` does not keep the order of its output points, so with downsampling the two maps held the same points in a different order and the centroids differed in the last bits.
    - `MapCompilerTest.LoadMapReportsTheSource`: dump, compiled map, a missing or non-string value, and a failed load.
    - `MapCompilerTest.RecompilingKeepsTheSource`: a dump and a PCD map, each compiled again, also with `--recompute-covariances`.
- **Verification**: `scripts/docker_run.sh scripts/ci.sh` (Docker, amd64): all checks pass, 352 core tests, the colcon tests, interfaces, the Python tests, the quick robustness suite and the replay e2e.
- **Open**: `scripts/render_demo.py` still reads only ascii and binary PCD for its background map. The deferred items above stay in [#12](https://github.com/atinfinity/glim_localization/issues/12).
