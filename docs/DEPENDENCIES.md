# Dependency provenance

This page records where the pinned C++ dependencies in `docker/Dockerfile` come from, how the
prebuilt packages were checked against upstream source, and their licenses. The image build checks
the SHA256 of each PPA deb, and `scripts/check_deps.sh` checks the pinned versions, the version
strings of the installed headers, and the commits on every CI run.

Last verified: 2026-11-27, on the amd64 image built from `docker/Dockerfile` and on the contents of
the amd64 and arm64 gtsam_points debs of 2026-11-25 (see "Replacement on 2026-11-25" below). The
earlier check of 2026-11-24 used the arm64 image.

## Summary

| Dependency | Pinned as | Source | Upstream revision | License |
|---|---|---|---|---|
| GTSAM | deb `libgtsam-notbb-dev=4.3.0`, SHA256 per architecture | koide3 PPA, `ubuntu2404` | borglab/gtsam tag `4.3a1`, commit `2f3e56c0ddbd3a1aa54ed043643b553d26a069f6` | BSD-3-Clause (`LICENSE`, `LICENSE.BSD`), with bundled third-party code under its own licenses |
| gtsam_points | deb `libgtsam-points-dev=1.2.2`, SHA256 per architecture | koide3 PPA, `ubuntu2404` | koide3/gtsam_points tag `v1.2.2`, commit `9d32e7dbecf6015560d84b4901d6b0a6f483ec46` | MIT, Copyright (c) 2024 koide3; some headers carry GTSAM's BSD notice or nanoflann's BSD license |
| gtsam_points (CUDA image) | deb `libgtsam-points-cuda13.1-dev=1.2.2`, SHA256 (amd64) | koide3 PPA, `ubuntu2404` | same as `libgtsam-points-dev` | same as `libgtsam-points-dev` |
| ROS base image | `ros:jazzy-ros-base@sha256:5147d3ae…` in all three Dockerfiles | Docker Hub official image | multi-platform index, created 2026-11-20 | ROS packages under their own licenses (mostly Apache-2.0 and BSD), Ubuntu packages under theirs |
| GLIM | source, `GLIM_COMMIT` | `https://github.com/koide3/glim.git` | `1a197c999e4865f5cab7c9cd25474128413ebbca` | MIT, Copyright (c) 2024 koide3 |
| glim_ros2 | source, `GLIM_ROS2_COMMIT`, with `third_party/patches/glim_ros2/` | `https://github.com/koide3/glim_ros2.git` | `4d4ec524ccf1b02aa09b0af2af767ecc54343798` | MIT, Copyright (c) 2024 koide3 |

GLIM and glim_ros2 are cloned at a fixed commit and `check_deps.sh` compares `git rev-parse HEAD`,
so their provenance is exact. The rest of this page is about the two deb packages, which carry no
commit hash.

## The koide3 PPA

- Repository: `deb [signed-by=/etc/apt/keyrings/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2404 ./`
  (a flat repository served by GitHub Pages from the `ppa` branch of `koide3/ppa`).
- Signing key: `https://koide3.github.io/ppa/ubuntu2404/KEY.gpg`, fetched over HTTPS at image build
  time. RSA 4096, created 2025-12-21, user ID "Kenji Koide", fingerprint
  `A0DA E9CF 75CF EF37 A509  FCD3 C2C4 4E47 B79E 8F3A` as gpg prints it, which is
  `A0DAE9CF75CFEF37A509FCD3C2C44E47B79E8F3A` in the form the Dockerfile uses. The Dockerfile pins
  this fingerprint (`KOIDE3_PPA_KEY_FPR`) and fails the build unless the downloaded file holds
  exactly this one primary key. It then imports the file into a temporary GnuPG home and exports only
  that key into `/etc/apt/keyrings/koide3_ppa.gpg`. If the PPA rotates its key, check the new key and
  update the argument with the 40 hexadecimal characters and no spaces (`gpg --with-colons` prints
  that form in field 10 of the `fpr` line); a value with spaces never matches.
- `InRelease` is dated 2026-11-25 14:20:18 UTC and `gpgv` reports a good signature from that key.
  The SHA256 of `Packages` in `InRelease` matches the downloaded index (checked 2026-11-27).
- Packages are built by GitHub Actions workflows in `koide3/ppa` (`build_gtsam.yml`,
  `build_gtsam_points.yml`) in Docker images based on `ubuntu:noble`, for amd64 and arm64, and
  committed to the `ppa` branch. The index is signed in the same workflow.

Pinned debs (`ubuntu2404/Packages`, checked 2026-11-27; each file was also downloaded and hashed with
`sha256sum`). The Dockerfiles pin these values as build arguments: `apt-get download` fetches the
pinned version, the build compares the file with the value for `dpkg --print-architecture`, fails on
a mismatch or on an architecture without a value, and only then installs the checked file with
`dpkg -i` (`apt-get install` on a local file would fetch the same version from the repository again).

| File | Size | SHA256 | Build argument |
|---|---|---|---|
| `libgtsam-notbb-dev_4.3.0_amd64.deb` | 4671410 | `257cbe2edf232cbce64f3bac45060c6e56f1b17ead939b415ae7247557cb4e66` | `GTSAM_SHA256_AMD64` |
| `libgtsam-notbb-dev_4.3.0_arm64.deb` | 4409498 | `211bc1693478a3afbd9cc70bbc386a9ec4c9d06b9f87beb9c2192f1bbc7303cb` | `GTSAM_SHA256_ARM64` |
| `libgtsam-points-dev_1.2.2_amd64.deb` | 1258728 | `bd947013c199789e04b1f2a234159a107213b9decc04a892877de05b1898f8b4` | `GTSAM_POINTS_SHA256_AMD64` |
| `libgtsam-points-dev_1.2.2_arm64.deb` | 1198220 | `246d03b234bf7904cc755e62ed506e5097d4845ce38241e9bdf65ad039e1b34e` | `GTSAM_POINTS_SHA256_ARM64` |
| `libgtsam-points-cuda13.1-dev_1.2.2_amd64.deb` | 4412582 | `320d224d18b9470cae2945b227425a8a74de620d9b3064da340261b8a85db848` | `GTSAM_POINTS_CUDA_SHA256_AMD64` (`docker/Dockerfile.cuda` only) |

## GTSAM (`libgtsam-notbb-dev 4.3.0`)

**The package version 4.3.0 is the GTSAM 4.3a1 pre-release, not the GTSAM 4.3.0 release.**

- Build recipe: `packages/docker/Dockerfile.gtsam` in `koide3/ppa` (unchanged since 2026-06-13)
  runs `git clone https://github.com/borglab/gtsam.git -b 4.3a1`, renames the CPack package to
  `libgtsam-notbb-dev`, and configures with `GTSAM_WITH_TBB=OFF`, `GTSAM_BUILD_WITH_MARCH_NATIVE=OFF`,
  `GTSAM_USE_SYSTEM_EIGEN=ON`, and `GTSAM_USE_SYSTEM_METIS=ON`. It does not use the `packages/gtsam`
  submodule of that repository.
- Tag `4.3a1` points to commit `2f3e56c` ("Merge branch 'release/4.3a1'", 2025-11-18). Its
  `CMakeLists.txt` sets version 4.3.0 with pre-release `a1`, which is where the package version
  comes from.
- The installed `gtsam/config.h` has `GTSAM_VERSION_STRING "4.3a1"` and `GTSAM_VERSION_NUMERIC 40300`,
  no `GTSAM_USE_TBB`, and defines `GTSAM_USE_SYSTEM_EIGEN`, `GTSAM_USE_SYSTEM_METIS`,
  `GTSAM_ENABLE_BOOST_SERIALIZATION`, and `GTSAM_ALLOW_DEPRECATED_SINCE_V43`.
- Header comparison: all 484 headers the package installs under `/usr/local/include`
  (`gtsam`, `gtsam_unstable`, `CppUnitLite`) were compared byte for byte with the source tree at
  `2f3e56c`. All are identical except the three files CMake generates (`gtsam/config.h`,
  `gtsam/dllexport.h`, `gtsam_unstable/dllexport.h`). The ten headers installed from other source
  directories (`3rdparty/cephes/cephes/*.h`, `3rdparty/CCOLAMD/Include/ccolamd.h`) are also identical.
- Other candidates are ruled out:
  - The `packages/gtsam` submodule in `koide3/ppa` points to `38f55ea` ("4.3.1"). It differs from
    `2f3e56c` only in `CMakeLists.txt` and `package.xml`, and would produce `GTSAM_VERSION_PATCH 1`
    and package version 4.3.1.
  - The GTSAM release tag `4.3.0` (`71a25ca`, 2026-09-18) is 2432 commits later. Of the 481
    installed headers that are not generated, 289 differ from that tag and 20 no longer exist there
    (with the same path mapping as above), and 172 are identical.
- The deb has been rebuilt in place under the same version. On the `ppa` branch of `koide3/ppa`,
  both `libgtsam-notbb-dev_4.3.0_amd64.deb` and `libgtsam-notbb-dev_4.3.0_arm64.deb` were added in
  `4cf106a` (2026-01-24 07:24 UTC) and then replaced five times, always together: three times on
  2026-01-24 (`7a6b94b` at 07:57, by the Iridescence workflow; `27704c2` at 08:13; `9fcefd1` at
  13:01), then `2636685` (2026-06-13) and `88fb3b0` (2026-07-11). Builds from 2026-06-13 on use tag
  `4.3a1`. Pinning the version alone therefore does not pin the bytes; the SHA256 values above are
  the ones in use since 2026-07-11, and the build checks them.

## gtsam_points (`libgtsam-points-dev 1.2.2`)

- Build recipe: `packages/docker/Dockerfile.gtsam_points` in `koide3/ppa` copies the
  `packages/gtsam_points` submodule, which points to `9d32e7d` (tag `v1.2.2`, 2026-07-11). It installs
  `libgtsam-notbb-dev` from the same PPA, then builds with `CMAKE_BUILD_TYPE=Release` and otherwise
  default options (no CUDA). The recipe also installs `libtbb-dev` and `libomp-dev`; TBB still ends
  up off.
  The deb was added to the `ppa` branch on 2026-07-11 22:49 UTC (`703f1b5`), after the GTSAM rebuild
  that day, and replaced under the same version on 2026-11-25 (see below).
- The installed `gtsam_points/config.hpp` has version 1.2.2, an empty `GTSAM_POINTS_GIT_HASH`,
  OpenMP on, and TBB, CUDA, and `-march=native` off.
- Header comparison: all 141 headers under `/usr/local/include/gtsam_points` are identical to
  `include/` at `9d32e7d`, except the generated `config.hpp`. The four later commits on the default
  branch change 11 files under `include/`, so the headers match the tag and not the current branch.
- Replacement on 2026-11-25: `6b45536` on the `ppa` branch (14:02 UTC, "Add gtsam_points packages
  from build_gtsam_points workflow") replaced `libgtsam-points-dev_1.2.2_amd64.deb`,
  `libgtsam-points-dev_1.2.2_arm64.deb` and `libgtsam-points-cuda13.1-dev_1.2.2_amd64.deb` (previous
  SHA256 `e00218a5…6eca`, `312b4f74…9795` and `985b80ab…6889`). It came from a `repository_dispatch`
  run of `build_gtsam_points.yml` at `koide3/ppa` `4777cfa`, whose changes since the previous build
  only add `apt-utils` for `apt-ftparchive` and update Iridescence; the `packages/gtsam_points`
  submodule still points to `9d32e7d`. The old debs (from `703f1b5`) and the new ones were extracted
  with `dpkg-deb` and compared file by file on 2026-11-27:
  - `libgtsam-points-dev` (amd64 and arm64): the control files and the file lists (149 entries) are
    equal, and every installed file, `libgtsam_points.so.1.2.2` included, is byte for byte identical.
    Only the archive metadata (timestamps) differs.
  - `libgtsam-points-cuda13.1-dev` (amd64): the control files and the file lists (152 entries) are
    equal, and every file is identical except `libgtsam_points_cuda.so.1.2.2`. Its size and its 770
    exported symbols are unchanged; the 28 differing bytes all lie in the non-loaded `.strtab`
    section, in the names of nvcc's temporary files (`tmpxft_<n>_…cudafe1.cpp`), which depend on the
    build process.
  - In all three new debs the 141 headers under `gtsam_points/` are identical to `include/` at
    `9d32e7d` except the generated `config.hpp`, which is identical to the previous one.

  The replacement therefore changes no header, ABI or code; the pins above are the 2026-11-25 files.
- The package depends on `libgtsam-notbb-dev` without a version, so it is only consistent with the
  GTSAM above as long as both come from the same PPA snapshot. The `=4.3.0` pin in
  `docker/Dockerfile` is what keeps a future PPA GTSAM build (for example 4.3.1, which the ppa
  submodule already points to) out of the image.

## Result

Both packages match the expected upstream revisions at the header level: GTSAM `2f3e56c` (4.3a1) and
gtsam_points `9d32e7d` (v1.2.2).

Decision: keep the PPA packages; no source build. The image build checks the PPA key fingerprint and,
since [#11](https://github.com/atinfinity/glim_localization/issues/11), the SHA256 of every PPA deb per
architecture, on a digest-pinned base image. `scripts/check_deps.sh` requires the exact deb versions
and the exact lines `#define GTSAM_VERSION_STRING "4.3a1"` and
`#define GTSAM_POINTS_VERSION_STRING "1.2.2"`.

Not verified:

- The compiled libraries were not rebuilt and compared. Bit-for-bit reproducibility of the PPA
  builds was not tested, so the check relies on the headers, the generated config headers, and the
  published build recipe.
- The GTSAM headers were compared on arm64 only; the amd64 GTSAM deb has the same SHA256 as at that
  check. The gtsam_points debs of 2026-11-25 were compared for both architectures (above).
- A cached image layer skips the SHA256 check: it runs only when the layer that installs the debs is
  rebuilt (for example after a change of the Dockerfile, or with `--no-cache`).
- The tags `4.3a1` and `v1.2.2` were assumed not to have moved since the packages were built. Git
  does not record tag history.

## CUDA image (`docker/Dockerfile.cuda`)

The optional CUDA image (amd64 only; [CUDA variant](build.md#cuda-variant)) uses the same GTSAM, GLIM and glim_ros2
pins as the CPU image (a Python test checks that the shared arguments are equal) and differs in:

| Dependency | Pinned as | Source | Notes |
|---|---|---|---|
| gtsam_points | deb `libgtsam-points-cuda13.1-dev=1.2.2` (instead of `libgtsam-points-dev`) | koide3 PPA, `ubuntu2404` | Same upstream tag `v1.2.2` built with CUDA 13.1 by the same `koide3/ppa` workflows. Links `libcudart.so.13`, SASS for sm_75/80/90/100/110/120 and PTX for sm_120, `GTSAM_POINTS_CUDA_VERSION 13.1.80`. Installs the same paths as `libgtsam-points-dev` and declares no `Conflicts`, so the two must never be installed together (`check_deps.sh` fails if both are). It declares no CUDA runtime dependency; the image provides cudart. |
| CUDA toolkit | debs `cuda-nvcc-13-1=13.1.115-1`, `cuda-cudart-dev-13-1=13.1.80-1`, `cuda-cccl-13-1=13.1.115-1` | NVIDIA apt repository `https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64` | Their unversioned dependencies (for example `cuda-toolkit-config-common`) come at whatever version the repository has at build time. `check_deps.sh` checks that `nvcc` reports 13.1. |
| GLIM | same commit, `BUILD_WITH_CUDA=ON`, plus `third_party/patches/glim_cuda/` | | GLIM `1a197c9` calls `IndexedSlidingWindow::has_index`, which gtsam_points added after v1.2.2 (commit `8f50081`, 2026-08-02). The CUDA-only patch replaces the one call with the same range test (docs/PROGRESS.md). |

The CUDA build of gtsam_points is checked against its own SHA256 (`GTSAM_POINTS_CUDA_SHA256_AMD64`,
in the table of "The koide3 PPA" above), and the GTSAM deb against the same value as in the CPU
image. The file pinned before 2026-11-25 (`985b80ab…6889`, 4412600 bytes) was replaced; see
"Replacement on 2026-11-25".

NVIDIA repository signing key: `3bf863cc.pub` from the repository URL above, fetched over HTTPS at
image build time. RSA 4096, created 2022-04-14, user ID "cudatools <cudatools@nvidia.com>", fingerprint
`EB69 3B30 35CD 5710 E231  E123 A4B4 6996 3BF8 63CC`. The Dockerfile pins it (`CUDA_REPO_KEY_FPR`) and
fails the build unless the file holds exactly this primary key, and exports only that key into
`/etc/apt/keyrings/cuda_repo.gpg`, as for the PPA key. `CUDA_REPO_KEY_FPR` takes the same form, 40
hexadecimal characters without spaces (`EB693B3035CD5710E231E123A4B469963BF863CC`).

The headers of the CUDA build of 2026-11-25 were compared with `include/` at `9d32e7d` like those of
`libgtsam-points-dev` (only `config.hpp` differs); its `config.hpp` reports version 1.2.2 with CUDA
on.

The CUDA toolkit is distributed under the NVIDIA CUDA Toolkit End User License Agreement. This
repository does not redistribute it or the image; anyone who publishes a built CUDA image must comply
with that license.

## Licenses and redistribution

- GTSAM: BSD-3-Clause text in `LICENSE.BSD` ("simplified BSD" in its `LICENSE`). The installed headers
  include bundled third-party code: CCOLAMD and SuiteSparse_config (BSD-3-Clause), the ceres
  auto-diff code (BSD), and cephes (MIT). Eigen and METIS come from the system packages.
- gtsam_points: MIT. Nine headers under `optimizers/` are derived from GTSAM and carry GTSAM's
  copyright notice, and `ann/small_kdtree.hpp` includes nanoflann's BSD license text.
- The debs ship no copyright or license files (there is nothing under `/usr/share/doc` for either
  package). This repository does not redistribute the debs or the image. Anyone who publishes a
  built image must add the GTSAM and gtsam_points license notices themselves. Eigen (MPL-2.0) and
  METIS (Apache-2.0), which GTSAM's `LICENSE` also lists, come from Ubuntu packages that ship their
  own copyright files.

## How to repeat the check

1. Inside the image: `apt-cache policy` and `dpkg -s` for both packages, and
   `gpg --show-keys /etc/apt/keyrings/koide3_ppa.gpg`.
2. Inside the image, in an empty directory such as `/tmp`, check the index against the published key:

   ```bash
   for f in InRelease Packages KEY.gpg; do curl -fsSLO "https://koide3.github.io/ppa/ubuntu2404/$f"; done
   gpg --show-keys --with-fingerprint KEY.gpg      # compare with the fingerprint above
   gpg --dearmor < KEY.gpg > ppa_key.gpg
   gpgv --keyring ./ppa_key.gpg InRelease          # expect "Good signature"
   grep -A3 '^SHA256:' InRelease | grep ' Packages$'; sha256sum Packages
   ```

   Then compare the `Size` and `SHA256` of the five package entries in `Packages` with the table
   above, and with `sha256sum` of each downloaded deb. (`gpgv --keyring /etc/apt/keyrings/koide3_ppa.gpg InRelease` checks against the key the
   image installed instead.)
3. Get the headers and the upstream source on the host:

   ```bash
   docker run --rm --entrypoint tar glim_localization:dev -C /usr/local/include \
     -cf - gtsam gtsam_unstable CppUnitLite gtsam_points | tar -xf - -C installed
   curl -fsSL https://codeload.github.com/borglab/gtsam/tar.gz/2f3e56c0ddbd3a1aa54ed043643b553d26a069f6 \
     | tar -xzf - -C gtsam_src --strip-components=1
   curl -fsSL https://codeload.github.com/koide3/gtsam_points/tar.gz/9d32e7dbecf6015560d84b4901d6b0a6f483ec46 \
     | tar -xzf - -C gtsam_points_src --strip-components=1
   ```

   (create the three directories first). Then compare each installed file with its source path:

   ```bash
   cd installed
   src_path() {
     case "$1" in
       gtsam_points/*) echo "../gtsam_points_src/include/$1" ;;
       gtsam/3rdparty/cephes/cephes.h) echo "../gtsam_src/$1" ;;
       gtsam/3rdparty/cephes/*) echo "../gtsam_src/gtsam/3rdparty/cephes/cephes/${1##*/}" ;;
       gtsam/3rdparty/CCOLAMD/ccolamd.h) echo "../gtsam_src/gtsam/3rdparty/CCOLAMD/Include/ccolamd.h" ;;
       *) echo "../gtsam_src/$1" ;;
     esac
   }
   find . -type f | sed 's|^\./||' | sort | while read -r f; do
     cmp -s "$f" "$(src_path "$f")" || echo "differs or missing: $f"
   done
   ```

   Expect exactly four lines, for the generated files `gtsam/config.h`, `gtsam/dllexport.h`,
   `gtsam_unstable/dllexport.h`, and `gtsam_points/config.hpp`.
4. Read `GTSAM_VERSION_STRING` in `gtsam/config.h` and `GTSAM_POINTS_VERSION_STRING` in
   `gtsam_points/config.hpp`.
5. Check the ruled-out candidates and the package history:
   - `gh api repos/borglab/gtsam/compare/2f3e56c...38f55ea` lists only `CMakeLists.txt` and
     `package.xml`.
   - Repeat step 3 with the source of tag `4.3.0`
     (`https://codeload.github.com/borglab/gtsam/tar.gz/refs/tags/4.3.0`) to see the differing headers.
   - `gh api repos/koide3/gtsam_points/compare/v1.2.2...HEAD` lists the later changes under `include/`.
   - `gh api 'repos/koide3/ppa/commits?sha=ppa&path=ubuntu2404/libgtsam-notbb-dev_4.3.0_arm64.deb'`
     (and the same for `_amd64.deb`) lists the commits that added and replaced the deb.

## Updating the pins

**A deb was replaced (the build fails with "SHA256 mismatch").** The PPA has replaced debs under the
same version before, so the message usually means a rebuilt file, not a tampered one. Do not just
copy the new hash:

1. Find the replacing commit with
   `gh api 'repos/koide3/ppa/commits?sha=ppa&path=ubuntu2404/<file>.deb'`, and what triggered the
   rebuild (the `koide3/ppa` commits and the `build_gtsam*.yml` runs of that day).
2. Download the previous file
   (`https://raw.githubusercontent.com/koide3/ppa/<previous commit>/ubuntu2404/<file>.deb`) and the
   new one, and compare `sha256sum` of the new one with its `Packages` entry.
3. Extract both with `dpkg-deb -x` and `dpkg-deb -e` and compare the control files, the file lists
   and every file, and repeat steps 3 and 4 of "How to repeat the check" for the headers. A changed
   header or generated config header, or a library with different exported symbols, needs a decision
   (and a progress entry) before the pin moves.
4. Update the build argument the message names in `docker/Dockerfile` and `docker/Dockerfile.cuda`
   (the shared arguments must be equal; `scripts/tests/test_dockerfile_cuda.py` checks it) and the
   table above, and record the comparison on this page.

**A new package version.** Change `GTSAM_VERSION` or `GTSAM_POINTS_VERSION` (both Dockerfiles and
`scripts/check_deps.sh`), the expected `*_VERSION_STRING` in `scripts/check_deps.sh`, and the SHA256
arguments, after repeating the full check above.

**The base image.** Dependabot (`.github/dependabot.yml`, monthly) opens one grouped pull request that
moves `ros:jazzy-ros-base@sha256:…` in all three Dockerfiles; its CI run rebuilds the image and so
also checks the deb hashes. To update by hand, read the digest of the multi-platform index and replace
it in `docker/Dockerfile`, `docker/Dockerfile.cuda` and `docker/sim/Dockerfile`:

```bash
docker buildx imagetools inspect ros:jazzy-ros-base   # the "Digest:" line of the index
```

Use the index digest, not the digest of a per-platform manifest, so the same line works on amd64 and
arm64.

**A signing key.** Check the new key as in "The koide3 PPA" above and set `KOIDE3_PPA_KEY_FPR` (or
`CUDA_REPO_KEY_FPR`) to its 40 hexadecimal characters without spaces.
