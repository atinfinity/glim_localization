# Pin the PPA package contents and check dependency versions ([#11](https://github.com/atinfinity/glim_localization/issues/11))

- **Change**: on 2026-11-25 the koide3 PPA replaced the files of `libgtsam-points-dev` 1.2.2
  (amd64, arm64) and of its CUDA variants, `libgtsam-points-cuda13.1-dev` 1.2.2 among them, under the same versions
  (PPA commit 6b45536, a rebuild from the same gtsam_points submodule commit 9d32e7d;
  `libgtsam-notbb-dev` 4.3.0 is unchanged since 2026-07-11). A version pin
  alone cannot notice that, so the images now pin the contents:
  - `docker/Dockerfile` and `docker/Dockerfile.cuda` fetch the debs with `apt-get download`,
    compare them with per-architecture SHA256 build arguments (`GTSAM_SHA256_AMD64`/`ARM64`,
    `GTSAM_POINTS_SHA256_AMD64`/`ARM64`, `GTSAM_POINTS_CUDA_SHA256_AMD64`) and install the checked
    files with `dpkg -i`. A mismatch fails the build, names the argument and points to
    [DEPENDENCIES.md](../DEPENDENCIES.md#updating-the-pins).
  - `ros:jazzy-ros-base` is pinned by its index digest in both Dockerfiles and in
    `docker/sim/Dockerfile`. Dependabot (`docker` ecosystem, monthly, one group) proposes digest updates.
  - The apt keyrings hold only the pinned key (`gpg --export <fingerprint>` from a temporary
    `GNUPGHOME`). An unparseable key file now reaches the mismatch message instead of stopping in
    the pipeline.
  - `scripts/check_deps.sh` matches the deb versions exactly and checks the
    `GTSAM_VERSION_STRING` and `GTSAM_POINTS_VERSION_STRING` define lines of the installed headers.
  - The pins, the replacement and the update procedure are in [DEPENDENCIES.md](../DEPENDENCIES.md).
- **Decisions**:
  - Hash the downloaded deb rather than trust the signed `Packages` index alone: the index is
    re-signed on every PPA update, so it does not show that the content changed.
  - `dpkg -i` instead of `apt-get install ./file.deb`: the first build log showed that apt fetched
    the same version from the repository again, so the hashed file was not the one installed.
    Dependencies come from the earlier `apt-get install` of the same `RUN`.
  - The replaced debs were accepted after a comparison (below). If it had shown a header or ABI
    difference, the change would have stopped for review.
- **Verification** (Docker, amd64, 2026-11-27):
  - All five debs were downloaded and hashed with `sha256sum`; each hash matches `Packages`, whose
    SHA256 matches the `InRelease` signed by `A0DAE9CF75CFEF37A509FCD3C2C44E47B79E8F3A`.
  - Old vs. new debs: the CPU debs (amd64 and arm64) are byte-identical file by file, including the
    `.so`; only archive timestamps differ. In the CUDA deb only `libgtsam_points_cuda.so` differs,
    by 28 bytes in `.strtab` (nvcc temporary names); the 770 dynamic symbols are unchanged. The 141
    headers match gtsam_points 9d32e7d.
  - Image rebuild and `scripts/ci.sh`: `PASS: all checks` (colcon
    test 24/24, Phase 0 dependency check, Python tests, replay e2e and robustness). The build log shows
    one `Get:` per deb and `dpkg` unpacking the checked files.
  - Negative builds (tag `glim_localization:negtest`, removed afterwards): a zero
    `GTSAM_POINTS_SHA256_AMD64` or `GTSAM_SHA256_AMD64` fails with the SHA256 mismatch message (expected,
    got, the argument name); a zero fingerprint and the fingerprint with spaces fail with the key
    mismatch message.
  - Scratch copies of `check_deps.sh` with a version prefix (`4.3`, `1.2`) or a wrong version
    string (`4.3`, `1.2.2.1`) fail in the new image with `FAIL:` lines that show the found value. An
    unparseable key file gives `got: none` and exit 1.
  - `scripts/tests/test_dockerfile_cuda.py`: 9 passed. `docker build --check`: no new warnings.
  - The CUDA image was not built (no practical GPU build here); its Dockerfile is covered by the
    Python test and `docker build --check`.
- **Review** (agent): no high findings. Fixed: this entry named the wrong replaced packages; the mismatch
  message now says that a wrong ARG is as likely as a replaced file and names every Dockerfile that declares
  the ARG (`GTSAM_POINTS_CUDA_SHA256_AMD64` is only in `Dockerfile.cuda`); the digest test also covers
  `docker/sim/Dockerfile`. Left as they are: the CUDA image is not built in CI; the PPA source stays enabled
  in the image, so a manual `apt-get upgrade` in a container could move gtsam.
- **Open**:
  - A cached image layer skips the check: a replaced deb is noticed only when the install layer is
    rebuilt (a new base digest, a changed argument or `--no-cache`). A scheduled job that compares
    `Packages` with the pinned hashes would notice it earlier; not done here.
  - arm64 builds were not run locally; the arm64 hashes were checked by download only, and the
    `build-and-test-arm64` CI job will exercise them.
