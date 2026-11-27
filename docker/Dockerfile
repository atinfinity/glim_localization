# Headless development image: Ubuntu 24.04 + ROS 2 Jazzy, CPU only.
# The base image is pinned by the digest of its multi-platform index; Dependabot proposes updates
# (.github/dependabot.yml), and docs/DEPENDENCIES.md ("Updating the pins") has the manual way.
FROM ros:jazzy-ros-base@sha256:5147d3ae250ce754afd399f6bfec15b33dd4e473ca357e30432a3182031bfad6

ARG GLIM_COMMIT=1a197c999e4865f5cab7c9cd25474128413ebbca
ARG GTSAM_VERSION=4.3.0
ARG GTSAM_POINTS_VERSION=1.2.2
ARG JOBS=4
# Primary key fingerprint of the koide3 PPA (docs/DEPENDENCIES.md). The build fails if KEY.gpg differs.
ARG KOIDE3_PPA_KEY_FPR=A0DAE9CF75CFEF37A509FCD3C2C44E47B79E8F3A
# SHA256 of the PPA debs per architecture (docs/DEPENDENCIES.md). The PPA has replaced debs under the
# same version, so the version alone does not pin the contents; the build fails on any other file.
ARG GTSAM_SHA256_AMD64=257cbe2edf232cbce64f3bac45060c6e56f1b17ead939b415ae7247557cb4e66
ARG GTSAM_SHA256_ARM64=211bc1693478a3afbd9cc70bbc386a9ec4c9d06b9f87beb9c2192f1bbc7303cb
ARG GTSAM_POINTS_SHA256_AMD64=bd947013c199789e04b1f2a234159a107213b9decc04a892877de05b1898f8b4
ARG GTSAM_POINTS_SHA256_ARM64=246d03b234bf7904cc755e62ed506e5097d4845ce38241e9bdf65ad039e1b34e

SHELL ["/bin/bash", "-euo", "pipefail", "-c"]

# The PPA debs are fetched with apt-get download (which checks them against the signed index),
# compared with the pinned SHA256 and installed with dpkg -i: apt-get install on a local file
# fetches the same version from the repository again instead of using the checked file. Their
# dependencies (Boost, METIS, the toolchain runtime) come from the first apt-get install of this RUN.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
         build-essential cmake git ca-certificates curl gpg \
         python3-colcon-common-extensions \
         libeigen3-dev libboost-all-dev libmetis-dev libfmt-dev libspdlog-dev libgtest-dev \
         nlohmann-json3-dev \
    && install -d -m 0755 /etc/apt/keyrings \
    && curl -fsSL --compressed https://koide3.github.io/ppa/ubuntu2404/KEY.gpg -o /tmp/koide3_ppa.asc \
    && export GNUPGHOME="$(mktemp -d)" \
    && { fprs="$(gpg --batch --show-keys --with-colons /tmp/koide3_ppa.asc \
         | awk -F: '$1 == "pub" { p = 1; next } $1 == "fpr" && p { print $10; p = 0 }')" || fprs=""; } \
    && if [ "${fprs}" != "${KOIDE3_PPA_KEY_FPR}" ]; then \
         echo "koide3 PPA key mismatch: expected ${KOIDE3_PPA_KEY_FPR}, got: ${fprs:-none}" >&2; exit 1; \
       fi \
    && gpg --batch --quiet --import /tmp/koide3_ppa.asc \
    && gpg --batch --export "${KOIDE3_PPA_KEY_FPR}" > /etc/apt/keyrings/koide3_ppa.gpg \
    && test -s /etc/apt/keyrings/koide3_ppa.gpg \
    && rm -rf /tmp/koide3_ppa.asc "${GNUPGHOME}" && unset GNUPGHOME \
    && echo "deb [signed-by=/etc/apt/keyrings/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2404 ./" \
         > /etc/apt/sources.list.d/koide3_ppa.list \
    && apt-get update \
    && arch="$(dpkg --print-architecture)" \
    && case "${arch}" in \
         amd64 | arm64) ;; \
         *) echo "no pinned SHA256 of the PPA debs for architecture ${arch} (docs/DEPENDENCIES.md)" >&2; exit 1 ;; \
       esac \
    && debs="$(mktemp -d)" && chown _apt "${debs}" \
    && check_sha256() { \
         local deb="${debs}/$1_${arch}.deb" expected="${!2:-}" actual; \
         actual="$(sha256sum "${deb}" | cut -d ' ' -f 1)" || actual=""; \
         if [ -z "${expected}" ] || [ "${actual}" != "${expected}" ]; then \
           echo "SHA256 mismatch for ${1%%_*} on ${arch}: expected ${expected:-none} (ARG $2), got ${actual:-none}." >&2; \
           echo "The PPA replaced this file under the same version; re-verify per docs/DEPENDENCIES.md, then update ARG $2 in docker/Dockerfile and docker/Dockerfile.cuda." >&2; \
           exit 1; \
         fi; \
       } \
    && (cd "${debs}" && apt-get download "libgtsam-notbb-dev=${GTSAM_VERSION}" "libgtsam-points-dev=${GTSAM_POINTS_VERSION}") \
    && check_sha256 "libgtsam-notbb-dev_${GTSAM_VERSION}" "GTSAM_SHA256_${arch^^}" \
    && check_sha256 "libgtsam-points-dev_${GTSAM_POINTS_VERSION}" "GTSAM_POINTS_SHA256_${arch^^}" \
    && dpkg -i "${debs}/libgtsam-notbb-dev_${GTSAM_VERSION}_${arch}.deb" \
         "${debs}/libgtsam-points-dev_${GTSAM_POINTS_VERSION}_${arch}.deb" \
    && rm -rf "${debs}" \
    && ldconfig \
    && rm -rf /var/lib/apt/lists/*

# Patches are applied on top of the pinned commit; GLIM itself stays unmodified upstream.
COPY third_party/patches/glim/ /opt/glim_patches/

RUN mkdir -p /opt/glim_ws/src \
    && git clone https://github.com/koide3/glim.git /opt/glim_ws/src/glim \
    && cd /opt/glim_ws/src/glim \
    && git checkout --detach "${GLIM_COMMIT}" \
    && test "$(git rev-parse HEAD)" = "${GLIM_COMMIT}" \
    && shopt -s nullglob \
    && for p in /opt/glim_patches/*.patch; do git apply "$p" || exit 1; done \
    && cd /opt/glim_ws \
    && set +u && source /opt/ros/jazzy/setup.bash && set -u \
    && MAKEFLAGS="-j${JOBS}" colcon build \
         --merge-install --install-base /opt/glim --parallel-workers 1 \
         --event-handlers console_direct+ \
         --cmake-args -DCMAKE_BUILD_TYPE=Release \
           -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF \
           -DBUILD_WITH_OPENCV=OFF -DBUILD_WITH_MARCH_NATIVE=OFF \
    && rm -rf /opt/glim_ws/build /opt/glim_ws/log

# glim_ros2 (R1 host and rosbag tools), built against the GLIM above. Placed before the Python
# layer and after GLIM so that neither rebuilds the other. cv_bridge is dropped from package.xml
# by a patch (all uses are behind BUILD_WITH_CV_BRIDGE, which stays OFF; see docs/PROGRESS.md).
# rviz_viewer is built unconditionally upstream, so it is deleted here; check_deps.sh stays strict.
ARG GLIM_ROS2_COMMIT=4d4ec524ccf1b02aa09b0af2af767ecc54343798
COPY third_party/patches/glim_ros2/ /opt/glim_ros2_patches/

RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-jazzy-image-transport \
    && rm -rf /var/lib/apt/lists/* \
    && git clone https://github.com/koide3/glim_ros2.git /opt/glim_ws/src/glim_ros2 \
    && cd /opt/glim_ws/src/glim_ros2 \
    && git checkout --detach "${GLIM_ROS2_COMMIT}" \
    && test "$(git rev-parse HEAD)" = "${GLIM_ROS2_COMMIT}" \
    && shopt -s nullglob \
    && patches=(/opt/glim_ros2_patches/*.patch) \
    && test "${#patches[@]}" -gt 0 \
    && for p in "${patches[@]}"; do git apply "$p" || exit 1; done \
    && cd /opt/glim_ws \
    && set +u && source /opt/ros/jazzy/setup.bash && source /opt/glim/setup.bash && set -u \
    && MAKEFLAGS="-j${JOBS}" colcon build --packages-select glim_ros \
         --merge-install --install-base /opt/glim --parallel-workers 1 \
         --event-handlers console_direct+ \
         --cmake-args -DCMAKE_BUILD_TYPE=Release \
           -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_CV_BRIDGE=OFF \
    && rm -f /opt/glim/lib/librviz_viewer.so \
    && sed -i 's/;rviz_viewer//' /opt/glim/share/glim_ros/cmake/ament_cmake_export_libraries-extras.cmake \
    && ! grep -rIl rviz_viewer /opt/glim/share/glim_ros \
    && rm -rf /opt/glim_ws/build /opt/glim_ws/log

# Python tooling for bag conversion and evaluation. Kept out of the PATH so that ROS and colcon
# keep using the system Python (Ubuntu 24.04 blocks system-wide pip installs, PEP 668).
# Placed after the GLIM build so that changing it does not rebuild GLIM.
# fonts-noto-core: Noto Sans (OFL 1.1) for the text of scripts/render_demo.py. The matplotlib font
# cache is built after the fonts are installed, so it lists them from the first use.
COPY scripts/requirements.txt /tmp/requirements.txt
RUN apt-get update \
    && apt-get install -y --no-install-recommends fonts-noto-core python3-venv \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m venv /opt/glim_localization_venv \
    && /opt/glim_localization_venv/bin/pip install --no-cache-dir -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt \
    && /opt/glim_localization_venv/bin/python -c "from matplotlib import font_manager as fm; fm.findfont(fm.FontProperties(family='Noto Sans'), fallback_to_default=False)"
ENV GLIM_LOC_PYTHON=/opt/glim_localization_venv/bin/python

# ccache, used only when CCACHE_DIR is set (CI mounts one; scripts/build_core.sh, build_colcon.sh).
RUN apt-get update \
    && apt-get install -y --no-install-recommends ccache \
    && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV CMAKE_BUILD_PARALLEL_LEVEL=${JOBS}
WORKDIR /ws/src/glim_localization
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# Lets scripts/ci_local.sh check that the image was built from this tree: every repository file the
# image bakes in is copied again, at the same path, under /etc/glim_localization/inputs/. Keep this
# list equal to image_inputs in scripts/image_inputs_hash.sh (a test checks it).
COPY docker/Dockerfile /etc/glim_localization/inputs/docker/Dockerfile
COPY docker/entrypoint.sh /etc/glim_localization/inputs/docker/entrypoint.sh
COPY scripts/requirements.txt /etc/glim_localization/inputs/scripts/requirements.txt
COPY third_party/patches/glim/ /etc/glim_localization/inputs/third_party/patches/glim/
COPY third_party/patches/glim_ros2/ /etc/glim_localization/inputs/third_party/patches/glim_ros2/
