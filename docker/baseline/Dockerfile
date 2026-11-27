# Baseline image for the benchmark of issue 80 (docs/benchmark.md): rsasaki0109/lidar_localization_ros2 (NDT)
# with ndt_omp_ros2, cloned at pinned commits and built in Release. Not vendored and not part of CI;
# scripts/sim_benchmark.sh builds it as glim_localization:baseline-lidarloc (docker/sim/compose.yaml, service
# baseline). Both packages are BSD-2-Clause; nothing of them is shipped by this repository.
# Same pinned base as docker/Dockerfile and docker/sim/Dockerfile.
FROM ros:jazzy-ros-base@sha256:5147d3ae250ce754afd399f6bfec15b33dd4e473ca357e30432a3182031bfad6

# lidar_localization_ros2 v1.12.0, and the ndt_omp_ros2 humble head that its dependencies.repos tracks
# (ndt_omp_ros2 has no jazzy branch; upstream CI builds humble on jazzy too).
ARG LIDARLOC_COMMIT=6f6cf9a348db2f2a9cf37acffd5e90199ba46248
ARG NDT_OMP_COMMIT=17ce1e19831447a1f5bc5a68a4522fa8087f8933
ARG JOBS=4

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      libomp-dev \
      procps \
      python3-colcon-common-extensions \
      python3-rosdep \
      ros-jazzy-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

# Clone by commit, not by branch or tag, so that the image does not change when upstream moves.
WORKDIR /opt/baseline_ws/src
RUN fetch() { mkdir "$1" && git -C "$1" init -q && git -C "$1" remote add origin "$2" \
        && git -C "$1" fetch -q --depth 1 origin "$3" && git -C "$1" checkout -q FETCH_HEAD \
        && test "$(git -C "$1" rev-parse HEAD)" = "$3"; } \
    && fetch lidar_localization_ros2 https://github.com/rsasaki0109/lidar_localization_ros2.git "${LIDARLOC_COMMIT}" \
    && fetch ndt_omp_ros2 https://github.com/rsasaki0109/ndt_omp_ros2.git "${NDT_OMP_COMMIT}"

RUN apt-get update \
    && (rosdep init 2>/dev/null || true) \
    && rosdep update --rosdistro jazzy \
    && rosdep install -r -y --rosdistro jazzy --from-paths /opt/baseline_ws/src --ignore-src \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/baseline_ws
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --parallel-workers "${JOBS}" --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf build log

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
