# Simulation image for the manual Gazebo evaluation (scripts/sim_eval.sh, docs/sim_eval.md).
# Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic (ros_gz) + sdf2map, headless. Rendering of gpu_lidar
# uses Mesa's software rasterizer (llvmpipe) through EGL (`gz sim --headless-rendering`), so no GPU is
# needed. Not used by CI and independent of glim_localization:dev: localization and evaluation run in
# the development image.
FROM ros:jazzy-ros-base

ARG SDF2MAP_COMMIT=66feb61af4c0fca37becb7e2ba4dcdd5731f58f1
ARG JOBS=4

SHELL ["/bin/bash", "-euo", "pipefail", "-c"]

# ros-jazzy-ros-gz brings Gazebo Harmonic through the gz_*_vendor packages (with headers, for the
# scripted motion plugin built by scripts/sim_eval.sh). Mesa provides llvmpipe for EGL.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
         build-essential cmake git ca-certificates \
         python3-colcon-common-extensions \
         ros-jazzy-ros-gz \
         ros-jazzy-rosbag2-storage-mcap \
         ros-jazzy-pcl-conversions libpcl-dev \
         libegl1 libegl-mesa0 libgl1-mesa-dri libglx-mesa0 libgles2 mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# sdf2map (Apache-2.0), pinned. Tests are not built.
RUN mkdir -p /opt/sdf2map_ws/src \
    && git clone https://github.com/atinfinity/sdf2map.git /opt/sdf2map_ws/src/sdf2map \
    && cd /opt/sdf2map_ws/src/sdf2map \
    && git checkout --detach "${SDF2MAP_COMMIT}" \
    && test "$(git rev-parse HEAD)" = "${SDF2MAP_COMMIT}" \
    && cd /opt/sdf2map_ws \
    && set +u && source /opt/ros/jazzy/setup.bash && set -u \
    && MAKEFLAGS="-j${JOBS}" colcon build \
         --merge-install --install-base /opt/sdf2map --parallel-workers 1 \
         --event-handlers console_direct+ \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf /opt/sdf2map_ws/build /opt/sdf2map_ws/log

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /ws/src/glim_localization
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
