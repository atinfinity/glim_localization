#!/bin/bash
set -e

# ROS setup scripts reference unset variables, so nounset must stay off here.
source /opt/ros/jazzy/setup.bash
source /opt/sdf2map/setup.bash

# The container is headless: Gazebo renders sensors offscreen with EGL (llvmpipe without a GPU).
unset DISPLAY WAYLAND_DISPLAY

exec "$@"
