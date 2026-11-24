#!/bin/bash
set -e

# ROS setup scripts reference unset variables, so nounset must stay off here.
source /opt/ros/jazzy/setup.bash
source /opt/glim/setup.bash

# The container is headless.
unset DISPLAY WAYLAND_DISPLAY

exec "$@"
