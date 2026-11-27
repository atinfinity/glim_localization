#!/bin/bash
set -e

# ROS setup scripts reference unset variables, so nounset must stay off here.
source /opt/ros/jazzy/setup.bash
source /opt/baseline_ws/install/setup.bash

exec "$@"
