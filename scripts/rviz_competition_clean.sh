#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source /home/rcj/Documents/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

config="/home/rcj/Documents/ros2_ws/src/rcj_localization/config/competition_localization_clean.rviz"

exec env LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}" rviz2 -d "$config"
