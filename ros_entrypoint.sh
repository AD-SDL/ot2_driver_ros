#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "$ROS_WS/install/setup.bash" --

# Check robot_ip environment variable is set. Alert and exit otherwise
if [[ -z $robot_ip ]]; then
    echo "[ERROR] robot_ip environment variable is not set. Must be set to connect to OT2."
    exit 1
fi

exec "$@"