#!/usr/bin/env bash
# Source this file from passive capture scripts. It configures ROS only.

REMOTE_CAPTURE_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REMOTE_CAPTURE_ROOT="$(cd -- "$REMOTE_CAPTURE_SCRIPT_DIR/.." && pwd)"

set +u
source /opt/ros/humble/setup.bash
source /home/brov/unitree_ros2/cyclonedds_ws/install/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="file://$REMOTE_CAPTURE_ROOT/config/cyclonedds_go2w.xml"
