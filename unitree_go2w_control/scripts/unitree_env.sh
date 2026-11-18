#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${UNITREE_PROJECT_ROOT:-$(cd -- "$SCRIPT_DIR/.." && pwd)}"
UNITREE_ROOT="${GO2W_UNITREE_ROOT:-$HOME/unitree_ros2}"
# shellcheck source=/dev/null
source "$PROJECT_ROOT/config/runtime.env"
[[ -f /opt/ros/humble/setup.bash ]] || { printf 'ROS 2 Humble is not installed\n' >&2; return 1 2>/dev/null || exit 1; }
# shellcheck source=/dev/null
set +u
source /opt/ros/humble/setup.bash
set -u
[[ -f "$UNITREE_ROOT/cyclonedds_ws/install/setup.bash" ]] || { printf 'Unitree message workspace is not built\n' >&2; return 1 2>/dev/null || exit 1; }
# shellcheck source=/dev/null
set +u
source "$UNITREE_ROOT/cyclonedds_ws/install/setup.bash"
[[ -f "$UNITREE_ROOT/example/install/setup.bash" ]] && source "$UNITREE_ROOT/example/install/setup.bash"
set -u

export ROS_DISTRO=humble
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PROJECT_ROOT/config/cyclonedds_go2w.xml"
export PYTHONPATH="$PROJECT_ROOT/vendor/unitree_sdk2_python${PYTHONPATH:+:$PYTHONPATH}"
