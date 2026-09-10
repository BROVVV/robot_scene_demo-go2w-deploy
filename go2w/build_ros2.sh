#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"

# ROS 2 is built with the system toolchain/Python.  A parent conda or ROS 1
# shell can otherwise inject an incompatible libstdc++/ament path into CMake.
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_ROOT ROS_PACKAGE_PATH
unset ROS_MASTER_URI ROSLISP_PACKAGE_DIRECTORIES AMENT_PREFIX_PATH COLCON_PREFIX_PATH
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL PYTHONHOME PYTHONPATH
unset LD_LIBRARY_PATH CMAKE_PREFIX_PATH
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

# The base distribution is selected explicitly above.  Use only the contents
# of existing overlays; their chained setup files may remember a stale ROS 1
# or older ROS 2 underlay from a previous build shell.
source_overlay_only() {
  local setup_file="$1"
  local local_setup="${setup_file%/setup.bash}/local_setup.bash"
  if [[ -f "$local_setup" ]]; then
    # shellcheck disable=SC1090
    source "$local_setup"
  else
    # shellcheck disable=SC1090
    source "$setup_file"
  fi
}

UNITREE_ROOT="${GO2W_UNITREE_ROOT:-$HOME/unitree_ros2}"
UNITREE_SETUP="${GO2W_UNITREE_SETUP:-}"
if [[ -z "$UNITREE_SETUP" ]]; then
  for candidate in \
    "$PROJECT_ROOT/external/unitree_ros2/cyclonedds_ws/install_system/setup.bash" \
    "$PROJECT_ROOT/external/unitree_ros2/cyclonedds_ws/install/setup.bash" \
    "$UNITREE_ROOT/cyclonedds_ws/install/setup.bash" \
    "/home/brov/robot/unitree_ros2/cyclonedds_ws/install/setup.bash"; do
    if [[ -f "$candidate" ]]; then UNITREE_SETUP="$candidate"; break; fi
  done
fi
if [[ -f "$UNITREE_SETUP" ]]; then
  source_overlay_only "$UNITREE_SETUP"
fi
CONTROL_ROOT="${GO2W_CONTROL_ROOT:-$PROJECT_ROOT/unitree_go2w_control}"
CONTROL_SETUP="${GO2W_CONTROL_SETUP:-$CONTROL_ROOT/ros2_ws/install/setup.bash}"
if [[ ! -f "$CONTROL_SETUP" ]]; then
  for candidate in \
    "$HOME/robot/unitree_go2w_control/ros2_ws/install/setup.bash" \
    "$HOME/unitree_go2w_control/ros2_ws/install/setup.bash" \
    "/home/brov/robot/unitree_go2w_control/ros2_ws/install/setup.bash"; do
    if [[ -f "$candidate" ]]; then CONTROL_SETUP="$candidate"; break; fi
  done
fi
if [[ -f "$CONTROL_SETUP" ]]; then
  # Provides the existing leased MotionCommand Action interface only.
  # Sourcing it does not start the lease holder or any control node.
  source_overlay_only "$CONTROL_SETUP"
fi
unset -f source_overlay_only
set -u

# plain_slam_ros2 must be vendored before building (plan §19.2); never fail
# silently later at launch time.  GO2W_SKIP_PLAIN_SLAM_CHECK=1 is an explicit
# escape hatch for offline machines that already ship the package elsewhere.
if [[ "${GO2W_SKIP_PLAIN_SLAM_CHECK:-0}" != "1" ]]; then
  if [[ ! -d "$PROJECT_ROOT/ros2_ws/src/plain_slam_ros2" ]]; then
    printf '%s\n' \
      'ERROR: ros2_ws/src/plain_slam_ros2 is missing.' >&2
    printf '%s\n' \
      'Run: bash scripts/go2w/vendor_plain_slam_ros2.sh' >&2
    exit 1
  fi
fi

cd "$PROJECT_ROOT/ros2_ws"
colcon build \
  --symlink-install \
  --event-handlers console_cohesion+ \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DWITH_PTCS_USE=OFF

printf 'ROS 2 workspace built with system Python: %s\n' \
  "$PROJECT_ROOT/ros2_ws/install/setup.bash"
