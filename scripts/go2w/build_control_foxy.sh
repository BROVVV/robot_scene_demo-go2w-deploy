#!/usr/bin/env bash
# Build the candidate motion controller natively; starts no node or SDK lease.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime="${GO2W_RUNTIME_ROOT:-/home/unitree/robotscene_runtime}"
unitree_setup="${GO2W_CONTROL_BUILD_UNDERLAY:-/home/unitree/robotscene/external/unitree_ros2/cyclonedds_ws/install/local_setup.bash}"
[[ "$(uname -m)" == aarch64 && -f /opt/ros/foxy/setup.bash ]] || {
  echo 'This build entry requires native aarch64 ROS 2 Foxy.' >&2; exit 2;
}
[[ -f "$unitree_setup" ]] || { echo 'Native Unitree message underlay is missing.' >&2; exit 2; }
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH
export PYTHONNOUSERSITE=1
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u
source /opt/ros/foxy/setup.bash
source "$unitree_setup"
set -u
[[ "$ROS_DISTRO" == foxy ]] || { echo 'Underlay changed the ROS distro; refusing build.' >&2; exit 2; }
export GO2W_UNITREE_ROOT="$root/external/unitree_ros2"
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
mkdir -p "$runtime/build/20261104_control" "$runtime/install/20261104_control" "$runtime/logs/control_colcon"
cd "$root/unitree_go2w_control/ros2_ws"
colcon --log-base "$runtime/logs/control_colcon" build --symlink-install --executor sequential \
  --base-paths "$root/unitree_go2w_control/ros2_ws/src" \
  --packages-select go2w_motion_interfaces go2w_motion_control \
  --build-base "$runtime/build/20261104_control" \
  --install-base "$runtime/install/20261104_control" \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
