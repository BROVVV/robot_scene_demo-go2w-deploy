#!/usr/bin/env bash
# Build versioned navigation, safety and LiDAR processing without starting nodes.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime="${GO2W_RUNTIME_ROOT:-/home/unitree/robotscene_runtime}"
release="$(basename -- "$root")"
[[ "$(uname -m)" == aarch64 && -f /opt/ros/foxy/setup.bash && "$release" =~ ^[0-9]{8}_[0-9]{2}$ ]] || exit 2
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH CYCLONEDDS_URI
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u
source /opt/ros/foxy/setup.bash
source "$runtime/install/ros2_ws/local_setup.bash"
source "$runtime/install/20261104_01/local_setup.bash"
set -u
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
cd "$root/ros2_ws"
nice -n 10 colcon --log-base "$runtime/logs/${release}_nav_colcon" build \
  --executor sequential --base-paths "$root/ros2_ws/src/robot_scene_nav_bringup" \
    "$root/ros2_ws/src/go2w_cmd_vel_bridge" "$root/ros2_ws/src/go2w_control_arbiter" \
    "$root/ros2_ws/src/go2w_lidar_preprocessor" \
    "$root/ros2_ws/src/go2w_lio_bringup" \
  --packages-select robot_scene_nav_bringup go2w_cmd_vel_bridge go2w_control_arbiter \
    go2w_lidar_preprocessor go2w_lio_bringup \
  --build-base "$runtime/build/${release}_nav" --install-base "$runtime/install/$release" \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
