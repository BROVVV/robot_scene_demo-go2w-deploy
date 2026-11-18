#!/usr/bin/env bash
# Native sidecar build; configured install must be new, old releases untouched.
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
[[ "$(uname -m)" == aarch64 && -f /opt/ros/foxy/setup.bash ]] || {
  echo 'Native Jetson Foxy build required.' >&2; exit 2;
}
set -a
source "$root/configs/go2w/jetson_foxy.env"
set +a
runtime="${GO2W_RUNTIME_ROOT:?}"
install="$(dirname -- "${GO2W_LIDAR_HOTPATH_SETUP:?}")"
[[ "$install" == "$runtime"/install/* && ! -e "$install" && ! -L "$install" ]] || {
  echo 'Configured overlay already exists or is outside runtime/install; refusing overwrite.' >&2; exit 2;
}
# Build against the pinned base, without sourcing the not-yet-created overlay.
unset GO2W_LIDAR_HOTPATH_SETUP
source "$root/scripts/go2w/setup_environment.sh"
export MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1
release="${root##*/}"
exec nice -n 10 colcon --log-base "$runtime/logs/${release}_navigation_guard_colcon" build \
  --base-paths "$root/ros2_ws/src/go2w_lidar_preprocessor" \
    "$root/ros2_ws/src/go2w_cmd_vel_bridge" "$root/ros2_ws/src/robot_scene_nav_bringup" \
    "$root/ros2_ws/src/go2w_sensor_time_bridge" \
  --build-base "$runtime/build/${release}_navigation_guard" --install-base "$install" \
  --executor sequential --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPYTHON_EXECUTABLE=/usr/bin/python3
