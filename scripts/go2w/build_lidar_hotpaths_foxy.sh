#!/usr/bin/env bash
# Native sidecar build of changed read-only packages. Starts no robot nodes.
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
[[ "$(uname -m)" == aarch64 && -f /opt/ros/foxy/setup.bash ]] || {
  echo 'Native Jetson Foxy build required.' >&2; exit 2;
}
set -a
source "$root/configs/go2w/jetson_foxy.env"
set +a
# The first build cannot source the overlay it is creating. Retain the
# explicitly verified base workspace and all existing vendor/Nav2 underlays.
unset GO2W_LIDAR_HOTPATH_SETUP
source "$root/scripts/go2w/setup_environment.sh"
export MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1
runtime="${GO2W_RUNTIME_ROOT:?}"
release="${root##*/}"
exec nice -n 10 colcon --log-base "$runtime/logs/${release}_lidar_hotpaths_colcon" build \
  --base-paths "$root/ros2_ws/src/go2w_lidar_preprocessor" "$root/ros2_ws/src/robot_scene_nav_bringup" \
  --build-base "$runtime/build/${release}_lidar_hotpaths" \
  --install-base "$runtime/install/${release}_lidar_hotpaths" \
  --executor sequential --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPYTHON_EXECUTABLE=/usr/bin/python3
