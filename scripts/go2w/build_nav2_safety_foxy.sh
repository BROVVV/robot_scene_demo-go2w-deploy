#!/usr/bin/env bash
# Build the isolated compatibility overlay; never starts any ROS node.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime="${GO2W_RUNTIME_ROOT:-/home/unitree/robotscene_runtime}"
[[ "$(uname -m)" == aarch64 && -f /opt/ros/foxy/setup.bash ]] || {
  echo 'This entry requires native aarch64 ROS 2 Foxy.' >&2
  exit 2
}
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH
export PYTHONNOUSERSITE=1
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u
source /opt/ros/foxy/setup.bash
set -u
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
source_root="$root/compat/foxy_nav2/src"
cd "$root/compat/foxy_nav2"
colcon --log-base "$runtime/logs/nav2_compat_colcon" build --symlink-install \
  --executor sequential \
  --base-paths "$source_root/go2w_nav2_compat" "$source_root/nav2_velocity_smoother" "$source_root/nav2_collision_monitor" \
  --packages-select go2w_nav2_compat nav2_velocity_smoother nav2_collision_monitor \
  --build-base "$runtime/build/20261106_nav2_compat" \
  --install-base "$runtime/install/20261106_nav2_compat" \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
