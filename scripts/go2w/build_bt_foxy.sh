#!/usr/bin/env bash
# Native build of the isolated BT action plugins; never starts a ROS node.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime="${GO2W_RUNTIME_ROOT:-/home/unitree/robotscene_runtime}"
[[ "$(uname -m)" == aarch64 && -f /opt/ros/foxy/setup.bash ]] || exit 2
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH CYCLONEDDS_URI
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u
source /opt/ros/foxy/setup.bash
set -u
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
cd "$root/compat/foxy_nav2"
nice -n 10 colcon --log-base "$runtime/logs/bt_compat_colcon" build \
  --executor sequential \
  --base-paths "$root/compat/foxy_nav2/src/go2w_nav2_bt_compat" \
  --packages-select go2w_nav2_bt_compat \
  --build-base "$runtime/build/20261106_bt_compat" \
  --install-base "$runtime/install/20261106_bt_compat" \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
