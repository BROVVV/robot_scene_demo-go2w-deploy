#!/usr/bin/env bash
# Build a distinct controller executable; never starts a node or loads the SDK.
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
set -u
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
cd "$root/compat/foxy_nav2"
nice -n 10 colcon --log-base "$runtime/logs/${release}_controller_colcon" build \
  --executor sequential \
  --base-paths "$root/compat/foxy_nav2/src/go2w_nav2_controller_compat" \
  --packages-select go2w_nav2_controller_compat \
  --build-base "$runtime/build/${release}_controller_compat" \
  --install-base "$runtime/install/${release}_controller_compat" \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
