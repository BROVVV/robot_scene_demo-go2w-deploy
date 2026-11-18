#!/usr/bin/env bash
# Native Foxy build for the Jetson sidecar release. Builds only; starts no nodes.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime="${GO2W_RUNTIME_ROOT:-/home/unitree/robotscene_runtime}"
export PYTHONNOUSERSITE=1
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME
set +u
source /opt/ros/foxy/setup.bash
unitree_overlay="${GO2W_UNITREE_UNDERLAY:-/home/unitree/robotscene/unitree_go2w_control/ros2_ws/install/local_setup.bash}"
if [[ -f "$unitree_overlay" ]]; then
  source "$unitree_overlay"
fi
set -u
mkdir -p "$runtime/build/ros2_ws" "$runtime/install/ros2_ws" "$runtime/logs/colcon"
cd "$root/ros2_ws"
colcon --log-base "$runtime/logs/colcon" build --symlink-install --executor sequential \
  --base-paths "$root/ros2_ws/src" \
  --build-base "$runtime/build/ros2_ws" \
  --install-base "$runtime/install/ros2_ws" \
  "$@" \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
