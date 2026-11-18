#!/usr/bin/env bash
# Execute deterministic policy tests only; no ROS context or robot connection.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime="${GO2W_RUNTIME_ROOT:-/home/unitree/robotscene_runtime}"
release="$(basename -- "$root")"
[[ "$(uname -m)" == aarch64 && -f /opt/ros/foxy/setup.bash && "$release" =~ ^[0-9]{8}_[0-9]{2}$ ]] || exit 2
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH CYCLONEDDS_URI
export PYTHONNOUSERSITE=1
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u
source /opt/ros/foxy/setup.bash
set -u
test_dir="$runtime/build/${release}_controller_compat/go2w_nav2_controller_compat"
"$test_dir/test_controller_patience"
exec "$test_dir/test_controller_semantics"
