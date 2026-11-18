#!/usr/bin/env bash
# Build only the read-only native state transport; no robot processes start.
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
[[ "$(uname -m)" == aarch64 && -f /opt/ros/foxy/setup.bash ]] || {
  echo 'This build requires the native Jetson Foxy environment.' >&2
  exit 2
}
set -a
source "$root/configs/go2w/jetson_foxy.env"
set +a
source "$root/scripts/go2w/setup_environment.sh"
export MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1
release="${root##*/}"
runtime="${GO2W_RUNTIME_ROOT:?}"
exec nice -n 10 colcon --log-base "$runtime/logs/${release}_state_transport_colcon" build \
  --base-paths "$root/ros2_ws/src/go2w_state_transport" \
  --build-base "$runtime/build/${release}_state_transport" \
  --install-base "$runtime/install/${release}_state_transport" \
  --executor sequential --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
