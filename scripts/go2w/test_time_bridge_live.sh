#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
output_dir="${repo_root}/outputs/go2w_acceptance/time_bridge_live"
mkdir -p "${output_dir}"

source /opt/ros/humble/setup.bash
source "${repo_root}/ros2_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${repo_root}/configs/go2w/cyclonedds_go2w.xml"

bridge_pid=""
cleanup() {
  if [[ -n "${bridge_pid}" ]] && kill -0 "${bridge_pid}" 2>/dev/null; then
    kill -TERM -- "-${bridge_pid}" 2>/dev/null || true
    wait "${bridge_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

setsid ros2 run go2w_sensor_time_bridge time_bridge --ros-args \
  -p "config_file:=${repo_root}/configs/go2w/time_sync.yaml" \
  >"${output_dir}/bridge.log" 2>&1 &
bridge_pid=$!

/usr/bin/python3 "${repo_root}/scripts/go2w/validate_time_bridge_ros.py" \
  --config "${repo_root}/configs/go2w/time_sync.yaml" \
  --output "${output_dir}/result.json"
