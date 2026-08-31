#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
session_id="acceptance_$(date +%s)"
output_dir="${repo_root}/outputs/go2w_acceptance/live_frame_bridge"
spool_root="${output_dir}/spool/${session_id}"
mkdir -p "${output_dir}" "${spool_root}"

source /opt/ros/humble/setup.bash
source "${GO2W_UNITREE_ROOT:-$HOME/unitree_ros2}/cyclonedds_ws/install/setup.bash"
source "${repo_root}/ros2_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${repo_root}/configs/go2w/cyclonedds_go2w.xml"

camera_pid=""
bridge_pid=""
cleanup() {
  for process_id in "${bridge_pid}" "${camera_pid}"; do
    if [[ -n "${process_id}" ]] && kill -0 "${process_id}" 2>/dev/null; then
      kill -TERM -- "-${process_id}" 2>/dev/null || true
      wait "${process_id}" 2>/dev/null || true
      for _ in {1..25}; do
        kill -0 -- "-${process_id}" 2>/dev/null || break
        sleep 0.2
      done
    fi
  done
}
trap cleanup EXIT INT TERM

setsid ros2 run go2w_camera_bridge camera_bridge --ros-args \
  -p source:=rpc \
  -p interface:=enp6s0 \
  -p "calibration_file:=${repo_root}/configs/go2w/camera_intrinsics.yaml" \
  >"${output_dir}/camera.log" 2>&1 &
camera_pid=$!

setsid ros2 run robot_scene_live_bridge live_bridge --ros-args \
  -p "spool_root:=${spool_root}" \
  -p "session_id:=${session_id}" \
  -p sensor_timeout_seconds:=0.3 \
  >"${output_dir}/bridge.log" 2>&1 &
bridge_pid=$!

deadline=$((SECONDS + 30))
while [[ ! -f "${spool_root}/latest/READY" ]]; do
  if (( SECONDS >= deadline )); then
    echo "timed out waiting for complete live frame bundle" >&2
    exit 1
  fi
  sleep 0.2
done

cd "${repo_root}"
/home/brov/miniconda3/envs/go2_robot_scene_demo/bin/python \
  scripts/go2w/validate_live_frame_bundle.py \
  --spool-root "${spool_root}" \
  --output "${output_dir}/result.json"
