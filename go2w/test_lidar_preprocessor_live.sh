#!/usr/bin/env bash
set -eo pipefail

# Read-only acceptance: no motion package, lease holder, cmd_vel, or Nav2 node.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
evidence_root="${project_root}/outputs/go2w_acceptance/lidar_preprocessor_live"
log_root="${evidence_root}/logs"
mkdir -p "${evidence_root}" "${log_root}"

source /opt/ros/humble/setup.bash
source "${GO2W_UNITREE_ROOT:-$HOME/unitree_ros2}/cyclonedds_ws/install/setup.bash"
source "${project_root}/ros2_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${project_root}/configs/go2w/cyclonedds_go2w.xml"

if ros2 topic list | grep -qx '/go2w/sensors/cloud'; then
  echo "Refusing acceptance: /go2w/sensors/cloud already has an active graph entry." >&2
  exit 2
fi

setsid ros2 launch go2w_description official_sensor_frames.launch.py \
  "reference_file:=${project_root}/configs/go2w/official_reference.yaml" \
  >"${log_root}/description.log" 2>&1 &
description_pid=$!
setsid ros2 run go2w_sensor_time_bridge time_bridge --ros-args \
  -p "config_file:=${project_root}/configs/go2w/time_sync.yaml" \
  >"${log_root}/time_bridge.log" 2>&1 &
time_pid=$!
setsid ros2 run go2w_lidar_preprocessor lidar_preprocessor --ros-args \
  -p "config_file:=${project_root}/configs/go2w/lidar_preprocess.yaml" \
  -p "geometry_file:=${project_root}/configs/go2w/official_reference.yaml" \
  >"${log_root}/preprocessor.log" 2>&1 &
preprocessor_pid=$!

cleanup() {
  kill -TERM -- "-${preprocessor_pid}" "-${time_pid}" "-${description_pid}" \
    2>/dev/null || true
  wait "${preprocessor_pid}" "${time_pid}" "${description_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

/usr/bin/python3 "${script_dir}/validate_lidar_preprocessor_ros.py" \
  --output "${evidence_root}/result.json" \
  --minimum-samples 20 \
  --timeout-seconds 15

# Stop only this script's process group and prove the 300 ms watchdog closes.
kill -TERM -- "-${time_pid}"
wait "${time_pid}" 2>/dev/null || true
/usr/bin/python3 "${script_dir}/validate_lidar_freshness_timeout_ros.py" \
  --output "${evidence_root}/freshness_timeout.json" \
  --configured-timeout-seconds 0.3 \
  --maximum-observed-seconds 0.6

echo "PASS: live read-only LiDAR preprocessing and freshness timeout"
