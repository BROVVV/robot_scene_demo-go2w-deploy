#!/usr/bin/env bash
set -eo pipefail

# Read-only stationary acceptance. This file contains no motion, lease, cmd_vel,
# posture, joint, navigation, or robot-control command.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
evidence_root="${project_root}/outputs/go2w_acceptance/lio_stationary"
log_root="${evidence_root}/logs"
mkdir -p "${evidence_root}" "${log_root}"

source /opt/ros/humble/setup.bash
source "${GO2W_UNITREE_ROOT:-$HOME/unitree_ros2}/cyclonedds_ws/install/setup.bash"
source "${project_root}/ros2_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${project_root}/configs/go2w/cyclonedds_go2w.xml"

if ros2 topic list | grep -Eq '^/lio/(odom|path|cloud_registered)$'; then
  echo "Refusing acceptance: a project LIO output already has an active graph entry." >&2
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
setsid ros2 launch go2w_lio_bringup lio.launch.py \
  "lio_config:=${project_root}/configs/go2w/lio.yaml" \
  "reference_config:=${project_root}/configs/go2w/official_reference.yaml" \
  "time_config:=${project_root}/configs/go2w/time_sync.yaml" \
  >"${log_root}/lio.log" 2>&1 &
lio_pid=$!

cleanup() {
  kill -TERM -- "-${lio_pid}" "-${time_pid}" "-${description_pid}" \
    2>/dev/null || true
  wait "${lio_pid}" "${time_pid}" "${description_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

stationary_rc=0
/usr/bin/python3 "${script_dir}/validate_lio_stationary_ros.py" \
  --reference "${project_root}/configs/go2w/official_reference.yaml" \
  --output "${evidence_root}/result.json" \
  --minimum-odom 300 \
  --collection-seconds 25 \
  --startup-timeout-seconds 20 || stationary_rc=$?

# Stop only this acceptance run's timestamp-preserving input bridge. RKO-LIO and
# its watchdog stay alive so the adapter's 300 ms fail-closed behavior is tested.
kill -TERM -- "-${time_pid}"
wait "${time_pid}" 2>/dev/null || true
stale_rc=0
/usr/bin/python3 "${script_dir}/validate_lio_stale_timeout_ros.py" \
  --output "${evidence_root}/stale_timeout.json" \
  --configured-timeout-seconds 0.3 \
  --maximum-observed-seconds 0.7 || stale_rc=$?

if (( stationary_rc != 0 || stale_rc != 0 )); then
  echo "FAIL: stationary_rc=${stationary_rc}, stale_rc=${stale_rc}" >&2
  exit 2
fi

echo "PASS: live stationary read-only RKO-LIO and stale timeout"
