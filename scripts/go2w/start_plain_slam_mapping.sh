#!/usr/bin/env bash
# Start the Go2-W plain_slam PandarXT-16 mapping-assist pipeline.
#
# Order (plan §14.1):
#   1. source ROS 2 + workspace
#   2. generate runtime plain_slam config (idempotent)
#   3. check /hesai/pandarxt16/points_raw; start the existing Hesai driver
#      only when no publisher exists (never a second driver)
#   4. wait for the Pandar cloud (timeout -> explicit mapping-launcher exit)
#   5. check /utlidar/imu
#   6. launch plain_slam_go2w.launch.py (LIO + SLAM + 4 bridge nodes)
#   7. wait for /go2w/slam/ready and print the topic/health summary
#
# Safety: this script starts mapping nodes only.  It never starts motion,
# never touches /go2w/odom/fused and never raises any authorization flag.
#
# Options:
#   --rviz            also open RViz with the pslam_odom fixed frame
#   --no-start-hesai  never start the Hesai driver (assume it runs elsewhere)
#   --record          record a debug rosbag of the mapping topics
#   --debug           verbose output + debug logging
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
workspace="${project_root}/ros2_ws"
runtime_dir="${project_root}/runtime/go2w/plain_slam"
pid_root="${project_root}/runtime/go2w/pids"
log_root="${project_root}/runtime/go2w/sessions"
mkdir -p "${runtime_dir}" "${pid_root}" "${log_root}"

use_rviz=0
start_hesai=1
record_bag=0
debug=0
while (( $# )); do
  case "$1" in
    --rviz) use_rviz=1; shift ;;
    --no-start-hesai) start_hesai=0; shift ;;
    --record) record_bag=1; shift ;;
    --debug) debug=1; shift ;;
    *) printf 'ERROR: unknown option: %s\n' "$1" >&2; exit 2 ;;
  esac
done

if [[ "${debug}" == "1" ]]; then
  export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
fi

# ---------------------------------------------------------------------------
# ROS environment
# ---------------------------------------------------------------------------
set +u
# shellcheck disable=SC1091
source "${script_dir}/setup_environment.sh"
set -u

if [[ -z "${CYCLONEDDS_URI:-}" || -z "${GO2W_HOST_IP:-}" ]]; then
  printf '%s\n' 'ERROR: GO2-W DDS environment was not pinned to the robot-facing address.' >&2
  exit 2
fi
printf '[OK] DDS: interface=%s address=%s config=%s\n' \
  "${GO2W_INTERFACE}" "${GO2W_HOST_IP}" "${CYCLONEDDS_URI}"

topic_alive() {
  local topic="$1"
  local info
  info="$(timeout 5 ros2 topic info "${topic}" -v 2>/dev/null || true)"
  if grep -q "Publisher count: [1-9]" <<<"${info}"; then
    return 0
  fi
  return 1
}

topic_receiving() {
  local topic="$1"
  local out
  out="$(timeout "$2" ros2 topic hz "${topic}" --window 3 2>/dev/null || true)"
  [[ -n "${out}" ]]
}

# ---------------------------------------------------------------------------
# 1. + 2. config generation
# ---------------------------------------------------------------------------
python3 "${script_dir}/generate_plain_slam_pandar_config.py" --check >/dev/null
python3 "${script_dir}/generate_plain_slam_pandar_config.py"
printf '[OK] Runtime plain_slam config generated in %s\n' "${runtime_dir}"

# ---------------------------------------------------------------------------
# 3. + 4. Hesai PandarXT-16 stream
# ---------------------------------------------------------------------------
if topic_alive /hesai/pandarxt16/points_raw; then
  printf '[OK] Hesai PandarXT-16: publisher already present (reused, no second driver)\n'
elif [[ "${start_hesai}" == "1" ]]; then
  printf '%s\n' 'Hesai publisher missing; starting the existing diagnostic driver...'
  nohup bash "${script_dir}/start_hesai_pandarxt16.sh" \
    >"${log_root}/hesai_pandarxt16.log" 2>&1 &
  hesai_pid=$!
  printf '%s\n' "${hesai_pid}" >"${pid_root}/hesai_pandarxt16.pid"
  for _ in {1..30}; do
    topic_alive /hesai/pandarxt16/points_raw && break
    sleep 1
  done
  if ! topic_alive /hesai/pandarxt16/points_raw; then
    printf 'ERROR: /hesai/pandarxt16/points_raw has no publisher after 30s.\n' >&2
    printf 'See %s\n' "${log_root}/hesai_pandarxt16.log" >&2
    exit 2
  fi
else
  printf 'ERROR: --no-start-hesai given but /hesai/pandarxt16/points_raw is absent.\n' >&2
  exit 2
fi

if ! topic_receiving /hesai/pandarxt16/points_raw 20; then
  printf 'ERROR: /hesai/pandarxt16/points_raw has a publisher but no data in 20s.\n' >&2
  exit 2
fi
printf '[OK] Hesai PandarXT-16: receiving\n'

# ---------------------------------------------------------------------------
# 5. IMU
# ---------------------------------------------------------------------------
if ! topic_alive /utlidar/imu; then
  printf '%s\n' 'WARNING: /utlidar/imu has no publisher; LIO may degrade (IMU_DEGRADED).' >&2
  printf '%s\n' 'The mapping launch continues; motion chain is unaffected.' >&2
else
  printf '[OK] IMU: /utlidar/imu publishing\n'
fi

# ---------------------------------------------------------------------------
# 6. + 7. launch + readiness
# ---------------------------------------------------------------------------
launch_log="${log_root}/plain_slam_go2w.launch.log"
printf 'Launching plain_slam mapping (log: %s)...\n' "${launch_log}"
nohup ros2 launch go2w_plain_slam_bridge plain_slam_go2w.launch.py \
  >"${launch_log}" 2>&1 &
launch_pid=$!
printf '%s\n' "${launch_pid}" >"${pid_root}/plain_slam_go2w.pid"

ready_ok=0
for _ in {1..60}; do
  if topic_alive /go2w/slam/ready; then
    ready_ok=1
    break
  fi
  sleep 1
done

if [[ "${record_bag}" == "1" ]]; then
  bag_dir="${project_root}/outputs/bags/plain_slam_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "${bag_dir}"
  nohup ros2 bag record -o "${bag_dir}" \
    /hesai/pandarxt16/points_raw /go2w/slam/pandar_points /utlidar/imu \
    /go2w/slam/imu \
    /go2w/slam/imu_pose_raw /go2w/slam/imu_odom_raw /go2w/slam/odom_base \
    /go2w/slam/aligned_scan /go2w/slam/deskewed_scan /go2w/slam/lio_map_cloud \
    /go2w/slam/map_3d /go2w/slam/map_2d /go2w/slam/health /go2w/slam/ready \
    >"${log_root}/plain_slam_bag.log" 2>&1 &
  bag_pid=$!
  printf '%s\n' "${bag_pid}" >"${pid_root}/plain_slam_bag.pid"
  printf '[OK] Recording debug bag to %s\n' "${bag_dir}"
fi

if [[ "${ready_ok}" == "1" ]]; then
  printf '[OK] plain_slam LIO: running\n'
  printf '[OK] Spatial mapping ready (mode=MAPPING_ASSIST)\n'
  printf '%s\n' '[INFO] /go2w/odom/fused remains motion authority'
else
  printf '%s\n' 'WARNING: /go2w/slam/ready not seen within 60s.' >&2
  printf 'See %s\n' "${launch_log}" >&2
fi

if [[ "${use_rviz}" == "1" ]]; then
  rviz_config="${project_root}/configs/go2w/plain_slam_mapping.rviz"
  nohup rviz2 -d "${rviz_config}" >"${log_root}/rviz_plain_slam.log" 2>&1 &
  printf '%s\n' $! >"${pid_root}/rviz_plain_slam.pid"
  printf '[OK] RViz opened (fixed frame pslam_odom)\n'
fi

# ---------------------------------------------------------------------------
# summary
# ---------------------------------------------------------------------------
printf '%s\n' '--- mapping-assist summary ---'
bash "${script_dir}/check_plain_slam_mapping.sh" || true
printf '%s\n' '--- done ---'
