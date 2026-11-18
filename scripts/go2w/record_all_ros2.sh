#!/usr/bin/env bash
# Start, inspect, or gracefully stop a full GO2-W ROS 2 recording.
#
# This script is read-only with respect to the robot: it only creates DDS
# subscriptions and never publishes motion commands or calls robot services.
#
# Usage:
#   bash scripts/go2w/record_all_ros2.sh start
#   bash scripts/go2w/record_all_ros2.sh start --d435-only-camera
#   bash scripts/go2w/record_all_ros2.sh status
#   bash scripts/go2w/record_all_ros2.sh stop
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
runtime_dir="${project_root}/runtime/go2w/rosbag_all"
output_root="${GO2W_ROSBAG_OUTPUT_ROOT:-${project_root}/outputs/rosbags}"
qos_file="${project_root}/configs/go2w/rosbag_all_qos.yaml"
pid_file="${runtime_dir}/recorder.pid"
guard_pid_file="${runtime_dir}/disk_guard.pid"
session_file="${runtime_dir}/current_session"
min_free_gib="${GO2W_ROSBAG_MIN_FREE_GIB:-15}"
# These vendor-only topics cannot be deserialized by the public Unitree ROS 2
# message package installed on this host.  They are control/UI channels or
# duplicates of standard topics (the standard /utlidar/voxel_map is kept).
excluded_topic_regex='^/(utlidar/voxel_map_compressed|config_change_status|arm_Command|arm_Feedback|pctoimage_local|qt_add_edge|qt_add_node|qt_command|query_result_edge|query_result_node)$'
camera_profile="all"

configure_profile() {
  case "${1:-}" in
    "") ;;
    --d435-only-camera)
      # /camera/front/* is a compatibility alias of the D435 stream in the
      # current bridge, while /frontvideostream is the Unitree native camera.
      # Record the canonical /go2w/d435/* RGB-D topics once and exclude both
      # legacy/original camera paths so replay has one unambiguous camera.
      excluded_topic_regex='^/(utlidar/voxel_map_compressed|config_change_status|arm_Command|arm_Feedback|pctoimage_local|qt_add_edge|qt_add_node|qt_command|query_result_edge|query_result_node|camera/front/.*|frontvideostream)$'
      camera_profile="d435_only"
      ;;
    *)
      printf 'ERROR: unknown recording profile: %s\n' "$1" >&2
      return 2
      ;;
  esac
}

mkdir -p "${runtime_dir}" "${output_root}"

source_environment() {
  # setup_environment.sh intentionally relaxes nounset while sourcing ROS.
  set +u
  # shellcheck disable=SC1091
  source "${script_dir}/setup_environment.sh" >/dev/null
  set -u
}

read_pid() {
  [[ -f "${pid_file}" ]] || return 1
  local pid
  pid="$(<"${pid_file}")"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${pid}"
}

is_running() {
  local pid
  pid="$(read_pid 2>/dev/null)" || return 1
  kill -0 "${pid}" 2>/dev/null
}

current_session() {
  [[ -f "${session_file}" ]] || return 1
  local path
  path="$(<"${session_file}")"
  [[ -n "${path}" ]] || return 1
  printf '%s\n' "${path}"
}

write_manifest() {
  local session="$1"
  {
    printf 'started_at=%s\n' "$(date -Ins)"
    printf 'hostname=%s\n' "$(hostname)"
    printf 'user=%s\n' "$(id -un)"
    printf 'ros_distro=%s\n' "${ROS_DISTRO:-unknown}"
    printf 'rmw_implementation=%s\n' "${RMW_IMPLEMENTATION:-unknown}"
    printf 'ros_domain_id=%s\n' "${ROS_DOMAIN_ID:-0}"
    printf 'network_interface=%s\n' "${GO2W_INTERFACE:-unknown}"
    printf 'host_ip=%s\n' "${GO2W_HOST_IP:-unknown}"
    printf 'qos_overrides=%s\n' "${qos_file}"
    printf 'camera_profile=%s\n' "${camera_profile}"
    printf 'excluded_ros_topic_regex=%s\n' "${excluded_topic_regex}"
    printf 'excluded_reason=%s\n' 'vendor type support unavailable; control/UI topics or standard equivalents are recorded'
    printf 'minimum_free_gib=%s\n' "${min_free_gib}"
  } >"${session}/session.env"
  ros2 topic list -t >"${session}/topics_at_start.txt" 2>&1 || true
  ps -eo pid,lstart,cmd >"${session}/processes_at_start.txt" 2>&1 || true
  df -h "${output_root}" >"${session}/disk_at_start.txt" 2>&1 || true
}

start_recording() {
  if is_running; then
    printf 'already recording: pid=%s session=%s\n' \
      "$(read_pid)" "$(current_session 2>/dev/null || printf '<unknown>')"
    return 0
  fi

  rm -f "${pid_file}" "${guard_pid_file}"
  source_environment
  [[ -f "${qos_file}" ]] || {
    printf 'ERROR: QoS file missing: %s\n' "${qos_file}" >&2
    return 2
  }

  local available_kib minimum_kib
  available_kib="$(df -Pk "${output_root}" | awk 'NR==2 {print $4}')"
  minimum_kib="$((min_free_gib * 1024 * 1024))"
  if (( available_kib <= minimum_kib )); then
    printf 'ERROR: only %s GiB free; at least %s GiB is required\n' \
      "$((available_kib / 1024 / 1024))" "${min_free_gib}" >&2
    return 2
  fi

  local run_id session log_file
  run_id="go2w_all_$(date +%Y%m%d_%H%M%S)"
  session="${output_root}/${run_id}"
  mkdir -p "${session}"
  printf '%s\n' "${session}" >"${session_file}"
  write_manifest "${session}"
  log_file="${session}/record.log"

  # Four-GiB splits keep each SQLite file recoverable and easy to copy.  The
  # 512-MiB cache absorbs short bursts from RGB-D and duplicated point-cloud
  # pipelines without spending CPU on compression during the live run.
  nohup setsid ros2 bag record \
    --all \
    --include-hidden-topics \
    --exclude "${excluded_topic_regex}" \
    --polling-interval 500 \
    --max-bag-size 4294967296 \
    --max-cache-size 536870912 \
    --compression-mode none \
    --qos-profile-overrides-path "${qos_file}" \
    --output "${session}/bag" \
    >"${log_file}" 2>&1 </dev/null &
  local recorder_pid=$!
  printf '%s\n' "${recorder_pid}" >"${pid_file}"

  local ready=0
  for _ in $(seq 1 30); do
    if ! kill -0 "${recorder_pid}" 2>/dev/null; then
      printf 'ERROR: recorder exited during startup\n' >&2
      tail -80 "${log_file}" >&2 || true
      rm -f "${pid_file}"
      return 1
    fi
    if find "${session}/bag" -maxdepth 1 -type f \
      \( -name '*.db3' -o -name '*.mcap' \) -print -quit 2>/dev/null \
      | grep -q .; then
      ready=1
      break
    fi
    sleep 1
  done
  if (( ready == 0 )); then
    printf 'ERROR: bag storage was not created within 30 seconds\n' >&2
    kill -INT -- "-${recorder_pid}" 2>/dev/null || true
    return 1
  fi

  # Stop cleanly before the filesystem becomes dangerously full.  The guard
  # records its reason in the session directory for post-run diagnosis.
  (
    while kill -0 "${recorder_pid}" 2>/dev/null; do
      available_kib="$(df -Pk "${output_root}" | awk 'NR==2 {print $4}')"
      if (( available_kib <= minimum_kib )); then
        printf '%s low_disk_space available_kib=%s threshold_kib=%s\n' \
          "$(date -Ins)" "${available_kib}" "${minimum_kib}" \
          >>"${session}/disk_guard.log"
        kill -INT -- "-${recorder_pid}" 2>/dev/null || true
        exit 0
      fi
      sleep 5
    done
  ) >/dev/null 2>&1 &
  printf '%s\n' "$!" >"${guard_pid_file}"

  sleep 3
  if ! kill -0 "${recorder_pid}" 2>/dev/null; then
    printf 'ERROR: recorder stopped immediately after storage creation\n' >&2
    tail -80 "${log_file}" >&2 || true
    return 1
  fi

  printf 'RECORDING\n'
  printf 'pid=%s\n' "${recorder_pid}"
  printf 'session=%s\n' "${session}"
  printf 'free_space_gib=%s\n' "$((available_kib / 1024 / 1024))"
  printf 'disk_guard_min_free_gib=%s\n' "${min_free_gib}"
  printf 'camera_profile=%s\n' "${camera_profile}"
}

show_status() {
  local session pid state size free
  session="$(current_session 2>/dev/null || true)"
  pid="$(read_pid 2>/dev/null || true)"
  if is_running; then
    state="RECORDING"
  else
    state="STOPPED"
  fi
  size="0"
  [[ -n "${session}" && -d "${session}" ]] && size="$(du -sh "${session}" | awk '{print $1}')"
  free="$(df -h "${output_root}" | awk 'NR==2 {print $4}')"
  printf 'state=%s\n' "${state}"
  printf 'pid=%s\n' "${pid:-<none>}"
  printf 'session=%s\n' "${session:-<none>}"
  printf 'session_size=%s\n' "${size}"
  printf 'disk_free=%s\n' "${free}"
  if [[ -n "${session}" && -f "${session}/record.log" ]]; then
    printf '%s\n' 'recent_log:'
    tail -20 "${session}/record.log"
  fi
}

stop_recording() {
  local pid session guard_pid
  session="$(current_session 2>/dev/null || true)"
  if ! is_running; then
    printf 'not recording; last_session=%s\n' "${session:-<none>}"
    rm -f "${pid_file}" "${guard_pid_file}"
    return 0
  fi
  pid="$(read_pid)"
  printf 'stopping recorder pid=%s gracefully...\n' "${pid}"
  kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
  for _ in $(seq 1 60); do
    kill -0 "${pid}" 2>/dev/null || break
    sleep 1
  done
  if kill -0 "${pid}" 2>/dev/null; then
    printf 'ERROR: recorder did not stop; it was left running to avoid bag corruption\n' >&2
    return 1
  fi
  if [[ -f "${guard_pid_file}" ]]; then
    guard_pid="$(<"${guard_pid_file}")"
    [[ "${guard_pid}" =~ ^[0-9]+$ ]] && kill "${guard_pid}" 2>/dev/null || true
  fi
  rm -f "${pid_file}" "${guard_pid_file}"
  if [[ -n "${session}" && -d "${session}/bag" ]]; then
    source_environment
    ros2 bag info "${session}/bag" >"${session}/bag_info.txt" 2>&1 || true
    printf 'STOPPED\n'
    printf 'session=%s\n' "${session}"
    printf 'size=%s\n' "$(du -sh "${session}" | awk '{print $1}')"
    tail -30 "${session}/bag_info.txt" || true
  fi
}

command="${1:-status}"
profile="${2:-}"
if [[ -n "${3:-}" ]]; then
  printf 'usage: %s {start [--d435-only-camera]|status|stop}\n' "$0" >&2
  exit 2
fi

case "${command}" in
  start)
    configure_profile "${profile}"
    start_recording
    ;;
  status) show_status ;;
  stop) stop_recording ;;
  *)
    printf 'usage: %s {start [--d435-only-camera]|status|stop}\n' "$0" >&2
    exit 2
    ;;
esac
