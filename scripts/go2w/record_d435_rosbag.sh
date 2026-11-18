#!/usr/bin/env bash
# One-command D435 RGB-D + full robot ROS 2 recording.
#
# This wrapper deliberately never opens the active SQLite database.  Runtime
# checks use only process state, recorder logs and directory size; bag metadata
# is inspected only after a graceful stop.
#
# Usage:
#   bash scripts/go2w/record_d435_rosbag.sh start
#   bash scripts/go2w/record_d435_rosbag.sh status
#   bash scripts/go2w/record_d435_rosbag.sh stop
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
runtime_dir="${project_root}/runtime/go2w/rosbag_d435"
bridge_pid_file="${runtime_dir}/bridge.pid"
bridge_owner_file="${runtime_dir}/bridge.started_by_recorder"
bridge_log="${runtime_dir}/d435_bridge.log"
recorder_script="${script_dir}/record_all_ros2.sh"
session_file="${project_root}/runtime/go2w/rosbag_all/current_session"
base_url="${D435_BASE_URL:-http://192.168.123.18:8080}"
bridge_rate="${GO2W_D435_RATE:-10}"

mkdir -p "${runtime_dir}"

source_ros() {
  set +u
  # shellcheck disable=SC1091
  source "${script_dir}/setup_environment.sh" >/dev/null
  set -u
}

read_bridge_pid() {
  [[ -f "${bridge_pid_file}" ]] || return 1
  local pid
  pid="$(<"${bridge_pid_file}")"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${pid}"
}

bridge_pid_alive() {
  local pid
  pid="$(read_bridge_pid 2>/dev/null)" || return 1
  kill -0 "${pid}" 2>/dev/null
}

find_existing_bridge() {
  pgrep -f '[r]ealsense_rgbd_bridge.py.*--base-url' | head -1
}

start_bridge() {
  local existing pid ready=0
  existing="$(find_existing_bridge || true)"
  if [[ -n "${existing}" ]] && kill -0 "${existing}" 2>/dev/null; then
    printf 'D435_BRIDGE_REUSED pid=%s\n' "${existing}"
    rm -f "${bridge_pid_file}" "${bridge_owner_file}"
    return 0
  fi

  rm -f "${bridge_pid_file}" "${bridge_owner_file}"
  : >"${bridge_log}"
  setsid env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY \
    -u ALL_PROXY -u all_proxy \
    /usr/bin/python3 "${script_dir}/realsense_rgbd_bridge.py" \
      --base-url "${base_url}" --rate "${bridge_rate}" \
      >"${bridge_log}" 2>&1 </dev/null &
  pid=$!
  printf '%s\n' "${pid}" >"${bridge_pid_file}"
  touch "${bridge_owner_file}"

  for _ in $(seq 1 40); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      printf 'ERROR: D435 ROS bridge exited during startup\n' >&2
      tail -40 "${bridge_log}" >&2 || true
      return 1
    fi
    if grep -q 'published D435 frame' "${bridge_log}"; then
      ready=1
      break
    fi
    sleep 0.5
  done
  if (( ready == 0 )); then
    printf 'ERROR: D435 produced no RGB-D frame within 20 seconds\n' >&2
    tail -40 "${bridge_log}" >&2 || true
    stop_owned_bridge
    return 1
  fi
  printf 'D435_BRIDGE_STARTED pid=%s base_url=%s\n' "${pid}" "${base_url}"
}

stop_owned_bridge() {
  [[ -f "${bridge_owner_file}" ]] || return 0
  local pid
  pid="$(read_bridge_pid 2>/dev/null || true)"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.2
    done
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    fi
  fi
  rm -f "${bridge_pid_file}" "${bridge_owner_file}"
}

current_session() {
  [[ -f "${session_file}" ]] || return 1
  local path
  path="$(<"${session_file}")"
  [[ -n "${path}" ]] || return 1
  printf '%s\n' "${path}"
}

recover_metadata_if_needed() {
  local session
  session="$(current_session 2>/dev/null || true)"
  [[ -n "${session}" && -d "${session}/bag" ]] || return 0
  if [[ ! -f "${session}/bag/metadata.yaml" ]] \
      && find "${session}/bag" -maxdepth 1 -name '*.db3' -print -quit | grep -q .; then
    printf 'Recovering rosbag metadata after an unclean recorder exit...\n'
    source_ros
    ros2 bag reindex "${session}/bag"
    ros2 bag info "${session}/bag" >"${session}/bag_info.txt"
  fi
}

start_all() {
  source_ros
  start_bridge
  if ! bash "${recorder_script}" start --d435-only-camera; then
    stop_owned_bridge
    return 1
  fi

  local session log_file ready=0
  session="$(current_session)"
  log_file="${session}/record.log"
  # Verify graph subscriptions from the recorder log only.  Never query the
  # active DB: doing so can lock sqlite3 and terminate rosbag2.
  for _ in $(seq 1 20); do
    if grep -q "Subscribed to topic '/go2w/d435/color/image_raw'" "${log_file}" \
        && grep -q "Subscribed to topic '/go2w/d435/depth/image_rect_raw'" "${log_file}"; then
      ready=1
      break
    fi
    sleep 0.5
  done
  if (( ready == 0 )); then
    printf 'ERROR: rosbag did not subscribe to both D435 image topics\n' >&2
    bash "${recorder_script}" stop || true
    stop_owned_bridge
    return 1
  fi

  printf 'READY_TO_DRIVE\n'
  printf 'session=%s\n' "${session}"
  printf 'camera=D435_RGB_PLUS_DEPTH\n'
  printf 'original_camera=EXCLUDED\n'
}

show_status() {
  bash "${recorder_script}" status
  local existing
  existing="$(find_existing_bridge || true)"
  printf 'd435_bridge=%s\n' "${existing:-STOPPED}"
  [[ -f "${bridge_log}" ]] && {
    printf '%s\n' 'recent_d435_log:'
    tail -5 "${bridge_log}"
  }
}

stop_all() {
  bash "${recorder_script}" stop
  stop_owned_bridge
  recover_metadata_if_needed
  printf 'D435_RECORDING_STOPPED\n'
}

case "${1:-status}" in
  start) start_all ;;
  status) show_status ;;
  stop) stop_all ;;
  *)
    printf 'usage: %s {start|status|stop}\n' "$0" >&2
    exit 2
    ;;
esac
