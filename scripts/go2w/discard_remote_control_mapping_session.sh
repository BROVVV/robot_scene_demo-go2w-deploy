#!/usr/bin/env bash
# Stop the teleoperated-mapping stack and irreversibly discard only its active
# session files. Historical rosbags, replay outputs and other WebUI modes are
# outside this script's explicit allow-list and are never removed.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
runtime_dir="${project_root}/outputs/remote_control_mapping/runtime"

quiet=0
while (( $# )); do
  case "$1" in
    --quiet) quiet=1; shift ;;
    *) printf 'ERROR: 未知选项: %s\n' "$1" >&2; exit 2 ;;
  esac
done

log() {
  if [[ "${quiet}" != "1" ]]; then
    printf '%s\n' "$*"
  fi
}

log '正在停止上一轮遥操建图进程……'
# The stop script validates every pid/cmdline before signalling it.  The
# explicit 2-D stop handles a previously reused project-owned map process too,
# which is necessary to guarantee that the next launch starts an empty map.
bash "${script_dir}/stop_remote_control_mapping.sh" || true
bash "${script_dir}/stop_2d_localization.sh" || true

# bootstrap_webui may have cold-started the read-only perception bridge.  Stop
# it only when this disposable launcher wrote the ownership marker; an already
# running perception stack belongs to its original operator and is reused.
perception_marker="${runtime_dir}/live_perception_started_by_one_click"
perception_pid_root="${project_root}/runtime/go2w/pids"
stop_owned_perception_pid() {
  local name="$1" expected="$2" pidfile="${perception_pid_root}/$1.pid"
  [[ -f "${pidfile}" ]] || return 0
  local pid cmdline pgid waited=0
  pid="$(tr -cd '0-9' <"${pidfile}")"
  if [[ -z "${pid}" || ! -d "/proc/${pid}" ]]; then
    rm -f -- "${pidfile}"
    return 0
  fi
  cmdline="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
  if [[ "${cmdline}" != *"${expected}"* ]]; then
    printf 'WARNING: %s pid=%s 已被其它进程复用，不发送信号：%s\n' \
      "${name}" "${pid}" "${cmdline:0:120}" >&2
    rm -f -- "${pidfile}"
    return 0
  fi
  pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ' || true)"
  if [[ -n "${pgid}" && "${pgid}" == "${pid}" ]]; then
    kill -TERM -- "-${pgid}" 2>/dev/null || true
  else
    kill -TERM "${pid}" 2>/dev/null || true
  fi
  while kill -0 "${pid}" 2>/dev/null && (( waited < 120 )); do
    sleep 0.1
    waited=$((waited + 1))
  done
  if kill -0 "${pid}" 2>/dev/null; then
    if [[ -n "${pgid}" && "${pgid}" == "${pid}" ]]; then
      kill -KILL -- "-${pgid}" 2>/dev/null || true
    else
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  fi
  rm -f -- "${pidfile}"
}

if [[ -f "${perception_marker}" ]]; then
  stop_owned_perception_pid start_live_perception start_live_perception.sh
  # Normally the supervisor's EXIT trap removes these children.  The exact,
  # validated fallbacks also cover a supervisor that was killed externally.
  stop_owned_perception_pid description go2w_description
  stop_owned_perception_pid camera realsense_rgbd_bridge.py
  stop_owned_perception_pid time go2w_sensor_time_bridge
  stop_owned_perception_pid lidar_watchdog lidar_freshness_watchdog
  stop_owned_perception_pid lidar go2w_lidar_preprocessor
  stop_owned_perception_pid fusion go2w_rgb_lidar_fusion
  stop_owned_perception_pid live_bridge robot_scene_live_bridge
  rm -f -- "${perception_marker}"
  log '本轮自动启动的只读激光/相机感知桥已停止。'
fi

mkdir -p "${runtime_dir}"

# This is intentionally an exact file allow-list: never use a wildcard or
# recursively remove outputs/remote_control_mapping as that directory also
# contains logs useful for diagnosing a failed launch.
active_files=(
  "${runtime_dir}/events.jsonl"
  "${runtime_dir}/status.json"
  "${runtime_dir}/slam_map_3d.json"
  "${runtime_dir}/slam_map_meta.json"
  "${runtime_dir}/slam_map_trajectory.json"
  "${runtime_dir}/semantic_objects.json"
  "${runtime_dir}/semantic_topology.json"
  "${runtime_dir}/target_watch_request.json"
  "${runtime_dir}/target_watch_state.json"
  "${runtime_dir}/semantic_frame.jpg"
  "${runtime_dir}/live_perception_started_by_one_click"
)
rm -f -- "${active_files[@]}"

evidence_dir="${runtime_dir}/target_watch_evidence"
if [[ -d "${evidence_dir}" ]]; then
  find "${evidence_dir}" -mindepth 1 -maxdepth 1 -type f -delete
fi
unknown_evidence_dir="${runtime_dir}/unknown_object_vlm_evidence"
if [[ -d "${unknown_evidence_dir}" ]]; then
  find "${unknown_evidence_dir}" -mindepth 1 -maxdepth 1 -type f -delete
fi

log '本轮活动地图、物体节点、拓扑图和目标复核证据已丢弃。'

