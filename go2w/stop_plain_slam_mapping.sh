#!/usr/bin/env bash
# Stop only the plain_slam mapping-assist processes started by
# start_plain_slam_mapping.sh.  Never touches:
#   - /go2w/odom/fused (wheel odometry)
#   - the /go2w/motion action server / control arbiter
#   - the camera stack or any existing safety chain
#   - a Hesai driver that was already running before mapping started
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
pid_root="${project_root}/runtime/go2w/pids"

stop_pid_file() {
  local name="$1"
  local pid_file="${pid_root}/${name}.pid"
  if [[ -f "${pid_file}" ]]; then
    local pid
    pid="$(tr -cd '0-9' < "${pid_file}")"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      printf 'Stopping %s (pid %s)\n' "${name}" "${pid}"
      kill -TERM "${pid}" 2>/dev/null || true
      sleep 1
      kill -KILL "${pid}" 2>/dev/null || true
    fi
    rm -f "${pid_file}"
  fi
}

# RViz / bag recorder / launch process.
stop_pid_file rviz_plain_slam
stop_pid_file plain_slam_bag
stop_pid_file plain_slam_go2w

# Bridge + plain_slam node processes (falls back to pgrep when the launcher
# pid file is stale).
mapfile -t mapping_pids < <(
  pgrep -f 'plain_slam_go2w.launch.py' 2>/dev/null || true
  pgrep -f 'pandar_slam_adapter' 2>/dev/null || true
  pgrep -f 'plain_slam_odom_adapter' 2>/dev/null || true
  pgrep -f 'pointcloud_to_occupancy' 2>/dev/null || true
  pgrep -f 'plain_slam_health_monitor' 2>/dev/null || true
  pgrep -f 'imu_fallback_adapter.py' 2>/dev/null || true
  pgrep -f 'lio_3d_node' 2>/dev/null || true
  pgrep -f 'slam_3d_node' 2>/dev/null || true
  pgrep -f 'go2w_plain_slam_lio' 2>/dev/null || true
  pgrep -f 'go2w_plain_slam_slam' 2>/dev/null || true
)
for pid in "${mapping_pids[@]:-}"; do
  pid="$(printf '%s\n' "${pid}" | tr -cd '0-9')"
  [[ -n "${pid}" && -r "/proc/${pid}/cmdline" ]] || continue
  [[ "${pid}" != "$$" && "${pid}" != "${PPID}" ]] || continue
  # Only kill processes that belong to this workspace (avoid touching the
  # robot-facing motion stack or a pre-existing Hesai driver).
  cmdline="$(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null || true)"
  case "${cmdline}" in
    *"${project_root}/ros2_ws"*|*go2w_plain_slam_bridge*|*plain_slam_go2w*)
      printf 'Stopping mapping process (pid %s)\n' "${pid}"
      kill -TERM "${pid}" 2>/dev/null || true
      ;;
  esac
done
# KILL fallback for processes that ignore SIGTERM (launcher orphans).
sleep 2
for pid in "${mapping_pids[@]:-}"; do
  pid="$(printf '%s\n' "${pid}" | tr -cd '0-9')"
  [[ -n "${pid}" && -r "/proc/${pid}/cmdline" ]] || continue
  [[ "${pid}" != "$$" && "${pid}" != "${PPID}" ]] || continue
  if kill -0 "${pid}" 2>/dev/null; then
    cmdline="$(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null || true)"
    case "${cmdline}" in
      *"${project_root}/ros2_ws"*|*go2w_plain_slam_bridge*|*plain_slam_go2w*)
        kill -KILL "${pid}" 2>/dev/null || true
        ;;
    esac
  fi
done

# The Hesai driver PID recorded by start_plain_slam_mapping.sh is only
# stopped when *this launcher* started it (never a pre-existing driver).
stop_pid_file hesai_pandarxt16

sleep 1
printf '%s\n' 'plain_slam mapping stopped. /go2w/odom/fused and motion stack untouched.'
