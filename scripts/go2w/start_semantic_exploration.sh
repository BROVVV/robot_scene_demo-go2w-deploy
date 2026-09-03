#!/usr/bin/env bash
# One-command launcher for the operator-supervised high-level semantic
# exploration experiment (plan section 23.2).
#
# For the real Go2-W backend it: checks the network, sources the ROS 2
# environment, starts the read-only perception stack and wheel odometry if
# they are not running, verifies the /go2w/motion action server, then runs
# scripts/go2w/run_semantic_exploration.py.  If the motion backend cannot be
# reached it reports MOTION_BACKEND_UNAVAILABLE and exits (no hang).
#
# For --backend mock / mock_metric it runs fully offline with the conda Python.

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
unitree_root="${GO2W_UNITREE_ROOT:-${HOME}/unitree_ros2}"
control_root="${GO2W_CONTROL_ROOT:-${project_root}/unitree_go2w_control}"
conda_python="/home/brov/miniconda3/envs/go2_robot_scene_demo/bin/python"
system_python="/usr/bin/python3"
log_root="${project_root}/runtime/go2w/sessions"
pid_root="${project_root}/runtime/go2w/pids"
mkdir -p "${log_root}" "${pid_root}"

target=""
backend="go2w_experimental"
max_seconds=600
max_motion_steps=50
turn_only=""
record_video=""
output=""
session_dir="outputs/live_runs"
allow_degraded=""
skip_stack_check=""
extra_args=()
# plain_slam mapping-assist integration (plan §15):
#   GO2W_SPATIAL_PROVIDER=plain_slam  -> --spatial-provider plain_slam
#   GO2W_START_PLAIN_SLAM=1           -> auto-start the mapping pipeline when
#                                        /go2w/slam/ready is not present
spatial_provider="${GO2W_SPATIAL_PROVIDER:-camera}"
start_plain_slam="${GO2W_START_PLAIN_SLAM:-0}"

while (( $# )); do
  case "$1" in
    --target) target="${2:-}"; shift 2 ;;
    --backend) backend="${2:-}"; shift 2 ;;
    --max-seconds) max_seconds="${2:-}"; shift 2 ;;
    --max-motion-steps) max_motion_steps="${2:-}"; shift 2 ;;
    --turn-only) turn_only="--turn-only"; shift ;;
    --record-video) record_video="${2:-}"; shift 2 ;;
    --output) output="${2:-}"; shift 2 ;;
    --session-dir) session_dir="${2:-}"; shift 2 ;;
    --allow-degraded) allow_degraded="--allow-degraded"; shift ;;
    --skip-stack-check) skip_stack_check="1"; shift ;;
    *) extra_args+=("$1"); shift ;;
  esac
done

if [[ -z "${target}" ]]; then
  printf '%s\n' 'ERROR: --target is required (e.g. 饮水机旁边的蓝色垃圾桶)' >&2
  exit 2
fi

cd "${project_root}"
# Load app env (API key etc.) without echoing secrets.
if [[ -f .env ]]; then
  set -a
  # shellcheck disable=SC1091
  source .env
  set +a
fi

run_id="semantic_exploration_$(date +%Y%m%d_%H%M%S)"
run_log="${log_root}/${run_id}.log"
printf 'Session: %s\nLog: %s\n' "${run_id}" "${run_log}"

# ---------------------------------------------------------------------------
# Offline mode: no ROS at all.
# ---------------------------------------------------------------------------
if [[ "${backend}" == "mock" || "${backend}" == "mock_metric" || "${skip_stack_check}" == "1" ]]; then
  args=(scripts/go2w/run_semantic_exploration.py --target "${target}" --backend "${backend}")
  [[ -n "${turn_only}" ]] && args+=( "${turn_only}" )
  [[ -n "${record_video}" ]] && args+=( --record-video "${record_video}" )
  [[ -n "${output}" ]] && args+=( --output "${output}" )
  args+=( --session-dir "${session_dir}" --max-seconds "${max_seconds}" \
          --max-motion-steps "${max_motion_steps}" )
  if [[ "${spatial_provider}" != "camera" ]]; then
    args+=( --spatial-provider "${spatial_provider}" --spatial-v2 )
  fi
  [[ -n "${allow_degraded}" ]] && args+=( "${allow_degraded}" )
  args+=( "${extra_args[@]}" )
  exec "${conda_python}" "${args[@]}"
fi

# ---------------------------------------------------------------------------
# Real Go2-W path.
# ---------------------------------------------------------------------------
if [[ ! -x "${system_python}" ]]; then
  printf '%s\n' 'ERROR: system python3 not found (required for rclpy).' >&2
  exit 2
fi
if ! command -v timeout >/dev/null 2>&1; then
  printf '%s\n' 'ERROR: coreutils timeout missing.' >&2
  exit 2
fi
# --- network check ----------------------------------------------------------
iface="$("${control_root}/scripts/detect_unitree_interface.sh" 2>/dev/null || true)"
if [[ -n "${iface}" ]]; then
  printf 'Robot interface: %s\n' "${iface}"
else
  printf '%s\n' 'WARNING: could not detect the Unitree interface; continuing.' >&2
fi
if ! ping -c 1 -W 1 192.168.123.18 >/dev/null 2>&1; then
  printf '%s\n' 'ERROR: robot 192.168.123.18 is not reachable.' >&2
  exit 2
fi

# --- ROS environment ---------------------------------------------------------
# Use the same interface-pinned, ROS1-cleansed environment as every other
# Go2-W launcher.  The previous inline setup always forced Humble and reused
# stale ROS variables from the caller's shell.
set +u
# shellcheck disable=SC1091
source "${project_root}/scripts/go2w/setup_environment.sh"
set -u
if ! command -v ros2 >/dev/null 2>&1; then
  printf '%s\n' 'ERROR: ros2 CLI still not available after sourcing ROS.' >&2
  exit 2
fi

# --- helper: is a topic alive? -------------------------------------------------
topic_alive() {
  local topic="$1"
  # Prefer topic-info publisher count; it is more reliable in this mixed
  # Foxy/Humble environment than `ros2 topic hz` (which can see the graph but
  # not receive data in some shells).  Also accept hz output as a fallback.
  local info
  info="$(timeout 5 ros2 topic info "${topic}" -v 2>/dev/null || true)"
  if grep -q "Publisher count: [1-9]" <<<"${info}"; then
    return 0
  fi
  local out
  out="$(timeout 5 ros2 topic hz "${topic}" --window 2 2>/dev/null)"
  [[ -n "${out}" ]]
}

started_stacks=()

cleanup() {
  for name in "${started_stacks[@]:-}"; do
    pid_file="${pid_root}/${name}.pid"
    if [[ -f "${pid_file}" ]]; then
      process_id="$(tr -cd '0-9' < "${pid_file}")"
      if [[ -n "${process_id}" ]] && kill -0 "${process_id}" 2>/dev/null; then
        printf 'Stopping stack %s (pid %s)\n' "${name}" "${process_id}"
        kill -TERM -- "-${process_id}" 2>/dev/null || kill -TERM "${process_id}" 2>/dev/null || true
        sleep 1
        kill -KILL -- "-${process_id}" 2>/dev/null || true
      fi
      rm -f "${pid_file}"
    fi
  done
}
trap cleanup EXIT

# --- perception stack ----------------------------------------------------------
if [[ -z "${skip_stack_check}" ]] && ! topic_alive /camera/front/image_raw; then
  printf '%s\n' 'Camera topic missing; starting read-only perception stack...'
  nohup bash "${script_dir}/start_live_perception.sh" >"${log_root}/start_live_perception.log" 2>&1 &
  perception_pid=$!
  printf '%s\n' "${perception_pid}" >"${pid_root}/start_live_perception.pid"
  started_stacks+=(start_live_perception)
  for _ in {1..30}; do
    topic_alive /camera/front/image_raw && break
    sleep 1
  done
  if ! topic_alive /camera/front/image_raw; then
    printf '%s\n' 'ERROR: perception stack did not publish /camera/front/image_raw.' >&2
    printf '%s\n' 'See runtime/go2w/sessions/start_live_perception.log' >&2
    exit 2
  fi
  printf '%s\n' 'Perception stack ready.'
fi

# --- wheel odometry -------------------------------------------------------------
if ! topic_alive /go2w/odom/fused; then
  printf '%s\n' 'Odom topic missing; starting wheel odometry...'
  nohup ros2 launch go2w_lio_bringup wheel_odom.launch.py \
    >"${log_root}/wheel_odom.log" 2>&1 &
  odom_pid=$!
  printf '%s\n' "${odom_pid}" >"${pid_root}/wheel_odom.pid"
  started_stacks+=(wheel_odom)
  for _ in {1..30}; do
    topic_alive /go2w/odom/fused && break
    sleep 1
  done
  if ! topic_alive /go2w/odom/fused; then
    printf '%s\n' 'ERROR: wheel odometry did not publish /go2w/odom/fused.' >&2
    printf '%s\n' 'See runtime/go2w/sessions/wheel_odom.log' >&2
    exit 2
  fi
  printf '%s\n' 'Wheel odometry ready.'
fi

# A topic being alive is insufficient: ROS collapses duplicate node names,
# while both publishers can still interleave poses from different origins.
odom_topic="${GO2W_ODOM_TOPIC:-/go2w/odom/fused}"
odom_info="$(timeout 5 ros2 topic info "$odom_topic" -v 2>&1 || true)"
odom_publishers="$(awk '/Publisher count:/ { print $3; exit }' <<<"$odom_info")"
odom_processes="$(ps -eo args= | awk '{ for (i=1; i<=NF; i++) { exe=$i; sub(/^.*\//, "", exe); if (exe == "go2w_wheel_odom") { count++; break } } } END { print count+0 }')"
odom_processes_valid=0
if [[ "$odom_processes" == 0 || "$odom_processes" == 1 ]]; then
  odom_processes_valid=1
fi
if [[ "$odom_publishers" != 1 || "$odom_processes_valid" != 1 ]]; then
  printf 'ODOM_BACKEND_UNAVAILABLE: %s requires exactly one publisher (ROS graph=%s, processes=%s).\n' \
    "$odom_topic" "${odom_publishers:-unknown}" "$odom_processes" >&2
  printf '%s\n' "$odom_info" >&2
  printf '%s\n' 'Stop duplicate wheel_odom launch processes before autonomous motion.' >&2
  exit 2
fi
if [[ "$odom_processes" == 0 ]]; then
  printf '%s\n' 'Using the single network wheel-odom publisher from the control host.' >&2
fi

# --- motion action server ---------------------------------------------------------
motion_action_info="$(timeout 5 ros2 action info /go2w/motion 2>&1 || true)"
motion_action_servers="$(awk '/Action servers:/ { print $3; exit }' <<<"$motion_action_info")"
motion_action_processes="$(ps -eo args= | awk '{ exe=$1; sub(/^.*\//, "", exe); if (exe == "go2w_motion_action_server") count++ } END { print count+0 }')"
if [[ "$motion_action_servers" != 1 || "$motion_action_processes" != 1 ]]; then
  printf 'MOTION_BACKEND_UNAVAILABLE: /go2w/motion requires exactly one server (ROS graph=%s, OS processes=%s).\n' \
    "${motion_action_servers:-unknown}" "$motion_action_processes" >&2
  printf '%s\n' "$motion_action_info" >&2
  printf '%s\n' 'Start it in another terminal:' >&2
  printf '  cd %s\n' "${control_root}" >&2
  printf '%s\n' '  source scripts/setup_go2w_ros2.sh' >&2
  printf '%s\n' '  ros2 launch go2w_motion_control go2w_motion_control.launch.py' >&2
  exit 3
fi
printf '%s\n' '/go2w/motion action server ready.'

# --- plain_slam mapping-assist (optional; plan §15) ---------------------------
# GO2W_START_PLAIN_SLAM=1 auto-starts the Pandar plain_slam mapping pipeline
# when /go2w/slam/ready is not yet published.  Failure to start degrades to the
# D435/camera provider and never blocks the existing exploration stack.
if [[ "${start_plain_slam}" == "1" ]]; then
  if topic_alive /go2w/slam/ready; then
    printf '%s\n' 'plain_slam mapping already running (/go2w/slam/ready present).'
  else
    printf '%s\n' 'Starting plain_slam mapping-assist pipeline (GO2W_START_PLAIN_SLAM=1)...'
    nohup bash "${script_dir}/start_plain_slam_mapping.sh" \
      >"${log_root}/start_plain_slam_mapping.log" 2>&1 &
    mapping_pid=$!
    printf '%s\n' "${mapping_pid}" >"${pid_root}/start_plain_slam_mapping.pid"
    started_stacks+=(start_plain_slam_mapping)
    ready_ok=0
    for _ in {1..60}; do
      if topic_alive /go2w/slam/ready; then ready_ok=1; break; fi
      sleep 1
    done
    if [[ "${ready_ok}" == "1" ]]; then
      printf '%s\n' 'plain_slam mapping ready (MAPPING_ASSIST, motion authority unchanged).'
    else
      printf '%s\n' \
        'WARNING: /go2w/slam/ready not seen within 60s; spatial provider will fall back.' >&2
      printf '%s\n' "See ${log_root}/start_plain_slam_mapping.log" >&2
    fi
  fi
fi

# --- VLM daemon (optional; subprocess fallback if unavailable) ------------------------
if [[ "${NO_VLM_DAEMON:-0}" != "1" ]]; then
  bash "${script_dir}/start_vlm_daemon.sh" >/dev/null 2>&1 || true
  if [[ -S "${project_root}/runtime/go2w/siliconflow_vlm.sock" ]]; then
    started_stacks+=(siliconflow_vlm)
    printf '%s\n' 'VLM daemon ready.'
  fi
fi

# --- run the exploration CLI ---------------------------------------------------------
args=(scripts/go2w/run_semantic_exploration.py --target "${target}" --backend go2w_experimental)
[[ -n "${turn_only}" ]] && args+=( "${turn_only}" )
[[ -n "${record_video}" ]] && args+=( --record-video "${record_video}" )
[[ -n "${output}" ]] && args+=( --output "${output}" )
args+=( --session-dir "${session_dir}" --max-seconds "${max_seconds}" \
        --max-motion-steps "${max_motion_steps}" \
        --operator-supervised-experiment \
        --profile-config configs/go2w/high_level_experiment.yaml )
if [[ "${spatial_provider}" != "camera" ]]; then
  args+=( --spatial-provider "${spatial_provider}" --spatial-v2 )
fi
[[ -n "${allow_degraded}" ]] && args+=( "${allow_degraded}" )
args+=( "${extra_args[@]}" )
exec "${system_python}" "${args[@]}"
