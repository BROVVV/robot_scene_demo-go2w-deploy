#!/usr/bin/env bash
set -euo pipefail

# Go2-W Autonomous Semantic Search WebUI launcher (plan book §76-§79).
#
#   bash scripts/go2w/start_autonomous_search_web.sh
#       # read-only start: camera + LLM + Manual WASD, search dry-run only
#   bash scripts/go2w/start_autonomous_search_web.sh --enable-autonomous-motion
#       # authorize the autonomous search to drive the robot (<=30 deg turns,
#       # <=0.30 m steps through the existing /go2w/motion safety stack)
#   bash scripts/go2w/start_autonomous_search_web.sh --mock
#       # offline frontend dev: mock backend, no robot / ROS required
#   bash scripts/go2w/start_autonomous_search_web.sh --with-plain-slam
#       # ensure mapping-assist is running and expose its live 3D cloud in WebUI
#
# This script never starts Nav2 / Point-LIO / the Pandar driver itself; it
# only launches the single FastAPI server (manual + autonomous console) and
# only stops processes the project itself owns.

ENABLE_MOTION=0
MOCK=0
WITH_PLAIN_SLAM=0
for arg in "$@"; do
  case "$arg" in
    --enable-autonomous-motion) ENABLE_MOTION=1 ;;
    --with-plain-slam) WITH_PLAIN_SLAM=1 ;;
    --mock) MOCK=1 ;;
    *) printf 'ERROR: unknown argument: %s\n' "$arg" >&2; exit 2 ;;
  esac
done

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
runtime_root="${project_root}/outputs/autonomous_search/runtime"
log_root="${project_root}/outputs/autonomous_search/logs"
mkdir -p "${runtime_root}" "${log_root}"

host="${AUTONOMOUS_SEARCH_HOST:-127.0.0.1}"
port="${AUTONOMOUS_SEARCH_PORT:-8765}"
probe_host="$host"
if [[ "$probe_host" == "0.0.0.0" || "$probe_host" == "::" ]]; then
  probe_host="127.0.0.1"
fi

# ---- 0. Port conflict: only stop project-owned Go2-W web servers ---------- #
if curl -fsS "http://${probe_host}:${port}/api/status" >/dev/null 2>&1; then
  printf 'Port %s already serves a Go2-W web demo.\n' "${port}" >&2
  for pidfile in \
    "${project_root}/outputs/manual_web_demo/runtime/web.pid" \
    "${runtime_root}/web.pid"; do
      if [[ -f "$pidfile" ]]; then
        pid="$(<"$pidfile")"
        if kill -0 "$pid" 2>/dev/null; then
          printf 'Stopping previous project-owned web server (pid %s) ...\n' "$pid" >&2
          pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]')"
          if [[ "$pgid" == "$pid" ]]; then
            kill -INT -- "-$pid" || true
          else
            kill "$pid" || true
          fi
        for _ in $(seq 1 40); do
          if [[ "$pgid" == "$pid" ]]; then
            kill -0 -- "-$pid" 2>/dev/null || break
          else
            kill -0 "$pid" 2>/dev/null || break
          fi
          sleep 0.25
          done
        # Uvicorn may not exit on the first group SIGINT when a worker is in
        # a blocking request.  Escalate only within the verified, project-
        # owned process group so a stale server cannot survive a restart.
        if [[ "$pgid" == "$pid" ]] && kill -0 -- "-$pid" 2>/dev/null; then
          kill -TERM -- "-$pid" || true
          for _ in $(seq 1 20); do
            kill -0 -- "-$pid" 2>/dev/null || break
            sleep 0.25
          done
        fi
        if [[ "$pgid" == "$pid" ]] && kill -0 -- "-$pid" 2>/dev/null; then
          kill -KILL -- "-$pid" || true
        fi
        fi
      fi
    done
fi

# ---- 1. Network preflight ------------------------------------------------ #
go2w_interface="${GO2W_INTERFACE:-}"
if [[ -z "$go2w_interface" ]]; then
  for candidate in eth0 enp6s0 enp3s0 enp4s0 enp5s0; do
    if [[ -r "/sys/class/net/${candidate}/carrier" ]] \
      && [[ "$(< "/sys/class/net/${candidate}/carrier")" == "1" ]] \
      && ip -4 -o address show dev "$candidate" 2>/dev/null \
        | awk '$4 ~ /^192[.]168[.]123[.][0-9]+\// { found=1 } END { exit !found }'; then
      go2w_interface="$candidate"
      break
    fi
  done
fi
go2w_interface="${go2w_interface:-enp6s0}"
export GO2W_INTERFACE="$go2w_interface"
if [[ ! -r "/sys/class/net/${go2w_interface}/carrier" ]] \
  || [[ "$(< "/sys/class/net/${go2w_interface}/carrier")" != "1" ]]; then
  printf 'WARNING: %s has no Ethernet carrier; camera/motion will be unavailable.\n' "$go2w_interface" >&2
fi

# ---- 2. Source ROS environment for the worker subprocess ----------------- #
source "${script_dir}/setup_environment.sh"

# ---- 2a. Optional one-step plain_slam mapping ---------------------------- #
# Reuse a healthy existing graph.  Never launch a duplicate LIO/SLAM stack.
if [[ "$MOCK" == 0 && "$WITH_PLAIN_SLAM" == 1 ]]; then
  slam_info="$(timeout 5 ros2 topic info /go2w/slam/aligned_scan -v 2>/dev/null || true)"
  if ! grep -q 'Publisher count: [1-9]' <<<"$slam_info"; then
    printf 'plain_slam is not running; starting mapping-assist first...\n' >&2
    bash "${script_dir}/start_plain_slam_mapping.sh"
  else
    printf 'plain_slam mapping-assist already running; reusing it.\n' >&2
  fi
fi

# ---- 3. Camera bridge check (read-only) ---------------------------------- #
# DDS discovery can take a moment after the environment is sourced.  Wait
# briefly so a healthy bridge is not reported as missing during startup.
camera_topic_found=0
for _ in $(seq 1 40); do
  if ros2 topic list 2>/dev/null \
      | grep -Eq '^/camera/front/image_raw(/compressed)?$'; then
    camera_topic_found=1
    break
  fi
  sleep 0.25
done
if [[ "$camera_topic_found" != 1 ]]; then
  printf 'WARNING: /camera/front/image_raw/compressed not found.\n' >&2
  printf '         Start the read-only perception stack first:\n' >&2
  printf '           bash %s/start_live_perception.sh\n' "${script_dir}" >&2
fi

# ---- 4. Autonomous motion authorization ---------------------------------- #
if [[ "$ENABLE_MOTION" == 1 && "$MOCK" == 0 ]]; then
  service_server_available() {
    local service="$1" node info
    while IFS= read -r node; do
      [[ -z "$node" ]] && continue
      info="$(timeout 5 ros2 node info "$node" 2>/dev/null || true)"
      if awk -v service="$service" '
        /^  Service Servers:/ { in_servers=1; next }
        /^  Service Clients:/ { in_servers=0 }
        in_servers && $1 == service ":" { found=1 }
        END { exit !found }
      ' <<<"$info"; then
        return 0
      fi
    done < <(timeout 5 ros2 node list 2>/dev/null || true)
    return 1
  }
  odom_topic="${GO2W_ODOM_TOPIC:-/go2w/odom/fused}"
  odom_info="$(timeout 5 ros2 topic info "$odom_topic" -v 2>&1 || true)"
  odom_publishers="$(awk '/Publisher count:/ { print $3; exit }' <<<"$odom_info")"
  odom_processes="$(ps -eo args= | awk '{ for (i=1; i<=NF; i++) { exe=$i; sub(/^.*\//, "", exe); if (exe == "go2w_wheel_odom") { count++; break } } } END { print count+0 }')"
  if [[ "$odom_publishers" != 1 || "$odom_processes" != 1 ]]; then
    printf 'ERROR: %s requires exactly one publisher (ROS graph=%s, wheel odom processes=%s).\n' \
      "$odom_topic" "${odom_publishers:-unknown}" "$odom_processes" >&2
    printf '%s\n' "$odom_info" >&2
    printf 'Duplicate odometry origins make motion verification and topology unsafe.\n' >&2
    exit 2
  fi
  motion_action_info="$(timeout 5 ros2 action info /go2w/motion 2>&1 || true)"
  motion_action_servers="$(awk '/Action servers:/ { print $3; exit }' <<<"$motion_action_info")"
  motion_action_processes="$(ps -eo args= | awk '{ exe=$1; sub(/^.*\//, "", exe); if (exe == "go2w_motion_action_server") count++ } END { print count+0 }')"
  if [[ "$motion_action_servers" != 1 || "$motion_action_processes" != 1 ]]; then
    printf 'ERROR: /go2w/motion requires exactly one server (ROS graph=%s, OS processes=%s).\n' \
      "${motion_action_servers:-unknown}" "$motion_action_processes" >&2
    printf '%s\n' "$motion_action_info" >&2
    printf 'Stop duplicate/stale go2w motion launch processes before autonomous motion.\n' >&2
    exit 2
  fi
  for service in /go2w/arm /go2w/emergency_stop; do
    if ! service_server_available "$service"; then
      printf 'ERROR: %s service server is not available; autonomous motion cannot start.\n' "$service" >&2
      exit 2
    fi
  done
  if [[ "${GO2W_AREA_CLEARED:-}" != "I_HAVE_CLEARED_THE_AREA" ]]; then
    printf 'Autonomous motion requested. You must keep a level, dry, obstacle-free\n' >&2
    printf 'area (>=2 m) and hold the remote emergency stop. Turns are capped at\n' >&2
    printf '<=30 deg, forward steps at <=0.30 m through the existing motion gate.\n' >&2
    read -r -p 'Type I_CONFIRM to authorize autonomous motion: ' answer
    if [[ "$answer" != "I_CONFIRM" ]]; then
      printf 'aborted.\n' >&2
      exit 2
    fi
  fi
fi

# ---- 5. Resolve the Conda Python for Web/LLM ----------------------------- #
conda_python=""
for candidate in \
  "${GO2W_CONDA_PYTHON:-}" \
  "${project_root}/.venv/bin/python" \
  "$HOME/anaconda3/envs/go2_robot_scene_demo/bin/python" \
  "$HOME/miniconda3/envs/go2_robot_scene_demo/bin/python" \
  /home/mxt/anaconda3/envs/go2_robot_scene_demo/bin/python \
  /home/brov/miniconda3/envs/go2_robot_scene_demo/bin/python \
  "${CONDA_PREFIX:-}/bin/python"; do
  if [[ -x "$candidate" ]]; then
    conda_python="$candidate"
    break
  fi
done
if [[ -z "$conda_python" ]]; then
  printf 'ERROR: go2_robot_scene_demo conda environment not found.\n' >&2
  exit 2
fi

# ---- 6. Idempotently add FastAPI/Uvicorn/WebSocket support --------------- #
if ! "$conda_python" -c 'import fastapi, uvicorn, websockets' >/dev/null 2>&1; then
  printf 'Installing fastapi + uvicorn + websockets into go2_robot_scene_demo...\n' >&2
  "$conda_python" -m pip install --quiet fastapi uvicorn websockets
fi

# A fresh bootstrap creates one environment containing both project packages
# and ROS system packages.  Prefer it for the real search worker when it can
# satisfy both sides; older installations retain the existing auto-detection
# fallback to /usr/bin/python3.
if [[ "$MOCK" == 0 && -z "${GO2W_WORKER_PYTHON:-}" ]] \
  && "$conda_python" -c \
    'import rclpy, sys; sys.path.insert(0, "scripts/go2w"); import run_semantic_exploration' \
    >/dev/null 2>&1; then
  export GO2W_WORKER_PYTHON="$conda_python"
fi

# ---- 7. Launch the Web server (spawns ROS worker + search worker) -------- #
cd "${project_root}"
export MANUAL_DEMO_RUNTIME_DIR="${MANUAL_DEMO_RUNTIME_DIR:-outputs/manual_web_demo/runtime}"
export MANUAL_DEMO_LOGS_DIR="${MANUAL_DEMO_LOGS_DIR:-outputs/manual_web_demo/logs}"
export AUTONOMOUS_SEARCH_RUNTIME_DIR="${runtime_root}"
export AUTONOMOUS_SEARCH_LOGS_DIR="${log_root}"
export GO2W_SLAM_MAP_SNAPSHOT="${runtime_root}/slam_map_3d.json"
# Search worker interpreter is auto-detected unless the unified bootstrap
# environment above has already proved it can import both ROS and the project.
if [[ "$MOCK" == 1 ]]; then
  export AUTONOMOUS_SEARCH_DEFAULT_BACKEND="mock"
  export AUTONOMOUS_SEARCH_ENABLE_AUTONOMOUS_MOTION="0"
else
  export AUTONOMOUS_SEARCH_DEFAULT_BACKEND="go2w_experimental"
  export AUTONOMOUS_SEARCH_ENABLE_AUTONOMOUS_MOTION="${ENABLE_MOTION}"
  # Carry the repository's existing operator-supervised experiment profile
  # into every WebUI start request so it cannot be dropped at the worker
  # boundary.  Motion still requires the explicit launcher opt-in above.
  export AUTONOMOUS_SEARCH_OPERATOR_SUPERVISED="$([[ "$ENABLE_MOTION" == 1 ]] && echo 1 || echo 0)"
  export AUTONOMOUS_SEARCH_SPATIAL_PROVIDER="plain_slam"
fi

# A ROS-system-Python sidecar decimates the PointCloud2 stream into an atomic
# JSON snapshot.  The FastAPI process remains free of rclpy/ABI coupling.
slam_bridge_pidfile="${runtime_root}/slam_web_bridge.pid"
if [[ -f "$slam_bridge_pidfile" ]]; then
  old_slam_bridge_pid="$(<"$slam_bridge_pidfile")"
  old_slam_bridge_cmd="$(tr '\0' ' ' <"/proc/${old_slam_bridge_pid}/cmdline" 2>/dev/null || true)"
  if [[ "$old_slam_bridge_cmd" == *"scripts/go2w/plain_slam_web_bridge.py"* ]]; then
    kill "$old_slam_bridge_pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$old_slam_bridge_pid" 2>/dev/null || break
      sleep 0.1
    done
  fi
  rm -f "$slam_bridge_pidfile"
fi
if [[ "$MOCK" == 0 ]]; then
  setsid /usr/bin/python3 "${script_dir}/plain_slam_web_bridge.py" \
    --output "$GO2W_SLAM_MAP_SNAPSHOT" \
    >"${log_root}/slam_web_bridge.log" 2>&1 &
  printf '%s\n' "$!" >"$slam_bridge_pidfile"
fi
setsid "$conda_python" -m uvicorn app.manual_web_demo.web_server:app \
  --host "${host}" --port "${port}" \
  > "${log_root}/web_server.log" 2>&1 &
printf '%s\n' "$!" > "${runtime_root}/web.pid"

# ---- 8. Wait for the API to become ready --------------------------------- #
ready=0
for _ in $(seq 1 60); do
  if curl -fsS "http://${probe_host}:${port}/api/status" >/dev/null 2>&1; then
    ready=1
    break
  fi
  if ! kill -0 "$(<"${runtime_root}/web.pid")" 2>/dev/null; then
    break
  fi
  sleep 0.25
done

if [[ "$ready" != 1 ]]; then
  if [[ -f "$slam_bridge_pidfile" ]]; then
    kill "$(<"$slam_bridge_pidfile")" 2>/dev/null || true
  fi
  printf 'ERROR: Web server did not become ready. See %s/web_server.log\n' "${log_root}" >&2
  exit 1
fi

# ---- 9. Open the browser ------------------------------------------------- #
printf 'Go2-W Autonomous Semantic Search WebUI: http://%s:%s\n' "${host}" "${port}"
printf 'Search motion: %s\n' "$([[ "$ENABLE_MOTION" == 1 && "$MOCK" == 0 ]] && echo ENABLED || echo DISABLED/read-only)"
printf 'Realtime 3D map: %s\n' "$([[ "$MOCK" == 0 ]] && echo /go2w/slam/map_3d+aligned_scan || echo mock-disabled)"
if command -v xdg-open >/dev/null 2>&1; then
  xdg-open "http://${host}:${port}" >/dev/null 2>&1 || true
fi
