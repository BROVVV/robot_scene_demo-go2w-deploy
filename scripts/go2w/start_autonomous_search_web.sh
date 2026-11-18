#!/usr/bin/env bash
set -euo pipefail

# Go2-W Autonomous Semantic Search WebUI launcher (plan book §76-§79).
#
# ⚠ 这是旧入口（计划书 §9）。它只起 WebUI 与 ROS 侧车，不启动 Nav2 bringup、
# 坐标桥、速度平滑和 Collision Monitor——单独用它做 Nav2 自主运动，速度链是
# 断的。Nav2 自主搜索请用新入口：
#
#     bash scripts/go2w/start_nav2_autonomous_search.sh [--enable-motion]
#
# 新入口内部会调用本脚本，所以 WebUI 功能不会少；本脚本保留是为了兼容已有
# 的停止脚本与现场习惯。
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
#   bash scripts/go2w/start_autonomous_search_web.sh --supervised-legacy-search
#       # explicit, bounded legacy relative-motion trial; read-only unless
#       # --enable-autonomous-motion is also given and the preflight passes
#   bash scripts/go2w/start_autonomous_search_web.sh --supervised-legacy-search \
#       --legacy-profile field_large
#       # same path, large-field envelope: unlimited navigation goals, <=3 m
#       # per forward goal (still split into <=0.30 m gated segments). The
#       # envelope lives in scripts/go2w/autonomous_search_worker.py and can
#       # only be selected here, never widened by a WebUI request.
#
# This script never starts Nav2 / Point-LIO / the Pandar driver itself; it
# only launches the single FastAPI server (manual + autonomous console) and
# only stops processes the project itself owns.

ENABLE_MOTION=0
MOCK=0
WITH_PLAIN_SLAM=0
SUPERVISED_LEGACY_SEARCH=0
LEGACY_PROFILE="supervised_short"
DETECTOR="${AUTONOMOUS_SEARCH_DEFAULT_DETECTOR:-grounded_sam}"
OPEN_BROWSER=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --enable-autonomous-motion) ENABLE_MOTION=1 ;;
    --with-plain-slam) WITH_PLAIN_SLAM=1 ;;
    --supervised-legacy-search) SUPERVISED_LEGACY_SEARCH=1 ;;
    --legacy-profile)
      # 取走标志本身；值可能就是最后一个参数，此时 $# 归零、循环末尾那次
      # shift 会返回 1 —— 在 set -e 下表现为"无任何输出、exit 1"的静默失败。
      shift
      LEGACY_PROFILE="${1:-}"
      ;;
    --legacy-profile=*) LEGACY_PROFILE="${1#*=}" ;;
    --detector)
      shift
      DETECTOR="${1:-}"
      ;;
    --detector=*) DETECTOR="${1#*=}" ;;
    --no-browser) OPEN_BROWSER=0 ;;
    --mock) MOCK=1 ;;
    *) printf 'ERROR: unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
  shift || true
done
if [[ "$DETECTOR" != "grounded_sam" && "$DETECTOR" != "yoloe_seg" ]]; then
  printf 'ERROR: --detector 只接受 grounded_sam 或 yoloe_seg。\n' >&2
  exit 2
fi
if [[ "$MOCK" == 1 && "$SUPERVISED_LEGACY_SEARCH" == 1 ]]; then
  printf 'ERROR: --mock and --supervised-legacy-search are mutually exclusive.\n' >&2
  exit 2
fi
# 档位只在旧受监督路径上有消费方；单独传它而没有 --supervised-legacy-search
# 会让人以为"参数被接受了"，这正是本项目反复踩的坑，所以宁可报错。
if [[ -z "$LEGACY_PROFILE" ]]; then
  printf 'ERROR: --legacy-profile 需要跟一个档位名。\n' >&2
  exit 2
fi
if [[ "$LEGACY_PROFILE" != "supervised_short" && "$SUPERVISED_LEGACY_SEARCH" != 1 ]]; then
  printf 'ERROR: --legacy-profile 只在 --supervised-legacy-search 下有意义。\n' >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"

# 运动包线的唯一来源是 worker 的档位表。这里读它、打印它、导出它，
# 不在 shell 里再抄一份数字（抄一份就会有一天和 Python 不一致）。
legacy_profile_name="$(
  cd "${project_root}" && /usr/bin/python3 -c '
import sys
sys.path.insert(0, "scripts/go2w")
from autonomous_search_worker import legacy_motion_profile
try:
    name, _ = legacy_motion_profile(sys.argv[1])
except ValueError as exc:
    sys.exit(str(exc))
print(name)
' "${LEGACY_PROFILE}"
)" || {
  printf 'ERROR: 未知的 --legacy-profile %s（见 scripts/go2w/autonomous_search_worker.py 的档位表）。\n' \
    "${LEGACY_PROFILE}" >&2
  exit 2
}
legacy_profile_bounds="$(
  cd "${project_root}" && /usr/bin/python3 -c '
import sys
sys.path.insert(0, "scripts/go2w")
from autonomous_search_worker import legacy_motion_profile
_, profile = legacy_motion_profile(sys.argv[1])
b = profile["bounds"]
print(
    "导航目标上限 %s 个；总时限 %s；规划轮数上限 %s；单次转向 ≤%s°；"
    "单次前进请求 ≤%s m；前进实测上限 %s m；起点半径 %s m（0=不限）"
    % (
        b["max_navigation_goals"] or "不限",
        (str(b["max_seconds"]) + " s") if b["max_seconds"] else "不限",
        b["max_planning_cycles"] or "不限",
        b["max_turn_deg"],
        b["forward_step_m"],
        b["max_observed_forward_m"],
        b["max_radius"],
    )
)
' "${legacy_profile_name}"
)"
runtime_root="${project_root}/outputs/autonomous_search/runtime"
log_root="${project_root}/outputs/autonomous_search/logs"
camera_runtime_root="${MANUAL_DEMO_RUNTIME_DIR:-${runtime_root}}"
camera_log_root="${MANUAL_DEMO_LOGS_DIR:-${log_root}}"
mkdir -p "${runtime_root}" "${log_root}"

# Load provider/runtime settings for the WebUI and its search worker without
# printing the values.  The VLM daemon remains the preferred credential
# boundary, but direct worker fallback also needs the same .env contract.
source "${script_dir}/load_env_defaults.sh"
go2w_load_env_defaults "${project_root}/.env"
# Keep external API traffic on the configured HTTP CONNECT forwarder.  The
# inherited socks5h ALL_PROXY alias is not understood by the pinned httpx
# runtime and would make every VLM request fail before it reaches the network.
if [[ "${ALL_PROXY:-}" == socks5h://* && \
      -n "${HTTPS_PROXY:-${https_proxy:-}}" ]]; then
  unset ALL_PROXY all_proxy
fi

# The console is an operator-facing LAN UI for the robot deployment.  Bind to
# all interfaces by default so the browser on the control workstation can
# reach 192.168.123.18; operators can still opt into loopback explicitly.
host="${AUTONOMOUS_SEARCH_HOST:-0.0.0.0}"
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

# A stale pid file must never make the launcher create a second WebUI on the
# same port.  If the port is still serving after the verified project-owned
# pidfile cleanup above, stop and let the operator inspect the existing
# process rather than interleaving two runtimes and two ROS workers.
if curl -fsS "http://${probe_host}:${port}/api/status" >/dev/null 2>&1; then
  printf 'ERROR: port %s is still occupied by an existing WebUI process.\n' "${port}" >&2
  printf '%s\n' 'Refusing to start a duplicate server; stop that exact process first.' >&2
  exit 1
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

# The WebUI's real search worker uses the local Unix-socket VLM daemon.  Make
# the launcher self-contained and idempotent so a stale/missing daemon cannot
# silently force every Quick request into the slower subprocess fallback.
if [[ "$MOCK" == 0 ]]; then
  if ! bash "${script_dir}/start_vlm_daemon.sh"; then
    printf 'WARNING: VLM daemon did not start; search may report LLM unavailable.\n' >&2
  fi
fi

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

# The robot-local RealSense HTTP stream is the reliable WebUI camera source.
# A ROS compressed-image alias is still accepted by the worker when available,
# but the HTTP capture prevents a cross-host JPEG burst from starving it.
camera_http_url="${MANUAL_DEMO_CAMERA_HTTP_URL:-}"
if [[ -z "$camera_http_url" ]]; then
  for camera_base in "http://127.0.0.1:8080" "http://192.168.123.18:8080"; do
    if curl --noproxy '*' -fsS --max-time 2 "${camera_base}/health" >/dev/null 2>&1; then
      camera_http_url="${camera_base}/color"
      break
    fi
  done
fi
# Capture ownership stays with this launcher even when image paths are customized.
camera_http_pidfile="${runtime_root}/camera_http_capture.pid"
if [[ "$MOCK" == 0 && -n "$camera_http_url" ]]; then
  mkdir -p "${camera_runtime_root}" "${camera_log_root}"
  camera_capture_alive=0
  if [[ -f "$camera_http_pidfile" ]]; then
    camera_capture_pid="$(<"$camera_http_pidfile")"
    if kill -0 "$camera_capture_pid" 2>/dev/null \
      && [[ "$(tr '\0' ' ' <"/proc/${camera_capture_pid}/cmdline" 2>/dev/null || true)" == *"robot_camera_http_capture.py"* ]]; then
      camera_capture_alive=1
    fi
  fi
  if [[ "$camera_capture_alive" != 1 ]]; then
    setsid /usr/bin/python3 "${script_dir}/robot_camera_http_capture.py" \
      --url "$camera_http_url" \
      --latest "${camera_runtime_root}/latest.jpg" \
      --status "${camera_runtime_root}/camera_status.json" \
      >"${camera_log_root}/camera_http_capture.log" 2>&1 &
    printf '%s\n' "$!" >"$camera_http_pidfile"
  fi
  printf '[OK] Robot HTTP camera capture: %s\n' "$camera_http_url"
else
  printf '%s\n' '[INFO] Robot HTTP camera capture unavailable; using ROS camera path only.'
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
  # Go2-W deployments may expose the project-authoritative odom through the
  # wheel-odom executable or through the SportModeState-backed bridge used
  # when /lf/sportmodestate is the only available robot state source.  Both
  # are valid, but exactly one must own the fused odom topic.
  odom_processes="$(ps -eo args= | awk '
    /(^|[[:space:]])go2w_[w]heel_odom([[:space:]]|$)/ { count++; next }
    /scripts\/go2w\/sport_[o]dom_bridge[.]py([[:space:]]|$)/ { count++ }
    END { print count+0 }
  ')"
  odom_processes_valid=0
  if [[ "$odom_processes" == 0 || "$odom_processes" == 1 ]]; then
    odom_processes_valid=1
  fi
  if [[ "$odom_publishers" != 1 || "$odom_processes_valid" != 1 ]]; then
    printf 'ERROR: %s requires exactly one publisher (ROS graph=%s, wheel odom processes=%s).\n' \
      "$odom_topic" "${odom_publishers:-unknown}" "$odom_processes" >&2
    printf '%s\n' "$odom_info" >&2
    printf 'Duplicate odometry origins make motion verification and topology unsafe.\n' >&2
    exit 2
  fi
  if [[ "$odom_processes" == 0 ]]; then
    printf '%s\n' 'Using the single network wheel-odom publisher from the control host.' >&2
  fi
  motion_action_info="$(timeout 5 ros2 action info /go2w/motion 2>&1 || true)"
  motion_action_servers="$(awk '/Action servers:/ { print $3; exit }' <<<"$motion_action_info")"
  motion_action_processes="$(ps -eo args= | awk '{ exe=$1; sub(/^.*\//, "", exe); if (exe == "go2w_motion_action_server") count++ } END { print count+0 }')"
  # 与上面 /go2w/odom/fused 同一口径：运动栈允许跑在控制主机上（本机 0 个进程），
  # ROS graph 的 Action servers 才是唯一性权威；本机出现第 2 个才是重复启动。
  if [[ "$motion_action_servers" != 1 || "$motion_action_processes" -gt 1 ]]; then
    printf 'ERROR: /go2w/motion requires exactly one server (ROS graph=%s, local OS processes=%s).\n' \
      "${motion_action_servers:-unknown}" "$motion_action_processes" >&2
    printf '%s\n' "$motion_action_info" >&2
    printf 'Stop duplicate/stale go2w motion launch processes before autonomous motion.\n' >&2
    exit 2
  fi
  if [[ "$motion_action_processes" == 0 ]]; then
    printf '%s\n' 'Using the single /go2w/motion action server from the control host.' >&2
  fi
  for service in /go2w/arm /go2w/emergency_stop; do
    if ! service_server_available "$service"; then
      printf 'ERROR: %s service server is not available; autonomous motion cannot start.\n' "$service" >&2
      exit 2
    fi
  done
  if [[ "$SUPERVISED_LEGACY_SEARCH" == 1 ]]; then
    printf 'Supervised legacy search requested (profile: %s).\n' "$legacy_profile_name" >&2
    printf 'Two observers, physical emergency stop in hand.\n' >&2
    printf 'This profile allows: %s.\n' "$legacy_profile_bounds" >&2
    printf 'Physical motion command attempt cap: %s (0=unlimited), including corrective turns.\n' \
      "$(cd "$project_root" && /usr/bin/python3 -c \
        'import sys; sys.path.insert(0, "scripts/go2w"); from autonomous_search_worker import legacy_motion_profile; print(legacy_motion_profile(sys.argv[1])[1]["bounds"]["max_motion_commands"])' \
        "$legacy_profile_name")" >&2
    if [[ "$legacy_profile_name" == "supervised_short" ]]; then
      printf 'Clear forward path >=1 m, side clearance >=0.4 m.\n' >&2
    else
      printf 'Clear in all directions out to the planning radius; the dog is NOT\n' >&2
      printf 'confined to the starting area in this profile.\n' >&2
    fi
    read -r -p 'Type I_CONFIRM to authorize this trial: ' answer
  elif [[ "${GO2W_AREA_CLEARED:-}" != "I_HAVE_CLEARED_THE_AREA" ]]; then
    printf 'Autonomous motion requested. You must keep a level, dry, obstacle-free\n' >&2
    printf 'area (>=2 m) and hold the remote emergency stop. Turns are capped at\n' >&2
    printf 'Map-frame targets <=3 m; each target may be shorter. Live collision/lease/stop gates remain active.\n' >&2
    read -r -p 'Type I_CONFIRM to authorize autonomous motion: ' answer
  else
    answer="I_CONFIRM"
  fi
  if [[ "$answer" != "I_CONFIRM" ]]; then
    printf 'aborted.\n' >&2
    exit 2
  fi
fi

# ---- 5. Resolve the Conda Python for Web/LLM ----------------------------- #
conda_python=""
for candidate in \
  "${GO2W_CONDA_PYTHON:-}" \
  "${project_root}/.runtime_venv/bin/python" \
  "${project_root}/.venv/bin/python" \
  /usr/bin/python3 \
  "$HOME/anaconda3/envs/go2_robot_scene_demo/bin/python" \
  "$HOME/miniconda3/envs/go2_robot_scene_demo/bin/python" \
  /home/mxt/anaconda3/envs/go2_robot_scene_demo/bin/python \
  /home/brov/miniconda3/envs/go2_robot_scene_demo/bin/python \
  "${CONDA_PREFIX:-}/bin/python"; do
  if [[ -x "$candidate" ]] && "$candidate" -c 'import sys' >/dev/null 2>&1; then
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
  "$conda_python" -m pip install --quiet 'fastapi<0.116' 'uvicorn<0.30' 'websockets<13'
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
export MANUAL_DEMO_RUNTIME_DIR="${camera_runtime_root}"
export MANUAL_DEMO_LOGS_DIR="${camera_log_root}"
export AUTONOMOUS_SEARCH_RUNTIME_DIR="${runtime_root}"
export AUTONOMOUS_SEARCH_LOGS_DIR="${log_root}"
export GO2W_SLAM_MAP_SNAPSHOT="${runtime_root}/slam_map_3d.json"
# 计划书 §2.3：三维对象与地图靠 map_epoch 配对，三个路径显式给出 —— 桥、对象
# 生产者（搜索 worker）和读取端都用这里的同一份取值，不靠 glob/mtime 猜。
export GO2W_SLAM_MAP_META="${runtime_root}/slam_map_meta.json"
export GO2W_SEMANTIC_OBJECTS_SNAPSHOT="${runtime_root}/semantic_objects.json"
export GO2W_SLAM_RESET_MARKER="${runtime_root}/slam_reset.marker"
export AUTONOMOUS_SEARCH_DEFAULT_DETECTOR="${DETECTOR}"
export GO2W_WEBUI_MODE="${GO2W_WEBUI_MODE:-autonomous_search}"
export GO2W_ACTIVE_DETECTOR="${DETECTOR}"
export GO2W_RUNTIME_ADMIN_ENABLED="${GO2W_RUNTIME_ADMIN_ENABLED:-1}"
# ---- 7a. Search replay (read-only capture + background encode) ----------- #
# The collector is a separate /usr/bin/python3 ROS process launched by the Web
# process.  It only subscribes to /map, /go2w/odom/fused, localization status,
# TF and the D435 MJPEG stream — it publishes nothing and never authorizes
# motion.  Mock mode records nothing by default: synthetic data must not end
# up in a real replay directory.
if [[ "$MOCK" == 1 ]]; then
  export GO2W_SEARCH_REPLAY_ENABLED="${GO2W_SEARCH_REPLAY_ENABLED:-0}"
else
  export GO2W_SEARCH_REPLAY_ENABLED="${GO2W_SEARCH_REPLAY_ENABLED:-1}"
fi
# 采集器必须用 ROS 系统解释器（Humble 3.10）；Web 进程自身不 import rclpy。
export GO2W_SEARCH_REPLAY_PYTHON="${GO2W_SEARCH_REPLAY_PYTHON:-/usr/bin/python3}"
export GO2W_SEARCH_REPLAY_ROOT="${GO2W_SEARCH_REPLAY_ROOT:-outputs/search_replay}"
export GO2W_SEARCH_REPLAY_CAMERA_URL="${GO2W_SEARCH_REPLAY_CAMERA_URL:-http://192.168.123.18:8080/color}"
# Search worker interpreter is auto-detected unless the unified bootstrap
# environment above has already proved it can import both ROS and the project.
if [[ "$MOCK" == 1 ]]; then
  export AUTONOMOUS_SEARCH_DEFAULT_BACKEND="mock"
  export AUTONOMOUS_SEARCH_ENABLE_AUTONOMOUS_MOTION="0"
else
  # Keep Nav2 as the normal path. The historical relative-motion path is a
  # separately named, bounded operator-supervised acceptance mode.
  if [[ "$SUPERVISED_LEGACY_SEARCH" == 1 ]]; then
    export AUTONOMOUS_SEARCH_DEFAULT_BACKEND="go2w_experimental"
    export GO2W_ALLOW_SUPERVISED_LEGACY_SEARCH="1"
    # 运动包线由 worker 的档位表唯一决定；这里只传档位名。
    export GO2W_LEGACY_SEARCH_MOTION_PROFILE="${legacy_profile_name}"
  else
    export AUTONOMOUS_SEARCH_DEFAULT_BACKEND="nav2"
    export GO2W_ALLOW_SUPERVISED_LEGACY_SEARCH="0"
  fi
  export AUTONOMOUS_SEARCH_ENABLE_AUTONOMOUS_MOTION="${ENABLE_MOTION}"
  # Carry the repository's existing operator-supervised experiment profile
  # into every WebUI start request so it cannot be dropped at the worker
  # boundary.  Motion still requires the explicit launcher opt-in above.
  export AUTONOMOUS_SEARCH_OPERATOR_SUPERVISED="$([[ "$ENABLE_MOTION" == 1 ]] && echo 1 || echo 0)"
  if [[ "$SUPERVISED_LEGACY_SEARCH" == 1 ]]; then
    export AUTONOMOUS_SEARCH_SPATIAL_PROVIDER="plain_slam"
  else
    export AUTONOMOUS_SEARCH_SPATIAL_PROVIDER="nav2"
    export GO2W_NAV2_SEARCH_PROFILE="field_large"
  fi
fi

# A ROS-system-Python sidecar decimates the PointCloud2 stream into an atomic
# JSON snapshot.  The FastAPI process remains free of rclpy/ABI coupling.
slam_bridge_pidfile="${runtime_root}/slam_web_bridge.pid"
if [[ -f "$slam_bridge_pidfile" ]]; then
  old_slam_bridge_pid="$(<"$slam_bridge_pidfile")"
  old_slam_bridge_cmd=""
  if [[ -r "/proc/${old_slam_bridge_pid}/cmdline" ]]; then
    old_slam_bridge_cmd="$(tr '\0' ' ' <"/proc/${old_slam_bridge_pid}/cmdline" 2>/dev/null || true)"
  fi
  if [[ "$old_slam_bridge_cmd" == *"scripts/go2w/plain_slam_web_bridge.py"* || "$old_slam_bridge_cmd" == *"scripts/go2w/nav2_web_map_bridge.py"* || "$old_slam_bridge_cmd" == *"scripts/go2w/map_epoch_meta_bridge.py"* ]]; then
    kill "$old_slam_bridge_pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$old_slam_bridge_pid" 2>/dev/null || break
      sleep 0.1
    done
  fi
  rm -f "$slam_bridge_pidfile"
fi
if [[ "$MOCK" == 0 ]]; then
  if [[ "${GO2W_WEB_POINTCLOUD_BRIDGE_ENABLED:-1}" == "0" && \
        ( "${GO2W_WEB_2D_MAP_ENABLED:-0}" != "1" || "$SUPERVISED_LEGACY_SEARCH" != 0 ) ]]; then
    setsid /usr/bin/python3 "${script_dir}/map_epoch_meta_bridge.py" \
      --meta-output "$GO2W_SLAM_MAP_META" \
      >"${log_root}/slam_web_bridge.log" 2>&1 &
  elif [[ "$SUPERVISED_LEGACY_SEARCH" == 0 ]]; then
    setsid /usr/bin/python3 "${script_dir}/nav2_web_map_bridge.py" \
      --output "$GO2W_SLAM_MAP_SNAPSHOT" --meta-output "$GO2W_SLAM_MAP_META" \
      >"${log_root}/slam_web_bridge.log" 2>&1 &
  else
  # 永久地图只来自权威 SLAM 地图（pslam_map）；aligned_scan 只做调试预览层。
  setsid /usr/bin/python3 "${script_dir}/plain_slam_web_bridge.py" \
    --output "$GO2W_SLAM_MAP_SNAPSHOT" \
    --meta-output "$GO2W_SLAM_MAP_META" \
    --map-topic "${GO2W_SLAM_WEB_MAP_TOPIC:-/go2w/slam/web_map}" \
    --map-info-topic "${GO2W_SLAM_WEB_MAP_INFO_TOPIC:-/go2w/slam/web_map_info}" \
    --map-frame "pslam_map" \
    --scan-topic "${GO2W_SLAM_WEB_SCAN_TOPIC:-/go2w/slam/web_scan}" \
    --scan-frame "pslam_odom" \
    --odom-topic "${GO2W_SLAM_ODOM_TOPIC:-/go2w/slam/odom_base}" \
    --motion-odom-topic "${GO2W_MOTION_ODOM_TOPIC:-/go2w/odom/fused}" \
    --require-motion-odom \
    --voxel-size "${GO2W_SLAM_WEB_VOXEL_SIZE:-0.12}" \
    --max-global-voxels "${GO2W_SLAM_MAX_GLOBAL_VOXELS:-300000}" \
    --max-web-points "${GO2W_SLAM_MAX_WEB_POINTS:-50000}" \
    --reset-marker "$GO2W_SLAM_RESET_MARKER" \
    >"${log_root}/slam_web_bridge.log" 2>&1 &
  fi
  printf '%s\n' "$!" >"$slam_bridge_pidfile"
fi
setsid "$conda_python" -m uvicorn app.manual_web_demo.web_server:app \
  --host "${host}" --port "${port}" --timeout-graceful-shutdown 5 \
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
printf 'Detector route: %s\n' "${DETECTOR}"
if [[ "$SUPERVISED_LEGACY_SEARCH" == 1 ]]; then
  printf 'Search profile: %s\nMotion envelope: %s\n' \
    "$legacy_profile_name" "$legacy_profile_bounds"
fi
if [[ "$MOCK" == 0 && "$SUPERVISED_LEGACY_SEARCH" == 0 ]]; then
  if [[ -n "${GO2W_NAV2_TRIAL_LEDGER:-}" ]]; then
    printf '%s\n' 'Navigation map: /map; bounded trial: limits and remaining allowance come from the selected fixed ledger (not unrestricted search).'
  else
    printf '%s\n' 'Navigation map: /map (2D occupancy, frame=map); target <=3 m, no session time/step cap.'
  fi
else
  printf 'Realtime 3D map: %s\n' "$([[ "$MOCK" == 0 ]] && echo /go2w/slam/map_3d+aligned_scan || echo mock-disabled)"
fi
if [[ "$OPEN_BROWSER" == 1 ]] && command -v xdg-open >/dev/null 2>&1; then
  xdg-open "http://${host}:${port}" >/dev/null 2>&1 || true
fi
