#!/usr/bin/env bash
set -euo pipefail

# Go2-W Manual WASD+QE Web Demo launcher (plan book §36).
#
#   bash scripts/go2w/start_manual_web_demo.sh                 # camera + LLM, motion disabled
#   bash scripts/go2w/start_manual_web_demo.sh --enable-motion # allow WASD+QE through /go2w/motion
#
# This script never starts Nav2, SemanticNavigation, the Pandar driver or Point-LIO.

ENABLE_MOTION=0
for arg in "$@"; do
  case "$arg" in
    --enable-motion) ENABLE_MOTION=1 ;;
    *) printf 'ERROR: unknown argument: %s\n' "$arg" >&2; exit 2 ;;
  esac
done

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
runtime_root="${project_root}/outputs/manual_web_demo/runtime"
log_root="${project_root}/outputs/manual_web_demo/logs"
mkdir -p "${runtime_root}" "${log_root}"

host="${MANUAL_DEMO_HOST:-127.0.0.1}"
port="${MANUAL_DEMO_PORT:-8765}"

# ---- 1. Network preflight ------------------------------------------- #
go2w_interface="${GO2W_INTERFACE:-}"
if [[ -z "$go2w_interface" ]]; then
  for candidate in enp6s0 enp3s0 enp4s0 enp5s0; do
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

# ---- 2. Source ROS environment for the worker subprocess ------------- #
# shellcheck source=setup_environment.sh
source "${script_dir}/setup_environment.sh"

# ---- 3. Camera bridge check (read-only) ------------------------------ #
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

# ---- 4. Motion stack check ------------------------------------------ #
if [[ "$ENABLE_MOTION" == 1 ]]; then
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
  if ! ros2 action info /go2w/motion 2>/dev/null \
    | grep -Eq 'Action servers:[[:space:]]*[1-9][0-9]*'; then
    printf 'WARNING: /go2w/motion Action server is not available; motion will be OFFLINE.\n' >&2
  fi
  for service in /go2w/arm /go2w/emergency_stop; do
    if ! service_server_available "$service"; then
      printf 'WARNING: %s service server is not available; motion will be OFFLINE.\n' "${service}" >&2
    fi
  done
  if [[ "${GO2W_AREA_CLEARED:-}" != "I_HAVE_CLEARED_THE_AREA" ]]; then
    printf 'Motion requested. You must keep a level, dry, obstacle-free area (>=2 m)\n' >&2
    printf 'and hold the remote emergency stop.\n' >&2
    read -r -p 'Type I_CONFIRM to authorize motion: ' answer
    if [[ "$answer" != "I_CONFIRM" ]]; then
      printf 'aborted.\n' >&2
      exit 2
    fi
  fi
fi

# ---- 5. Resolve the Conda Python for Web/LLM ------------------------- #
conda_python=""
for candidate in \
  "${GO2W_CONDA_PYTHON:-}" \
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

# ---- 6. Idempotently add FastAPI/uvicorn ----------------------------- #
if ! "$conda_python" -c 'import fastapi, uvicorn' >/dev/null 2>&1; then
  printf 'Installing fastapi + uvicorn into go2_robot_scene_demo...\n' >&2
  "$conda_python" -m pip install --quiet fastapi uvicorn
fi

# ---- 7. Launch the Web server (spawns the ROS worker) ---------------- #
cd "${project_root}"
export MANUAL_DEMO_RUNTIME_DIR="${MANUAL_DEMO_RUNTIME_DIR:-outputs/manual_web_demo/runtime}"
export MANUAL_DEMO_LOGS_DIR="${MANUAL_DEMO_LOGS_DIR:-outputs/manual_web_demo/logs}"
# 计划书 §2.3：地图快照、地图元数据、对象快照三个路径显式给出，写端与读取端
# 用的是同一份取值（本模式没有语义建图进程，所以对象快照通常不存在 —— 页面
# 显示地图、不显示历史归档物体）。
export GO2W_SLAM_MAP_SNAPSHOT="${GO2W_SLAM_MAP_SNAPSHOT:-${runtime_root}/slam_map_3d.json}"
export GO2W_SLAM_MAP_META="${GO2W_SLAM_MAP_META:-${runtime_root}/slam_map_meta.json}"
export GO2W_SEMANTIC_OBJECTS_SNAPSHOT="${GO2W_SEMANTIC_OBJECTS_SNAPSHOT:-${runtime_root}/semantic_objects.json}"
setsid "$conda_python" -m uvicorn app.manual_web_demo.web_server:app \
  --host "${host}" --port "${port}" \
  > "${log_root}/web_server.log" 2>&1 &
printf '%s\n' "$!" > "${runtime_root}/web.pid"

# ---- 7b. WebUI 地图/物体快照写入器 ------------------------------------- #
# 背景：WebUI 的 /api/slam/map3d **不订阅实时话题**，只读
#   outputs/manual_web_demo/runtime/slam_map_3d.json 这个快照文件；
# 该文件曾长期停留在 8 天前（内含 pose=2.28e11 的坏数据），这就是"WebUI 上地图重叠"的
# 直接来源。这里按 WebUI 要求的格式周期性写入**权威 SLAM 地图**。
#
# 计划书 §2.3：**物体不在这里写**。本模式没有语义建图进程，于是就没有对象源 ——
# 页面显示地图、不显示历史归档物体。旧写法是从 events.jsonl 里挑一份（glob 猜
# 最新的 ∪ 写死的 Dataset D 归档），2026-09-17 实测挑中的是 09-10 的归档，当天
# 搜索新定位的物体一个都没上屏。在线对象只由语义建图进程按 map_epoch 写。
# 关闭：GO2W_WEBUI_SNAPSHOT_WRITER=0
if [[ "${GO2W_WEBUI_SNAPSHOT_WRITER:-1}" == "1" ]]; then
  WEBUI_SNAPSHOT_PYTHON="/usr/bin/python3"
  # 同一份地图 JSON 只能有一个写进程：上一轮留下的写手（没走 stop 脚本就重启、
  # 或 stop 脚本早先没清这个 pid）会带着它自己的 map_epoch 继续写同一个文件，
  # 于是两个 epoch 交替上屏。与自主模式对地图桥的处理同一口径：先读 cmdline
  # 验明正身，不是这个脚本才敢停。
  if [[ -f "${runtime_root}/snapshot_writer.pid" ]]; then
    old_writer_pid="$(<"${runtime_root}/snapshot_writer.pid")"
    old_writer_cmd=""
    if [[ -n "$old_writer_pid" && -r "/proc/${old_writer_pid}/cmdline" ]]; then
      old_writer_cmd="$(tr '\0' ' ' <"/proc/${old_writer_pid}/cmdline" 2>/dev/null || true)"
    fi
    if [[ -n "$old_writer_pid" ]] && kill -0 "$old_writer_pid" 2>/dev/null \
      && [[ "$old_writer_cmd" == *"scripts/go2w/webui_map_snapshot_writer.py"* ]]; then
      printf 'Stopping previous map snapshot writer (pid %s) ...\n' "$old_writer_pid" >&2
      kill "$old_writer_pid" 2>/dev/null || true
      for _ in $(seq 1 20); do
        kill -0 "$old_writer_pid" 2>/dev/null || break
        sleep 0.1
      done
    fi
    rm -f "${runtime_root}/snapshot_writer.pid"
  fi
  PYTHONPATH="${HOME}/.local/lib/python3.10/site-packages${PYTHONPATH:+:${PYTHONPATH}}" \
  setsid "${WEBUI_SNAPSHOT_PYTHON}" "${script_dir}/webui_map_snapshot_writer.py" --ros-args \
    -p map_topic:="${GO2W_WEBUI_SNAPSHOT_MAP_TOPIC:-/go2w/slam/map_3d}" \
    -p max_points:="${GO2W_WEBUI_SNAPSHOT_MAX_POINTS:-60000}" \
    -p max_range_m:="${GO2W_WEBUI_SNAPSHOT_MAX_RANGE_M:-25.0}" \
    -p min_range_m:=1.0 \
    -p voxel_size_m:="${GO2W_WEBUI_SNAPSHOT_VOXEL_M:-0.05}" \
    -p min_z_m:="${GO2W_WEBUI_SNAPSHOT_MIN_Z_M:-0.15}" \
    -p max_z_m:="${GO2W_WEBUI_SNAPSHOT_MAX_Z_M:-2.80}" \
    -p trajectory_radius_m:="${GO2W_WEBUI_SNAPSHOT_TRAJ_RADIUS_M:-12.0}" \
    -p focus_margin_m:="${GO2W_WEBUI_SNAPSHOT_FOCUS_MARGIN_M:-3.0}" \
    -p snapshot_path:="${GO2W_SLAM_MAP_SNAPSHOT}" \
    -p meta_path:="${GO2W_SLAM_MAP_META}" \
    > "${log_root}/webui_map_snapshot_writer.log" 2>&1 &
  printf '%s\n' "$!" > "${runtime_root}/snapshot_writer.pid"
  printf '[OK] WebUI map snapshot writer (map=%s)\n' \
    "${GO2W_WEBUI_SNAPSHOT_MAP_TOPIC:-/go2w/slam/map_3d}"
fi

# ---- 8. Wait for the API to become ready ------------------------------ #
ready=0
for _ in $(seq 1 60); do
  if curl -fsS "http://${host}:${port}/api/status" >/dev/null 2>&1; then
    ready=1
    break
  fi
  if ! kill -0 "$(<"${runtime_root}/web.pid")" 2>/dev/null; then
    break
  fi
  sleep 0.25
done

if [[ "$ready" != 1 ]]; then
  printf 'ERROR: Web server did not become ready. See %s/web_server.log\n' "${log_root}" >&2
  exit 1
fi

# ---- 9. Open the browser --------------------------------------------- #
printf 'Go2-W Manual WASD+QE Demo: http://%s:%s\n' "${host}" "${port}"
if command -v xdg-open >/dev/null 2>&1; then
  xdg-open "http://${host}:${port}" >/dev/null 2>&1 || true
fi
