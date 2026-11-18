#!/usr/bin/env bash
set -euo pipefail

# Stop the Go2-W Autonomous Semantic Search WebUI (plan book §76).
# Only stops the project-owned web.pid process (and, through the server
# shutdown path, its ROS worker + search worker).  Never kills other
# ROS / user processes.

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
runtime_root="${project_root}/outputs/autonomous_search/runtime"
pidfile="${runtime_root}/web.pid"

# Search has its own capture PID record; never consume the manual demo's record.
# Exact executable identity and bounded TERM waiting are shared with host stop.
if [[ -f "${runtime_root}/camera_http_capture.pid" ]]; then
  /usr/bin/python3 "${script_dir}/stop_owned_host_processes.py" "$project_root" \
    --camera-pidfile "${runtime_root}/camera_http_capture.pid"
fi

# ---- 本模式自己的 ROS 地图桥 ---------------------------------------------- #
# 桥是 setsid 起的**独立进程组**，不在 Web 服务器的进程组里，所以上面那套
# "停 web.pid 的进程组"碰不到它。不在这里停掉，它会继续用它启动时那个
# map_epoch 写 slam_map_3d.json —— 看起来 WebUI 停了、地图却还在动；
# 下次启动自主模式会出现两个写手互相覆盖，正是计划书 §0.3 禁止的。
#
# HTTP capture now has a separate per-mode record and is stopped above.
slam_bridge_pidfile="${runtime_root}/slam_web_bridge.pid"
if [[ -f "$slam_bridge_pidfile" ]]; then
  bridge_pid="$(tr -cd '0-9' <"$slam_bridge_pidfile")"
  bridge_cmd=""
  if [[ -n "$bridge_pid" && -r "/proc/${bridge_pid}/cmdline" ]]; then
    bridge_cmd="$(tr '\0' ' ' <"/proc/${bridge_pid}/cmdline" 2>/dev/null || true)"
  fi
  if [[ -n "$bridge_pid" ]] && kill -0 "$bridge_pid" 2>/dev/null \
    && [[ "$bridge_cmd" == *"scripts/go2w/plain_slam_web_bridge.py"* || "$bridge_cmd" == *"scripts/go2w/nav2_web_map_bridge.py"* || "$bridge_cmd" == *"scripts/go2w/map_epoch_meta_bridge.py"* ]]; then
    printf 'Stopping SLAM web bridge (pid %s)...\n' "$bridge_pid" >&2
    kill -TERM "$bridge_pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$bridge_pid" 2>/dev/null || break
      sleep 0.25
    done
    kill -0 "$bridge_pid" 2>/dev/null && kill -KILL "$bridge_pid" 2>/dev/null || true
  elif [[ -n "$bridge_pid" ]] && kill -0 "$bridge_pid" 2>/dev/null; then
    printf 'WARNING: refusing to stop pid %s; it is not the project SLAM bridge.\n' \
      "$bridge_pid" >&2
  fi
  rm -f "$slam_bridge_pidfile"
fi

if [[ ! -f "$pidfile" ]]; then
  printf 'No web.pid at %s; nothing to stop.\n' "$pidfile" >&2
  exit 0
fi

pid="$(<"$pidfile")"
if ! kill -0 "$pid" 2>/dev/null; then
  printf 'Web server pid %s is not running.\n' "$pid" >&2
  rm -f "$pidfile"
  exit 0
fi

printf 'Stopping Go2-W Autonomous Search WebUI (pid %s)...\n' "$pid" >&2
cmdline=""
if [[ -r "/proc/${pid}/cmdline" ]]; then
  cmdline="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
fi
if [[ "$cmdline" != *"uvicorn app.manual_web_demo.web_server:app"* ]]; then
  printf 'ERROR: refusing to stop pid %s; it is not the project WebUI.\n' "$pid" >&2
  exit 2
fi
pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]')"
if [[ "$pgid" == "$pid" ]]; then
  # The launcher uses setsid, so the WebUI and its owned ROS/search workers
  # share this exact process group.  Stop that group, never an unrelated PID.
  kill -INT -- "-$pgid" 2>/dev/null || true
else
  kill -INT "$pid" 2>/dev/null || true
fi
for _ in $(seq 1 40); do
  if [[ "$pgid" == "$pid" ]]; then
    alive="$(kill -0 -- "-$pgid" 2>/dev/null && echo 1 || echo 0)"
  else
    alive="$(kill -0 "$pid" 2>/dev/null && echo 1 || echo 0)"
  fi
  if [[ "$alive" == 0 ]]; then
    rm -f "$pidfile"
    printf 'Stopped.\n' >&2
    exit 0
  fi
  sleep 0.25
done
if [[ "$pgid" == "$pid" ]] && kill -0 -- "-$pgid" 2>/dev/null; then
  kill -TERM -- "-$pgid" 2>/dev/null || true
elif kill -0 "$pid" 2>/dev/null; then
  kill -TERM "$pid" 2>/dev/null || true
fi
for _ in $(seq 1 20); do
  if [[ "$pgid" == "$pid" ]]; then
    kill -0 -- "-$pgid" 2>/dev/null || { rm -f "$pidfile"; printf 'Stopped.\n' >&2; exit 0; }
  else
    kill -0 "$pid" 2>/dev/null || { rm -f "$pidfile"; printf 'Stopped.\n' >&2; exit 0; }
  fi
  sleep 0.25
done
if [[ "$pgid" == "$pid" ]] && kill -0 -- "-$pgid" 2>/dev/null; then
  # The server can be inside a blocking camera/LLM stream and ignore both
  # graceful signals.  This is still restricted to the verified project-owned
  # process group, so a stale UI cannot block a clean restart indefinitely.
  kill -KILL -- "-$pgid" 2>/dev/null || true
elif kill -0 "$pid" 2>/dev/null; then
  kill -KILL "$pid" 2>/dev/null || true
fi
sleep 0.2
if [[ "$pgid" == "$pid" ]] && ! kill -0 -- "-$pgid" 2>/dev/null; then
  rm -f "$pidfile"
  printf 'Stopped (forced).\n' >&2
  exit 0
fi
if [[ "$pgid" != "$pid" ]] && ! kill -0 "$pid" 2>/dev/null; then
  rm -f "$pidfile"
  printf 'Stopped (forced).\n' >&2
  exit 0
fi
printf 'WARNING: WebUI process group %s did not exit; left as-is.\n' "$pid" >&2
exit 1
