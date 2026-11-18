#!/usr/bin/env bash
set -euo pipefail

# Go2-W Manual WASD+QE Web Demo stop script (plan book §40).
#
# Shuts down the Web server, asks the ROS worker to stop/cancel any active
# goal, and never touches the camera bridge or other ROS nodes it does not own.

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
runtime_root="${project_root}/outputs/manual_web_demo/runtime"
mkdir -p "${runtime_root}"

web_pid="$(cat "${runtime_root}/web.pid" 2>/dev/null || true)"
worker_pid="$(cat "${runtime_root}/worker.pid" 2>/dev/null || true)"
camera_capture_pid="$(cat "${runtime_root}/camera_http_capture.pid" 2>/dev/null || true)"
snapshot_writer_pid="$(cat "${runtime_root}/snapshot_writer.pid" 2>/dev/null || true)"

# 定向停止：先验明正身再杀。pid 文件里的数字可能早已被系统回收给别的进程 ——
# 不校验 cmdline 就可能杀掉无关进程（本仓 app/manual_web_demo 的启动脚本同样
# 按这个口径写）。$2 是需要出现在 cmdline 里的关键字。
stop_verified() {
  local name="$1" pid="$2" expect="$3"
  [[ -n "${pid}" ]] || return 0
  kill -0 "${pid}" 2>/dev/null || return 0
  if [[ ! -r "/proc/${pid}/cmdline" ]] \
    || [[ "$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)" != *"${expect}"* ]]; then
    printf 'WARNING: %s pid %s 的 cmdline 不含 %s，判为已回收，不杀。\n' \
      "${name}" "${pid}" "${expect}" >&2
    return 0
  fi
  kill -TERM "${pid}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "${pid}" 2>/dev/null || return 0
    sleep 0.2
  done
  kill -KILL "${pid}" 2>/dev/null || true
}

# 1. Graceful Web server shutdown: the FastAPI lifespan exit stops the ROS
#    worker (disable control -> cancel/estop active goal -> worker shutdown).
if [[ -n "${web_pid}" ]] && kill -0 "${web_pid}" 2>/dev/null; then
  kill -TERM "${web_pid}" 2>/dev/null || true
  for _ in $(seq 1 40); do
    kill -0 "${web_pid}" 2>/dev/null || break
    sleep 0.2
  done
  kill -KILL "${web_pid}" 2>/dev/null || true
fi

# 2. Belt-and-braces: make sure the worker process is gone.
if [[ -n "${worker_pid}" ]] && kill -0 "${worker_pid}" 2>/dev/null; then
  kill -TERM "${worker_pid}" 2>/dev/null || true
  sleep 1
  if kill -0 "${worker_pid}" 2>/dev/null; then
    kill -KILL "${worker_pid}" 2>/dev/null || true
  fi
fi

# 3. Stop the optional robot-local HTTP camera capture owned by the WebUI.
if [[ -n "${camera_capture_pid}" ]] && kill -0 "${camera_capture_pid}" 2>/dev/null; then
  kill -TERM "${camera_capture_pid}" 2>/dev/null || true
  sleep 1
  kill -KILL "${camera_capture_pid}" 2>/dev/null || true
fi

# 4. 地图快照写入器。它是 setsid 起的**独立进程组**，既不在 Web 服务器的进程组里，
#    也不会随 Web 服务器退出 —— 不在这里停掉，它会带着自己的 map_epoch 一直写
#    outputs/manual_web_demo/runtime/slam_map_3d.json。演示台停了、地图还在动，
#    下次启动本模式就会变成两个写手互相覆盖。
stop_verified 'snapshot writer' "${snapshot_writer_pid}" 'webui_map_snapshot_writer'

rm -f "${runtime_root}/web.pid" "${runtime_root}/worker.pid" \
      "${runtime_root}/camera_http_capture.pid" "${runtime_root}/snapshot_writer.pid"
printf 'Manual web demo stopped. Camera bridge and other ROS nodes untouched.\n'
