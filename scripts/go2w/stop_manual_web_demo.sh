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

rm -f "${runtime_root}/web.pid" "${runtime_root}/worker.pid" "${runtime_root}/camera_http_capture.pid"
printf 'Manual web demo stopped. Camera bridge and other ROS nodes untouched.\n'
