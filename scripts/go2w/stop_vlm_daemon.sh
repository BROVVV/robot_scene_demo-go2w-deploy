#!/usr/bin/env bash
# Stop the SiliconFlow VLM daemon gracefully.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
pid_file="${project_root}/runtime/go2w/pids/siliconflow_vlm.pid"
socket_path="${project_root}/runtime/go2w/siliconflow_vlm.sock"

if [[ -f "${pid_file}" ]]; then
  pid="$(tr -cd '0-9' < "${pid_file}")"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    cmdline=""
    [[ -r "/proc/${pid}/cmdline" ]] && cmdline="$(tr '\0' ' ' <"/proc/${pid}/cmdline")"
    if [[ "${cmdline}" != *'siliconflow_vision_daemon.py'* ]]; then
      echo "ERROR: pid ${pid} is not the SiliconFlow daemon; refusing to kill: ${cmdline}" >&2
      exit 3
    fi
    kill -TERM "${pid}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "${pid}" 2>/dev/null; then
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  fi
  rm -f "${pid_file}"
fi
rm -f "${socket_path}"
echo "VLM daemon stopped"
