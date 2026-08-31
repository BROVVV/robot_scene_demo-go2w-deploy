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
    kill -TERM "${pid}" 2>/dev/null || true
    sleep 1
    kill -KILL "${pid}" 2>/dev/null || true
  fi
  rm -f "${pid_file}"
fi
rm -f "${socket_path}"
echo "VLM daemon stopped"
