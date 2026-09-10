#!/usr/bin/env bash
# Start the long-running SiliconFlow VLM daemon (Conda Python).
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
# Keep the provider credential in the daemon process, where the API client
# actually lives.  The ROS worker can then use the Unix socket without
# receiving the secret in its own environment.
if [[ -f "${project_root}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${project_root}/.env"
  set +a
fi
conda_python="${SILICONFLOW_PYTHON:-${GO2W_CONDA_PYTHON:-/home/brov/miniconda3/envs/go2_robot_scene_demo/bin/python}}"
if [[ ! -x "${conda_python}" && -x "${project_root}/.venv/bin/python" ]]; then
  conda_python="${project_root}/.venv/bin/python"
fi
socket_path="${project_root}/runtime/go2w/siliconflow_vlm.sock"
pid_file="${project_root}/runtime/go2w/pids/siliconflow_vlm.pid"

mkdir -p "$(dirname "${pid_file}")" "$(dirname "${socket_path}")" "${project_root}/runtime/go2w/sessions"

if [[ -S "${socket_path}" ]]; then
  old_pid=""
  if [[ -f "${pid_file}" ]]; then
    old_pid="$(tr -cd '0-9' < "${pid_file}")"
  fi
  if [[ -n "${old_pid}" ]] && kill -0 "${old_pid}" 2>/dev/null; then
    echo "VLM daemon already running (pid ${old_pid})" >&2
    exit 0
  fi
  # Stale socket: remove it so the daemon can bind again.
  rm -f "${socket_path}"
  rm -f "${pid_file}"
fi

if [[ -f "${pid_file}" ]]; then
  old_pid="$(tr -cd '0-9' < "${pid_file}")"
  if [[ -n "${old_pid}" ]] && kill -0 "${old_pid}" 2>/dev/null; then
    echo "VLM daemon already running (pid ${old_pid})" >&2
    exit 0
  fi
  rm -f "${pid_file}"
fi

# Start in its own session: the development launcher cleans up the caller's
# process group after the shell exits, and a plain nohup is not sufficient for
# a long-lived Unix-socket service in that environment.
setsid "${conda_python}" "${project_root}/app/detectors/siliconflow_vision_daemon.py" \
  --socket "${socket_path}" \
  >"${project_root}/runtime/go2w/sessions/siliconflow_vlm_daemon.log" 2>&1 &
echo $! > "${pid_file}"
# Wait for the socket to appear.
for _ in $(seq 1 20); do
  if [[ -S "${socket_path}" ]]; then
    echo "VLM daemon started (pid $(cat "${pid_file}"))"
    exit 0
  fi
  sleep 0.2
done
echo "ERROR: VLM daemon did not create socket" >&2
exit 1
