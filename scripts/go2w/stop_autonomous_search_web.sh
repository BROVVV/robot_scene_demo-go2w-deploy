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
kill "$pid" || true
for _ in $(seq 1 40); do
  if ! kill -0 "$pid" 2>/dev/null; then
    rm -f "$pidfile"
    printf 'Stopped.\n' >&2
    exit 0
  fi
  sleep 0.25
done
printf 'WARNING: pid %s did not exit within 10s; left as-is.\n' "$pid" >&2
exit 1
