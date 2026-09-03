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
