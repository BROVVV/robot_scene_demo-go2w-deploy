#!/usr/bin/env bash
set -euo pipefail
pidfile=/home/unitree/robotscene_runtime/web_smoke_8876.pid
[[ -f "$pidfile" ]] || exit 0
pid="$(tr -cd '0-9' < "$pidfile")"
[[ -n "$pid" ]] || exit 1
if [[ -r "/proc/$pid/cmdline" ]]; then
  cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline")"
  if [[ "$cmd" == *"uvicorn app.manual_web_demo.web_server:app"* &&
        "$cmd" == *"--port 8876"* ]]; then
    kill -TERM "$pid"
  else
    printf 'Refusing to stop unrelated PID %s\n' "$pid" >&2
    exit 1
  fi
fi
