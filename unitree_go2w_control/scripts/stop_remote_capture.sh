#!/usr/bin/env bash
set -Eeuo pipefail

PID_FILE="${1:?usage: stop_remote_capture.sh PID_FILE}"
[[ -f "$PID_FILE" ]] || {
  printf 'NO_ACTIVE_PID_FILE=%s\n' "$PID_FILE"
  exit 0
}

mapfile -t entries <"$PID_FILE"
pid_matches_label() {
  local label="$1" pid="$2" cmdline
  [[ -r "/proc/$pid/cmdline" ]] || return 1
  cmdline="$(tr '\0' ' ' <"/proc/$pid/cmdline")"
  case "$label" in
    rosbag) [[ "$cmdline" == *"ros2 bag record"* ]] ;;
    remote_keys) [[ "$cmdline" == *"monitor_remote_keys.py"* ]] ;;
    timeline) [[ "$cmdline" == *"capture_remote_timeline.py"* ]] ;;
    tcpdump) [[ "$cmdline" == *"tcpdump"* ]] ;;
    *) return 1 ;;
  esac
}

for entry in "${entries[@]}"; do
  label="${entry%% *}"
  pid="${entry##* }"
  [[ "$pid" =~ ^[0-9]+$ ]] || continue
  if kill -0 "$pid" 2>/dev/null && pid_matches_label "$label" "$pid"; then
    printf 'STOPPING label=%s pid=%s\n' "$label" "$pid"
    kill -INT "$pid" 2>/dev/null || true
  fi
done

for _ in {1..50}; do
  active=0
  for entry in "${entries[@]}"; do
    pid="${entry##* }"
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    if kill -0 "$pid" 2>/dev/null && pid_matches_label "${entry%% *}" "$pid"; then
      active=1
    fi
  done
  (( active == 0 )) && break
  sleep 0.1
done

for entry in "${entries[@]}"; do
  label="${entry%% *}"
  pid="${entry##* }"
  [[ "$pid" =~ ^[0-9]+$ ]] || continue
  if kill -0 "$pid" 2>/dev/null && pid_matches_label "$label" "$pid"; then
    printf 'TERMINATING label=%s pid=%s\n' "$label" "$pid"
    kill -TERM "$pid" 2>/dev/null || true
  fi
done

for entry in "${entries[@]}"; do
  pid="${entry##* }"
  [[ "$pid" =~ ^[0-9]+$ ]] || continue
  wait "$pid" 2>/dev/null || true
done

: >"$PID_FILE"
printf 'CAPTURE_PROCESSES_STOPPED=true\n'
