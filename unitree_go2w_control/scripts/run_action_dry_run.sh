#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/setup_go2w_ros2.sh"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$ROOT/logs/$STAMP"
mkdir -p "$LOG_DIR"
SERVER_PID=""
cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  if [[ -n "$SERVER_PID" ]] && kill -0 "$SERVER_PID" 2>/dev/null; then
    kill -TERM "$SERVER_PID" 2>/dev/null || true
    for _ in {1..50}; do
      kill -0 "$SERVER_PID" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$SERVER_PID" 2>/dev/null; then
      kill -KILL "$SERVER_PID" 2>/dev/null || true
    fi
    wait "$SERVER_PID" 2>/dev/null || true
  fi
  exit "$rc"
}
trap cleanup EXIT INT TERM
ros2 run go2w_motion_control go2w_motion_action_server --ros-args \
  --params-file "$ROOT/ros2_ws/src/go2w_motion_control/config/motion_control.yaml" \
  -p dry_run:=true -p yaw_command_sign:=1 \
  -p sport_state_topic:=/dry_run/sportmodestate \
  -p low_state_topic:=/dry_run/lowstate \
  >"$LOG_DIR/dry_run_server.txt" 2>&1 &
SERVER_PID=$!
timeout 90s python3 "$SCRIPT_DIR/test_action_dry_run.py" \
  | tee "$LOG_DIR/dry_run_tests.jsonl"
printf 'DRY_RUN_LOG_DIR=%s\n' "$LOG_DIR"
