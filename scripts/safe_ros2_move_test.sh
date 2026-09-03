#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
UNITREE_ROOT="${GO2W_UNITREE_ROOT:-$HOME/unitree_ros2}"
VENV_PYTHON="$ROOT/.venv/bin/python"
IFACE="$($SCRIPT_DIR/detect_unitree_interface.sh)"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$ROOT/logs/$STAMP"
mkdir -p "$LOG_DIR"
chmod 700 "$LOG_DIR"
LOG_FILE="$LOG_DIR/safe_ros2_move_test.txt"
BIN="$ROOT/ros2_ws/install/safe_go2w_control/lib/safe_go2w_control/safe_go2w_ros2_move"
LEASE_FILE="$LOG_DIR/sport_lease_id.txt"
LEASE_PID=""

cleanup() {
  local rc=$?
  trap - EXIT INT TERM HUP
  if [[ -n "$LEASE_PID" ]] && kill -0 "$LEASE_PID" 2>/dev/null; then
    kill -TERM "$LEASE_PID" 2>/dev/null || true
    wait "$LEASE_PID" || rc=1
  fi
  "$VENV_PYTHON" "$SCRIPT_DIR/safe_sdk_stop.py" --interface "$IFACE" \
    >>"$LOG_FILE" 2>&1 || true
  if pgrep -af '[s]afe_go2w_ros2_move|[h]old_sport_lease.py' >>"$LOG_FILE"; then
    printf 'ERROR: residual control process detected\n' >>"$LOG_FILE"
    rc=1
  else
    printf 'CONTROL_PROCESS_RESIDUE=none\n' >>"$LOG_FILE"
  fi
  chmod 600 "$LOG_FILE"
  exit "$rc"
}
trap cleanup EXIT INT TERM HUP

set +u
source /opt/ros/humble/setup.bash
source "$UNITREE_ROOT/cyclonedds_ws/install/setup.bash"
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="file://$ROOT/config/cyclonedds_go2w.xml"

{
  printf 'INTERFACE=%s\n' "$IFACE"
  printf 'RMW_IMPLEMENTATION=%s\n' "$RMW_IMPLEMENTATION"
  printf 'CYCLONEDDS_URI=%s\n' "$CYCLONEDDS_URI"
  "$VENV_PYTHON" "$SCRIPT_DIR/inspect_motion_mode.py" --interface "$IFACE"
  topics="$(timeout 10s ros2 topic list)"
  grep -Fx '/api/sport/request' <<<"$topics"
  grep -Fx '/api/sport/response' <<<"$topics"
  grep -Fx '/lf/sportmodestate' <<<"$topics"
  [[ -x "$BIN" ]]
} 2>&1 | tee "$LOG_FILE"

read -r -p "Type I_HAVE_CLEARED_THE_AREA: " confirmation
if [[ "$confirmation" != I_HAVE_CLEARED_THE_AREA ]]; then
  printf 'ERROR: safety confirmation rejected\n' | tee -a "$LOG_FILE"
  exit 2
fi

"$VENV_PYTHON" "$SCRIPT_DIR/hold_sport_lease.py" \
  --interface "$IFACE" --ready-file "$LEASE_FILE" >>"$LOG_FILE" 2>&1 &
LEASE_PID=$!
for _ in {1..60}; do
  [[ -s "$LEASE_FILE" ]] && break
  kill -0 "$LEASE_PID" 2>/dev/null || break
  sleep 0.1
done
if [[ ! -s "$LEASE_FILE" ]]; then
  printf 'ERROR: Sport lease was not acquired\n' | tee -a "$LOG_FILE"
  exit 1
fi
LEASE_ID="$(tr -d '[:space:]' <"$LEASE_FILE")"
if [[ ! "$LEASE_ID" =~ ^[1-9][0-9]*$ ]]; then
  printf 'ERROR: invalid Sport lease ID\n' | tee -a "$LOG_FILE"
  exit 1
fi
printf 'SPORT_LEASE_ID=%s\n' "$LEASE_ID" | tee -a "$LOG_FILE"

set +e
timeout --signal=INT --kill-after=3s 15s "$BIN" --confirmed \
  --lease-id "$LEASE_ID" --vx 0.0 --vy 0.0 --vyaw 0.08 --duration 0.4 \
  2>&1 | tee -a "$LOG_FILE"
test_rc=${PIPESTATUS[0]}
set -e
printf 'ROS2_TEST_RC=%s\nLOG_DIR=%s\n' "$test_rc" "$LOG_DIR" | tee -a "$LOG_FILE"
exit "$test_rc"
