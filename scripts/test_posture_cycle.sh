#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"
new_log_dir
# shellcheck source=/dev/null
source "$SCRIPT_DIR/unitree_env.sh"

BIN="$(official_example_bin go2_sport_client)" || die "go2_sport_client is not built"
STATE_TOPIC=/lf/sportmodestate
cleanup() {
  "$SCRIPT_DIR/stop_robot.sh" || true
}
trap cleanup EXIT INT TERM HUP

topics="$(timeout 10s ros2 topic list || true)"
grep -Fxq /api/sport/request <<<"$topics" || die "/api/sport/request is missing"
grep -Fxq "$STATE_TOPIC" <<<"$topics" || die "$STATE_TOPIC is missing"

sample_state() {
  local label="$1"
  local output="$LOG_DIR/state_${label}.txt"
  timeout 10s ros2 topic echo --once "$STATE_TOPIC" >"$output"
  awk '
    /^mode:/ {mode=$2}
    /^velocity:/ {getline; vx=$2; getline; vy=$2; getline; vz=$2}
    /^yaw_speed:/ {yaw=$2}
    END {
      printf "mode=%s velocity=[%s,%s,%s] yaw_speed=%s\n", mode, vx, vy, vz, yaw
      if (vx < -0.05 || vx > 0.05 || vy < -0.05 || vy > 0.05 ||
          vz < -0.05 || vz > 0.05 || yaw < -0.05 || yaw > 0.05) exit 1
    }
  ' "$output" | tee -a "$LOG_DIR/posture_cycle.txt"
}

current_mode() {
  timeout 5s ros2 topic echo --once "$STATE_TOPIC" | awk '/^mode:/ {print $2; exit}'
}

wait_for_mode() {
  local expected="$1"
  local label="$2"
  local mode=""
  for _ in {1..20}; do
    mode="$(current_mode || true)"
    printf '%s mode=%s expected=%s\n' "$label" "${mode:-none}" "$expected" | tee -a "$LOG_DIR/posture_cycle.txt"
    [[ "$mode" == "$expected" ]] && return 0
    sleep 0.5
  done
  return 1
}

send_mode() {
  local mode="$1"
  local name="$2"
  set +e
  timeout --signal=KILL 2s "$BIN" "$mode" >>"$LOG_DIR/${name}.txt" 2>&1
  local rc=$?
  set -e
  [[ "$rc" -eq 0 || "$rc" -eq 124 || "$rc" -eq 137 ]] || die "$name failed with rc=$rc"
  log "$name request initialized" | tee -a "$LOG_DIR/posture_cycle.txt"
}

log "Posture cycle started; no walking command is permitted" | tee "$LOG_DIR/posture_cycle.txt"
sample_state before
send_mode 3 stand_down
wait_for_mode 5 LIE_DOWN || die "robot did not reach lie-down mode 5"
sample_state down
sleep 2
send_mode 4 stand_up
wait_for_mode 1 STAND_UP || die "robot did not return to stand mode 1"
sample_state up
"$SCRIPT_DIR/stop_robot.sh"
trap - EXIT INT TERM HUP
log "Posture cycle completed and final STOP passed" | tee -a "$LOG_DIR/posture_cycle.txt"
printf 'POSTURE_CYCLE_LOG_DIR=%s\n' "$LOG_DIR"
