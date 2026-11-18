#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"
new_log_dir
# shellcheck source=/dev/null
source "$SCRIPT_DIR/unitree_env.sh"

topics="$(timeout 10s ros2 topic list || true)"
grep -Fxq '/api/sport/request' <<<"$topics" || die "/api/sport/request not found; refusing to publish"
STATE_TOPIC="$(grep -Ex '/?(lf/)?sportmodestate' <<<"$topics" | head -n1 || true)"
[[ -n "$STATE_TOPIC" ]] || die "state topic not found; refusing to publish"
BIN="$(official_example_bin go2_sport_client)" || die "go2_sport_client is not built"

for attempt in 1 2 3; do
  set +e
  timeout --signal=KILL 2s "$BIN" 10 2>&1 | tee -a "$LOG_DIR/stop_robot.txt"
  rc=${PIPESTATUS[0]}
  set -e
  if [[ "$rc" -eq 0 || "$rc" -eq 124 || "$rc" -eq 137 ]]; then
    timeout 5s ros2 topic echo --once "$STATE_TOPIC" >"$LOG_DIR/state_after_stop.txt"
    log "STOP request initialized on attempt $attempt"
    exit 0
  fi
  sleep 0.3
done
die "STOP request failed after 3 attempts"
