#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"
new_log_dir
# shellcheck source=/dev/null
source "$SCRIPT_DIR/unitree_env.sh"

topics="$(timeout 20s ros2 topic list || true)"
SPORT_STATE_TOPIC="$(grep -Ex '/?(lf/)?sportmodestate' <<<"$topics" | head -n1 || true)"
[[ -n "$SPORT_STATE_TOPIC" ]] || die "SportModeState topic not found"
printf 'SPORT_STATE_TOPIC=%s\n' "$SPORT_STATE_TOPIC" | tee "$LOG_DIR/state_topic.txt"
timeout 15s ros2 topic echo --once "$SPORT_STATE_TOPIC" 2>&1 | tee "$LOG_DIR/state_once.txt"
ros2 topic info -v "$SPORT_STATE_TOPIC" 2>&1 | tee "$LOG_DIR/state_info.txt"

BIN="$(official_example_bin read_motion_state)" || die "read_motion_state is not built"
set +e
timeout 10s "$BIN" 2>&1 | tee "$LOG_DIR/read_motion_state.txt"
rc=${PIPESTATUS[0]}
set -e
[[ "$rc" -eq 0 || "$rc" -eq 124 ]] || exit "$rc"
