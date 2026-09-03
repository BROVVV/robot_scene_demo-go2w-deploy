#!/usr/bin/env bash
set -Eeuo pipefail

# Owner safety policy for this run: the robot must not move. This script therefore
# performs only read-only state validation and an optional STOP request. It never
# invokes go2_sport_client modes 0, 2, 4, 6, 8, or 9.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"
new_log_dir
# shellcheck source=/dev/null
source "$SCRIPT_DIR/unitree_env.sh"

log "PHYSICAL MOTION DISABLED by owner instruction" | tee "$LOG_DIR/official_demo.txt"
"$SCRIPT_DIR/read_state.sh"
"$SCRIPT_DIR/stop_robot.sh"
log "Read-only validation and STOP completed; no stand or motion command was sent" | tee -a "$LOG_DIR/official_demo.txt"
