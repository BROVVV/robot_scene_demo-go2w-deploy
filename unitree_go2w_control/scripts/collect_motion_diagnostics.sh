#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/setup_go2w_ros2.sh"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="$ROOT/logs/$STAMP/motion_diagnostics"
mkdir -p "$OUTPUT"
{
  ip -br addr
  ip route
  ip route get "$GO2W_ROBOT_IP"
  ping -c 3 -W 1 "$GO2W_ROBOT_IP" || true
  env | rg '^(ROS|RMW|CYCLONEDDS|AMENT|COLCON)_' | sort || true
} >"$OUTPUT/environment.txt"
ros2 node list >"$OUTPUT/nodes.txt" || true
ros2 action list -t >"$OUTPUT/actions.txt" || true
ros2 service list -t >"$OUTPUT/services.txt" || true
for topic in /api/sport/request /api/sport/response /lf/sportmodestate /lf/lowstate /go2w/sport_lease/id /go2w/sport_lease/alive; do
  safe_name="${topic//\//_}"
  timeout 5s ros2 topic info -v "$topic" >"$OUTPUT/${safe_name}.txt" 2>&1 || true
done
printf '%s\n' "$OUTPUT"
