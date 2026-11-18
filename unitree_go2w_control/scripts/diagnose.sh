#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"
new_log_dir
# shellcheck source=/dev/null
source "$SCRIPT_DIR/unitree_env.sh"

{
  printf 'RMW=%s\nURI=%s\nDOMAIN=%s\nIFACE=%s\n' \
    "$RMW_IMPLEMENTATION" "$CYCLONEDDS_URI" "$ROS_DOMAIN_ID" "$UNITREE_IFACE"
  ip -br addr show dev "$UNITREE_IFACE"
  ip route get "$UNITREE_ROBOT_IP"
  ip maddr show dev "$UNITREE_IFACE"
  ros2 daemon stop || true
  ros2 daemon start
  timeout 20s ros2 topic list | sort
} 2>&1 | tee "$LOG_DIR/diagnose.txt"
