#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
IFACE="$($SCRIPT_DIR/detect_unitree_interface.sh)"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$ROOT/logs/$STAMP"
mkdir -p "$LOG_DIR"
chmod 700 "$LOG_DIR"

set +u
source /opt/ros/humble/setup.bash
source "${GO2W_UNITREE_ROOT:-$HOME/unitree_ros2}/cyclonedds_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="file://$ROOT/config/cyclonedds_go2w.xml"

{
  date --iso-8601=seconds
  printf 'INTERFACE=%s\n' "$IFACE"
  ip -br addr
  ip route
  ip route get 192.168.123.18
  ping -c 3 -W 1 192.168.123.18 || true
  printenv | grep -E '^(ROS|RMW|CYCLONEDDS|AMENT|COLCON)_' | sort || true
  ros2 node list
  ros2 topic list
  ros2 topic info -v /api/sport/request
  ros2 topic info -v /api/sport/response
  ros2 topic info -v /lf/sportmodestate
  ps -ef | grep -E 'unitree|sport|go2|ros2' | grep -v grep || true
} >"$LOG_DIR/diagnostics.txt" 2>&1

timeout 5s ros2 topic echo /lf/sportmodestate \
  >"$LOG_DIR/sportmodestate_5s.txt" 2>&1 || true
timeout 5s ros2 topic echo /api/sport/response \
  >"$LOG_DIR/sport_response_5s.txt" 2>&1 || true
"$ROOT/.venv/bin/python" "$SCRIPT_DIR/inspect_motion_mode.py" --interface "$IFACE" \
  >"$LOG_DIR/motion_mode.json" 2>&1 || true
chmod 600 "$LOG_DIR"/*
printf '%s\n' "$LOG_DIR"
