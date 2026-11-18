#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONTROL_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
CONTROL_CONFIG="${GO2W_CONTROL_CONFIG:-$CONTROL_ROOT/ros2_ws/src/go2w_motion_control/config/motion_control.yaml}"
if ! rg -q '^\s*yaw_command_sign:\s*(-1|1)\s*$' "$CONTROL_CONFIG"; then
  echo "拒绝验收：yaw_command_sign 尚未完成 ±1 方向标定。" >&2
  echo "先运行 $SCRIPT_DIR/calibrate_yaw_direction.sh，随后重启控制节点。" >&2
  exit 3
fi
read -r -p '将按计划执行完整低速到 90° 验收。输入 I_HAVE_CLEARED_THE_AREA: ' confirmation
[[ "$confirmation" == I_HAVE_CLEARED_THE_AREA ]] || exit 2
export GO2W_AREA_CLEARED=I_HAVE_CLEARED_THE_AREA
trap '"$SCRIPT_DIR/go2w_stop.sh" >/dev/null 2>&1 || true; "$SCRIPT_DIR/go2w_arm.sh" off >/dev/null 2>&1 || true' EXIT INT TERM
"$SCRIPT_DIR/go2w_stop.sh"
"$SCRIPT_DIR/go2w_arm.sh" on
"$SCRIPT_DIR/go2w_move_time.sh" --vx 0.05 --seconds 0.5 --yes
"$SCRIPT_DIR/go2w_move_time.sh" --vx -0.05 --seconds 0.5 --yes
"$SCRIPT_DIR/go2w_move_time.sh" --vx 0.08 --seconds 1.0 --yes
"$SCRIPT_DIR/go2w_move_time.sh" --vx 0.05 --seconds 3.0 --cancel-after 1.0 --yes
"$SCRIPT_DIR/go2w_turn_angle.sh" --degrees 10 --max-yaw-rate 0.08 --yes
"$SCRIPT_DIR/go2w_turn_angle.sh" --degrees -10 --max-yaw-rate 0.08 --yes
"$SCRIPT_DIR/go2w_turn_angle.sh" --degrees 30 --max-yaw-rate 0.12 --yes
"$SCRIPT_DIR/go2w_turn_angle.sh" --degrees -30 --max-yaw-rate 0.12 --yes
for _ in 1 2 3; do
  "$SCRIPT_DIR/go2w_turn_angle.sh" --degrees -90 --max-yaw-rate 0.20 --yes
done
