#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
read -r -p '将依次执行 +0.05 和 -0.05 rad/s，各 0.3 秒。输入 I_HAVE_CLEARED_THE_AREA: ' confirmation
[[ "$confirmation" == I_HAVE_CLEARED_THE_AREA ]] || exit 2
# shellcheck source=/dev/null
source "$SCRIPT_DIR/setup_go2w_ros2.sh"
python3 "$SCRIPT_DIR/calibrate_yaw_direction.py" \
  --config "$ROOT/ros2_ws/src/go2w_motion_control/config/motion_control.yaml"
printf '%s\n' '方向已写入 YAML；请重启 launch 后再执行定角转向。'
