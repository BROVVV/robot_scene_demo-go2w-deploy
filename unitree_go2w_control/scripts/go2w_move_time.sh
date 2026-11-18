#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VX=0.0; VY=0.0; YAW_RATE=0.0; DURATION_SEC=""; TIMEOUT=0.0; YES=0; CANCEL_AFTER=0.0
while (($#)); do
  case "$1" in
    --vx) VX="$2"; shift 2 ;;
    --vy) VY="$2"; shift 2 ;;
    --yaw-rate) YAW_RATE="$2"; shift 2 ;;
    --seconds) DURATION_SEC="$2"; shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    --cancel-after) CANCEL_AFTER="$2"; shift 2 ;;
    --yes) YES=1; shift ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done
[[ -n "$DURATION_SEC" ]] || { printf '%s\n' '--seconds is required' >&2; exit 2; }
printf '将执行：vx=%s vy=%s yaw_rate=%s duration=%s 秒\n' "$VX" "$VY" "$YAW_RATE" "$DURATION_SEC"
if ((YES)); then
  [[ "${GO2W_AREA_CLEARED:-}" == I_HAVE_CLEARED_THE_AREA ]] || {
    printf '%s\n' '--yes requires GO2W_AREA_CLEARED=I_HAVE_CLEARED_THE_AREA' >&2
    exit 2
  }
else
  read -r -p '平整地面周围 2 米已清空，遥控器可急停。输入 I_HAVE_CLEARED_THE_AREA: ' confirmation
  [[ "$confirmation" == I_HAVE_CLEARED_THE_AREA ]] || exit 2
fi
# shellcheck source=/dev/null
source "$SCRIPT_DIR/setup_go2w_ros2.sh"
exec python3 "$SCRIPT_DIR/go2w_action_client.py" --mode timed \
  --vx "$VX" --vy "$VY" --yaw-rate "$YAW_RATE" --seconds "$DURATION_SEC" \
  --timeout "$TIMEOUT" --cancel-after "$CANCEL_AFTER"
