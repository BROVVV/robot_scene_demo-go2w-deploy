#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEGREES=""; MAX_YAW_RATE=""; TIMEOUT=0.0; YES=0
while (($#)); do
  case "$1" in
    --degrees) DEGREES="$2"; shift 2 ;;
    --max-yaw-rate) MAX_YAW_RATE="$2"; shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    --yes) YES=1; shift ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done
[[ -n "$DEGREES" && -n "$MAX_YAW_RATE" ]] || {
  printf '%s\n' '--degrees and --max-yaw-rate are required' >&2; exit 2;
}
printf '将执行相对转向：%s 度，最大角速度 %s rad/s。正角度左转，负角度右转。\n' "$DEGREES" "$MAX_YAW_RATE"
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
exec python3 "$SCRIPT_DIR/go2w_action_client.py" --mode yaw \
  --degrees "$DEGREES" --max-yaw-rate "$MAX_YAW_RATE" --timeout "$TIMEOUT"
