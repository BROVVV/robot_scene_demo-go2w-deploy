#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/setup_go2w_ros2.sh"
case "${1:-}" in
  on) value=true ;;
  off) value=false ;;
  *) printf 'Usage: %s on|off\n' "$0" >&2; exit 2 ;;
esac
ros2 service call /go2w/arm std_srvs/srv/SetBool "{data: $value}"
