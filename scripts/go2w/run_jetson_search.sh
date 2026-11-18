#!/usr/bin/env bash
# Apply the robot profile and enter the production search flow, planning only
# by default. Original execution gates remain in start_nav2_autonomous_search.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
[[ "$(uname -m)" == aarch64 && -f /opt/ros/foxy/setup.bash ]] || {
  echo 'This entry requires the Jetson Foxy deployment.' >&2
  exit 2
}
set -a
source "$root/configs/go2w/jetson_foxy.env"
set +a
exec bash "$root/scripts/go2w/bootstrap_webui.sh" --mode search --detector yoloe_seg --no-browser "$@"
