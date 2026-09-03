#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/setup_go2w_ros2.sh"
ros2 service call /go2w/emergency_stop std_srvs/srv/Trigger '{}'
