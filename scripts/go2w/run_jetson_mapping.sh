#!/usr/bin/env bash
# Apply the candidate robot profile, then use the unchanged production flow.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
[[ "$(uname -m)" == aarch64 && -f /opt/ros/foxy/setup.bash ]] || {
  echo 'This entry requires the Jetson Foxy deployment.' >&2
  exit 2
}
set -a
source "$root/configs/go2w/jetson_foxy.env"
set +a
exec bash "$root/scripts/go2w/one_click_remote_mapping.sh"
