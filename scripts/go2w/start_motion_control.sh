#!/usr/bin/env bash
set -Eeuo pipefail

# Operator-facing launcher for the bundled high-level controller. It starts
# the lease holder and the bounded /go2w/motion Action server, but does not
# authorize a motion goal by itself.

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
control_root="${GO2W_CONTROL_ROOT:-${project_root}/unitree_go2w_control}"

existing_servers="$(ps -eo pid=,args= | awk '{ exe=$2; sub(/^.*\//, "", exe); if (exe == "go2w_motion_action_server") print $1 }')"
if [[ -n "$existing_servers" ]]; then
  printf 'ERROR: refusing to start a duplicate go2w_motion_action_server; existing PID(s): %s\n' \
    "$(tr '\n' ' ' <<<"$existing_servers" | xargs)" >&2
  exit 3
fi

[[ -f "${control_root}/scripts/setup_go2w_ros2.sh" ]] || {
  printf 'ERROR: control workspace is not bundled/built: %s\n' "${control_root}" >&2
  printf 'Run: bash %s/setup_go2w_control.sh\n' "${script_dir}" >&2
  exit 2
}

# shellcheck disable=SC1090
source "${control_root}/scripts/setup_go2w_ros2.sh"
exec ros2 launch go2w_motion_control go2w_motion_control.launch.py \
  "robot_ip:=${GO2W_ROBOT_IP:-192.168.123.18}"
