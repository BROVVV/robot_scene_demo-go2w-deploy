#!/usr/bin/env bash
set -euo pipefail

# Ten-minute stationary, read-only camera/LiDAR transport soak. This wrapper
# starts no Sport, lease, cmd_vel, Nav2, posture, or joint-control component.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "${script_dir}/run_level_a_acceptance.py" "$@"
