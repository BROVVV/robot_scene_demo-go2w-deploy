#!/usr/bin/env bash
set -euo pipefail
request_path="${1:?usage: start_nav2_worker.sh REQUEST_JSON}"
source "${NAV2_SETUP_BASH:-/opt/ros/humble/setup.bash}"
if [[ -n "${NAV2_WORKSPACE_SETUP:-}" ]]; then source "$NAV2_WORKSPACE_SETUP"; fi
exec "${NAV2_SYSTEM_PYTHON:-/usr/bin/python3}" scripts/nav2_bridge_worker.py --request "$request_path"
