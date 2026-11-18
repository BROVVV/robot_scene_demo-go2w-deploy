#!/usr/bin/env bash
# Phase-2 Web smoke: no ROS worker, no search execution, no motion output.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
profile="$root/configs/go2w/jetson_foxy.env"
set -a
source "$profile"
set +a
export MANUAL_DEMO_DISPLAY_ONLY=1
export GO2W_RUNTIME_ADMIN_ENABLED=0
runtime=/home/unitree/robotscene_runtime
mkdir -p "$runtime/logs"
cd "$root"
exec "$GO2W_CONDA_PYTHON" -m uvicorn app.manual_web_demo.web_server:app \
  --host 0.0.0.0 --port "${GO2W_SMOKE_WEB_PORT:-8876}" --lifespan off
