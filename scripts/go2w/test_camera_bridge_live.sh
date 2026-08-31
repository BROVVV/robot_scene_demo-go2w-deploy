#!/usr/bin/env bash
set -euo pipefail

# Read-only live acceptance. This script subscribes to the built-in camera and
# publishes only derived host-side ROS image topics. It never initializes the
# Unitree SDK, acquires a Sport lease, or publishes a robot input topic.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
OUTPUT_DIR="${GO2W_CAMERA_LIVE_OUTPUT_DIR:-$PROJECT_ROOT/outputs/go2w_acceptance/camera_bridge_live}"
mkdir -p "$OUTPUT_DIR"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${GO2W_UNITREE_ROOT:-$HOME/unitree_ros2}/cyclonedds_ws/install/setup.bash"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/setup_environment.sh"
set -u

bridge_pid=""
cleanup() {
  [[ -z "$bridge_pid" ]] || kill -TERM "$bridge_pid" 2>/dev/null || true
  [[ -z "$bridge_pid" ]] || wait "$bridge_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 run go2w_camera_bridge camera_bridge --ros-args \
  -p source:="${GO2W_CAMERA_LIVE_SOURCE:-rpc}" \
  -p calibration_file:="$PROJECT_ROOT/configs/go2w/camera_intrinsics.yaml" \
  >"$OUTPUT_DIR/bridge.log" 2>&1 &
bridge_pid=$!

/usr/bin/python3 "$SCRIPT_DIR/validate_camera_bridge_ros.py" \
  --output "$OUTPUT_DIR/result.json" --frames 10 --timeout 20

printf 'Live read-only camera acceptance passed: %s\n' "$OUTPUT_DIR/result.json"
