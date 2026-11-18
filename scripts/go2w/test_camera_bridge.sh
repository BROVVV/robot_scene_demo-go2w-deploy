#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
FIXTURE="${1:-}"
OUTPUT_DIR="${GO2W_CAMERA_TEST_OUTPUT_DIR:-$PROJECT_ROOT/outputs/go2w_acceptance/camera_bridge_bag}"

if [[ -z "$FIXTURE" || ! -f "$FIXTURE" ]]; then
  printf 'ERROR: pass a saved JPEG fixture as the first argument.\n' >&2
  exit 2
fi
mkdir -p "$OUTPUT_DIR"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${GO2W_UNITREE_ROOT:-$HOME/unitree_ros2}/cyclonedds_ws/install/setup.bash"
# shellcheck disable=SC1091
source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="${GO2W_TEST_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="file://$PROJECT_ROOT/configs/go2w/cyclonedds_camera_test.xml"
export ROS_DOMAIN_ID="${GO2W_TEST_ROS_DOMAIN_ID:-87}"

bridge_pid=""
fixture_pid=""
cleanup() {
  [[ -z "$fixture_pid" ]] || kill -TERM "$fixture_pid" 2>/dev/null || true
  [[ -z "$bridge_pid" ]] || kill -TERM "$bridge_pid" 2>/dev/null || true
  [[ -z "$fixture_pid" ]] || wait "$fixture_pid" 2>/dev/null || true
  [[ -z "$bridge_pid" ]] || wait "$bridge_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 run go2w_camera_bridge camera_bridge --ros-args \
  -p source:=topic \
  -p calibration_file:="$PROJECT_ROOT/configs/go2w/camera_intrinsics.yaml" \
  >"$OUTPUT_DIR/bridge.log" 2>&1 &
bridge_pid=$!

/usr/bin/python3 "$SCRIPT_DIR/publish_camera_fixture.py" \
  --image "$FIXTURE" --count 8 --hz 0.5 \
  >"$OUTPUT_DIR/fixture_publisher.log" 2>&1 &
fixture_pid=$!

/usr/bin/python3 "$SCRIPT_DIR/validate_camera_bridge_ros.py" \
  --output "$OUTPUT_DIR/result.json" --frames 5 --timeout 25

printf 'Offline camera bridge acceptance passed: %s\n' "$OUTPUT_DIR/result.json"
