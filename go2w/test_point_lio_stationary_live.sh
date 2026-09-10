#!/usr/bin/env bash
set -eo pipefail

# Live read-only stationary acceptance for the official Unitree Point-LIO
# fallback. No motion, lease, cmd_vel, posture, joint, navigation, or robot
# control process is started by this script.
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd -- "$SCRIPT_DIR/../.." && pwd)
EVIDENCE_ROOT="$PROJECT_ROOT/outputs/go2w_acceptance/point_lio_stationary"
LOG_ROOT="$EVIDENCE_ROOT/logs"
COLLECTION_SECONDS=${POINT_LIO_COLLECTION_SECONDS:-25}
MINIMUM_ODOM=${POINT_LIO_MINIMUM_ODOM:-100}
RESULT_NAME=${POINT_LIO_RESULT_NAME:-result.json}
STALE_RESULT_NAME=${POINT_LIO_STALE_RESULT_NAME:-stale_timeout.json}
mkdir -p "$EVIDENCE_ROOT" "$LOG_ROOT"

source /opt/ros/humble/setup.bash
source "${GO2W_UNITREE_ROOT:-$HOME/unitree_ros2}/cyclonedds_ws/install/setup.bash"
source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PROJECT_ROOT/configs/go2w/cyclonedds_go2w.xml"

if ros2 topic list | grep -Eq '^/lio/(odom|path|cloud_registered)$'; then
  printf 'refusing acceptance: an LIO output already exists\n' >&2
  exit 2
fi

owned_groups=()
cleanup() {
  trap - EXIT INT TERM
  for group in "${owned_groups[@]}"; do
    if kill -0 "$group" 2>/dev/null; then
      kill -TERM -- "-$group" 2>/dev/null || true
    fi
  done
  for group in "${owned_groups[@]}"; do wait "$group" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

setsid "$SCRIPT_DIR/run_point_lio_ros1.sh" >"$LOG_ROOT/ros1_stack.log" 2>&1 &
ros1_pid=$!
owned_groups+=("$ros1_pid")
for _ in $(seq 1 200); do
  if grep -q 'isolated Point-LIO running' "$LOG_ROOT/ros1_stack.log" 2>/dev/null; then break; fi
  if ! kill -0 "$ros1_pid" 2>/dev/null; then
    printf 'isolated ROS 1 Point-LIO stack exited during startup\n' >&2
    exit 2
  fi
  sleep 0.1
done
if ! grep -q 'isolated Point-LIO running' "$LOG_ROOT/ros1_stack.log"; then
  printf 'timed out waiting for isolated Point-LIO startup\n' >&2
  exit 2
fi

setsid ros2 launch go2w_description official_sensor_frames.launch.py \
  "reference_file:=$PROJECT_ROOT/configs/go2w/official_reference.yaml" \
  >"$LOG_ROOT/description.log" 2>&1 &
description_pid=$!
owned_groups+=("$description_pid")

setsid ros2 run go2w_sensor_time_bridge time_bridge --ros-args \
  -p "config_file:=$PROJECT_ROOT/configs/go2w/time_sync.yaml" \
  >"$LOG_ROOT/time_bridge.log" 2>&1 &
time_pid=$!
owned_groups+=("$time_pid")

setsid ros2 launch go2w_lio_bringup point_lio.launch.py \
  "lio_config:=$PROJECT_ROOT/configs/go2w/point_lio.yaml" \
  "reference_config:=$PROJECT_ROOT/configs/go2w/official_reference.yaml" \
  "time_config:=$PROJECT_ROOT/configs/go2w/time_sync.yaml" \
  >"$LOG_ROOT/point_lio_bridge.log" 2>&1 &
bridge_pid=$!
owned_groups+=("$bridge_pid")

stationary_rc=0
/usr/bin/python3 "$SCRIPT_DIR/validate_lio_stationary_ros.py" \
  --reference "$PROJECT_ROOT/configs/go2w/official_reference.yaml" \
  --output "$EVIDENCE_ROOT/$RESULT_NAME" \
  --minimum-odom "$MINIMUM_ODOM" \
  --collection-seconds "$COLLECTION_SECONDS" \
  --startup-timeout-seconds 30 || stationary_rc=$?

# Stop only this run's timestamp-preserving sensor-copy process. Point-LIO and
# the ROS 2 watchdog remain alive to prove outputs become stale rather than
# replaying the final pose.
kill -TERM -- "-$time_pid" 2>/dev/null || true
wait "$time_pid" 2>/dev/null || true
stale_rc=0
/usr/bin/python3 "$SCRIPT_DIR/validate_lio_stale_timeout_ros.py" \
  --output "$EVIDENCE_ROOT/$STALE_RESULT_NAME" \
  --configured-timeout-seconds 0.3 \
  --maximum-observed-seconds 0.7 || stale_rc=$?

if (( stationary_rc != 0 || stale_rc != 0 )); then
  printf 'FAIL: stationary_rc=%d, stale_rc=%d\n' "$stationary_rc" "$stale_rc" >&2
  exit 2
fi
printf 'PASS: official Point-LIO stationary read-only trial and stale timeout\n'
