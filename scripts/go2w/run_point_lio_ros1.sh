#!/usr/bin/env bash
set -euo pipefail

# Starts only an isolated ROS 1 master, the localhost read-only endpoint, and
# official Point-LIO. It has no Unitree control client and no command topic.
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd -- "$SCRIPT_DIR/../.." && pwd)
CONDA_BIN=${CONDA_BIN:-/home/brov/miniconda3/bin/conda}
ENVIRONMENT=go2w_point_lio_noetic
WORKSPACE="$PROJECT_ROOT/point_lio_ws"
CONFIG=${POINT_LIO_CONFIG:-"$PROJECT_ROOT/configs/go2w/point_lio_unilidar_l2.yaml"}
OUTPUT_DIR=${POINT_LIO_OUTPUT_DIR:-$PROJECT_ROOT/outputs/go2w_acceptance/point_lio_stationary}
USE_IMU_AS_INPUT=${POINT_LIO_USE_IMU_AS_INPUT:-false}
FILTER_SIZE_SURF=${POINT_LIO_FILTER_SIZE_SURF:-0.4}
FILTER_SIZE_MAP=${POINT_LIO_FILTER_SIZE_MAP:-0.4}

# The parent is normally a sourced Humble shell. Remove every ROS/ament path
# that could make Noetic import host ROS 2 packages; conda run rebuilds the
# isolated environment for each command below.
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_ETC_DIR
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH LD_LIBRARY_PATH

ROS_MASTER_URI=http://127.0.0.1:11319
ROS_IP=127.0.0.1
export ROS_MASTER_URI ROS_IP
mkdir -p "$OUTPUT_DIR"

if [[ ! -x "$WORKSPACE/devel/lib/point_lio_unilidar/pointlio_mapping" ]]; then
  printf 'Point-LIO has not been built; run setup_point_lio_noetic.sh first\n' >&2
  exit 1
fi

owned_groups=()
cleanup() {
  trap - EXIT INT TERM
  for group in "${owned_groups[@]}"; do
    if kill -0 "$group" 2>/dev/null; then
      kill -TERM -- "-$group" 2>/dev/null || true
    fi
  done
  for _ in $(seq 1 30); do
    any_alive=false
    for group in "${owned_groups[@]}"; do
      if kill -0 "$group" 2>/dev/null; then any_alive=true; fi
    done
    if [[ "$any_alive" == false ]]; then return; fi
    sleep 0.1
  done
  for group in "${owned_groups[@]}"; do
    if kill -0 "$group" 2>/dev/null; then
      kill -KILL -- "-$group" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

setsid "$CONDA_BIN" run --no-capture-output -n "$ENVIRONMENT" roscore -p 11319 \
  >"$OUTPUT_DIR/roscore.log" 2>&1 &
owned_groups+=("$!")
for _ in $(seq 1 100); do
  if "$CONDA_BIN" run -n "$ENVIRONMENT" rosparam list >/dev/null 2>&1; then break; fi
  sleep 0.1
done
if ! "$CONDA_BIN" run -n "$ENVIRONMENT" rosparam list >/dev/null 2>&1; then
  printf 'isolated ROS 1 master failed to start\n' >&2
  exit 1
fi

setsid "$CONDA_BIN" run --no-capture-output -n "$ENVIRONMENT" \
  python "$SCRIPT_DIR/point_lio_ros1_endpoint.py" \
  >"$OUTPUT_DIR/ros1_endpoint.log" 2>&1 &
owned_groups+=("$!")

"$CONDA_BIN" run -n "$ENVIRONMENT" rosparam load "$CONFIG" /laserMapping
setsid "$CONDA_BIN" run --no-capture-output -n "$ENVIRONMENT" bash --noprofile --norc -c \
  "source '$WORKSPACE/devel/setup.bash'; exec rosrun point_lio_unilidar pointlio_mapping __name:=laserMapping _use_imu_as_input:=${USE_IMU_AS_INPUT} _prop_at_freq_of_imu:=true _check_satu:=true _init_map_size:=10 _point_filter_num:=1 _space_down_sample:=true _filter_size_surf:=${FILTER_SIZE_SURF} _filter_size_map:=${FILTER_SIZE_MAP} _cube_side_length:=1000.0 _runtime_pos_log_enable:=false" \
  >"$OUTPUT_DIR/point_lio.log" 2>&1 &
owned_groups+=("$!")

printf 'isolated Point-LIO running; ROS_MASTER_URI=%s; motion_authorized=false\n' "$ROS_MASTER_URI"
while true; do
  for group in "${owned_groups[@]}"; do
    if ! kill -0 "$group" 2>/dev/null; then
      printf 'owned Point-LIO process exited unexpectedly: %s\n' "$group" >&2
      exit 1
    fi
  done
  sleep 0.5
done
