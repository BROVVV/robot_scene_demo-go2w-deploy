#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
BOARD=""
SQUARE_M=""
OPERATOR=""
INSTALL_FROM=""
CAMERA_PID=""

usage() {
  cat <<'EOF'
Usage:
  calibrate_camera.sh --board COLSxROWS --square-m METERS --operator NAME
  calibrate_camera.sh --board COLSxROWS --square-m METERS --operator NAME \
    --install-from /path/to/ost.yaml

COLSxROWS is the measured number of inner chessboard corners. METERS is the
physically measured square edge length. No defaults are provided by design.
The robot must remain stationary throughout collection.
EOF
}

while (( $# )); do
  case "$1" in
    --board) BOARD="${2:-}"; shift 2 ;;
    --square-m) SQUARE_M="${2:-}"; shift 2 ;;
    --operator) OPERATOR="${2:-}"; shift 2 ;;
    --install-from) INSTALL_FROM="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'ERROR: unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! "$BOARD" =~ ^[1-9][0-9]*x[1-9][0-9]*$ || -z "$SQUARE_M" || -z "$OPERATOR" ]]; then
  printf 'ERROR: --board, --square-m, and --operator are required.\n' >&2
  usage >&2
  exit 2
fi
if ! awk -v value="$SQUARE_M" 'BEGIN { exit !(value > 0) }'; then
  printf 'ERROR: --square-m must be positive.\n' >&2
  exit 2
fi

if [[ -n "$INSTALL_FROM" ]]; then
  /usr/bin/python3 "$SCRIPT_DIR/install_camera_calibration.py" \
    --input "$INSTALL_FROM" \
    --output "$PROJECT_ROOT/configs/go2w/camera_intrinsics.yaml" \
    --operator "$OPERATOR" \
    --board "$BOARD" \
    --square-m "$SQUARE_M"
  exit 0
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${GO2W_UNITREE_ROOT:-$HOME/unitree_ros2}/cyclonedds_ws/install/setup.bash"
if [[ -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
fi
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${PROJECT_ROOT}/configs/go2w/cyclonedds_go2w.xml"

cleanup() {
  if [[ -n "$CAMERA_PID" ]] && kill -0 "$CAMERA_PID" 2>/dev/null; then
    kill -TERM -- "-$CAMERA_PID" 2>/dev/null || true
    wait "$CAMERA_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if ros2 node list 2>/dev/null | grep -qx '/go2w_camera_bridge'; then
  printf '%s\n' 'Using the already-running go2w_camera_bridge.'
else
  mkdir -p "$PROJECT_ROOT/runtime/go2w"
  setsid ros2 run go2w_camera_bridge camera_bridge --ros-args \
    -p source:=rpc \
    -p interface:=enp6s0 \
    -p "calibration_file:=${PROJECT_ROOT}/configs/go2w/camera_intrinsics.yaml" \
    >"$PROJECT_ROOT/runtime/go2w/camera_calibration_bridge.log" 2>&1 &
  CAMERA_PID=$!
fi

# camera_calibration chooses its subscriber QoS from publishers discovered at
# construction time. Starting it before DDS discovery makes it fall back to
# Reliable, which is incompatible with this sensor-data Best Effort publisher.
camera_publisher_ready=false
for _ in {1..150}; do
  topic_info="$(ros2 topic info /camera/front/image_raw 2>/dev/null || true)"
  if grep -Eq '^Publisher count: [1-9][0-9]*$' <<<"$topic_info"; then
    camera_publisher_ready=true
    break
  fi
  sleep 0.1
done
if [[ "$camera_publisher_ready" != true ]]; then
  printf '%s\n' 'ERROR: camera publisher was not discovered within 15 seconds.' >&2
  exit 2
fi

# XWayland is more stable for the long-running GTK calibrator on this host.
export GDK_BACKEND="${GDK_BACKEND:-x11}"

cat <<EOF
Camera calibration collection will start with:
  board inner corners: $BOARD
  measured square size: $SQUARE_M m
  operator: $OPERATOR

Keep the Go2-W completely stationary. Move only the printed board through
different distances, tilts, and image positions. In the calibrator, collect
until the bars are green, then CALIBRATE and SAVE. Do not press COMMIT because
the bridge has no camera set_camera_info service.

The image source is the read-only VideoHub RPC; no Sport client or motion node
is started by this script.
EOF

ros2 run camera_calibration cameracalibrator \
  --size "$BOARD" \
  --square "$SQUARE_M" \
  --no-service-check \
  image:=/camera/front/image_raw \
  camera:=/camera/front

cat <<EOF
Collection ended. Extract the saved calibration archive (normally
/tmp/calibrationdata.tar.gz), inspect ost.yaml, then install it with:

  bash scripts/go2w/calibrate_camera.sh --board '$BOARD' \\
    --square-m '$SQUARE_M' --operator '$OPERATOR' \\
    --install-from /path/to/ost.yaml
EOF
