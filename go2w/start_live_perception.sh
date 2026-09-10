#!/usr/bin/env bash
set -eo pipefail

# Read-only live perception. No motion package, Sport request, lease holder,
# cmd_vel bridge, or Nav2 controller is launched by this script.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
go2w_interface="${GO2W_INTERFACE:-}"
if [[ -z "$go2w_interface" ]]; then
  for candidate in eth0 enp6s0 enp3s0 enp4s0 enp5s0; do
    if [[ -r "/sys/class/net/${candidate}/carrier" ]] \
      && [[ "$(< "/sys/class/net/${candidate}/carrier")" == "1" ]] \
      && ip -4 -o address show dev "$candidate" 2>/dev/null \
        | awk '$4 ~ /^192[.]168[.]123[.][0-9]+\// { found=1 } END { exit !found }'; then
      go2w_interface="$candidate"
      break
    fi
  done
fi
go2w_interface="${go2w_interface:-enp6s0}"
export GO2W_INTERFACE="$go2w_interface"
go2w_conda_python="${GO2W_CONDA_PYTHON:-${GO2W_CONTROL_PYTHON:-$project_root/unitree_go2w_control/.venv/bin/python}}"
go2w_sdk_path="${GO2W_SDK_PYTHON_PATH:-$project_root/unitree_go2w_control/vendor/unitree_sdk2_python}"
camera_source="${GO2W_CAMERA_SOURCE:-auto}"
d435_base_url="${D435_BASE_URL:-http://192.168.123.18:8080}"
session_id="live_$(date +%Y%m%d_%H%M%S)"
spool_root="${GO2W_FRAME_SPOOL_DIR:-${project_root}/runtime/go2w/spool}"
log_root="${project_root}/runtime/go2w/sessions/${session_id}"
pid_root="${project_root}/runtime/go2w/pids"
mkdir -p "${spool_root}" "${log_root}" "${pid_root}"

source "${script_dir}/setup_environment.sh"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [[ ! -r "/sys/class/net/${go2w_interface}/carrier" ]] \
  || [[ "$(< "/sys/class/net/${go2w_interface}/carrier")" != "1" ]]; then
  printf 'ERROR: %s has no Ethernet carrier; refusing a stale read-only session.\n' "$go2w_interface" >&2
  exit 2
fi
if ! ip -4 -o address show dev "$go2w_interface" \
  | awk '$4 ~ /^192[.]168[.]123[.][0-9]+\// { found=1 } END { exit !found }'; then
  printf 'ERROR: %s has no 192.168.123.0/24 host address.\n' "$go2w_interface" >&2
  exit 2
fi

if [[ "$camera_source" == auto ]]; then
  if env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY \
      -u ALL_PROXY -u all_proxy curl -fsS --max-time 2 \
      "$d435_base_url/health" >/dev/null 2>&1; then
    camera_source=d435
  else
    camera_source=builtin
  fi
fi

process_ids=()
start_read_only_node() {
  local name="$1"
  shift
  setsid "$@" >"${log_root}/${name}.log" 2>&1 &
  local process_id=$!
  process_ids+=("${process_id}")
  printf '%s\n' "${process_id}" >"${pid_root}/${name}.pid"
}

cleanup() {
  # ros2 run is a Python wrapper; its actual node can outlive the wrapper while
  # retaining the same owned process group. Address groups, verify them, and
  # use KILL only after bounded graceful INT/TERM windows.
  for process_id in "${process_ids[@]}"; do
    kill -INT -- "-${process_id}" 2>/dev/null || true
  done
  for _ in {1..25}; do
    groups_alive=0
    for process_id in "${process_ids[@]}"; do
      if kill -0 -- "-${process_id}" 2>/dev/null; then
        groups_alive=1
      fi
    done
    (( groups_alive == 0 )) && break
    sleep 0.1
  done
  for process_id in "${process_ids[@]}"; do
    kill -TERM -- "-${process_id}" 2>/dev/null || true
  done
  for _ in {1..25}; do
    groups_alive=0
    for process_id in "${process_ids[@]}"; do
      if kill -0 -- "-${process_id}" 2>/dev/null; then
        groups_alive=1
      fi
    done
    (( groups_alive == 0 )) && break
    sleep 0.1
  done
  for process_id in "${process_ids[@]}"; do
    if kill -0 -- "-${process_id}" 2>/dev/null; then
      kill -KILL -- "-${process_id}" 2>/dev/null || true
    fi
    wait "${process_id}" 2>/dev/null || true
  done
  for name in description camera time lidar fusion live_bridge; do
    rm -f "${pid_root}/${name}.pid"
  done
}
trap cleanup EXIT INT TERM

start_read_only_node description ros2 launch go2w_description official_sensor_frames.launch.py \
  "reference_file:=${project_root}/configs/go2w/official_reference.yaml"
if [[ "$camera_source" == d435 ]]; then
  start_read_only_node camera env -u http_proxy -u https_proxy \
    -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY -u all_proxy \
    /usr/bin/python3 \
    "${project_root}/scripts/go2w/realsense_rgbd_bridge.py" \
    --base-url "$d435_base_url" --rate "${GO2W_D435_RATE:-10}"
else
  start_read_only_node camera ros2 run go2w_camera_bridge camera_bridge --ros-args \
    -p source:=rpc \
    -p "interface:=${go2w_interface}" \
    -p "sdk_python_path:=${go2w_sdk_path}" \
    -p "rpc_worker_python:=${go2w_conda_python}" \
    -p "rpc_worker_script:=${project_root}/scripts/go2w/videohub_rpc_worker.py" \
    -p "calibration_file:=${project_root}/configs/go2w/camera_intrinsics.yaml"
fi
start_read_only_node time ros2 run go2w_sensor_time_bridge time_bridge --ros-args \
  -p "config_file:=${project_root}/configs/go2w/time_sync.yaml"
start_read_only_node lidar ros2 run go2w_lidar_preprocessor lidar_preprocessor --ros-args \
  -p "config_file:=${project_root}/configs/go2w/lidar_preprocess.yaml" \
  -p "geometry_file:=${project_root}/configs/go2w/official_reference.yaml"
start_read_only_node fusion ros2 run go2w_rgb_lidar_fusion fusion_node --ros-args \
  -p "fusion_config:=${project_root}/configs/go2w/rgb_lidar_fusion.yaml" \
  -p "camera_config:=${project_root}/configs/go2w/camera_intrinsics.yaml" \
  -p "extrinsics_config:=${project_root}/configs/go2w/sensor_extrinsics.yaml" \
  -p "cloud_topic:=/go2w/sensors/cloud"
start_read_only_node live_bridge ros2 run robot_scene_live_bridge live_bridge --ros-args \
  -p "spool_root:=${spool_root}" \
  -p "session_id:=${session_id}" \
  -p sensor_timeout_seconds:=0.3

printf 'Read-only Go2-W perception session: %s\n' "${session_id}"
printf 'Spool: %s\nLogs: %s\n' "${spool_root}" "${log_root}"
printf 'Camera source: %s\n' "${camera_source}"
printf 'Motion/lease/Nav2 execution nodes: NOT STARTED\n'
wait
