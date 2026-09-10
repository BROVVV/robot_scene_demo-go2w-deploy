#!/usr/bin/env bash
set -euo pipefail

# Diagnostic-only external LiDAR bring-up. This script does not launch Sport,
# a motion Action, a lease holder, Nav2, or any project safety publisher.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
workspace="${project_root}/ros2_ws"

set +u
# shellcheck disable=SC1091
source "${script_dir}/setup_environment.sh"
set -u

config="${GO2W_HESAI_CONFIG:-${project_root}/configs/go2w/hesai_pandarxt16.yaml}"
interface="${GO2W_HESAI_INTERFACE:-${GO2W_INTERFACE}}"
host_address="${GO2W_HESAI_HOST_ADDRESS:-${GO2W_HOST_IP}}"
device_address="${GO2W_HESAI_DEVICE_ADDRESS:-192.168.123.20}"
preprocess_config="${GO2W_HESAI_PREPROCESS_CONFIG:-${project_root}/configs/go2w/hesai_pandarxt16_preprocess.yaml}"
with_preprocessor="${GO2W_HESAI_WITH_PREPROCESSOR:-0}"
while (( $# )); do
  case "$1" in
    --with-preprocessor) with_preprocessor=1; shift ;;
    *) break ;;
  esac
done

if [[ ! -r "${config}" ]]; then
  printf 'ERROR: missing Hesai configuration: %s\n' "${config}" >&2
  exit 2
fi
if [[ ! -r "/sys/class/net/${interface}/carrier" ]] \
  || [[ "$(< "/sys/class/net/${interface}/carrier")" != "1" ]]; then
  printf 'ERROR: %s has no Ethernet carrier.\n' "${interface}" >&2
  exit 2
fi
if ! ip -4 -o address show dev "${interface}" \
  | awk -v address="${host_address}" '$4 == address "/24" { found=1 } END { exit !found }'; then
  printf 'ERROR: %s does not own %s/24.\n' "${interface}" "${host_address}" >&2
  exit 2
fi
if ! ip route get "${device_address}" \
  | awk -v dev="${interface}" -v src="${host_address}" \
      '$0 ~ ("dev " dev " ") && $0 ~ ("src " src "([[:space:]]|$)") { found=1 } END { exit !found }'; then
  printf 'ERROR: route to %s is not pinned to %s with source %s.\n' \
    "${device_address}" "${interface}" "${host_address}" >&2
  exit 2
fi
if [[ ! -r "${workspace}/install/setup.bash" ]]; then
  printf 'ERROR: ROS workspace is not built: %s\n' "${workspace}" >&2
  exit 2
fi

if ! ros2 pkg prefix hesai_ros_driver >/dev/null 2>&1; then
  printf '%s\n' 'ERROR: hesai_ros_driver is not installed in this workspace.' >&2
  exit 2
fi

printf 'Starting diagnostic-only PandarXT-16 stream from %s to %s.\n' \
  "${device_address}" "${host_address}"
printf '%s\n' 'Output: /hesai/pandarxt16/points_raw (unvalidated sensor frame)'
printf '%s\n' 'Motion/safety integration: DISABLED'
ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args \
  -p "config_path:=${config}" &
driver_pid=$!
if [[ "${with_preprocessor}" == "1" ]]; then
  if [[ ! -r "${preprocess_config}" ]]; then
    printf 'ERROR: missing Pandar preprocess configuration: %s\n' "${preprocess_config}" >&2
    kill -TERM "${driver_pid}" 2>/dev/null || true
    exit 2
  fi
  printf '%s\n' 'Also starting diagnostic preprocessor (zero-return filter + status).'
  ros2 run go2w_lidar_preprocessor hesai_pandarxt16_preprocessor --ros-args \
    -p "config_file:=${preprocess_config}" &
  preprocess_pid=$!
  trap 'kill -INT ${driver_pid} ${preprocess_pid} 2>/dev/null || true' EXIT INT TERM
  wait "${preprocess_pid}"
  kill -TERM "${driver_pid}" 2>/dev/null || true
  wait "${driver_pid}" 2>/dev/null || true
else
  wait "${driver_pid}"
fi
