#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 4 ]]; then
  echo "usage: $0 OUTPUT_DIR DURATION_SECONDS OPERATOR SCENE_LABEL" >&2
  exit 2
fi
output_dir="$1"
duration="$2"
operator="$3"
scene_label="$4"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source /opt/ros/humble/setup.bash
source "${repo_root}/ros2_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${repo_root}/configs/go2w/cyclonedds_go2w.xml"

if ! [[ "${duration}" =~ ^[0-9]+$ ]] || (( duration < 10 )); then
  echo "duration must be an integer of at least 10 seconds" >&2
  exit 2
fi
read -r -p "确认 Go2-W 静止、区域清空且不执行任何运动命令；请输入 STATIONARY: " answer
if [[ "${answer}" != "STATIONARY" ]]; then
  echo "operator did not confirm stationary capture" >&2
  exit 3
fi

mkdir -p "${output_dir}"
metadata="${output_dir}/capture_metadata.yaml"
if [[ -e "${metadata}" ]]; then
  echo "refusing to overwrite existing capture: ${output_dir}" >&2
  exit 4
fi
{
  echo "robot_model: Unitree Go2-W"
  echo "operator: '${operator}'"
  echo "scene_label: '${scene_label}'"
  echo "stationary_confirmed: true"
  echo "duration_seconds: ${duration}"
  echo "captured_at: '$(date --iso-8601=seconds)'"
  echo "motion_commands_sent: false"
} > "${metadata}"

bag_log="${output_dir}/rosbag_record.log"
setsid ros2 bag record -o "${output_dir}/rosbag" \
  /camera/front/image_raw/compressed \
  /camera/front/camera_info \
  /go2w/sensors/cloud \
  /go2w/lidar/cloud_filtered \
  /go2w/lidar/obstacles \
  /go2w/sensors/time_status \
  /tf /tf_static >"${bag_log}" 2>&1 &
bag_pid=$!

cleanup_bag() {
  if kill -0 -- "-${bag_pid}" 2>/dev/null; then
    kill -INT -- "-${bag_pid}" 2>/dev/null || true
    for _ in {1..150}; do
      kill -0 -- "-${bag_pid}" 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM -- "-${bag_pid}" 2>/dev/null || true
    for _ in {1..50}; do
      kill -0 -- "-${bag_pid}" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL -- "-${bag_pid}" 2>/dev/null || true
  fi
  wait "${bag_pid}" 2>/dev/null || true
}
trap cleanup_bag EXIT INT TERM

bag_ready=false
for _ in {1..300}; do
  if ! kill -0 "${bag_pid}" 2>/dev/null; then
    echo "ros2 bag recorder exited before storage became ready" >&2
    tail -n 40 "${bag_log}" >&2 || true
    exit 5
  fi
  db3_path="$(find "${output_dir}/rosbag" -maxdepth 1 -type f -name '*.db3' \
    -print -quit 2>/dev/null || true)"
  if [[ -n "${db3_path}" ]]; then
    bag_ready=true
    break
  fi
  sleep 0.1
done
if [[ "${bag_ready}" != true ]]; then
  echo "ros2 bag storage was not ready within 30 seconds" >&2
  exit 6
fi

echo "ros2 bag storage ready; recording ${duration} measured seconds"
sleep "${duration}"
cleanup_bag
trap - EXIT INT TERM

if [[ ! -f "${output_dir}/rosbag/metadata.yaml" ]]; then
  echo "ros2 bag metadata.yaml is missing after graceful shutdown" >&2
  tail -n 40 "${bag_log}" >&2 || true
  exit 7
fi
echo "stationary RGB-LiDAR bag complete: ${output_dir}/rosbag"
