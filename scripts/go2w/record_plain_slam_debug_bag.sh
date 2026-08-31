#!/usr/bin/env bash
# Record a debug rosbag of the plain_slam mapping topics (optional helper).
# Requires the pipeline to be running.  Only reads topics; changes nothing.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
workspace="${project_root}/ros2_ws"
bag_root="${project_root}/outputs/bags"
mkdir -p "${bag_root}"

set +u
# shellcheck disable=SC1091
source "${script_dir}/setup_environment.sh"
set -u

bag_dir="${bag_root}/plain_slam_$(date +%Y%m%d_%H%M%S)"
printf 'Recording to %s (Ctrl-C to stop)\n' "${bag_dir}"
exec ros2 bag record -o "${bag_dir}" \
  /hesai/pandarxt16/points_raw \
  /go2w/slam/pandar_points \
  /utlidar/imu \
  /go2w/slam/imu \
  /go2w/slam/imu_pose_raw \
  /go2w/slam/imu_odom_raw \
  /go2w/slam/odom_base \
  /go2w/slam/aligned_scan \
  /go2w/slam/deskewed_scan \
  /go2w/slam/lio_map_cloud \
  /go2w/slam/map_3d \
  /go2w/slam/map_2d \
  /go2w/slam/health \
  /go2w/slam/ready \
  /go2w/slam/point_status \
  /go2w/slam/occupancy_status
