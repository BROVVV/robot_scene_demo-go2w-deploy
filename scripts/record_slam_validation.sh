#!/usr/bin/env bash
# 录制 SLAM / 语义验证数据集（20260908 计划书 §18、§28、§29）。
#
# 用法（控制主机 /home/mxt/robotscene 下执行）：
#   bash scripts/record_slam_validation.sh --dataset A --duration 60
#   bash scripts/record_slam_validation.sh --dataset D --duration 90 --with-rgbd
#
# 本脚本只录包，绝不发送任何运动命令。运动由操作者按受控流程另行执行
# （见 docs/GO2W_SLAM重构_交接书_20260908.md §六）。
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
dataset="A"
duration=60
with_rgbd=0
with_raw=0
output_root="${project_root}/artifacts/slam_validation"

while (( $# )); do
  case "$1" in
    --dataset) dataset="$2"; shift 2 ;;
    --duration) duration="$2"; shift 2 ;;
    --with-rgbd) with_rgbd=1; shift ;;
    --with-raw) with_raw=1; shift ;;
    --output-root) output_root="$2"; shift 2 ;;
    *) printf 'unknown option: %s\n' "$1" >&2; exit 2 ;;
  esac
done

topics=(
  /go2w/slam/map_3d
  /go2w/slam/odom_base
  /go2w/slam/base_pose
  /go2w/slam/aligned_scan
  /go2w/slam/pose_graph_poses
  /go2w/slam/pose_graph_loop_edges
  /go2w/slam/health
  /go2w/slam/imu
  /go2w/odom/fused
  /tf
  /tf_static
)
if [[ "${with_raw}" == "1" ]]; then
  # 原始传感器流：`ros2 bag play` 重放 A/B 的唯一输入来源
  # （验证时间偏移 / 零偏等输入层改动的收益必须先有可重放数据）。
  topics+=(
    /hesai/pandarxt16/points_raw
    /utlidar/imu
  )
fi
if [[ "${with_rgbd}" == "1" ]]; then
  topics+=(
    /go2w/d435/color/image_raw
    /go2w/d435/depth/image_rect_raw
    /go2w/d435/color/camera_info
  )
fi

run_id="dataset${dataset}_$(date +%Y%m%d_%H%M%S)"
out="${output_root}/${run_id}"
mkdir -p "${out}"

# shellcheck disable=SC1091
source "${project_root}/scripts/go2w/setup_environment.sh" >/dev/null

printf '[record] dataset=%s duration=%ss -> %s\n' "${dataset}" "${duration}" "${out}"
ros2 bag record -o "${out}/bag" "${topics[@]}" >"${out}/record.log" 2>&1 &
rec_pid=$!
trap 'kill -INT "${rec_pid}" 2>/dev/null || true' EXIT

sleep "${duration}"

kill -INT "${rec_pid}" 2>/dev/null || true
wait "${rec_pid}" 2>/dev/null || true
trap - EXIT

printf '[record] done: %s/bag\n' "${out}"
ros2 bag info "${out}/bag" 2>&1 | tail -20
printf '[record] evaluate with:\n  bash scripts/run_slam_validation.sh %s\n' "${out}"
