#!/usr/bin/env bash
# §9.4：有序重建 mapping session。
# plain_slam 上游没有 reset service，所以顺序必须是：停止地图写入 → 清网页快照
# → 有序重启 mapping launch → 等 IMU/LiDAR/odom/map 重新健康 → 新 session 才显示。
# 本脚本不发布任何命令话题，不改运动权限。
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
runtime_root="${project_root}/outputs/autonomous_search/runtime"
marker="${GO2W_SLAM_RESET_MARKER:-${runtime_root}/slam_reset.marker}"
snapshot="${GO2W_SLAM_MAP_SNAPSHOT:-${runtime_root}/slam_map_3d.json}"

set +u
# shellcheck disable=SC1091
source "${script_dir}/setup_environment.sh"
set -u

read_field() {
  /usr/bin/python3 - "${snapshot}" "$1" <<'PY'
import json
import sys

try:
    with open(sys.argv[1], encoding="utf-8") as handle:
        print(json.load(handle).get(sys.argv[2], ""))
except Exception:
    print("")
PY
}

session_before="$(read_field mapping_session_id)"
printf '[1/6] 停止地图写入并清空网页快照 (marker %s)\n' "${marker}"
mkdir -p "$(dirname "${marker}")"
touch "${marker}"

session_now="${session_before}"
for _ in $(seq 1 20); do
  session_now="$(read_field mapping_session_id)"
  if [[ -n "${session_now}" && "${session_now}" != "${session_before}" ]]; then
    break
  fi
  sleep 0.5
done
if [[ "${session_now}" == "${session_before}" ]]; then
  printf '[WARN] 网页桥没有确认新 session（桥可能没运行或在机器狗上）\n' >&2
fi
printf '[2/6] 快照已清空: session %s -> %s, available=%s\n' \
  "${session_before:-n/a}" "${session_now:-n/a}" "$(read_field available)"

printf '[3/6] 有序停止 mapping 栈（上游没有 reset service）\n'
bash "${script_dir}/stop_plain_slam_mapping.sh" || true
sleep 2

printf '[4/6] 重启 mapping 栈\n'
bash "${script_dir}/start_plain_slam_mapping.sh" "$@"

printf '[5/6] 等待 IMU / LiDAR / odom / map 重新健康\n'
health_ok=1
for topic in /go2w/slam/imu /go2w/slam/pandar_points /go2w/slam/odom_base \
             /go2w/slam/map_3d; do
  live=0
  for _ in $(seq 1 30); do
    if timeout 4 ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
      live=1
      break
    fi
  done
  if [[ "${live}" == "0" ]]; then
    health_ok=0
  fi
  printf '  %-32s %s\n' "${topic}" "$([[ "${live}" == "1" ]] && echo live || echo MISSING)"
done

printf '[6/6] 等待新 session 的第一张地图变成 HEALTHY\n'
for _ in $(seq 1 60); do
  if [[ "$(read_field mapping_health)" == "HEALTHY" ]]; then
    break
  fi
  sleep 1
done
printf 'mapping_session_id=%s mapping_health=%s revision=%s web_points=%s\n' \
  "$(read_field mapping_session_id)" "$(read_field mapping_health)" \
  "$(read_field map_revision)" "$(read_field web_display_points)"
printf 'health_reason=%s last_rejected=%s\n' \
  "$(read_field health_reason)" "$(read_field last_rejected_reason)"
printf '%s\n' '[NOTE] 网页桥若运行在机器狗上，需要在机器狗上 touch 同一个 marker。'
[[ "${health_ok}" == "1" ]]
