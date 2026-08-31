#!/usr/bin/env bash
# Read-only health summary for the plain_slam mapping-assist pipeline.
# Publishes nothing, moves nothing, changes no authorization state.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
workspace="${project_root}/ros2_ws"

set +u
# shellcheck disable=SC1091
source "${script_dir}/setup_environment.sh"
set -u

topics=(
  /hesai/pandarxt16/points_raw
  /go2w/slam/pandar_points
  /utlidar/imu
  /go2w/slam/imu
  /go2w/slam/odom_base
  /go2w/slam/aligned_scan
  /go2w/slam/map_3d
  /go2w/slam/map_2d
)

for topic in "${topics[@]}"; do
  info="$(timeout 5 ros2 topic info "${topic}" -v 2>/dev/null || true)"
  pub_count="$(awk '/Publisher count:/ { print $3; exit }' <<<"${info}")"
  sub_count="$(awk '/Subscription count:/ { print $3; exit }' <<<"${info}")"
  printf '%-42s pub=%-3s sub=%-3s' "${topic}" "${pub_count:-n/a}" "${sub_count:-n/a}"
  if [[ "${pub_count:-0}" =~ ^[1-9] ]]; then
    hz="$(timeout 6 ros2 topic hz "${topic}" --window 3 2>/dev/null | head -n 1 || true)"
    printf ' hz=%s' "${hz:-n/a}"
  fi
  printf '\n'
done

printf '%s\n' '--- /go2w/slam/ready ---'
timeout 5 ros2 topic echo /go2w/slam/ready --once 2>/dev/null || printf 'no data\n'

printf '%s\n' '--- /go2w/slam/health (first status) ---'
timeout 5 ros2 topic echo /go2w/slam/health --once 2>/dev/null \
  | awk '/^  name:|^    level:|^    message:/ { print }' | head -n 12 \
  || printf 'no data\n'

printf '%s\n' '--- /go2w/odom/fused authority check ---'
info="$(timeout 5 ros2 topic info /go2w/odom/fused -v 2>/dev/null || true)"
printf '%s\n' "${info}" | grep -E 'Publisher count|Subscription count' || printf 'no data\n'
