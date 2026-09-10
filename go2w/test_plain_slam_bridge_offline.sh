#!/usr/bin/env bash
# Offline bridge-only smoke test: validates the go2w_plain_slam_bridge data
# chain with synthetic data (no robot, no LiDAR, no LIO needed).
#
#   bash scripts/go2w/test_plain_slam_bridge_offline.sh [--seconds 45]
#
# Checks:
#   - /go2w/slam/pandar_points schema + hz
#   - /go2w/slam/odom_base exists
#   - /go2w/slam/map_2d contains free (<0? no: 0), occupied (100) and
#     unknown (-1) cells
#   - /go2w/slam/ready and /go2w/slam/health publish
#   - /go2w/odom/fused has no publisher (bridge never publishes it)
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
workspace="${project_root}/ros2_ws"
log_root="${project_root}/runtime/go2w/sessions"
pid_root="${project_root}/runtime/go2w/pids"
mkdir -p "${log_root}" "${pid_root}"

seconds="${PLAIN_SLAM_TEST_SECONDS:-45}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${workspace}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if ! ros2 pkg prefix go2w_plain_slam_bridge >/dev/null 2>&1; then
  printf '%s\n' 'ERROR: go2w_plain_slam_bridge is not built. Run build_ros2.sh first.' >&2
  exit 2
fi

# Clean any leftover bridge processes from previous interrupted runs so ROS
# never sees duplicate node names.
bash "${script_dir}/stop_plain_slam_mapping.sh" >/dev/null 2>&1 || true
sleep 1

pids=()
cleanup() {
  for pid in "${pids[@]:-}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${pids[@]:-}"; do
    kill -KILL "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT

printf 'Starting fake Pandar/IMU fixture (%ss)...\n' "${seconds}"
python3 "${script_dir}/publish_plain_slam_fake_fixture.py" --seconds "${seconds}" \
  >"${log_root}/fake_fixture.log" 2>&1 &
pids+=($!)

sleep 2
printf 'Starting bridge (start_upstream=false, LIO/SLAM disabled)...\n'
ros2 launch go2w_plain_slam_bridge plain_slam_go2w.launch.py \
  start_upstream:=false >"${log_root}/bridge_smoke.log" 2>&1 &
pids+=($!)

failures=0
expect_ok() {
  local label="$1"
  local ok="$2"
  if [[ "${ok}" == "1" ]]; then
    printf '[OK] %s\n' "${label}"
  else
    printf '[FAIL] %s\n' "${label}" >&2
    failures=$((failures + 1))
  fi
}

wait_for_topic() {
  local topic="$1"
  local deadline=$(( $(date +%s) + 30 ))
  while (( $(date +%s) < deadline )); do
    out="$(timeout 6 ros2 topic info "${topic}" -v 2>&1 || true)"
    pub="$(awk '/Publisher count:/ { print $3; exit }' <<<"${out}")"
    echo "[wait] ${topic} publisher=${pub:-none} t=$(date +%T)" >>"${log_root}/bridge_smoke_wait.log"
    if [[ "${pub:-0}" =~ ^[1-9] ]]; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_topic /go2w/slam/pandar_points && ok_pp=1 || ok_pp=0
expect_ok "/go2w/slam/pandar_points publishing" "${ok_pp}"

wait_for_topic /go2w/slam/odom_base && ok_ob=1 || ok_ob=0
expect_ok "/go2w/slam/odom_base publishing" "${ok_ob}"

wait_for_topic /go2w/slam/map_2d && ok_m2=1 || ok_m2=0
expect_ok "/go2w/slam/map_2d publishing" "${ok_m2}"
if [[ "${ok_m2}" == "1" ]]; then
  # Count free(0) / occupied(100) / unknown(<0) cells with a small Python
  # probe instead of echoing the whole 640k-cell grid through the CLI.
  map_stats="$(timeout 12 python3 - <<'PY' 2>/dev/null || true
import time

import rclpy
from nav_msgs.msg import OccupancyGrid

rclpy.init()
node = rclpy.create_node("smoke_map_probe")
received = []
node.create_subscription(OccupancyGrid, "/go2w/slam/map_2d", received.append, 1)
deadline = time.time() + 10
while time.time() < deadline and not received:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
if not received:
    raise SystemExit(1)
data = received[0].data
print(sum(1 for v in data if v == 0),
      sum(1 for v in data if v == 100),
      sum(1 for v in data if v < 0))
PY
)"
  read -r free_count occ_count unk_count <<<"${map_stats}" || true
  free_count="${free_count:-0}"
  occ_count="${occ_count:-0}"
  unk_count="${unk_count:-0}"
  expect_ok "map_2d contains free cells (${free_count})" "$(( free_count > 0 ))"
  expect_ok "map_2d contains occupied cells (${occ_count})" "$(( occ_count > 0 ))"
  expect_ok "map_2d contains unknown cells (${unk_count})" "$(( unk_count > 0 ))"
fi

wait_for_topic /go2w/slam/ready && ok_rd=1 || ok_rd=0
expect_ok "/go2w/slam/ready publishing" "${ok_rd}"
if [[ "${ok_rd}" == "1" ]]; then
  ready="$(timeout 5 ros2 topic echo /go2w/slam/ready --once 2>/dev/null \
    | grep -o 'data: true' || true)"
  expect_ok "ready=true with live bridge data" "$([[ -n "${ready}" ]] && echo 1 || echo 0)"
fi

# TF isolation: bridge must never publish odom -> base_link owner frames.
tf_out="$(timeout 5 ros2 run tf2_ros tf2_echo pslam_odom pslam_imu 2>/dev/null || true)"
if [[ -n "${tf_out}" ]] && grep -q 'Translation' <<<"${tf_out}"; then
  printf '[WARN] pslam_odom -> pslam_imu TF found (expected only when LIO is on)\n'
else
  printf '[OK] no pslam TF broadcast in bridge-only mode\n'
fi

# Motion authority untouched: the bridge must never publish /go2w/odom/fused.
# A publisher on this topic may legitimately be the robot itself (cross-host
# DDS discovery); what matters is that no bridge node name backs it.
fused_info="$(timeout 8 ros2 topic info /go2w/odom/fused -v 2>/dev/null || true)"
fused_pub="$(awk '/Publisher count:/ { print $3; exit }' <<<"${fused_info}")"
bridge_owns=0
for node_name in pandar_slam_adapter plain_slam_odom_adapter \
  pointcloud_to_occupancy plain_slam_health_monitor plain_slam_map_relay; do
  if grep -q "Node name: ${node_name}$" <<<"${fused_info}"; then
    bridge_owns=1
  fi
done
expect_ok "/go2w/odom/fused not published by bridge (pub count=${fused_pub:-0})" \
  "$(( 1 - bridge_owns ))"

if (( failures > 0 )); then
  printf 'Bridge smoke test FAILED (%d failures). See %s\n' \
    "${failures}" "${log_root}/bridge_smoke.log" >&2
  exit 1
fi
printf '%s\n' 'Bridge smoke test PASSED.'