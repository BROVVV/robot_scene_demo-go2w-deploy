#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/remote_capture_env.sh"

ROBOT_IP=192.168.123.18
CAPTURE_ROOT="${1:-}"
if [[ -z "$CAPTURE_ROOT" ]]; then
  STAMP="$(date +%Y%m%d_%H%M%S)"
  CAPTURE_ROOT="$ROOT/captures/$STAMP"
fi
case "$CAPTURE_ROOT" in
  "$ROOT"/captures/*) ;;
  *) printf 'ERROR: capture root must be under %s/captures\n' "$ROOT" >&2; exit 2 ;;
esac

mkdir -p \
  "$CAPTURE_ROOT/metadata" \
  "$CAPTURE_ROOT/rosbag" \
  "$CAPTURE_ROOT/text" \
  "$CAPTURE_ROOT/pcap" \
  "$CAPTURE_ROOT/analysis" \
  "$CAPTURE_ROOT/checksums"
chmod 700 "$CAPTURE_ROOT"
printf '%s\n' "$CAPTURE_ROOT" >"$ROOT/.latest_remote_capture_attempt"
preflight_exit() {
  local rc=$?
  trap - EXIT
  if (( rc != 0 )); then
    printf 'PREFLIGHT=FAIL\nRETURN_CODE=%s\nCAPTURE_ROOT=%s\n' \
      "$rc" "$CAPTURE_ROOT" >"$CAPTURE_ROOT/metadata/preflight_result.txt"
  fi
  exit "$rc"
}
trap preflight_exit EXIT

ROBOT_IFACE="$(ip route get "$ROBOT_IP" | awk '{for(i=1;i<=NF;i++) if($i=="dev"){print $(i+1); exit}}')"
HOST_IP="$(ip route get "$ROBOT_IP" | awk '{for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}')"
[[ -n "$ROBOT_IFACE" && -n "$HOST_IP" ]]
ip link show dev "$ROBOT_IFACE" | grep -q 'state UP'
python3 - "$HOST_IP" "$ROBOT_IP" <<'PY'
import ipaddress, sys
host = ipaddress.ip_address(sys.argv[1])
robot = ipaddress.ip_address(sys.argv[2])
if host not in ipaddress.ip_network('192.168.123.0/24') or robot not in ipaddress.ip_network('192.168.123.0/24'):
    raise SystemExit('host or robot is outside 192.168.123.0/24')
PY

{
  printf 'CAPTURE_ROOT=%s\nROBOT_IFACE=%s\nHOST_IP=%s\nROBOT_IP=%s\n' \
    "$CAPTURE_ROOT" "$ROBOT_IFACE" "$HOST_IP" "$ROBOT_IP"
  ip -br addr
  ip route
  ip route get "$ROBOT_IP"
  ping -c 3 -W 1 "$ROBOT_IP"
} >"$CAPTURE_ROOT/metadata/network.txt" 2>&1

{
  printenv | rg '^(ROS|RMW|CYCLONEDDS|AMENT|COLCON)_' | sort || true
  printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-}"
  printf 'RMW_IMPLEMENTATION=%s\n' "$RMW_IMPLEMENTATION"
  printf 'ROS_DOMAIN_ID=%s\n' "$ROS_DOMAIN_ID"
  printf 'CYCLONEDDS_URI=%s\n' "$CYCLONEDDS_URI"
} >"$CAPTURE_ROOT/metadata/ros_environment.txt"

[[ "${ROS_DISTRO:-}" == humble ]]
[[ "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp ]]
[[ "$ROS_DOMAIN_ID" == 0 ]]
[[ "$CYCLONEDDS_URI" == *cyclonedds_go2w.xml ]]
xml_path="${CYCLONEDDS_URI#file://}"
xml_iface="$(sed -n 's/.*NetworkInterface name="\([^"]*\)".*/\1/p' "$xml_path" | head -n 1)"
[[ -n "$xml_iface" && "$xml_iface" == "$ROBOT_IFACE" ]] || {
  printf 'ERROR: CycloneDDS interface %s does not match route interface %s\n' \
    "${xml_iface:-missing}" "$ROBOT_IFACE" >&2
  exit 1
}

ps -eo pid,ppid,user,lstart,comm,args >"$CAPTURE_ROOT/metadata/processes_before.txt"
set +e
control_processes="$(ps -C python -C python3 -C safe_go2w_ros2_move -o pid=,comm=,args= | \
  awk '/hold_sport_lease|safe_sdk_move|safe_ros2_move|safe_sdk_posture|safe_go2w_ros2|posture_api_cycle|ros2[ ]topic[ ]pub|rqt_publisher/ {print}')"
set -e
if [[ -n "$control_processes" ]]; then
  printf 'ERROR: computer-side control process present:\n%s\n' "$control_processes" >&2
  exit 1
fi

timeout 15s ros2 topic list -t >"$CAPTURE_ROOT/metadata/topics_preflight.txt"
for entry in \
  '/wirelesscontroller unitree_go/msg/WirelessController' \
  '/wirelesscontroller_unprocessed unitree_go/msg/WirelessController' \
  '/api/sport/request unitree_api/msg/Request' \
  '/api/sport/response unitree_api/msg/Response' \
  '/lf/sportmodestate unitree_go/msg/SportModeState' \
  '/lf/lowstate unitree_go/msg/LowState'; do
  topic="${entry%% *}"
  type="${entry#* }"
  grep -Fxq "$topic [$type]" "$CAPTURE_ROOT/metadata/topics_preflight.txt"
done

timeout 12s ros2 topic info -v /api/sport/request \
  >"$CAPTURE_ROOT/metadata/sport_request_publishers_before.txt"

mkdir -p "$CAPTURE_ROOT/metadata/interfaces"
for type in \
  unitree_go/msg/WirelessController \
  unitree_api/msg/Request \
  unitree_api/msg/Response \
  unitree_go/msg/SportModeState \
  unitree_go/msg/LowState; do
  name="${type//\//_}"
  ros2 interface show "$type" >"$CAPTURE_ROOT/metadata/interfaces/$name.txt"
done

if git -C "$ROOT" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  git -C "$ROOT" status --short >"$CAPTURE_ROOT/metadata/git_status_before.txt"
  git -C "$ROOT" diff >"$CAPTURE_ROOT/metadata/git_diff_before.patch"
  git -C "$ROOT" rev-parse HEAD >"$CAPTURE_ROOT/metadata/project_git_head.txt"
else
  printf 'PROJECT_IS_NOT_A_GIT_WORKTREE\n' >"$CAPTURE_ROOT/metadata/git_status_before.txt"
fi

{
  for repo in /home/brov/unitree_ros2 "$ROOT/vendor/unitree_sdk2_python"; do
    [[ -d "$repo/.git" ]] || continue
    printf 'REPOSITORY=%s\n' "$repo"
    git -C "$repo" rev-parse HEAD
    git -C "$repo" status --short
    git -C "$repo" remote -v
  done
} >"$CAPTURE_ROOT/metadata/unitree_repositories.txt"

free_kb="$(df -Pk "$ROOT" | awk 'NR==2 {print $4}')"
(( free_kb >= 10 * 1024 * 1024 )) || {
  printf 'ERROR: less than 10 GiB free disk space\n' >&2
  exit 1
}

command -v tcpdump >"$CAPTURE_ROOT/metadata/tcpdump_path.txt"
if sudo -n true 2>/dev/null; then
  printf 'TCPDUMP_PRIVILEGE=sudo_noninteractive\n' >"$CAPTURE_ROOT/metadata/pcap_capability.txt"
elif [[ "$(id -u)" -eq 0 ]] || getcap "$(command -v tcpdump)" | rg -q 'cap_net_raw'; then
  printf 'TCPDUMP_PRIVILEGE=direct\n' >"$CAPTURE_ROOT/metadata/pcap_capability.txt"
else
  printf 'ERROR: tcpdump capture privilege is unavailable without prompting\n' >&2
  exit 1
fi

"$SCRIPT_DIR/verify_capture_is_read_only.sh" \
  "$CAPTURE_ROOT/metadata/read_only_audit.txt"
"$SCRIPT_DIR/snapshot_ros_graph.sh" \
  "$CAPTURE_ROOT/metadata/ros_graph_preflight"

printf '%s\n' "$CAPTURE_ROOT" >"$ROOT/.latest_remote_capture"
printf 'PREFLIGHT=PASS\nCAPTURE_ROOT=%s\nROBOT_IFACE=%s\n' \
  "$CAPTURE_ROOT" "$ROBOT_IFACE" | tee "$CAPTURE_ROOT/metadata/preflight_result.txt"
trap - EXIT
