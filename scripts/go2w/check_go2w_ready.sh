#!/usr/bin/env bash
# Go2-W 功能健康检查（operator-supervised experiment 前置探针）。
#
# 用法：
#   bash scripts/go2w/check_go2w_ready.sh            # 人类可读 + 机器可读 JSON
#   bash scripts/go2w/check_go2w_ready.sh --json     # 只输出机器可读 JSON
#
# 检查项（全部自动，不需要人工标定/摆场）：
#   network    robot-facing interface carrier + 192.168.123.0/24 + ping 机器人 192.168.123.18
#   sport      /lf/sportmodestate（mode=1、error_code=0）
#   odom       /go2w/odom/fused、/go2w/odom/wheel（20 Hz）
#   camera     /camera/front/image_raw（~15-28 Hz）与 CameraInfo
#   safety     /go2w/safety/lidar_fresh、front_clearance、rotation_clearance_valid
#   motion     /go2w/motion Action、/go2w/arm、/go2w/emergency_stop 服务
#   spool      最近 Frame Bundle（READY + sensor_health.camera/lidar）
#   llm        .env 中 SILICONFLOW_API_KEY 是否配置
#
# 退出码：0=ready（可开始实验） 1=degraded（非阻塞项缺失） 2=unreachable/配置错误

set -uo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
unitree_root="${GO2W_UNITREE_ROOT:-${HOME}/unitree_ros2}"
control_root="${GO2W_CONTROL_ROOT:-${project_root}/unitree_go2w_control}"
go2w_interface="${GO2W_INTERFACE:-}"
if [[ -z "$go2w_interface" ]]; then
  # The workstation normally uses enp3s0/enp6s0; the Jetson deployment uses
  # eth0.  Keep the probe portable so the same one-click check is useful on
  # both sides of the deployment.
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
go2w_interface="${go2w_interface:-eth0}"
export GO2W_INTERFACE="$go2w_interface"
json_only="0"
for arg in "$@"; do
  case "$arg" in
    --json) json_only="1" ;;
    *) printf 'Unknown argument: %s\n' "$arg" >&2; exit 2 ;;
  esac
done

checks="{}"
declare -A results

set_check() { results["$1"]="$2"; }

# ---------------------------------------------------------------------------
# 1. network
# ---------------------------------------------------------------------------
iface_carrier="0"
host_ip="none"
robot_reachable="false"
if [[ -r "/sys/class/net/${go2w_interface}/carrier" ]]; then
  iface_carrier="$(< "/sys/class/net/${go2w_interface}/carrier")"
fi
host_ip="$(ip -4 -o address show dev "$go2w_interface" 2>/dev/null | awk '{print $4}' | head -1)"
if [[ "${host_ip:-none}" == none ]]; then
  host_ip="none"
fi
if ping -c 1 -W 1 192.168.123.18 >/dev/null 2>&1; then
  robot_reachable="true"
fi
if [[ "${iface_carrier}" == "1" && "${robot_reachable}" == "true" ]]; then
  set_check network ok
else
  set_check network fail
fi

# ---------------------------------------------------------------------------
# ROS environment (best effort; topic checks are skipped when unavailable)
# ---------------------------------------------------------------------------
ros_ok="false"
set +u
# shellcheck disable=SC1091
source "${project_root}/scripts/go2w/setup_environment.sh" >/dev/null 2>&1 || true
set -u
command -v ros2 >/dev/null 2>&1 && ros_ok="true"

topic_alive() {
  # ros2 topic hz never exits; treat any captured rate output as alive.
  local out
  out="$(timeout 5 ros2 topic hz "$1" --window 2 2>/dev/null)"
  if [[ -n "${out}" && "${out}" != *"does not appear to be published yet"* ]]; then
    return 0
  fi
  # Foxy CLI can fail to print a rate when the endpoint is discovered through
  # a remote DDS participant.  The publisher-count fallback still verifies
  # that the expected live endpoint exists, while the WebUI performs the
  # stricter freshness checks from its own worker.
  timeout 5 ros2 topic info "$1" 2>/dev/null \
    | awk '/Publisher count:/ { found = ($3 + 0 > 0); exit } END { exit !found }' \
    && return 0
  return 1
}

service_ok() {
  local service="$1" node info
  while IFS= read -r node; do
    [[ -z "$node" ]] && continue
    info="$(timeout 5 ros2 node info "$node" 2>/dev/null || true)"
    if awk -v service="$service" '
      /^  Service Servers:/ { in_servers=1; next }
      /^  Service Clients:/ { in_servers=0 }
      in_servers && $1 == service ":" { found=1 }
      END { exit !found }
    ' <<<"$info"; then
      return 0
    fi
  done < <(timeout 5 ros2 node list 2>/dev/null || true)
  return 1
}

action_server_ok() {
  local info
  info="$(timeout 5 ros2 action info "$1" 2>/dev/null || true)"
  grep -Eq 'Action servers:[[:space:]]*[1-9][0-9]*' <<<"$info"
}

if [[ "${ros_ok}" == "true" ]]; then
  # sport mode
  sport_ok="false"
  # Foxy has no --once/--field options.  Read one bounded full sample and
  # extract the two scalar fields, which is compatible with Foxy and Humble.
  sport_sample="$(timeout 6 ros2 topic echo --qos-reliability reliable /lf/sportmodestate 2>/dev/null || true)"
  sport_mode="$(printf '%s\n' "$sport_sample" | awk '$1 == "mode:" {print $2; exit}' | tr -d '[:space:]')"
  sport_error="$(printf '%s\n' "$sport_sample" | awk '$1 == "error_code:" {print $2; exit}' | tr -d '[:space:]')"
  if [[ "${sport_mode}" == "1" && "${sport_error}" == "0" ]]; then
    sport_ok="true"
  fi
  set_check sport "${sport_ok}"

  # odom
  odom_fused="false"; odom_wheel="false"
  topic_alive /go2w/odom/fused && odom_fused="true"
  topic_alive /go2w/odom/wheel && odom_wheel="true"
  if [[ "${odom_fused}" == "true" || "${odom_wheel}" == "true" ]]; then
    set_check odom ok
  else
    set_check odom fail
  fi

  # camera
  camera="false"
  topic_alive /camera/front/image_raw && camera="true"
  set_check camera "${camera}"

  # safety topics
  lidar_fresh="false"
  fresh_value="$(timeout 6 ros2 topic echo --qos-reliability reliable /go2w/safety/lidar_fresh 2>/dev/null | awk '$1 == "data:" {print $2; exit}' | tr -d '[:space:]')"
  [[ "${fresh_value}" == "True" || "${fresh_value}" == "true" ]] && lidar_fresh="true"
  set_check lidar_fresh "${lidar_fresh}"
  rotation_clearance="false"
  rc_value="$(timeout 6 ros2 topic echo --qos-reliability reliable /go2w/safety/rotation_clearance_valid 2>/dev/null | awk '$1 == "data:" {print $2; exit}' | tr -d '[:space:]')"
  [[ "${rc_value}" == "True" || "${rc_value}" == "true" ]] && rotation_clearance="true"
  set_check rotation_clearance_valid "${rotation_clearance}"

  # motion action + services
  motion="false"
  action_server_ok /go2w/motion && motion="true"
  set_check motion_action "${motion}"
  set_check arm_service "$(service_ok /go2w/arm && echo true || echo false)"
  set_check emergency_stop_service "$(service_ok /go2w/emergency_stop && echo true || echo false)"
else
  set_check ros unavailable
fi

# ---------------------------------------------------------------------------
# spool bundle freshness (latest READY within 30 s)
# ---------------------------------------------------------------------------
spool_ok="false"
spool_root="${GO2W_FRAME_SPOOL_DIR:-${project_root}/runtime/go2w/spool}"
spool_latest="${spool_root}/latest"
if [[ -f "${spool_latest}/READY" && -f "${spool_latest}/frame_bundle.json" ]]; then
  age_sec="$(python3 - "${spool_latest}/READY" <<'PYEOF'
import os, sys, time
try:
    print(int(time.time() - os.path.getmtime(sys.argv[1])))
except OSError:
    print(99999)
PYEOF
)"
  if [[ "${age_sec}" -le 30 ]]; then
    spool_ok="true"
  fi
fi
set_check spool_bundle "${spool_ok}"

# ---------------------------------------------------------------------------
# LLM key
# ---------------------------------------------------------------------------
llm_key="false"
if [[ -f "${project_root}/.env" ]] && grep -q '^SILICONFLOW_API_KEY=.\+' "${project_root}/.env" 2>/dev/null; then
  llm_key="true"
fi
set_check llm_api_key "${llm_key}"

# ---------------------------------------------------------------------------
# aggregate
# ---------------------------------------------------------------------------
read_json="$(python3 - "${results[network]}" "${results[sport]:-na}" "${results[odom]:-na}" \
  "${results[camera]:-na}" "${results[lidar_fresh]:-na}" \
  "${results[rotation_clearance_valid]:-na}" "${results[motion_action]:-na}" \
  "${results[arm_service]:-na}" "${results[emergency_stop_service]:-na}" \
  "${results[spool_bundle]:-na}" "${results[llm_api_key]:-na}" "${results[ros]:-na}" <<'PYEOF'
import json, sys
network, sport, odom, camera, lidar, rot, motion, arm, stop, spool, llm, ros = sys.argv[1:13]
checks = {
    "network": {"ok": network == "ok", "carrier": network == "ok",
                "robot_ip": "192.168.123.18"},
    "sport_mode": {"ok": sport == "true"},
    "odom": {"ok": odom == "ok"},
    "camera": {"ok": camera == "true"},
    "lidar_fresh": {"ok": lidar == "true"},
    "rotation_clearance_valid": {"ok": rot == "true"},
    "motion_action": {"ok": motion == "true"},
    "arm_service": {"ok": arm == "true"},
    "emergency_stop_service": {"ok": stop == "true"},
    "spool_bundle": {"ok": spool == "true"},
    "llm_api_key": {"ok": llm == "true"},
    "ros_env": {"ok": ros == "true" if ros != "na" else True},
}
hard = ["network", "sport_mode", "camera", "motion_action", "emergency_stop_service",
        "llm_api_key"]
soft = ["odom", "lidar_fresh", "spool_bundle", "arm_service"]
ready = all(checks[k]["ok"] for k in hard)
degraded = [k for k in soft if not checks[k]["ok"]]
unreachable = checks["network"]["ok"] is False
state = "unreachable" if unreachable else ("ready" if ready else "degraded")
print(json.dumps({
    "state": state,
    # ``state=ready`` means every hard prerequisite is met.  The soft
    # checks remain visible in ``degraded`` and keep their exit-code 1 path,
    # but must not contradict the state by reporting ready=false here.
    "ready": ready,
    "degraded": degraded,
    "checks": checks,
    "backend": "go2w_experimental",
}, ensure_ascii=False))
PYEOF
)"

if [[ "${json_only}" == "1" ]]; then
  printf '%s\n' "${read_json}"
else
  printf '%s\n' "${read_json}" | python3 -m json.tool --no-ensure-ascii
  state="$(printf '%s\n' "${read_json}" | python3 -c 'import json,sys; print(json.load(sys.stdin)["state"])')"
  printf '\n状态: %s\n' "${state}"
  printf '提示: 机器狗未上电/未连接时 network=unreachable；充电并插好网线后重试。\n'
fi

case "$(printf '%s\n' "${read_json}" | python3 -c 'import json,sys; print(json.load(sys.stdin)["state"])')" in
  ready) exit 0 ;;
  degraded) exit 1 ;;
  *) exit 2 ;;
esac
