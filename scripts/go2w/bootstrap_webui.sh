#!/usr/bin/env bash
# One-command Go2-W bootstrap after the robot is powered on.
#
# Examples:
#   bash scripts/go2w/bootstrap_webui.sh --mode mapping --detector groundingdino
#   bash scripts/go2w/bootstrap_webui.sh --mode mapping --detector yoloe
#   bash scripts/go2w/bootstrap_webui.sh --mode search  --detector groundingdino
#
# Search is plan-only by default.  Physical execution still requires both
# --enable-motion and --operator-supervised-rotation plus all existing gates.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
cd "$project_root"

mode=""
detector="grounded_sam"
enable_motion=0
operator_rotation=0
open_browser=1
while (( $# )); do
  case "$1" in
    --mode) mode="${2:?--mode requires search or mapping}"; shift 2 ;;
    --mode=*) mode="${1#*=}"; shift ;;
    --detector) detector="${2:?--detector requires grounded_sam or yoloe_seg}"; shift 2 ;;
    --detector=*) detector="${1#*=}"; shift ;;
    --enable-motion) enable_motion=1; shift ;;
    --operator-supervised-rotation) operator_rotation=1; shift ;;
    --no-browser) open_browser=0; shift ;;
    -h|--help)
      awk 'NR > 1 { if (/^#/) { sub(/^# ?/, ""); print } else exit }' "$0"
      exit 0
      ;;
    *) printf 'ERROR: unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done

[[ "$mode" == search || "$mode" == mapping ]] || {
  echo 'ERROR: --mode must be search or mapping' >&2; exit 2;
}
if [[ "$detector" == groundingdino ]]; then
  detector=grounded_sam
elif [[ "$detector" == yoloe ]]; then
  detector=yoloe_seg
fi
[[ "$detector" == grounded_sam || "$detector" == yoloe_seg ]] || {
  echo 'ERROR: --detector must be groundingdino/grounded_sam or yoloe/yoloe_seg' >&2; exit 2;
}
if [[ "$mode" != search && ( "$enable_motion" == 1 || "$operator_rotation" == 1 ) ]]; then
  echo 'ERROR: motion options are only valid with --mode search' >&2
  exit 2
fi
if [[ "$enable_motion" != "$operator_rotation" ]]; then
  echo 'ERROR: real search execution requires both --enable-motion and --operator-supervised-rotation' >&2
  exit 2
fi

robot_ip="${ROBOT_IP:-192.168.123.18}"
reconnect_timeout_s="${REMOTE_MAPPING_RECONNECT_TIMEOUT_SECONDS:-180}"
rgbd_recovery_timeout_s="${REMOTE_MAPPING_RGBD_RECOVERY_TIMEOUT_SECONDS:-90}"
[[ "${reconnect_timeout_s}" =~ ^[1-9][0-9]*$ ]] || {
  echo 'ERROR: REMOTE_MAPPING_RECONNECT_TIMEOUT_SECONDS must be a positive integer.' >&2
  exit 2
}
[[ "${rgbd_recovery_timeout_s}" =~ ^[1-9][0-9]*$ ]] || {
  echo 'ERROR: REMOTE_MAPPING_RGBD_RECOVERY_TIMEOUT_SECONDS must be a positive integer.' >&2
  exit 2
}

robot_endpoint_reachable() {
  ping -c 1 -W 1 "${robot_ip}" >/dev/null 2>&1 \
    || curl --noproxy '*' -fsS --connect-timeout 1 --max-time 2 \
      "http://${robot_ip}:8080/health" >/dev/null 2>&1 \
    || timeout 2 bash -c 'exec 3<>"/dev/tcp/${1}/22"' _ "${robot_ip}" \
      >/dev/null 2>&1
}

printf '== Robot network\n'
# Replugging Ethernet is asynchronous: carrier, the 192.168.123.x address and
# the route can recover at different times.  A one-shot ping turns that normal
# settling window into a failed launch.  Wait here so the operator may start
# the launcher immediately after inserting the cable (or even just before it).
bash "${script_dir}/wait_robot_reconnect.sh" \
  --robot-ip "${robot_ip}" --timeout "${reconnect_timeout_s}"

printf '== Robot RGB-D service\n'
if ! curl --noproxy '*' -fsS --max-time 3 \
    "http://${robot_ip}:8080/health" >/dev/null 2>&1; then
  echo 'RGB-D service is not ready; recovering the robot systemd service...'
  rgbd_deadline=$((SECONDS + rgbd_recovery_timeout_s))
  rgbd_next_action=0
  rgbd_attempt=0
  rgbd_ready=0
  while (( SECONDS < rgbd_deadline )); do
    if curl --noproxy '*' -fsS --connect-timeout 1 --max-time 3 \
        "http://${robot_ip}:8080/health" >/dev/null 2>&1; then
      rgbd_ready=1
      break
    fi
    if (( SECONDS >= rgbd_next_action )) && robot_endpoint_reachable; then
      rgbd_attempt=$((rgbd_attempt + 1))
      if (( rgbd_attempt == 1 )); then
        rgbd_action=start
      else
        rgbd_action=restart
      fi
      printf '[WAIT] RGB-D 未就绪，执行第 %s 次 %s……\n' \
        "${rgbd_attempt}" "${rgbd_action}"
      ROBOT_IP="${robot_ip}" \
        bash "${script_dir}/start_realsense_stream.sh" "${rgbd_action}" \
        >/dev/null 2>&1 || true
      rgbd_next_action=$((SECONDS + 20))
    fi
    sleep 2
  done
  [[ "${rgbd_ready}" == 1 ]] || {
    echo "ERROR: RGB-D service did not become healthy within ${rgbd_recovery_timeout_s}s." >&2
    exit 3
  }
fi
echo '[OK] RGB-D service healthy'

# Mapping mode does not pass through the Nav2 launcher, so bootstrap its
# read-only perception owner here.  Search mode already performs the same
# unique-publisher/ownership checks inside start_nav2_autonomous_search.sh.
if [[ "$mode" == mapping ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${script_dir}/setup_environment.sh"
  set -u
  cloud_count="$(timeout 5 ros2 topic info /go2w/sensors/cloud 2>/dev/null \
    | awk '/Publisher count:/ {print $3; exit}' || true)"
  cloud_count="${cloud_count:-0}"
  if (( cloud_count > 1 )); then
    echo 'ERROR: /go2w/sensors/cloud has duplicate publishers.' >&2
    exit 3
  fi
  if (( cloud_count == 0 )); then
    leftovers=()
    for owner_name in description camera time lidar lidar_watchdog fusion live_bridge start_live_perception; do
      owner_file="${project_root}/runtime/go2w/pids/${owner_name}.pid"
      [[ -f "$owner_file" ]] || continue
      owner_pid="$(tr -cd '0-9' <"$owner_file")"
      [[ -n "$owner_pid" ]] && kill -0 "$owner_pid" 2>/dev/null \
        && leftovers+=("${owner_name}:${owner_pid}")
    done
    if (( ${#leftovers[@]} )); then
      printf 'ERROR: perception is partially alive (%s); refusing duplicates.\n' \
        "${leftovers[*]}" >&2
      echo 'Run scripts/go2w/stop_all.sh, then retry this one-command launcher.' >&2
      exit 3
    fi
    mkdir -p runtime/go2w/pids runtime/go2w/sessions
    # Write ownership before spawning so even a terminal/SIGKILL in the tiny
    # launch window leaves enough evidence for the next one-click cleanup.
    if [[ -n "${GO2W_BOOTSTRAP_OWNER_MARKER:-}" ]]; then
      mkdir -p "$(dirname -- "${GO2W_BOOTSTRAP_OWNER_MARKER}")"
      : >"${GO2W_BOOTSTRAP_OWNER_MARKER}"
    fi
    setsid bash "${script_dir}/start_live_perception.sh" \
      >runtime/go2w/sessions/bootstrap_live_perception.log 2>&1 &
    perception_pid=$!
    printf '%s\n' "$perception_pid" >runtime/go2w/pids/start_live_perception.pid
    cloud_ready=0
    for _ in $(seq 1 120); do
      count="$(timeout 3 ros2 topic info /go2w/sensors/cloud 2>/dev/null \
        | awk '/Publisher count:/ {print $3; exit}' || true)"
      if [[ "${count:-0}" == 1 ]]; then cloud_ready=1; break; fi
      kill -0 "$perception_pid" 2>/dev/null || break
      sleep 1
    done
    if [[ "$cloud_ready" != 1 ]]; then
      echo 'ERROR: read-only perception did not publish /go2w/sensors/cloud.' >&2
      echo 'See runtime/go2w/sessions/bootstrap_live_perception.log' >&2
      exit 3
    fi
  fi
  echo '[OK] read-only perception ready'
fi

export GO2W_WEBUI_MODE="$([[ "$mode" == mapping ]] && echo teleop_mapping || echo autonomous_search)"
export GO2W_ACTIVE_DETECTOR="$detector"
export GO2W_RUNTIME_ADMIN_ENABLED=1

if [[ "$mode" == mapping ]]; then
  args=(--map-mode 2d --detector "$detector")
  [[ "$open_browser" == 1 ]] || args+=(--no-browser)
  exec bash "${script_dir}/start_remote_control_mapping.sh" "${args[@]}"
fi

args=("--detector=${detector}")
[[ "$open_browser" == 1 ]] || args+=(--no-browser)
if [[ "$enable_motion" == 1 ]]; then
  args+=(--enable-motion --operator-supervised-rotation)
fi
exec bash "${script_dir}/start_nav2_autonomous_search.sh" "${args[@]}"
