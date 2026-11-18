#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/remote_capture_env.sh"

CAPTURE_ROOT=""
ROUND=""
ACTION=""
DRY_RUN=false
while (($#)); do
  case "$1" in
    --capture-root) CAPTURE_ROOT="$2"; shift 2 ;;
    --round) ROUND="$2"; shift 2 ;;
    --action) ACTION="$2"; shift 2 ;;
    --dry-run) DRY_RUN=true; shift ;;
    *) printf 'ERROR: unknown argument %s\n' "$1" >&2; exit 2 ;;
  esac
done
[[ -n "$CAPTURE_ROOT" && -n "$ROUND" && -n "$ACTION" ]]
case "$CAPTURE_ROOT" in "$ROOT"/captures/*) ;; *) exit 2 ;; esac
[[ "$ROUND" =~ ^[A-Za-z0-9_-]+$ ]]
case "$ACTION" in idle_baseline|remote_key_calibration|remote_stand_down|down_idle|remote_stand_up|up_idle|dry_run) ;; *) exit 2 ;; esac

ROBOT_IP=192.168.123.18
ROBOT_IFACE="$($SCRIPT_DIR/detect_unitree_interface.sh)"
PID_FILE="$CAPTURE_ROOT/metadata/${ROUND}_active_capture.pids"
BAG_PATH="$CAPTURE_ROOT/rosbag/$ROUND"
PCAP_PATH="$CAPTURE_ROOT/pcap/${ROUND}.pcap"
TIMELINE="$CAPTURE_ROOT/text/${ROUND}_combined_timeline.jsonl"
KEY_PREFIX="$CAPTURE_ROOT/text/${ROUND}_remote_keys"
MARKERS="$CAPTURE_ROOT/text/${ROUND}_operator_markers.jsonl"
MANIFEST="$CAPTURE_ROOT/metadata/${ROUND}_round_manifest.json"
START_TIME="$(date --iso-8601=ns)"
: >"$PID_FILE"
ps -eo pid,ppid,user,lstart,comm,args \
  >"$CAPTURE_ROOT/metadata/${ROUND}_processes_before.txt"

cleanup() {
  local rc=$?
  trap - EXIT INT TERM HUP
  "$SCRIPT_DIR/stop_remote_capture.sh" "$PID_FILE" || rc=1
  "$SCRIPT_DIR/snapshot_ros_graph.sh" \
    "$CAPTURE_ROOT/metadata/ros_graph_after/$ROUND" >/dev/null || rc=1
  ps -eo pid,ppid,user,lstart,comm,args \
    >"$CAPTURE_ROOT/metadata/${ROUND}_processes_after.txt"
  exit "$rc"
}
trap cleanup EXIT INT TERM HUP

"$SCRIPT_DIR/snapshot_ros_graph.sh" \
  "$CAPTURE_ROOT/metadata/ros_graph_before/$ROUND" >/dev/null
mkdir -p "$CAPTURE_ROOT/metadata/topic_qos/$ROUND"
for topic in \
  /wirelesscontroller \
  /wirelesscontroller_unprocessed \
  /api/sport/request \
  /api/sport/response \
  /lf/sportmodestate \
  /lf/lowstate; do
  safe_name="${topic#/}"
  safe_name="${safe_name//\//_}"
  timeout 12s ros2 topic info -v "$topic" \
    >"$CAPTURE_ROOT/metadata/topic_qos/$ROUND/${safe_name}.txt" 2>&1 || true
done

python3 - "$MANIFEST" "$ROUND" "$ACTION" "$START_TIME" "$ROBOT_IFACE" "$BAG_PATH" "$PCAP_PATH" <<'PY'
import json, sys
path, round_name, action, start, iface, bag, pcap = sys.argv[1:]
data = {
    "round": round_name,
    "action": action,
    "operator_trigger_source": "official_remote" if action.startswith("remote_") else "none",
    "computer_control_publishers_enabled": False,
    "sport_lease_acquired_by_computer": False,
    "start_time": start,
    "operator_action_time": None,
    "end_time": None,
    "robot_interface": iface,
    "robot_ip": "192.168.123.18",
    "bag_path": bag,
    "pcap_path": pcap,
    "notes": "",
}
with open(path, "w", encoding="utf-8") as handle:
    json.dump(data, handle, ensure_ascii=False, indent=2)
    handle.write("\n")
PY

ros2 bag record \
  --qos-profile-overrides-path "$ROOT/config/remote_capture_qos.yaml" \
  --max-bag-duration 60 \
  --max-bag-size 2147483648 \
  -o "$BAG_PATH" \
  /wirelesscontroller \
  /wirelesscontroller_unprocessed \
  /api/sport/request \
  /api/sport/response \
  /lf/sportmodestate \
  /lf/lowstate \
  /rosout \
  /parameter_events \
  >"$CAPTURE_ROOT/text/${ROUND}_rosbag_record.log" 2>&1 &
printf 'rosbag %s\n' "$!" >>"$PID_FILE"

python3 "$SCRIPT_DIR/monitor_remote_keys.py" \
  --output-prefix "$KEY_PREFIX" \
  >"$CAPTURE_ROOT/text/${ROUND}_remote_keys.log" 2>&1 &
printf 'remote_keys %s\n' "$!" >>"$PID_FILE"

python3 "$SCRIPT_DIR/capture_remote_timeline.py" \
  --output "$TIMELINE" \
  >"$CAPTURE_ROOT/text/${ROUND}_timeline.log" 2>&1 &
printf 'timeline %s\n' "$!" >>"$PID_FILE"

if sudo -n true 2>/dev/null; then
  sudo -n tcpdump -i "$ROBOT_IFACE" -s 0 -U -w "$PCAP_PATH" \
    'net 192.168.123.0/24 and (udp or arp or icmp)' \
    >"$CAPTURE_ROOT/text/${ROUND}_tcpdump.log" 2>&1 &
else
  tcpdump -i "$ROBOT_IFACE" -s 0 -U -w "$PCAP_PATH" \
    'net 192.168.123.0/24 and (udp or arp or icmp)' \
    >"$CAPTURE_ROOT/text/${ROUND}_tcpdump.log" 2>&1 &
fi
printf 'tcpdump %s\n' "$!" >>"$PID_FILE"

sleep 2
while read -r label pid; do
  kill -0 "$pid" 2>/dev/null || {
    printf 'ERROR: %s capture process exited early\n' "$label" >&2
    exit 1
  }
done <"$PID_FILE"

if [[ "$DRY_RUN" == true || "$ACTION" == dry_run ]]; then
  sleep 5
elif [[ "$ACTION" == idle_baseline ]]; then
  printf '保持机器人站立静止 15 秒；不要操作遥控器。\n'
  sleep 15
elif [[ "$ACTION" == down_idle || "$ACTION" == up_idle ]]; then
  printf '保持机器人当前姿态静止 12 秒；不要操作遥控器。\n'
  sleep 12
elif [[ "$ACTION" == remote_key_calibration ]]; then
  printf '%s\n' \
    '本轮只标定操作者确认安全的单键短按。' \
    '每个键输入 PRESS 才会进入记录提示；不确定或可能触发动作时输入 SKIP。'
  sleep 3
  for button in A B X Y L1 L2 R1 R2; do
    read -r -p "确认单独短按 $button 安全：输入 PRESS 或 SKIP: " decision
    if [[ "$decision" != PRESS ]]; then
      printf 'SKIPPED button=%s\n' "$button"
      continue
    fi
    python3 - "$MARKERS" "$ROUND" "$button" calibration_key_imminent <<'PY'
import json, sys, time
from datetime import datetime, timezone
path, round_name, button, event = sys.argv[1:]
record = {
    "round": round_name,
    "action": "remote_key_calibration",
    "event": event,
    "button": button,
    "receive_monotonic_ns": time.monotonic_ns(),
    "receive_wall_time": datetime.now(timezone.utc).astimezone().isoformat(timespec="microseconds"),
}
with open(path, "a", encoding="utf-8") as handle:
    handle.write(json.dumps(record, ensure_ascii=False) + "\n")
PY
    read -r -t 15 -p "现在短按并释放 $button，完成后按 Enter: " _ || {
      printf 'ERROR: key calibration timed out for %s\n' "$button" >&2
      exit 1
    }
    python3 - "$MARKERS" "$ROUND" "$button" calibration_key_complete <<'PY'
import json, sys, time
from datetime import datetime, timezone
path, round_name, button, event = sys.argv[1:]
record = {
    "round": round_name,
    "action": "remote_key_calibration",
    "event": event,
    "button": button,
    "receive_monotonic_ns": time.monotonic_ns(),
    "receive_wall_time": datetime.now(timezone.utc).astimezone().isoformat(timespec="microseconds"),
}
with open(path, "a", encoding="utf-8") as handle:
    handle.write(json.dumps(record, ensure_ascii=False) + "\n")
PY
    sleep 2
  done
else
  printf '动作前静止记录 8 秒；不要操作遥控器。\n'
  sleep 8
  if [[ "$ACTION" == remote_stand_down ]]; then
    if [[ "$ROUND" == D3 ]]; then
      prompt='输入 START_CAPTURE 后立即使用官方遥控器执行趴下组合；本轮可略微延长组合保持时间，但不要快速连按。'
    else
      prompt='输入 START_CAPTURE 后立即使用官方遥控器执行一次已知有效的趴下组合。'
    fi
  else
    if [[ "$ROUND" == U3 ]]; then
      prompt='输入 START_CAPTURE 后立即使用官方遥控器执行起立组合；本轮可略微延长组合保持时间，但不要快速连按。'
    else
      prompt='输入 START_CAPTURE 后立即使用官方遥控器执行一次已知有效的起立组合。'
    fi
  fi
  printf '%s\n' "$prompt"
  read -r -t 180 start_marker || {
    printf 'ERROR: operator marker timed out\n' >&2
    exit 1
  }
  [[ "$start_marker" == START_CAPTURE ]] || {
    printf 'ERROR: expected START_CAPTURE marker\n' >&2
    exit 1
  }
  python3 - "$MARKERS" "$MANIFEST" "$ROUND" "$ACTION" <<'PY'
import json, sys, time
from datetime import datetime, timezone
path, manifest, round_name, action = sys.argv[1:]
record = {
    "round": round_name,
    "action": action,
    "event": "operator_action_imminent",
    "receive_monotonic_ns": time.monotonic_ns(),
    "receive_wall_time": datetime.now(timezone.utc).astimezone().isoformat(timespec="microseconds"),
}
with open(path, "a", encoding="utf-8") as handle:
    handle.write(json.dumps(record, ensure_ascii=False) + "\n")
with open(manifest, encoding="utf-8") as handle:
    data = json.load(handle)
data["operator_action_time"] = record["receive_wall_time"]
with open(manifest, "w", encoding="utf-8") as handle:
    json.dump(data, handle, ensure_ascii=False, indent=2)
    handle.write("\n")
PY
  printf '请立即操作遥控器；机器人完全达到目标姿态后输入 POSTURE_STABLE。异常时只使用遥控器处置。\n'
  read -r -t 180 stable_marker || {
    printf 'ERROR: robot posture completion marker timed out\n' >&2
    exit 1
  }
  [[ "$stable_marker" == POSTURE_STABLE ]] || {
    printf 'ERROR: expected POSTURE_STABLE marker\n' >&2
    exit 1
  }
  python3 - "$MARKERS" "$ROUND" "$ACTION" <<'PY'
import json, sys, time
from datetime import datetime, timezone
path, round_name, action = sys.argv[1:]
record = {
    "round": round_name,
    "action": action,
    "event": "operator_reports_posture_stable",
    "receive_monotonic_ns": time.monotonic_ns(),
    "receive_wall_time": datetime.now(timezone.utc).astimezone().isoformat(timespec="microseconds"),
}
with open(path, "a", encoding="utf-8") as handle:
    handle.write(json.dumps(record, ensure_ascii=False) + "\n")
PY
  printf '正在记录动作后的稳定状态 10 秒。异常时只使用遥控器处置。\n'
  sleep 10
fi

"$SCRIPT_DIR/stop_remote_capture.sh" "$PID_FILE"
trap - EXIT INT TERM HUP
"$SCRIPT_DIR/snapshot_ros_graph.sh" \
  "$CAPTURE_ROOT/metadata/ros_graph_after/$ROUND" >/dev/null
ps -eo pid,ppid,user,lstart,comm,args \
  >"$CAPTURE_ROOT/metadata/${ROUND}_processes_after.txt"

ros2 bag info "$BAG_PATH" >"$CAPTURE_ROOT/analysis/${ROUND}_bag_info.txt"
[[ -s "$TIMELINE" ]]
[[ -s "$PCAP_PATH" ]]
for topic in /wirelesscontroller /lf/sportmodestate /lf/lowstate; do
  grep -Fq "Topic: $topic" "$CAPTURE_ROOT/analysis/${ROUND}_bag_info.txt"
done
count_for_topic() {
  local topic="$1"
  local value
  value="$(awk -v topic="$topic" '
    index($0, "Topic: " topic " |") {
      for (i = 1; i <= NF; i++) if ($i == "Count:") {print $(i + 1); exit}
    }
  ' "$CAPTURE_ROOT/analysis/${ROUND}_bag_info.txt")"
  printf '%s\n' "${value:-0}"
}
wireless_count="$((
  $(count_for_topic /wirelesscontroller) +
  $(count_for_topic /wirelesscontroller_unprocessed)
))"
if [[ "$ACTION" == remote_key_calibration || "$ACTION" == remote_stand_down || "$ACTION" == remote_stand_up ]]; then
  [[ -s "${KEY_PREFIX}.jsonl" ]] || {
    printf 'ERROR: action round remote-key JSONL is empty\n' >&2
    exit 1
  }
  (( wireless_count > 0 )) || {
    printf 'ERROR: action round captured no wireless-controller messages\n' >&2
    exit 1
  }
fi
printf 'WIRELESS_MESSAGE_COUNT=%s\n' "$wireless_count" \
  >"$CAPTURE_ROOT/metadata/${ROUND}_wireless_count.txt"

END_TIME="$(date --iso-8601=ns)"
python3 - "$MANIFEST" "$END_TIME" <<'PY'
import json, sys
path, end = sys.argv[1:]
with open(path, encoding="utf-8") as handle:
    data = json.load(handle)
data["end_time"] = end
with open(path, "w", encoding="utf-8") as handle:
    json.dump(data, handle, ensure_ascii=False, indent=2)
    handle.write("\n")
PY

find "$CAPTURE_ROOT/metadata" "$CAPTURE_ROOT/text" "$CAPTURE_ROOT/rosbag/$ROUND" "$PCAP_PATH" \
  -type f -print0 | sort -z | xargs -0 sha256sum \
  >"$CAPTURE_ROOT/checksums/${ROUND}_SHA256SUMS"
printf 'ROUND_CAPTURE=PASS round=%s action=%s\n' "$ROUND" "$ACTION"
