#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

printf '%s\n' \
  '本轮只监听，不会向机器人发布控制指令。' \
  '' \
  '请确认：' \
  '1. 机器人位于平整、防滑地面；' \
  '2. 四周 2 米无人员和障碍物；' \
  '3. 机身下方无物体；' \
  '4. 遥控器在手且可立即急停；' \
  '5. 所有电脑端 lease/control 程序均已关闭。'
read -r -p '输入 I_CONFIRM_PASSIVE_CAPTURE: ' confirmation
[[ "$confirmation" == I_CONFIRM_PASSIVE_CAPTURE ]] || {
  printf 'ERROR: passive capture confirmation rejected\n' >&2
  exit 2
}

STAMP="$(date +%Y%m%d_%H%M%S)"
CAPTURE_ROOT="$ROOT/captures/$STAMP"
"$SCRIPT_DIR/preflight_remote_capture.sh" "$CAPTURE_ROOT"

"$SCRIPT_DIR/capture_remote_posture.sh" --capture-root "$CAPTURE_ROOT" --round 00_qos_dry_run --action dry_run --dry-run
"$SCRIPT_DIR/capture_remote_posture.sh" --capture-root "$CAPTURE_ROOT" --round 01_idle_baseline --action idle_baseline
"$SCRIPT_DIR/capture_remote_posture.sh" --capture-root "$CAPTURE_ROOT" --round 02_key_calibration --action remote_key_calibration
"$SCRIPT_DIR/capture_remote_posture.sh" --capture-root "$CAPTURE_ROOT" --round D1 --action remote_stand_down
"$SCRIPT_DIR/capture_remote_posture.sh" --capture-root "$CAPTURE_ROOT" --round 03_down_idle --action down_idle
"$SCRIPT_DIR/capture_remote_posture.sh" --capture-root "$CAPTURE_ROOT" --round U1 --action remote_stand_up
"$SCRIPT_DIR/capture_remote_posture.sh" --capture-root "$CAPTURE_ROOT" --round 05_up_idle --action up_idle
"$SCRIPT_DIR/capture_remote_posture.sh" --capture-root "$CAPTURE_ROOT" --round D2 --action remote_stand_down
"$SCRIPT_DIR/capture_remote_posture.sh" --capture-root "$CAPTURE_ROOT" --round U2 --action remote_stand_up
"$SCRIPT_DIR/capture_remote_posture.sh" --capture-root "$CAPTURE_ROOT" --round D3 --action remote_stand_down
"$SCRIPT_DIR/capture_remote_posture.sh" --capture-root "$CAPTURE_ROOT" --round U3 --action remote_stand_up

read -r -p '确认机器人最终已安全站立且静止，输入 I_CONFIRM_ROBOT_SAFE_STANDING: ' final_confirmation
[[ "$final_confirmation" == I_CONFIRM_ROBOT_SAFE_STANDING ]] || {
  printf 'ERROR: final robot state was not confirmed\n' >&2
  exit 1
}
printf '%s\n' "$final_confirmation" >"$CAPTURE_ROOT/metadata/final_operator_safety_confirmation.txt"
ps -eo pid,ppid,user,lstart,comm,args >"$CAPTURE_ROOT/metadata/processes_final.txt"
set +e
residue="$(ps -C python -C python3 -C tcpdump -o pid=,comm=,args= | \
  awk '/monitor_remote_keys|capture_remote_timeline|ros2 bag record|tcpdump.*192\.168\.123\.18/ {print}')"
set -e
if [[ -n "$residue" ]]; then
  printf 'ERROR: residual passive capture process detected:\n%s\n' "$residue" >&2
  exit 1
fi
printf 'CAPTURE_PROCESS_RESIDUE=none\n' \
  >"$CAPTURE_ROOT/metadata/final_process_residue_check.txt"
"$SCRIPT_DIR/analyze_remote_capture.sh" "$CAPTURE_ROOT"

printf 'REMOTE_POSTURE_CAPTURE_ROUNDS=COMPLETE\nCAPTURE_ROOT=%s\n' "$CAPTURE_ROOT"
