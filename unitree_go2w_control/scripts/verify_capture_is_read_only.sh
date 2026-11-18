#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
OUTPUT="${1:-}"

runtime_files=(
  "$SCRIPT_DIR/remote_capture_env.sh"
  "$SCRIPT_DIR/preflight_remote_capture.sh"
  "$SCRIPT_DIR/snapshot_ros_graph.sh"
  "$SCRIPT_DIR/monitor_remote_keys.py"
  "$SCRIPT_DIR/capture_remote_timeline.py"
  "$SCRIPT_DIR/capture_remote_posture.sh"
  "$SCRIPT_DIR/stop_remote_capture.sh"
  "$SCRIPT_DIR/run_remote_posture_capture.sh"
)

for file in "${runtime_files[@]}"; do
  [[ -f "$file" ]] || {
    printf 'READ_ONLY_AUDIT=FAIL missing=%s\n' "$file"
    exit 1
  }
done

forbidden='create_publisher|ros2[[:space:]]+topic[[:space:]]+pub|ChannelPublisher|SportClient|\.Move\(|\.StandDown\(|\.StandUp\(|\.StopMove\(|\.ReleaseMode\(|\.Damp\(|ros2[[:space:]]+bag[[:space:]]+play|LowCmd|/lowcmd.*(publish|publisher|pub)'
set +e
matches="$(rg -n --no-heading -e "$forbidden" "${runtime_files[@]}" 2>/dev/null)"
rg_rc=$?
set -e
if [[ $rg_rc -eq 0 ]]; then
  printf 'READ_ONLY_AUDIT=FAIL\n%s\n' "$matches"
  exit 1
fi
if [[ $rg_rc -ne 1 ]]; then
  printf 'READ_ONLY_AUDIT=FAIL scanner_error=%s\n' "$rg_rc"
  exit 1
fi

{
  printf 'READ_ONLY_AUDIT=PASS\n'
  printf 'ROOT=%s\n' "$ROOT"
  printf 'FILES_CHECKED=%s\n' "${#runtime_files[@]}"
  printf 'NO_CONTROL_PUBLISHER_CODE=true\n'
  printf 'NO_SPORT_CLIENT=true\n'
  printf 'NO_ROSBAG_PLAYBACK=true\n'
} | if [[ -n "$OUTPUT" ]]; then tee "$OUTPUT"; else cat; fi
