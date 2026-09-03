#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CAPTURE_ROOT="${1:-}"
if [[ -z "$CAPTURE_ROOT" ]]; then
  ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
  CAPTURE_ROOT="$(<"$ROOT/.latest_remote_capture")"
fi
source "$SCRIPT_DIR/remote_capture_env.sh"
python3 "$SCRIPT_DIR/analyze_remote_posture_bag.py" --capture-root "$CAPTURE_ROOT"
python3 "$SCRIPT_DIR/analyze_sport_requests.py" --capture-root "$CAPTURE_ROOT"
python3 "$SCRIPT_DIR/correlate_remote_timeline.py" --capture-root "$CAPTURE_ROOT"
python3 "$SCRIPT_DIR/compare_remote_ros_graph.py" --capture-root "$CAPTURE_ROOT"
"$SCRIPT_DIR/inspect_pcap_summary.sh" "$CAPTURE_ROOT"
python3 "$SCRIPT_DIR/generate_remote_posture_report.py" \
  --capture-root "$CAPTURE_ROOT"
find "$CAPTURE_ROOT/metadata" "$CAPTURE_ROOT/text" "$CAPTURE_ROOT/rosbag" \
  "$CAPTURE_ROOT/pcap" "$CAPTURE_ROOT/analysis" -type f -print0 | \
  sort -z | xargs -0 sha256sum >"$CAPTURE_ROOT/checksums/FINAL_SHA256SUMS"
printf 'REMOTE_CAPTURE_ANALYSIS=PASS capture_root=%s\n' "$CAPTURE_ROOT"
