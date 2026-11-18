#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/remote_capture_env.sh"

OUTPUT_DIR="${1:?usage: snapshot_ros_graph.sh OUTPUT_DIR}"
mkdir -p "$OUTPUT_DIR"
chmod 700 "$OUTPUT_DIR"

timeout 15s ros2 node list >"$OUTPUT_DIR/nodes.txt" 2>&1 || true
timeout 15s ros2 topic list -t >"$OUTPUT_DIR/topics.txt" 2>&1
timeout 15s ros2 service list -t >"$OUTPUT_DIR/services.txt" 2>&1 || true
timeout 15s ros2 action list -t >"$OUTPUT_DIR/actions.txt" 2>&1 || true

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
    >"$OUTPUT_DIR/topic_${safe_name}.txt" 2>&1 || true
done

date --iso-8601=ns >"$OUTPUT_DIR/captured_at.txt"
printf 'ROS_GRAPH_SNAPSHOT=%s\n' "$OUTPUT_DIR"
