#!/usr/bin/env bash
# 真机 A/B 录制：同时记录 plain_slam（现权威）与 rko 链路（新方案）的输出。
# 只录包、不发运动命令；用户遥控走完回到胶带原点后，agent touch /tmp/stop_real_ab 结束。
set -uo pipefail
cd /home/mxt/robotscene
RUN_ID="real_ab_$(date +%Y%m%d_%H%M%S)"
OUT="artifacts/slam_validation/${RUN_ID}"
mkdir -p "$OUT"
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE CONDA_SHLVL
source scripts/go2w/setup_environment.sh > "$OUT/env.log" 2>&1
export PYTHONPATH="${HOME}/.local/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
PY=/usr/bin/python3
rm -f /tmp/stop_real_ab
echo "$OUT" > /tmp/real_ab_dir.txt

$PY scripts/go2w/odom_pose_probe.py /go2w/odom/fused > "$OUT/pose_start_fused.json" 2>&1

ros2 bag record -o "$OUT/bag" \
  /go2w/slam/map_3d /go2w/slam/odom_base /go2w/slam/aligned_scan \
  /go2w/slam/pose_graph_loop_edges /go2w/slam/pose_graph_poses \
  /go2w/rko_slam/map_3d /go2w/rko_slam/odom_base /go2w/rko_slam/aligned_scan \
  /go2w/rko_slam/pose_graph_loop_edges /go2w/rko_slam/pose_graph_poses \
  /go2w/odom/fused \
  --qos-profile-overrides-path /tmp/record_qos.yaml \
  > "$OUT/record.log" 2>&1 &
REC=$!
echo "$REC" > "$OUT/recorder.pid"
echo "RECORDING $OUT" > /tmp/real_ab_status.txt

for _ in $(seq 1 2400); do
  [[ -f /tmp/stop_real_ab ]] && break
  sleep 1
done

sleep 6
kill -INT "$REC" 2>/dev/null
wait "$REC" 2>/dev/null
$PY scripts/go2w/odom_pose_probe.py /go2w/odom/fused > "$OUT/pose_end_fused.json" 2>&1
echo "DONE $OUT" > /tmp/real_ab_status.txt
