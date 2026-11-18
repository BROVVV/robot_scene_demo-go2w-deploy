#!/usr/bin/env bash
# Dataset D：D435 多视角语义观测采集（worker 不自己发运动；运动由受控 runner 执行）。
set -uo pipefail
cd /home/mxt/robotscene
RUN_ID="semanticD_$(date +%Y%m%d_%H%M%S)"
OUT="artifacts/slam_validation/${RUN_ID}"
mkdir -p "$OUT"
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE CONDA_SHLVL
source scripts/go2w/setup_environment.sh > "$OUT/env.log" 2>&1
export PYTHONPATH="${HOME}/.local/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
PY=/usr/bin/python3
echo "$OUT" > /tmp/semantic_D_dir.txt

bash scripts/go2w/start_vlm_daemon.sh > "$OUT/vlm.log" 2>&1 || true

$PY scripts/go2w/run_semantic_exploration.py \
  --target "办公椅" --backend go2w_experimental \
  --rgbd-source --dry-run-motion --spatial-v2 --spatial-provider plain_slam \
  --max-seconds 160 --session-dir "$OUT" --output "$OUT/events.jsonl" \
  --operator-supervised-experiment \
  --profile-config configs/go2w/high_level_experiment.yaml \
  > "$OUT/worker.log" 2>&1 &
WORKER=$!
sleep 20

pattern=""
for _ in $(seq 1 12); do pattern="${pattern}r30,"; done
pattern="${pattern%,}"
timeout 220 $PY scripts/go2w/run_autonomous_loop.py --mode pattern --pattern "$pattern" \
  --output "$OUT/rotation.jsonl" --operator-authorized-rotation \
  --odom-topic /go2w/odom/fused --max-seconds 200 > "$OUT/rotation.log" 2>&1
echo "rotation_rc=$?" >> "$OUT/rotation.log"

wait "$WORKER"
echo "worker_rc=$?" >> "$OUT/worker.log"
echo "DONE RUN_ID=$RUN_ID OUT=$OUT"
