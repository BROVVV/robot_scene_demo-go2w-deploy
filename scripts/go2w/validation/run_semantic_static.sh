#!/usr/bin/env bash
# 定点语义观测：机器人静止不动，对正前方同一物体连续观测，用于分离"深度采样误差"与"视角差异"。
set -uo pipefail
cd /home/mxt/robotscene
RUN_ID="semanticStatic_$(date +%Y%m%d_%H%M%S)"
OUT="artifacts/slam_validation/${RUN_ID}"
mkdir -p "$OUT"
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE CONDA_SHLVL
source scripts/go2w/setup_environment.sh > "$OUT/env.log" 2>&1
export PYTHONPATH="${HOME}/.local/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
PY=/usr/bin/python3
echo "$OUT" > /tmp/semantic_static_dir.txt

bash scripts/go2w/start_vlm_daemon.sh > "$OUT/vlm.log" 2>&1 || true

timeout 120 $PY scripts/go2w/run_semantic_exploration.py \
  --target "桌子" --backend go2w_experimental \
  --rgbd-source --dry-run-motion --spatial-v2 --spatial-provider plain_slam \
  --max-seconds 60 --session-dir "$OUT" --output "$OUT/events.jsonl" \
  --operator-supervised-experiment \
  --profile-config configs/go2w/high_level_experiment.yaml \
  > "$OUT/worker.log" 2>&1
echo "worker_rc=$?" >> "$OUT/worker.log"
echo "DONE $OUT" > /tmp/semantic_static_status.txt
