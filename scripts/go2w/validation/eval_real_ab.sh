#!/usr/bin/env bash
# 真机 A/B 评估：同一盘 bag 分别评 plain_slam（现权威）与 rko 链路（新方案）。
set -uo pipefail
cd /home/mxt/robotscene
D="$1"
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE CONDA_SHLVL
source scripts/go2w/setup_environment.sh > /dev/null 2>&1
export PYTHONPATH="${HOME}/.local/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
PY=/usr/bin/python3
$PY -m app.mapping.map_quality_evaluator --bag "$D/bag" \
  --topic-prefix /go2w/slam --output "$D/metrics_plain.json" > "$D/eval_plain.log" 2>&1
$PY -m app.mapping.map_quality_evaluator --bag "$D/bag" \
  --topic-prefix /go2w/rko_slam --output "$D/metrics_rko.json" > "$D/eval_rko.log" 2>&1
echo "DONE $D" > "$D/eval_status.txt"
