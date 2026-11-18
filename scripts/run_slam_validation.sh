#!/usr/bin/env bash
# 评估一个已录制的 SLAM 验证数据集（20260908 计划书 §29）。
#
# 用法（控制主机）：
#   bash scripts/run_slam_validation.sh artifacts/slam_validation/datasetA_20260908_190000
#
# 退出码：0 = 所有已测得指标 PASS；1 = 存在 FAIL 或未测得任何指标。
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
run_dir="${1:-}"
if [[ -z "${run_dir}" || ! -d "${run_dir}/bag" ]]; then
  printf 'usage: %s <run_dir containing bag/>\n' "$0" >&2
  exit 2
fi

# ROS 2 Humble 的 rclpy 是 Python 3.10 扩展；当前交互 shell 可能处于
# Conda/Python 3.13。评估 rosbag 前显式移除 Conda，避免加载错误 ABI。
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE CONDA_SHLVL
# shellcheck disable=SC1091
source "${project_root}/scripts/go2w/setup_environment.sh" >/dev/null
if [[ -d "${HOME}/.local/lib/python3.10/site-packages" ]]; then
  export PYTHONPATH="${HOME}/.local/lib/python3.10/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
fi

/usr/bin/python3 -m app.mapping.map_quality_evaluator \
  --bag "${run_dir}/bag" \
  --output "${run_dir}/metrics.json"

/usr/bin/python3 - "${run_dir}/metrics.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as handle:
    report = json.load(handle)
acceptance = report.get("acceptance") or {}
for name, item in (acceptance.get("checks") or {}).items():
    print(f"{item['status']:<13} {name:<32} value={item['value']} limit={item['limit']}")
print("failed:", acceptance.get("failed"))
sys.exit(1 if acceptance.get("failed") else 0)
PY
