#!/usr/bin/env bash
set -euo pipefail
if [[ "${NAV2_ALLOW_EXECUTE:-false}" != "true" ]]; then
  echo "NAV2_ALLOW_EXECUTE 必须显式设为 true" >&2; exit 2
fi
gate_file="outputs/go2w_acceptance/navigation_gate/execute.json"
/home/brov/miniconda3/envs/go2_robot_scene_demo/bin/python \
  scripts/go2w/evaluate_navigation_gate.py --mode nav2_execute --output "${gate_file}"
printf '%s\n' 'Type I_CONFIRM_GO2W_NAVIGATION_EXECUTION to continue:'
read -r confirmation
if [[ "${confirmation}" != "I_CONFIRM_GO2W_NAVIGATION_EXECUTION" ]]; then
  echo "Nav2 execute 二次确认失败" >&2; exit 2
fi
python run_demo.py --mock --enable-nav2 --nav2-mode execute \
  --nav2-goal-x "${1:-1.0}" --nav2-goal-y "${2:-0.0}" --nav2-goal-yaw "${3:-0.0}" \
  --nav2-use-current-start --nav2-allow-execute --nav2-safety-confirmed \
  --nav2-footprint-confirmed --nav2-estop-confirmed \
  --nav2-capability-gate-json "${gate_file}" --nav2-wait
