#!/usr/bin/env bash
set -euo pipefail
gate_file="outputs/go2w_acceptance/navigation_gate/plan_only.json"
/home/brov/miniconda3/envs/go2_robot_scene_demo/bin/python \
  scripts/go2w/evaluate_navigation_gate.py --mode nav2_plan_only --output "${gate_file}"
python run_demo.py --mock --enable-nav2 --nav2-mode plan_only \
  --nav2-goal-x "${1:-1.0}" --nav2-goal-y "${2:-0.0}" --nav2-goal-yaw "${3:-0.0}" \
  --nav2-use-current-start --nav2-wait
