"""用户要求：机器人每次前进最大距离改为 1.5 米（少跑几步）。

锁定：
  * --forward-step-m 默认 1.5
  * 实验 profile 的 max_forward_step_m 允许 1.5（不再被 0.30 clamp）
  * LocalGoalExecutor 默认单步前进 1.5m（无参时的生成 goal.relative_dx）
"""

from __future__ import annotations

from app.navigation.exploration_config import load_go2w_experiment_profile
from app.navigation.local_goal_executor import LocalGoalExecutor
from app.navigation.models import GOAL_RELATIVE_MOVE, GOAL_ROTATE_VIEW
from app.navigation.robot_backend import RobotCapabilities
from app.spatial.models import INTENT_EXPLORE_FRONTIER, ExplorationIntent
from scripts.go2w.run_semantic_exploration import build_parser


def test_cli_default_forward_step_is_1_5():
    args = build_parser().parse_args(["--target", "x", "--backend", "mock"])
    assert args.forward_step_m == 1.5


def test_profile_allows_1_5_step():
    profile = load_go2w_experiment_profile("configs/go2w/high_level_experiment.yaml")
    limits = profile.get("limits") or {}
    assert float(limits.get("max_forward_step_m")) >= 1.5
    execution = profile.get("execution") or {}
    assert 1.0 < float(execution.get("forward_command_duration_scale")) <= 2.0


def test_local_executor_default_goal_is_1_5_m():
    executor = LocalGoalExecutor()  # 不传 forward_step_m -> 默认 1.5
    executor.begin(ExplorationIntent(
        intent_id="i", intent_type=INTENT_EXPLORE_FRONTIER,
        target_frontier_id="F1", preferred_bearing_deg=30.0,
        spatial_gain=0.5, semantic_reason="t",
    ))
    caps = RobotCapabilities(supports_relative_translation=True, supports_relative_rotation=True)
    # 先转向，然后前进
    assert executor.next_goal(current_yaw_deg=0.0, capabilities=caps).goal_type == GOAL_ROTATE_VIEW
    forward = executor.next_goal(current_yaw_deg=30.0, capabilities=caps)
    assert forward is not None and forward.goal_type == GOAL_RELATIVE_MOVE
    assert forward.relative_dx == 1.5
