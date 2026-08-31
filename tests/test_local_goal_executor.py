"""Tests for LocalGoalExecutor primitive decomposition."""

from __future__ import annotations

from app.navigation.local_goal_executor import LocalGoalExecutor
from app.navigation.models import GOAL_RELATIVE_MOVE, GOAL_ROTATE_VIEW
from app.navigation.robot_backend import RobotCapabilities
from app.spatial.models import INTENT_EXPLORE_FRONTIER, ExplorationIntent


def _intent(bearing=30.0) -> ExplorationIntent:
    return ExplorationIntent(
        intent_id="i1",
        intent_type=INTENT_EXPLORE_FRONTIER,
        target_frontier_id="F1",
        preferred_bearing_deg=bearing,
        spatial_gain=0.5,
        semantic_reason="test",
    )


def test_explore_frontier_decomposes_rotate_then_forward():
    executor = LocalGoalExecutor(forward_step_m=0.25)
    executor.begin(_intent(bearing=30.0))
    caps = RobotCapabilities(supports_relative_translation=True, supports_relative_rotation=True)
    first = executor.next_goal(current_yaw_deg=0.0, capabilities=caps)
    assert first is not None and first.goal_type == GOAL_ROTATE_VIEW
    assert first.relative_dyaw == 30.0
    second = executor.next_goal(current_yaw_deg=30.0, capabilities=caps)
    assert second is not None and second.goal_type == GOAL_RELATIVE_MOVE
    assert second.relative_dx == 0.25
    assert executor.next_goal(current_yaw_deg=30.0, capabilities=caps) is None


def test_turn_only_never_forward():
    executor = LocalGoalExecutor(turn_only=True)
    executor.begin(_intent(bearing=10.0))
    caps = RobotCapabilities(supports_relative_translation=True, supports_relative_rotation=True)
    first = executor.next_goal(current_yaw_deg=0.0, capabilities=caps)
    assert first.goal_type == GOAL_ROTATE_VIEW
    assert executor.next_goal(current_yaw_deg=10.0, capabilities=caps) is None


def test_already_facing_skips_rotation():
    executor = LocalGoalExecutor()
    executor.begin(_intent(bearing=10.0))
    caps = RobotCapabilities(supports_relative_translation=True, supports_relative_rotation=True)
    first = executor.next_goal(current_yaw_deg=10.0, capabilities=caps)
    assert first is not None and first.goal_type == GOAL_RELATIVE_MOVE
