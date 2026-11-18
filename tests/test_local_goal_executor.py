"""Tests for LocalGoalExecutor primitive decomposition."""

from __future__ import annotations

import pytest
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


@pytest.mark.parametrize("distance,expected", [(0.45, 0.45), (2.0, 2.0), (5.0, 3.0)])
def test_forward_goal_uses_range_below_three_metre_cap(distance, expected):
    executor = LocalGoalExecutor(forward_step_m=0.3, max_forward_step_m=3.0)
    intent = _intent(bearing=0)
    intent.provenance = {"frontier": {"distance_m": distance}}
    executor.begin(intent)
    caps = RobotCapabilities(supports_relative_translation=True)
    goal = executor.next_goal(current_yaw_deg=0, capabilities=caps)
    assert goal.relative_dx == expected


def test_unknown_range_uses_short_fallback_with_three_metre_cap():
    executor = LocalGoalExecutor(forward_step_m=0.3, max_forward_step_m=3.0)
    executor.begin(_intent(bearing=0))
    goal = executor.next_goal(capabilities=RobotCapabilities(supports_relative_translation=True))
    assert goal.relative_dx == 0.3

@pytest.mark.parametrize('distance,expected', [(.4,.4),(2.,2.),(5.,3.)])
def test_metric_goal_limits_distance_without_forcing_three_metres(distance, expected):
    executor = LocalGoalExecutor(max_forward_step_m=3.)
    intent = _intent(0)
    intent.preferred_position = (10.+distance,20.)
    executor.begin(intent)
    goal = executor.next_goal(current_position=(10.,20.), capabilities=RobotCapabilities(supports_metric_navigation=True))
    assert goal.position == pytest.approx((10.+expected,20.))


def test_metric_turn_only_preserves_current_position():
    executor = LocalGoalExecutor(turn_only=True, max_forward_step_m=3.)
    intent = _intent(30)
    intent.preferred_position = (10.,20.)
    executor.begin(intent)
    goal = executor.next_goal(current_position=(1.,2.), capabilities=RobotCapabilities(supports_metric_navigation=True))
    assert goal.position == (1.,2.)


def test_metric_intent_emits_one_goal_until_a_new_intent_begins():
    executor = LocalGoalExecutor(max_forward_step_m=3.)
    intent = _intent(0)
    intent.preferred_position = (1.,0.)
    executor.begin(intent)
    caps = RobotCapabilities(supports_metric_navigation=True)
    assert executor.next_goal(current_position=(0.,0.), capabilities=caps) is not None
    assert executor.next_goal(current_position=(1.,0.), capabilities=caps) is None
