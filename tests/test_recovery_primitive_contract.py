"""计划书 §32.1：BACKWARD_RECOVERY primitive 契约。

历史 bug：候选生成器用 ``"REVERSE" in allowed_primitives`` 判断负 dx 目标，而
backend 公布的词表是 ``BACKWARD_RECOVERY``，导致所有 recovery 目标在规划阶段
被静默丢弃，机器狗只能原地转或继续往墙里顶。
"""

from __future__ import annotations

import unittest

from app.navigation.candidate_goal_generator import (
    _goal_primitive_allowed,
    generate_live_exploration_candidates,
)
from app.navigation.exploration_graph import ExplorationGraph
from app.navigation.go2w_experimental_backend import (
    GO2W_OPERATOR_SUPERVISED_PRIMITIVES,
    Go2WBackendConfig,
    Go2WExperimentalBackend,
)
from app.navigation.models import (
    GOAL_RELATIVE_MOVE,
    ExplorationGoal,
    LiveObservation,
)
from app.navigation.robot_backend import NavigationStatus, RobotCapabilities
from app.navigation.safety_models import (
    build_local_safety_snapshot,
    evaluate_perception_quality,
)
from app.live_robot.recovery_motion import evaluate_backward_safety


class _FakeMotion:
    def __init__(self) -> None:
        self.steps: list[str] = []
        self.odom = [0.0, 0.0, 0.0]

    def execute(self, step: str) -> tuple[bool, str, dict]:
        self.steps.append(step)
        if step.startswith("f"):
            self.odom[0] += float(step[1:])
        elif step.startswith("b"):
            self.odom[0] -= float(step[1:])
        return True, "ok", {"step": step}

    def odometry(self) -> tuple[float, float, float]:
        return tuple(self.odom)


def _backward_goal(distance_m: float) -> ExplorationGoal:
    return ExplorationGoal(
        goal_id="recovery_001",
        goal_type=GOAL_RELATIVE_MOVE,
        relative_dx=-abs(distance_m),
        semantic_reason="front blocked recovery",
        provenance={"recovery_reason": "FRONT_BLOCKED"},
    )


def _observation(**kwargs) -> LiveObservation:
    payload = {
        "bundle_id": "bundle_0001",
        "timestamp": 1000.0,
        "scene_objects": [{"label": "wall"}],
        "heading_sector": 0,
    }
    payload.update(kwargs)
    return LiveObservation(**payload)


def _go2w_capabilities() -> RobotCapabilities:
    return RobotCapabilities(
        supports_relative_translation=True,
        supports_relative_rotation=True,
        supports_heading_control=True,
        allowed_motion_primitives=GO2W_OPERATOR_SUPERVISED_PRIMITIVES,
    )


class TestRecoveryPrimitiveContract(unittest.TestCase):
    def test_backward_goal_allowed_under_backward_recovery_vocabulary(self) -> None:
        allowed = set(GO2W_OPERATOR_SUPERVISED_PRIMITIVES)
        self.assertTrue(_goal_primitive_allowed(_backward_goal(0.10), allowed))

    def test_legacy_reverse_vocabulary_no_longer_authorizes_backward(self) -> None:
        """``REVERSE`` 是已废除的名字，不得再授权 recovery 后退。"""
        self.assertFalse(
            _goal_primitive_allowed(_backward_goal(0.10), {"FORWARD", "REVERSE"})
        )

    def test_forward_and_rotation_contract_unchanged(self) -> None:
        allowed = set(GO2W_OPERATOR_SUPERVISED_PRIMITIVES)
        forward = ExplorationGoal(
            goal_id="cand_001", goal_type=GOAL_RELATIVE_MOVE, relative_dx=0.20
        )
        self.assertTrue(_goal_primitive_allowed(forward, allowed))
        lateral = ExplorationGoal(
            goal_id="cand_002", goal_type=GOAL_RELATIVE_MOVE, relative_dy=0.20
        )
        self.assertFalse(_goal_primitive_allowed(lateral, allowed))

    def test_normal_exploration_never_emits_backward_candidates(self) -> None:
        """后退只能来自 recovery 决策，普通候选生成不得出现负 dx。"""
        candidates = generate_live_exploration_candidates(
            observation=_observation(),
            graph=ExplorationGraph(),
            capabilities=_go2w_capabilities(),
        )
        self.assertTrue(candidates)
        for goal in candidates:
            self.assertGreaterEqual(float(goal.relative_dx or 0.0), 0.0)

    def test_blind_forward_fallback_is_refused_when_front_is_unsafe(self) -> None:
        """§44：不得靠"反正往前 0.2m"改视角；前方不安全就不许前进。"""
        blocked = build_local_safety_snapshot(
            sector_evidence=[
                {"bearing_deg": bearing * 15.0, "clearance_m": 0.25, "state": "occupied"}
                for bearing in range(-3, 4)
            ],
            lidar_fresh=True,
            geometry_validated=True,
        )
        candidates = generate_live_exploration_candidates(
            observation=_observation(),
            graph=ExplorationGraph(),
            capabilities=_go2w_capabilities(),
            local_safety=blocked,
        )
        sources = {goal.provenance.get("source") for goal in candidates}
        self.assertNotIn("fallback_forward", sources)

    def test_blind_forward_fallback_is_refused_when_camera_is_blocked(self) -> None:
        clear = build_local_safety_snapshot(
            sector_evidence=[
                {"bearing_deg": index * 15.0, "clearance_m": 3.0, "state": "clear"}
                for index in range(24)
            ],
            lidar_fresh=True,
            geometry_validated=True,
        )
        blind_camera = evaluate_perception_quality(
            front_clearance_m=0.30,
            near_field_ratio=0.9,
            estimated_information_gain=0.05,
        )
        self.assertTrue(blind_camera.camera_blocked)
        candidates = generate_live_exploration_candidates(
            observation=_observation(),
            graph=ExplorationGraph(),
            capabilities=_go2w_capabilities(),
            local_safety=clear,
            perception_quality=blind_camera,
        )
        sources = {goal.provenance.get("source") for goal in candidates}
        self.assertNotIn("fallback_forward", sources)

    def test_forward_fallback_survives_a_clear_corridor(self) -> None:
        clear = build_local_safety_snapshot(
            sector_evidence=[
                {"bearing_deg": index * 15.0, "clearance_m": 3.0, "state": "clear"}
                for index in range(24)
            ],
            lidar_fresh=True,
            geometry_validated=True,
        )
        candidates = generate_live_exploration_candidates(
            observation=_observation(),
            graph=ExplorationGraph(),
            capabilities=_go2w_capabilities(),
            local_safety=clear,
        )
        sources = {goal.provenance.get("source") for goal in candidates}
        self.assertIn("fallback_forward", sources)

    def test_negative_dx_goal_reaches_the_backend_backward_primitive(self) -> None:
        motion = _FakeMotion()
        backend = Go2WExperimentalBackend(
            execute_step=motion.execute,
            odometry=motion.odometry,
            config=Go2WBackendConfig(),
        )
        result = backend._dispatch(_backward_goal(0.10))
        self.assertEqual(result.status, NavigationStatus.SUCCEEDED)
        self.assertEqual(motion.steps, ["b0.100"])
        self.assertEqual(result.requested_motion["primitive"], "BACKWARD_RECOVERY")
        self.assertEqual(result.provenance["recovery_reason"], "FRONT_BLOCKED")

    def test_backward_step_is_clamped_to_the_configured_maximum(self) -> None:
        motion = _FakeMotion()
        backend = Go2WExperimentalBackend(
            execute_step=motion.execute,
            odometry=motion.odometry,
            config=Go2WBackendConfig(),
        )
        result = backend._dispatch(_backward_goal(0.40))
        self.assertEqual(motion.steps, ["b0.120"])
        self.assertEqual(result.requested_motion["distance_m"], 0.12)
        self.assertTrue(result.requested_motion["safety_limited"])

    def test_backward_without_breadcrumb_fails_closed(self) -> None:
        decision = evaluate_backward_safety(
            None,
            current_pose=(0.0, 0.0, 0.0),
            requested_distance_m=0.10,
            max_backward_step_m=0.12,
            min_backward_step_m=0.05,
            max_age_sec=8.0,
            heading_tolerance_deg=8.0,
        )
        self.assertFalse(decision.allowed)
        self.assertIn("NO_VALID_REVERSE_CORRIDOR", decision.reason)

    def test_backend_publishes_split_platform_safety_capabilities(self) -> None:
        """§16.1：不得再用单一 platform_obstacle_avoidance 标志描述平台。"""
        motion = _FakeMotion()
        backend = Go2WExperimentalBackend(
            execute_step=motion.execute,
            odometry=motion.odometry,
            config=Go2WBackendConfig(),
        )
        caps = backend.capabilities()
        self.assertFalse(caps.supports_platform_obstacle_avoidance)
        self.assertFalse(caps.emergency_collision_stop_supported)
        self.assertTrue(caps.clearance_aware_navigation_supported)
        self.assertFalse(caps.autonomous_obstacle_bypass_supported)
        restored = RobotCapabilities.from_dict(caps.to_dict())
        self.assertEqual(restored, caps)


if __name__ == "__main__":
    unittest.main()
