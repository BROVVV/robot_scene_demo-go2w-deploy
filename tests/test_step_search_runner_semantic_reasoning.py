from __future__ import annotations

import unittest

from app.live_robot.search_state_machine import SensorSnapshot
from app.live_robot.step_search_runner import (
    Detection, StepSearchConfig, StepSearchRunner, VerificationResult,
)
from app.reasoning.semantic_navigation.models import SearchDirective, SearchDirectiveKind


def _directive(_context):
    return SearchDirective(
        directive_id="semantic_d1", kind=SearchDirectiveKind.INSPECT_ANCHOR,
        source_backend="hybrid", match_state="partial_match", confidence=0.9,
        preferred_heading_delta_deg=20.0, allow_forward=False,
        reason_zh="inspect anchor",
    )


class RunnerSemanticTests(unittest.TestCase):
    def _run(self, mode):
        detections = [[], [Detection("phone", 0.9, (0.4, 0.1, 0.6, 0.9))]]
        steps = []
        runner = StepSearchRunner(
            StepSearchConfig(
                target="phone", reach_area_ratio=0.15,
                semantic_reasoning_enabled=True, search_reasoner_backend="hybrid",
                search_reasoner_mode=mode,
            ),
            detect=lambda: detections.pop(0),
            verify=lambda _bbox: VerificationResult("手机", True, 0.9, "ok"),
            execute_step=lambda step: (steps.append(step) is None, ""),
            snapshot=lambda: SensorSnapshot(True, True, True),
            odometry=lambda: (0.0, 0.0, 0.0),
            reason_next_view=_directive,
            semantic_observe=lambda: {"scene_graph": {"nodes": [], "edges": []}},
        )
        return runner.run(), steps

    def test_shadow_executes_legacy_even_when_directive_differs(self):
        result, steps = self._run("shadow")
        self.assertEqual(result["status"], "target_reached")
        self.assertEqual(steps, ["r30"])
        artifact = next(
            e for e in result["events"]
            if e["event"] == "reasoner_shadow_compare"
        )
        self.assertEqual(artifact["legacy_step"], "r30")
        self.assertEqual(artifact["semantic_step"], "l20")
        self.assertTrue(artifact["semantic_disagrees_with_legacy"])
        self.assertFalse(artifact["dangerous_forward_request"])

    def test_active_executes_semantic_turn(self):
        result, steps = self._run("active")
        self.assertEqual(result["status"], "target_reached")
        self.assertEqual(steps, ["l20"])

    def test_target_candidate_never_calls_reasoner(self):
        calls = []
        runner = StepSearchRunner(
            StepSearchConfig(
                target="phone", reach_area_ratio=0.15,
                semantic_reasoning_enabled=True, search_reasoner_backend="hybrid",
                search_reasoner_mode="active",
            ),
            detect=lambda: [Detection("phone", 0.9, (0.4, 0.1, 0.6, 0.9))],
            verify=lambda _bbox: VerificationResult("手机", True, 0.9, "ok"),
            execute_step=lambda step: (True, ""),
            snapshot=lambda: SensorSnapshot(True, True, True),
            odometry=lambda: (0.0, 0.0, 0.0),
            reason_next_view=lambda context: calls.append(context),
        )
        self.assertEqual(runner.run()["status"], "target_reached")
        self.assertEqual(calls, [])

    def test_existing_observation_memory_is_passed_read_only_to_reasoner(self):
        records = [{"memory_id": "mem_existing", "label": "手机"}]
        contexts = []

        def reason(context):
            contexts.append(context)
            context.observation_memory.append({"memory_id": "local_mutation"})
            return _directive(context)

        runner = StepSearchRunner(
            StepSearchConfig(
                target="phone", semantic_reasoning_enabled=True,
                search_reasoner_backend="hybrid", search_reasoner_mode="shadow",
                max_motion_steps=1,
            ),
            detect=lambda: [],
            verify=lambda _bbox: VerificationResult("", False, 0.0, ""),
            execute_step=lambda _step: (True, ""),
            snapshot=lambda: SensorSnapshot(True, True, True),
            odometry=lambda: (0.0, 0.0, 0.0),
            reason_next_view=reason,
            observation_memory=records,
        )
        result = runner.run()
        self.assertEqual(records, [{"memory_id": "mem_existing", "label": "手机"}])
        self.assertEqual(contexts[0].observation_memory[0]["memory_id"], "mem_existing")
        artifact = next(
            event for event in result["events"]
            if event["event"] == "reasoner_shadow_compare"
        )
        self.assertEqual(artifact["observation_memory"]["retrieved_count"], 1)
        self.assertFalse(
            artifact["observation_memory"]["persistent_write_attempted"]
        )

    def test_min_replan_interval_reuses_only_explicit_observer_cache_hit(self):
        detections = [
            [],
            [],
            [Detection("phone", 0.9, (0.4, 0.1, 0.6, 0.9))],
        ]
        observations = [
            {
                "frame_id": "stable_1", "heading_sector": 0,
                "cache_hit": False,
                "scene_graph": {"nodes": [], "edges": []},
            },
            {
                "frame_id": "stable_1", "heading_sector": 0,
                "cache_hit": True,
                "scene_graph": {"nodes": [], "edges": []},
            },
        ]
        reasoner_calls = []
        runner = StepSearchRunner(
            StepSearchConfig(
                target="phone", reach_area_ratio=0.15,
                semantic_reasoning_enabled=True,
                search_reasoner_backend="hybrid",
                search_reasoner_mode="shadow",
                reasoner_min_replan_seconds=100.0,
            ),
            detect=lambda: detections.pop(0),
            verify=lambda _bbox: VerificationResult("手机", True, 0.9, "ok"),
            execute_step=lambda _step: (True, ""),
            snapshot=lambda: SensorSnapshot(True, True, True),
            odometry=lambda: (0.0, 0.0, 0.0),
            reason_next_view=lambda context: (
                reasoner_calls.append(context) or _directive(context)
            ),
            semantic_observe=lambda: observations.pop(0),
        )
        result = runner.run()
        self.assertEqual(result["status"], "target_reached")
        self.assertEqual(len(reasoner_calls), 1)
        throttled = [
            event for event in result["events"]
            if event["event"] == "semantic_replan_throttled"
        ]
        self.assertEqual(len(throttled), 1)
        artifacts = [
            event for event in result["events"]
            if event["event"] == "reasoner_shadow_compare"
        ]
        self.assertFalse(artifacts[0]["replan"]["throttled"])
        self.assertTrue(artifacts[1]["replan"]["throttled"])

    def test_auxiliary_hints_are_audited_and_passed_to_reasoner(self):
        contexts = []
        hint = {
            "hint_id": "psg:view_left",
            "source": "psg",
            "heading_delta_deg": 30.0,
            "confidence": 0.7,
            "can_confirm_target": False,
            "allow_forward": False,
        }
        runner = StepSearchRunner(
            StepSearchConfig(
                target="phone", semantic_reasoning_enabled=True,
                search_reasoner_backend="hybrid", search_reasoner_mode="shadow",
                max_motion_steps=1,
            ),
            detect=lambda: [],
            verify=lambda _bbox: VerificationResult("", False, 0.0, ""),
            execute_step=lambda _step: (True, ""),
            snapshot=lambda: SensorSnapshot(True, True, True),
            odometry=lambda: (0.0, 0.0, 0.0),
            reason_next_view=lambda context: (
                contexts.append(context) or _directive(context)
            ),
            semantic_observe=lambda: {
                "scene_graph": {"nodes": [], "edges": []}
            },
            build_auxiliary_hints=lambda _semantic: {
                "hints": [hint],
                "status": {"psg": {"available": True}},
            },
        )
        result = runner.run()
        self.assertEqual(contexts[0].auxiliary_hints, [hint])
        artifact = next(
            event for event in result["events"]
            if event["event"] == "reasoner_shadow_compare"
        )
        self.assertEqual(artifact["auxiliary_reasoning"]["hints"], [hint])
        self.assertFalse(
            artifact["auxiliary_reasoning"]["can_confirm_target"]
        )


if __name__ == "__main__":
    unittest.main()
