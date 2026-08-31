"""Deterministic replay regression (plan section 36).

A mock session writes a JSONL; replaying its observation events with the mock
backend must reproduce the same outcome without the robot or LLM.
"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from app.live_robot.autonomous_explorer import AutonomousExplorer
from app.live_robot.mock_observation_scene import scenario_anchor_then_target
from app.navigation.backend_factory import MockBackend
from app.navigation.exploration_config import load_exploration_policy
from app.navigation.exploration_graph import ExplorationGraph
from app.navigation.models import LiveObservation


def _run_session() -> tuple[AutonomousExplorer, list[dict]]:
    scene = scenario_anchor_then_target()
    explorer = AutonomousExplorer(
        target="蓝色垃圾桶",
        observer=scene.observer(),
        matcher=scene.matcher(),
        verifier=scene.verifier(),
        backend=MockBackend(),
        graph=ExplorationGraph(session_id="replay_src"),
        policy=load_exploration_policy(),
        negative_target_key="蓝色垃圾桶",
    )
    result = explorer.run()
    assert result.result == "TARGET_FOUND"
    return explorer, explorer.events


def _replay(events: list[dict], *, target: str) -> AutonomousExplorer:
    observations: list[LiveObservation] = []
    for event in events:
        if event.get("event") == "observation":
            observation = LiveObservation.from_dict(event)
            observation.target_match = {
                "target_present": bool(event.get("target_present", False)),
                "score": (
                    float((event.get("target_match") or {}).get("score", 0.0))
                    if isinstance(event.get("target_match"), dict) else 0.0
                ),
            }
            observations.append(observation)
    index = [0]

    def observe() -> LiveObservation:
        obs = observations[min(index[0], len(observations) - 1)]
        index[0] += 1
        return obs

    def matcher(observation: LiveObservation):
        from app.live_robot.autonomous_explorer import SemanticMatch
        return SemanticMatch(
            has_candidate=bool(observation.target_present),
            target_match=observation.target_match,
            target_score=float((observation.target_match or {}).get("score", 0.0)),
            target_match_level="candidate" if observation.target_present else "none",
            provenance={"source": "replay"},
        )

    def verifier(observation, match):
        from app.live_robot.autonomous_explorer import VerificationOutcome
        return VerificationOutcome(
            confirmed=bool(observation.target_present), attempts=1,
            reason_zh="replay verify",
        )

    explorer = AutonomousExplorer(
        target=target,
        observer=observe,
        matcher=matcher,
        verifier=verifier,
        backend=MockBackend(),
        graph=ExplorationGraph(session_id="replay_run"),
        policy=load_exploration_policy(),
        negative_target_key=target,
    )
    return explorer


class TestExplorationReplay(unittest.TestCase):
    def test_replay_reproduces_target_found(self) -> None:
        _, events = _run_session()
        explorer = _replay(events, target="蓝色垃圾桶")
        result = explorer.run()
        self.assertEqual(result.result, "TARGET_FOUND")
        self.assertGreaterEqual(result.observations, 1)

    def test_replay_is_deterministic(self) -> None:
        """Replay 两次应产生相同结果；真实执行时长（duration_s）允许不同。"""
        _, events = _run_session()
        first = _replay(events, target="蓝色垃圾桶").run()
        second = _replay(events, target="蓝色垃圾桶").run()

        # Replay 的核心确定性结果必须完全一致；真实时间戳（duration_s、
        # object_topology.generated_at 等 wall-clock 字段）允许不同。
        def _core(session) -> dict:
            d = session.to_dict()
            return {
                "result": d.get("result"),
                "planning_cycles": d.get("planning_cycles"),
                "motion_steps": d.get("motion_steps"),
                "observations": d.get("observations"),
                "finish_reason": d.get("finish_reason"),
                "summary_result": (d.get("summary") or {}).get("result"),
                "summary_cycles": (d.get("summary") or {}).get("planning_cycles"),
                "unique_objects": (d.get("summary") or {}).get("unique_objects"),
                "unique_places": (d.get("summary") or {}).get("unique_places"),
                "semantic_topology_nodes": len(
                    (((d.get("spatial") or {}).get("semantic_graph") or {})
                        .get("object_topology") or {}).get("nodes") or []
                ),
            }

        self.assertEqual(_core(first), _core(second))

    def test_replay_jsonl_roundtrip(self) -> None:
        _, events = _run_session()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "session.jsonl"
            with path.open("w", encoding="utf-8") as handle:
                for event in events:
                    handle.write(json.dumps(event, ensure_ascii=False) + "\n")
            loaded: list[dict] = []
            for line in path.read_text(encoding="utf-8").splitlines():
                loaded.append(json.loads(line))
        self.assertEqual(len([e for e in loaded if e["event"] == "observation"]),
                         len([e for e in events if e["event"] == "observation"]))
        explorer = _replay(loaded, target="蓝色垃圾桶")
        self.assertEqual(explorer.run().result, "TARGET_FOUND")


if __name__ == "__main__":
    unittest.main()
