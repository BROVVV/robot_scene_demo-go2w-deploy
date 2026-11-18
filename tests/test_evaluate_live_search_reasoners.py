from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from scripts.evaluate_live_search_reasoners import evaluate

REPO_ROOT = Path(__file__).resolve().parents[1]
# 固定录制放在 fixtures，避免 outputs/live_runs 的运行期保留策略删除测试输入。
RECORDED_LIVE_SESSION = (
    REPO_ROOT / "tests/fixtures/live_search_reasoners/search_20260901_120501_96392a00"
)
RECORDED_NO_SCENE_GRAPH = (
    REPO_ROOT / "tests/fixtures/live_search_reasoners/search_20260901_121219_0a3965d1"
)


def _observation(node_id: str, object_labels: list[str]) -> dict:
    """造一个 exploration_graph.json 的观测节点，内嵌一份最小的合法 SceneGraph。"""
    nodes = [
        {
            "node_id": "place_001", "node_type": "place", "label": "unknown",
            "label_zh": "未知地点", "category": "place", "source": "observed",
            "confidence": 0.9, "evidence_level": "observed_confirmed",
            "based_on": ["frame_1"], "can_confirm_target": True,
            "attributes": {},
        }
    ]
    edges = []
    for index, label in enumerate(object_labels):
        object_id = f"obj_{label}_{index:03d}"
        nodes.append({
            "node_id": object_id, "node_type": "object", "label": label,
            "label_zh": label, "category": "object", "source": "observed",
            "confidence": 0.9, "evidence_level": "observed_confirmed",
            "based_on": ["frame_1"], "can_confirm_target": False,
            "attributes": {},
        })
        edges.append({
            "edge_id": f"edge_{index:04d}", "source_node_id": "place_001",
            "target_node_id": object_id, "relation": "contains",
            "source": "observed", "confidence": 0.9,
            "evidence_level": "observed_confirmed", "reason": None,
            "attributes": {},
        })
    return {
        "node_id": node_id,
        "objects": object_labels,
        "scene_graph": {"nodes": nodes, "edges": edges},
    }


def _aimed_observation(node_id: str, sector: int, heading: float) -> dict:
    """一个只有地点、没有任何目标物体的观测 —— 让图匹配落到 ZERO，
    从而走 `_best_unseen_heading` 那条会看 `observed_heading_sectors` 的分支。"""
    observation = _observation(node_id, ["办公椅"])
    observation["heading"] = heading
    observation["heading_sector"] = sector
    return observation


def _exploration_session(root: Path, observations: list[dict]) -> Path:
    session = root / "exploration_session"
    session.mkdir()
    (session / "exploration_graph.json").write_text(
        json.dumps({"session_id": "expl", "nodes": observations}), encoding="utf-8"
    )
    return session


class ReasonerReplayEvaluatorTests(unittest.TestCase):
    def _session(self, root: Path) -> Path:
        session = root / "session"
        session.mkdir()
        graph = {
            "nodes": [
                {
                    "node_id": "trash", "node_type": "object",
                    "label": "trash bin", "label_zh": "垃圾桶",
                    "category": "container", "source": "observed",
                    "confidence": 0.9, "evidence_level": "observed_confirmed",
                    "attributes": {"attributes": ["blue"], "stable_position_2d": "left"},
                },
                {
                    "node_id": "water", "node_type": "object",
                    "label": "water cooler", "label_zh": "饮水机",
                    "category": "appliance", "source": "observed",
                    "confidence": 0.9, "evidence_level": "observed_confirmed",
                    "attributes": {"stable_position_2d": "left"},
                },
            ],
            "edges": [
                {
                    "edge_id": "near", "source_node_id": "trash",
                    "target_node_id": "water", "relation": "near",
                    "source": "observed", "confidence": 0.9,
                    "evidence_level": "observed_confirmed",
                }
            ],
        }
        (session / "scene_graph.json").write_text(json.dumps(graph), encoding="utf-8")
        observations = [{"frame_id": value} for value in (1, 2, 3)]
        (session / "frame_observations.json").write_text(
            json.dumps(observations), encoding="utf-8"
        )
        (session / "target_profile.json").write_text(
            json.dumps({"resolver_source": "llm"}), encoding="utf-8"
        )
        (session / "crop_verify_results.json").write_text(
            json.dumps({"attempted": 2}), encoding="utf-8"
        )
        (session / "search_directive.json").write_text(
            json.dumps({"kind": "reobserve_sector"}), encoding="utf-8"
        )
        (session / "target_search.json").write_text(json.dumps({
            "task": {"detector": "llm"},
            "best_evidence": {"timestamp_sec": 1.2},
            "timeline": [
                {"type": "direct_detection", "timestamp_sec": 1.2}
            ],
        }), encoding="utf-8")
        (session / "evidence_gating_report.json").write_text(json.dumps({
            "target_found": False,
            "candidates": [
                {"blocking_rules": ["crop"]}, {"blocking_rules": []}
            ],
        }), encoding="utf-8")
        return session

    def test_reports_real_artifact_metrics_and_shadow_invariant(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session = self._session(root)
            report = evaluate(
                session=str(session),
                target="饮水机旁边的蓝色垃圾桶",
                output_dir=str(root / "report"),
            )
            self.assertEqual(report["detector_calls"], 3)
            self.assertEqual(report["recorded_llm_calls_inferred"], 6)
            self.assertEqual(report["verify_reject_count"], 1)
            self.assertEqual(report["time_to_candidate"], 1.2)
            self.assertIsNone(report["time_to_target"])
            self.assertTrue(report["success_proxy"])
            self.assertFalse(report["target_confirmed"])
            self.assertTrue(report["actual_shadow_behavior_matches_legacy"])
            self.assertEqual(report["dangerous_forward_request_count"], 0)
            self.assertTrue(all(
                row["shadow_executed_step"] == row["legacy_step"]
                for row in report["results"]
            ))
            self.assertTrue((root / "report/reasoner_comparison.json").is_file())
            self.assertTrue((root / "report/reasoner_metrics.json").is_file())
            self.assertTrue((root / "report/shadow_comparison.json").is_file())

    def test_event_log_counts_only_verified_physical_progress(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session = self._session(root)
            event_log = root / "events.jsonl"
            event_log.write_text("\n".join([
                json.dumps({"event": "step_start", "step": "l20"}),
                json.dumps({"event": "step_verified", "step": "l20", "distance_m": 0.02}),
                json.dumps({"event": "step_start", "step": "f"}),
                json.dumps({"event": "step_verified", "step": "f", "distance_m": 0.18}),
            ]) + "\n", encoding="utf-8")
            report = evaluate(
                session=str(session), target="手机",
                output_dir=str(root / "report"), event_log=str(event_log),
            )
            self.assertEqual(report["search_steps"], 2)
            self.assertEqual(report["total_turn_deg"], 20.0)
            self.assertAlmostEqual(report["estimated_distance"], 0.2)
            self.assertIsNone(report["negative_revisit_count"])

    def test_exploration_graph_fallback_supplies_the_scene_graph(self) -> None:
        """真机会话（run_semantic_exploration.py）不写 scene_graph.json，
        必须能从 exploration_graph.json 的 nodes[].scene_graph 回退读到图。"""
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session = _exploration_session(
                root, [_observation("node_a", ["办公椅", "垃圾桶"])]
            )
            self.assertFalse((session / "scene_graph.json").exists())
            report = evaluate(
                session=str(session), target="垃圾桶",
                output_dir=str(root / "report"),
            )
            self.assertEqual(
                report["scene_graph_source"],
                "exploration_graph.json:nodes[].scene_graph",
            )
            self.assertEqual(report["scene_nodes"], 3)   # place + 2 objects
            self.assertEqual(report["scene_edges"], 2)
            self.assertEqual(len(report["results"]), 3)  # 三个 backend 都跑到了
            self.assertEqual(report["llm_calls"], 0)

    def test_observed_sectors_are_fed_into_the_replay_context(self) -> None:
        """回放必须把当时的 yaw 与已观测扇区喂进 SearchReasoningContext。

        不喂的后果不是"略有偏差"：`_best_unseen_heading` 的六个候选会全部平局、
        退化成恒选 −30，于是报告会给出一个**看起来像发现**的假结论。
        这里用 yaw=0、已观测扇区=[−1]（即 −30° 那个方向已经看过）：
        喂进去了 ⇒ 避开 −30 选 +30；没喂 ⇒ 平局选 −30。
        """
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session = _exploration_session(
                root, [_aimed_observation("node_a", sector=-1, heading=0.0)]
            )
            report = evaluate(session=str(session), target="垃圾桶",
                              output_dir=str(root / "report"))
            self.assertEqual(report["replay_context"]["robot_yaw_deg"], 0.0)
            self.assertEqual(
                report["replay_context"]["observed_heading_sectors"], [-1]
            )
            semantic = [r for r in report["results"]
                        if r["backend"] == "semantic_navigation"][0]
            self.assertEqual(semantic["directive"]["kind"], "explore_unseen")
            self.assertEqual(
                semantic["directive"]["preferred_heading_delta_deg"], 30.0
            )
            self.assertEqual(semantic["proposed_step"], "l30")

    def test_a_different_observed_sector_flips_the_chosen_heading(self) -> None:
        """反面对照：同一张图，只把"已观测扇区"从 −1 换成 0（没有任何候选
        方向落在扇区 0）⇒ 不再有惩罚、回到平局，选回 −30。"""
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session = _exploration_session(
                root, [_aimed_observation("node_a", sector=0, heading=0.0)]
            )
            report = evaluate(session=str(session), target="垃圾桶",
                              output_dir=str(root / "report"))
            self.assertEqual(report["replay_context"]["observed_heading_sectors"], [0])
            semantic = [r for r in report["results"]
                        if r["backend"] == "semantic_navigation"][0]
            self.assertEqual(
                semantic["directive"]["preferred_heading_delta_deg"], -30
            )

    def test_yaw_comes_from_the_selected_observation(self) -> None:
        """yaw 必须取"被选中的那次观测"，不是整场最后一次。"""
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session = _exploration_session(root, [
                _aimed_observation("node_early", sector=0, heading=12.5),
                _aimed_observation("node_late", sector=4, heading=123.5),
            ])
            early = evaluate(session=str(session), target="垃圾桶",
                             output_dir=str(root / "early"), node_index=0)
            late = evaluate(session=str(session), target="垃圾桶",
                            output_dir=str(root / "late"), node_index=-1)
            self.assertEqual(early["replay_context"]["robot_yaw_deg"], 12.5)
            self.assertEqual(late["replay_context"]["robot_yaw_deg"], 123.5)
            # 扇区取"到这次观测为止"，早期观测不该看到后面才走过的扇区。
            self.assertEqual(early["replay_context"]["observed_heading_sectors"], [0])
            self.assertEqual(late["replay_context"]["observed_heading_sectors"], [0, 4])

    def test_node_index_picks_the_requested_observation(self) -> None:
        """不同观测看到的物体数不同；--node-index 必须真的换一张图。"""
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session = _exploration_session(root, [
                _observation("node_early", ["办公椅"]),
                _observation("node_late", ["办公椅", "垃圾桶", "墙壁"]),
            ])
            first = evaluate(session=str(session), target="垃圾桶",
                             output_dir=str(root / "r0"), node_index=0)
            last = evaluate(session=str(session), target="垃圾桶",
                            output_dir=str(root / "r-1"), node_index=-1)
            self.assertEqual(first["scene_nodes"], 2)
            self.assertEqual(last["scene_nodes"], 4)
            self.assertEqual(first["scene_node_index"], 0)
            self.assertEqual(last["scene_node_index"], -1)

    def test_node_index_out_of_range_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session = _exploration_session(root, [_observation("node_a", ["垃圾桶"])])
            with self.assertRaises(ValueError) as caught:
                evaluate(session=str(session), target="垃圾桶",
                         output_dir=str(root / "report"), node_index=7)
            self.assertIn("越界", str(caught.exception))

    def test_exploration_graph_without_scene_graph_is_rejected(self) -> None:
        """有 exploration_graph.json 但里面没有 scene_graph 时，
        必须报错而不是回退成一张空图 —— 空图会让三个 backend 都"没分歧"，
        看起来像"评估通过"，实际是什么都没测。"""
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session = root / "session"
            session.mkdir()
            (session / "exploration_graph.json").write_text(
                json.dumps({"session_id": "expl", "nodes": [
                    {"node_id": "node_a", "objects": [], "scene_graph": None},
                ]}), encoding="utf-8",
            )
            with self.assertRaises(ValueError) as caught:
                evaluate(session=str(session), target="垃圾桶",
                         output_dir=str(root / "report"))
            self.assertIn("没有带 scene_graph", str(caught.exception))

    def test_recorded_live_session_replays_from_the_tracked_recording(self) -> None:
        """对着**被 git 跟踪的真机录制**回放，端到端走一遍。"""
        report = evaluate(
            session=str(RECORDED_LIVE_SESSION), target="粉色大象",
            output_dir=str(Path(tempfile.mkdtemp()) / "report"),
        )
        self.assertEqual(
            report["scene_graph_source"],
            "exploration_graph.json:nodes[].scene_graph",
        )
        self.assertEqual(report["scene_nodes"], 5)
        self.assertEqual(report["scene_edges"], 4)
        self.assertEqual({row["backend"] for row in report["results"]},
                         {"legacy", "semantic_navigation", "hybrid"})
        # 这条录制的 exploration_graph 里一个观测节点都没有 —— 必须是报错，
        # 不能是"回放成功但没测到东西"。
        with self.assertRaises(ValueError):
            evaluate(session=str(RECORDED_NO_SCENE_GRAPH), target="垃圾桶",
                     output_dir=str(Path(tempfile.mkdtemp()) / "report"))

    def test_degenerate_embedded_scene_graph_is_reported_as_such(self) -> None:
        """实测 outputs/live_runs 的 102 个可回放会话里有 45 个内嵌 scene_graph
        只有四个键（mock/早期会话）。这必须报成"这份录制装不下"，不能是
        裸 TypeError —— 那看起来像评估器自己坏了。"""
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session = root / "session"
            session.mkdir()
            (session / "exploration_graph.json").write_text(json.dumps({
                "session_id": "mock", "nodes": [{
                    "node_id": "node_a",
                    "scene_graph": {"nodes": [
                        {"node_id": "mock_desk", "label": "desk",
                         "label_zh": "desk", "attributes": {}},
                    ], "edges": []},
                }],
            }), encoding="utf-8")
            with self.assertRaises(ValueError) as caught:
                evaluate(session=str(session), target="垃圾桶",
                         output_dir=str(root / "report"))
            self.assertIn("exploration_graph.json", str(caught.exception))
            self.assertIn("无法回放", str(caught.exception))

    def test_absent_video_artifacts_are_flagged_not_reported_as_zero(self) -> None:
        """真机会话没有视频管线的产物，报告必须明说这是"缺席"，
        否则 detector_calls=0 / success_proxy=false 会被读成实测结论。"""
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session = _exploration_session(root, [_observation("node_a", ["垃圾桶"])])
            report = evaluate(session=str(session), target="垃圾桶",
                              output_dir=str(root / "report"))
            self.assertTrue(any("缺席" in item for item in report["limitations"]),
                            report["limitations"])
            self.assertEqual(report["detector_calls"], 0)
            self.assertFalse(report["success_proxy"])

    def test_video_pipeline_session_is_not_flagged_as_absent(self) -> None:
        """反面对照：有 frame_observations.json 的会话不该被贴"缺席"标签。"""
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session = self._session(root)
            report = evaluate(session=str(session), target="手机",
                              output_dir=str(root / "report"))
            self.assertFalse(any("缺席" in item for item in report["limitations"]))
            self.assertEqual(report["detector_calls"], 3)


if __name__ == "__main__":
    unittest.main()
