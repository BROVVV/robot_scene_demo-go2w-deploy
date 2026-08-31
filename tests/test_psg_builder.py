from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import networkx as nx

from app.knowledge.retrieval import retrieve_relevant_knowledge
from app.schemas import (
    BoundingBox2D,
    Position,
    RobotTask,
    RoutePlan,
    SceneAnalysisResult,
    SceneObject,
    TargetDecision,
    TopologyGraph,
)
from app.services.psg_builder import (
    build_predictive_scene_graph,
    export_predictive_scene_graph_graphml,
)


ROOT = Path(__file__).resolve().parents[1]


class PSGBuilderTest(unittest.TestCase):
    def test_builds_phone_candidate_from_office_context(self) -> None:
        scene = _load_scene("examples/mock_knowledge_aware_result.json").base_scene
        task = RobotTask(
            task_id="task_001",
            raw_text="找到手机",
            task_type="find_object",
            target_object="phone",
            scope="current_room",
            confidence=0.9,
        )
        knowledge = retrieve_relevant_knowledge(
            target_text="手机",
            room_type="office",
            kb_dir=ROOT / "data" / "scene_kb",
        )

        graph = build_predictive_scene_graph(scene, knowledge, task)

        inferred = {node.id: node for node in graph.nodes if not node.visible}
        self.assertIn("psg_inferred_phone_candidate_001", inferred)
        self.assertIn("psg_inferred_phone_candidate_001", graph.inferred_node_ids)
        self.assertIn("键盘旁", graph.recommended_verification_targets)
        self.assertTrue(
            any(edge.edge_type == "containment_prior" for edge in graph.edges)
        )

    def test_exports_predictive_scene_graph_graphml(self) -> None:
        scene = _load_scene("examples/mock_knowledge_aware_result.json").base_scene
        task = RobotTask(
            task_id="task_001",
            raw_text="找到手机",
            task_type="find_object",
            target_object="phone",
            scope="current_room",
            confidence=0.9,
        )
        graph = build_predictive_scene_graph(scene, [], task)

        with tempfile.TemporaryDirectory() as tmpdir:
            path = export_predictive_scene_graph_graphml(
                graph, Path(tmpdir) / "predictive_scene_graph.graphml"
            )
            loaded = nx.read_graphml(path)

        self.assertIn("psg_obj_001", loaded.nodes)
        self.assertIn("node_type", loaded.nodes["psg_obj_001"])
        self.assertGreaterEqual(loaded.number_of_edges(), 1)

    def test_builds_door_inspection_candidates(self) -> None:
        scene = _door_scene()
        task = RobotTask(
            task_id="task_002",
            raw_text="巡查这层楼看看有几个房间的门是打开的",
            task_type="inspect_area",
            target_object="door",
            scope="current_floor",
            confidence=0.88,
        )
        knowledge = retrieve_relevant_knowledge(
            location_hint="floor_5",
            kb_dir=ROOT / "data" / "scene_kb",
        )

        graph = build_predictive_scene_graph(scene, knowledge, task)

        self.assertTrue(
            any(node.id.startswith("psg_door_state_check_") for node in graph.nodes)
        )
        self.assertTrue(
            any(edge.edge_type == "verification_relation" for edge in graph.edges)
        )
        self.assertIn("门", "".join(graph.recommended_verification_targets))


def _load_scene(relative_path: str):
    data = json.loads((ROOT / relative_path).read_text(encoding="utf-8"))
    from app.schemas import KnowledgeAwareSceneResult

    return KnowledgeAwareSceneResult.model_validate(data)


def _door_scene() -> SceneAnalysisResult:
    door = SceneObject(
        id="obj_door_001",
        name="door",
        name_zh="门",
        category="architecture",
        color="white",
        attributes=["doorplate 503", "in corridor"],
        visible=True,
        position=Position(
            horizontal="right",
            vertical="front",
            relative_to_robot="front-right",
            estimated_distance_m=2.0,
        ),
        bbox_2d=BoundingBox2D(x1=0.62, y1=0.18, x2=0.82, y2=0.86),
        confidence=0.82,
    )
    return SceneAnalysisResult(
        scene_summary_zh="当前在走廊，可见右前方一扇门。",
        objects=[door],
        relations=[],
        topology=TopologyGraph(nodes=[], edges=[]),
        target_decision=TargetDecision(
            target_text="巡查这层楼看看有几个房间的门是打开的",
            is_present=True,
            matched_object_ids=["obj_door_001"],
            match_reason_zh="画面中可见一扇门，但门状态仍需确认。",
            confidence=0.72,
        ),
        route_plan=RoutePlan(
            route_type="explore_likely_location",
            summary_zh="靠近门并重新观察门缝。",
            steps=[],
            safety_notes_zh=[],
        ),
    )


if __name__ == "__main__":
    unittest.main()
