from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import networkx as nx

from app.schemas import Actionability, NodeObservationStatus
from app.planning.quadruped_viewpoint_planner import plan_quadruped_viewpoints
from app.reasoning.llm_situated_search_reasoner import LLMSituatedSearchReasoner
from app.reasoning.observed_facts_builder import build_observed_scene_facts
from app.schemas import LLMReasoningRequest, SceneAnalysisResult, default_quadruped_capability
from app.services.predictive_scene_graph import (
    build_reasoned_predictive_scene_graph,
    export_reasoned_predictive_scene_graph,
)
from streamlit_app import _reasoned_graph_dot


ROOT = Path(__file__).resolve().parents[1]


class ReasonedPSGExportTest(unittest.TestCase):
    def test_exports_source_status_and_actionability(self) -> None:
        scene = SceneAnalysisResult.model_validate(
            json.loads(
                (ROOT / "examples/mock_knowledge_aware_result.json").read_text(
                    encoding="utf-8"
                )
            )["base_scene"]
        )
        facts = build_observed_scene_facts(scene, "找到手机")
        contract = default_quadruped_capability()
        reasoning = LLMSituatedSearchReasoner(
            auto_create_client=False
        ).reason(
            LLMReasoningRequest(
                target_text="找到手机",
                observed_facts=facts,
                capability_contract=contract,
            )
        )
        unreachable = reasoning.hypotheses[0].model_copy(
            update={
                "hypothesis_id": "hyp_unreachable",
                "candidate_region_zh": "封闭柜体内部",
                "status": NodeObservationStatus.UNREACHABLE,
                "actionability": Actionability.NEEDS_HUMAN,
                "should_not_mark_found": True,
            }
        )
        reasoning = reasoning.model_copy(
            update={"hypotheses": [*reasoning.hypotheses, unreachable]}
        )
        plan = plan_quadruped_viewpoints(
            target_text="找到手机",
            target_found=False,
            hypotheses=reasoning.hypotheses,
            contract=contract,
        )
        graph = build_reasoned_predictive_scene_graph(
            scene,
            reasoning,
            plan,
            retrieved_experiences=[
                {
                    "memory_id": "mem_001",
                    "hypothesis_region_zh": "历史桌面区域",
                    "confidence_after_outcome": 0.7,
                    "retrieval_score": 0.8,
                    "visual_evidence_summary_zh": "历史视觉证据",
                    "hypothesis_rationale_zh": "相似办公室经验",
                }
            ],
        )
        self.assertTrue(
            any(edge.relation_type == "visible_relation" for edge in graph.edges)
        )
        node_types = {node.node_type for node in graph.nodes}
        self.assertTrue(
            {
                "observed_object",
                "inferred_target_area",
                "unreachable_area",
                "memory_episode",
                "viewpoint_action",
            }
            <= node_types
        )
        dot = _reasoned_graph_dot(graph)
        self.assertIn("dashed", dot)
        self.assertIn("unreachable", dot)
        with tempfile.TemporaryDirectory() as tmpdir:
            json_path, graphml_path = export_reasoned_predictive_scene_graph(
                graph,
                json_path=Path(tmpdir) / "graph.json",
                graphml_path=Path(tmpdir) / "graph.graphml",
            )
            payload = json.loads(json_path.read_text(encoding="utf-8"))
            loaded = nx.read_graphml(graphml_path)
        self.assertTrue(payload["nodes"])
        node_data = next(iter(loaded.nodes.values()))
        self.assertIn("observation_status", node_data)
        self.assertIn("node_id", node_data)
        self.assertIn("source", node_data)
        self.assertIn("generated_by", node_data)
        self.assertIn("display_style", node_data)


if __name__ == "__main__":
    unittest.main()
