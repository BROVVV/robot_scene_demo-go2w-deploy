"""Build and export the source-aware Predictive Scene Graph v2."""

from __future__ import annotations

import json
from pathlib import Path

import networkx as nx

from app.reasoning.observed_facts_builder import infer_object_source
from app.schemas import (
    Actionability,
    EvidenceSource,
    LLMReasoningResult,
    NodeObservationStatus,
    QuadrupedSearchPlan,
    ReasonedPSGEdge,
    ReasonedPSGNode,
    ReasonedPredictiveSceneGraph,
    SceneAnalysisResult,
)


def build_reasoned_predictive_scene_graph(
    base_scene: SceneAnalysisResult,
    llm_reasoning: LLMReasoningResult,
    quadruped_plan: QuadrupedSearchPlan,
    retrieved_experiences: list[dict] | None = None,
) -> ReasonedPredictiveSceneGraph:
    nodes: list[ReasonedPSGNode] = []
    edges: list[ReasonedPSGEdge] = []
    for obj in base_scene.objects:
        if not obj.visible:
            continue
        nodes.append(
            ReasonedPSGNode(
                node_id=f"observed_{obj.id}",
                label_zh=obj.name_zh,
                node_type="observed_object",
                observation_status=NodeObservationStatus.OBSERVED,
                source=infer_object_source(obj),
                confidence=obj.final_score or obj.confidence,
                bbox=[
                    obj.bbox_2d.x1,
                    obj.bbox_2d.y1,
                    obj.bbox_2d.x2,
                    obj.bbox_2d.y2,
                ],
                image_region_hint=obj.position.horizontal,
                evidence_summary_zh="来自当前视觉检测结果。",
                generated_by="ObservedSceneFactsBuilder",
                display_style="solid",
                border_color="#166534",
                fill_color="#dcfce7",
            )
        )
    observed_ids = {node.node_id for node in nodes}
    for relation in base_scene.relations:
        source_id = f"observed_{relation.source_id}"
        target_id = f"observed_{relation.target_id}"
        if source_id not in observed_ids or target_id not in observed_ids:
            continue
        edges.append(
            ReasonedPSGEdge(
                source_id=source_id,
                target_id=target_id,
                relation_type="visible_relation",
                observation_status=NodeObservationStatus.OBSERVED,
                source=EvidenceSource.GEOMETRIC_RELATION,
                confidence=relation.confidence,
                evidence_summary_zh=(
                    relation.description_zh or relation.relation_type
                ),
                line_style="solid",
            )
        )
    target_node_id = "task_target"
    matched_objects = [
        obj
        for obj in base_scene.objects
        if obj.id in base_scene.target_decision.matched_object_ids
    ]
    nodes.append(
        ReasonedPSGNode(
            node_id=target_node_id,
            label_zh=f"目标：{base_scene.target_decision.target_text}",
            node_type="task_target",
            observation_status=(
                NodeObservationStatus.OBSERVED
                if base_scene.target_decision.is_present
                and base_scene.target_decision.matched_object_ids
                else NodeObservationStatus.INFERRED
            ),
            source=(
                infer_object_source(matched_objects[0])
                if matched_objects
                else EvidenceSource.LLM_SITUATED_REASONING
            ),
            confidence=base_scene.target_decision.confidence,
            evidence_summary_zh=base_scene.target_decision.match_reason_zh,
            generated_by="VisualGroundingGate",
            display_style=(
                "solid"
                if matched_objects
                else "dashed"
            ),
            border_color="#dc2626",
            fill_color="#fee2e2",
        )
    )
    for matched in matched_objects:
        observed_id = f"observed_{matched.id}"
        if observed_id in observed_ids:
            edges.append(
                ReasonedPSGEdge(
                    source_id=observed_id,
                    target_id=target_node_id,
                    relation_type="visible_relation",
                    observation_status=NodeObservationStatus.OBSERVED,
                    source=infer_object_source(matched),
                    confidence=base_scene.target_decision.confidence,
                    evidence_summary_zh="该视觉对象被确认匹配任务目标。",
                    line_style="solid",
                )
            )
    if not base_scene.target_decision.is_present:
        negative_node_id = "negative_evidence_current_view"
        nodes.append(
            ReasonedPSGNode(
                node_id=negative_node_id,
                label_zh="当前视野未确认目标",
                node_type="inferred_context",
                observation_status=NodeObservationStatus.OBSERVED,
                source=EvidenceSource.NEGATIVE_EVIDENCE,
                confidence=base_scene.target_decision.confidence,
                evidence_summary_zh=base_scene.target_decision.match_reason_zh,
                generated_by="VisualGroundingGate",
                display_style="solid",
                border_color="#ca8a04",
                fill_color="#fef9c3",
            )
        )
        edges.append(
            ReasonedPSGEdge(
                source_id=negative_node_id,
                target_id=target_node_id,
                relation_type="negative_evidence",
                observation_status=NodeObservationStatus.OBSERVED,
                source=EvidenceSource.NEGATIVE_EVIDENCE,
                confidence=base_scene.target_decision.confidence,
                evidence_summary_zh="当前画面没有足够视觉证据确认目标。",
                line_style="dashed",
            )
        )
    for index, memory in enumerate(retrieved_experiences or [], start=1):
        memory_id = str(memory.get("memory_id") or f"memory_{index:03d}")
        node_id = f"memory_{memory_id}"
        nodes.append(
            ReasonedPSGNode(
                node_id=node_id,
                label_zh=str(
                    memory.get("hypothesis_region_zh") or "历史搜索经验"
                ),
                node_type="memory_episode",
                observation_status=NodeObservationStatus.INFERRED,
                source=EvidenceSource.EPISODIC_MEMORY,
                confidence=float(memory.get("confidence_after_outcome", 0.5)),
                evidence_summary_zh=str(
                    memory.get("visual_evidence_summary_zh") or "历史经验上下文"
                ),
                generated_by="LLMExperienceMemory",
                display_style="dotted",
                border_color="#7e22ce",
                fill_color="#f3e8ff",
            )
        )
        edges.append(
            ReasonedPSGEdge(
                source_id=node_id,
                target_id=target_node_id,
                relation_type="memory_supports",
                observation_status=NodeObservationStatus.INFERRED,
                source=EvidenceSource.EPISODIC_MEMORY,
                confidence=float(memory.get("retrieval_score", 0.5)),
                evidence_summary_zh=str(
                    memory.get("hypothesis_rationale_zh") or "相似历史经验"
                ),
                line_style="dotted",
            )
        )
    for hypothesis in llm_reasoning.hypotheses:
        node_type = (
            "unreachable_area"
            if hypothesis.status == NodeObservationStatus.UNREACHABLE
            else "inferred_target_area"
            if hypothesis.status != NodeObservationStatus.OBSERVED
            else "observed_place"
        )
        nodes.append(
            ReasonedPSGNode(
                node_id=hypothesis.hypothesis_id,
                label_zh=hypothesis.candidate_region_zh,
                node_type=node_type,
                observation_status=hypothesis.status,
                source=EvidenceSource.LLM_SITUATED_REASONING,
                confidence=hypothesis.confidence,
                image_region_hint=hypothesis.image_region_hint,
                evidence_summary_zh=hypothesis.human_like_rationale_zh,
                actionability=hypothesis.actionability,
                generated_by="LLMSituatedSearchReasoner",
                display_style=(
                    "solid"
                    if hypothesis.status == NodeObservationStatus.OBSERVED
                    else "dashed"
                ),
                border_color=(
                    "#6b7280"
                    if hypothesis.status == NodeObservationStatus.UNREACHABLE
                    else "#dc2626"
                ),
                fill_color=(
                    "#e5e7eb"
                    if hypothesis.status == NodeObservationStatus.UNREACHABLE
                    else "#fef3c7"
                ),
            )
        )
        edges.append(
            ReasonedPSGEdge(
                source_id=target_node_id,
                target_id=hypothesis.hypothesis_id,
                relation_type=(
                    "cannot_verify"
                    if hypothesis.status == NodeObservationStatus.UNREACHABLE
                    else "needs_viewpoint"
                ),
                observation_status=hypothesis.status,
                source=EvidenceSource.LLM_SITUATED_REASONING,
                confidence=hypothesis.confidence,
                evidence_summary_zh=hypothesis.uncertainty_zh,
                line_style="dashed",
            )
        )
        for anchor_id in hypothesis.supporting_visible_anchor_ids:
            observed_id = f"observed_{anchor_id}"
            if any(node.node_id == observed_id for node in nodes):
                edges.append(
                    ReasonedPSGEdge(
                        source_id=observed_id,
                        target_id=hypothesis.hypothesis_id,
                        relation_type="llm_reasoned_relation",
                        observation_status=NodeObservationStatus.INFERRED,
                        source=EvidenceSource.LLM_SITUATED_REASONING,
                        confidence=hypothesis.confidence,
                        evidence_summary_zh=hypothesis.human_like_rationale_zh,
                        line_style="dashed",
                    )
                )
    for step in quadruped_plan.steps:
        nodes.append(
            ReasonedPSGNode(
                node_id=step.step_id,
                label_zh=step.reason_zh,
                node_type="viewpoint_action",
                observation_status=NodeObservationStatus.INFERRED,
                source=EvidenceSource.LLM_SITUATED_REASONING,
                confidence=step.expected_information_gain,
                image_region_hint=step.target_image_region,
                evidence_summary_zh=f"机械狗动作原语：{step.primitive.value}",
                actionability=Actionability.ROBOT_EXECUTABLE,
                generated_by="QuadrupedViewpointPlanner",
                display_style="solid",
                border_color="#2563eb",
                fill_color="#dbeafe",
                motion_horizon_m=step.motion_horizon_m,
                motion_policy=step.motion_policy,
                requires_stop_observation=step.requires_stop_observation,
                platform_obstacle_avoidance_assumed=(
                    step.platform_obstacle_avoidance_assumed
                ),
            )
        )
        edges.append(
            ReasonedPSGEdge(
                source_id=target_node_id,
                target_id=step.step_id,
                relation_type="needs_viewpoint",
                observation_status=NodeObservationStatus.INFERRED,
                source=EvidenceSource.LLM_SITUATED_REASONING,
                confidence=step.expected_information_gain,
                evidence_summary_zh=step.reason_zh,
                line_style="dashed",
            )
        )
    return ReasonedPredictiveSceneGraph(nodes=nodes, edges=edges)


def export_reasoned_predictive_scene_graph(
    graph: ReasonedPredictiveSceneGraph,
    *,
    json_path: str | Path,
    graphml_path: str | Path,
) -> tuple[Path, Path]:
    json_output = Path(json_path)
    graphml_output = Path(graphml_path)
    json_output.parent.mkdir(parents=True, exist_ok=True)
    graphml_output.parent.mkdir(parents=True, exist_ok=True)
    json_output.write_text(
        json.dumps(graph.model_dump(mode="json"), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    nx_graph = nx.DiGraph()
    for node in graph.nodes:
        payload = node.model_dump(mode="json")
        nx_graph.add_node(
            node.node_id,
            node_id=node.node_id,
            **{
                key: (
                    json.dumps(value, ensure_ascii=False)
                    if isinstance(value, (list, dict))
                    else "" if value is None else value
                )
                for key, value in payload.items()
                if key != "node_id"
            },
        )
    for edge in graph.edges:
        payload = edge.model_dump(mode="json")
        nx_graph.add_edge(
            edge.source_id,
            edge.target_id,
            **{
                key: value
                for key, value in payload.items()
                if key not in {"source_id", "target_id"}
            },
        )
    nx.write_graphml(nx_graph, graphml_output)
    return json_output, graphml_output
