"""Build Predictive Scene Graphs from visible scene results and knowledge."""

from __future__ import annotations

import json
from pathlib import Path

import networkx as nx

from app.schemas import (
    KnowledgeItem,
    PredictiveSceneGraph,
    PredictiveSceneGraphEdge,
    PredictiveSceneGraphNode,
    RobotTask,
    SceneAnalysisResult,
    SceneObject,
)


PHONE_ALIASES = {"phone", "mobile_phone", "cell_phone", "手机"}
DESK_CONTEXT = {"desk", "table", "keyboard", "charger", "monitor", "办公桌", "桌子", "键盘", "充电器", "显示器"}
DOOR_ALIASES = {"door", "doorplate", "room_number", "门", "门牌", "房间号"}


def build_predictive_scene_graph(
    scene: SceneAnalysisResult,
    retrieved_knowledge: list[KnowledgeItem],
    task: RobotTask,
) -> PredictiveSceneGraph:
    builder = _PSGBuilder(scene, retrieved_knowledge, task)
    return builder.build()


def export_predictive_scene_graph_graphml(
    graph: PredictiveSceneGraph,
    output_path: str | Path,
) -> Path:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    nx.write_graphml(_to_networkx_graph(graph), path)
    return path


def _to_networkx_graph(graph: PredictiveSceneGraph) -> nx.DiGraph:
    nx_graph = nx.DiGraph()
    for node in graph.nodes:
        nx_graph.add_node(
            node.id,
            label=node.label,
            node_type=node.node_type,
            object_id=node.object_id or "",
            visible=node.visible,
            confidence=node.confidence,
            reason_zh=node.reason_zh or "",
            source_ids=json.dumps(node.source_ids, ensure_ascii=False),
        )
    for edge in graph.edges:
        nx_graph.add_edge(
            edge.source_id,
            edge.target_id,
            edge_type=edge.edge_type,
            label=edge.label or "",
            confidence=edge.confidence,
            reason_zh=edge.reason_zh or "",
            source_ids=json.dumps(edge.source_ids, ensure_ascii=False),
        )
    return nx_graph


class _PSGBuilder:
    def __init__(
        self,
        scene: SceneAnalysisResult,
        retrieved_knowledge: list[KnowledgeItem],
        task: RobotTask,
    ) -> None:
        self.scene = scene
        self.retrieved_knowledge = retrieved_knowledge
        self.task = task
        self.nodes: dict[str, PredictiveSceneGraphNode] = {}
        self.edges: list[PredictiveSceneGraphEdge] = []
        self.observed_node_ids: list[str] = []
        self.inferred_node_ids: list[str] = []
        self.uncertainty_notes: list[str] = []
        self.recommended_verification_targets: list[str] = []

    def build(self) -> PredictiveSceneGraph:
        self._add_observed_objects()
        self._add_visible_relations()
        self._add_task_target_node()
        self._add_phone_candidates()
        self._add_adjacent_room_candidates()
        self._add_door_inspection_candidates()

        return PredictiveSceneGraph(
            nodes=list(self.nodes.values()),
            edges=self.edges,
            observed_node_ids=self.observed_node_ids,
            inferred_node_ids=self.inferred_node_ids,
            uncertainty_notes=self.uncertainty_notes,
            recommended_verification_targets=self.recommended_verification_targets,
        )

    def _add_observed_objects(self) -> None:
        for obj in self.scene.objects:
            node_id = _observed_node_id(obj.id)
            self.nodes[node_id] = PredictiveSceneGraphNode(
                id=node_id,
                label=obj.name_zh or obj.name,
                node_type="observed_object",
                object_id=obj.id,
                visible=obj.visible,
                confidence=obj.confidence,
                reason_zh="来自当前视觉检测结果。",
                source_ids=[obj.id],
            )
            if obj.visible:
                self.observed_node_ids.append(node_id)

    def _add_visible_relations(self) -> None:
        for index, relation in enumerate(self.scene.relations, start=1):
            source_id = _observed_node_id(relation.source_id)
            target_id = _observed_node_id(relation.target_id)
            if source_id not in self.nodes or target_id not in self.nodes:
                continue
            self.edges.append(
                PredictiveSceneGraphEdge(
                    source_id=source_id,
                    target_id=target_id,
                    edge_type="visible_relation",
                    label=relation.description_zh or relation.relation_type,
                    confidence=relation.confidence,
                    reason_zh="来自当前场景显式关系或关系补全结果。",
                    source_ids=[f"relation_{index}", relation.source_id, relation.target_id],
                )
            )

    def _add_task_target_node(self) -> None:
        target_label = self.task.target_object or self.task.target_room or self.task.raw_text
        node_id = "psg_task_target"
        self.nodes[node_id] = PredictiveSceneGraphNode(
            id=node_id,
            label=f"目标：{target_label}",
            node_type="task_target_node",
            object_id=None,
            visible=False,
            confidence=self.task.confidence,
            reason_zh="来自用户任务解析结果。",
            source_ids=[self.task.task_id],
        )
        self.inferred_node_ids.append(node_id)

    def _add_phone_candidates(self) -> None:
        target = (self.task.target_object or self.task.raw_text).lower()
        if not any(alias in target for alias in PHONE_ALIASES):
            return
        if self._target_present():
            return

        context_objects = [
            obj for obj in self.scene.objects if _object_matches_any(obj, DESK_CONTEXT)
        ]
        if not context_objects:
            return

        knowledge_sources = [
            item.id
            for item in self.retrieved_knowledge
            if item.knowledge_type == "object_location_prior"
        ]
        best_context = _prefer_context_object(context_objects)
        candidate_id = "psg_inferred_phone_candidate_001"
        label = "候选手机位置"
        location = _phone_candidate_location(best_context)

        self.nodes[candidate_id] = PredictiveSceneGraphNode(
            id=candidate_id,
            label=label,
            node_type="inferred_object",
            object_id=None,
            visible=False,
            confidence=_bounded_confidence(0.55 + best_context.confidence * 0.25),
            reason_zh=(
                f"当前目标不可见，但画面中可见{best_context.name_zh}；"
                "知识库或常识显示手机常出现在桌面、键盘旁或充电器附近。"
            ),
            source_ids=[best_context.id, *knowledge_sources],
        )
        self.inferred_node_ids.append(candidate_id)
        self._append_edge(
            candidate_id,
            _observed_node_id(best_context.id),
            "containment_prior",
            f"手机可能在{location}",
            0.68,
            "由目标物体位置先验和当前可见上下文物体推断。",
            [best_context.id, *knowledge_sources],
        )
        self._append_edge(
            "psg_task_target",
            candidate_id,
            "verification_relation",
            f"需要重新观察{location}",
            0.72,
            "候选位置尚未被当前视角验证。",
            [self.task.task_id],
        )
        self.uncertainty_notes.append("当前视角未直接看到手机，候选位置需要移动或转向后验证。")
        self.recommended_verification_targets.append(location)

    def _add_adjacent_room_candidates(self) -> None:
        if self.task.task_type not in {"find_room", "navigate_to_location"}:
            return

        visible_doors = [obj for obj in self.scene.objects if _object_matches_any(obj, DOOR_ALIASES)]
        floor_items = [
            item
            for item in self.retrieved_knowledge
            if item.knowledge_type == "environment_layout"
        ]
        if not visible_doors and not floor_items:
            return

        room_label = self.task.target_room or self.task.target_location or "相邻房间"
        candidate_id = "psg_inferred_adjacent_room_001"
        source_ids = [obj.id for obj in visible_doors] + [item.id for item in floor_items]
        self.nodes[candidate_id] = PredictiveSceneGraphNode(
            id=candidate_id,
            label=f"候选房间：{room_label}",
            node_type="place_node",
            object_id=None,
            visible=False,
            confidence=0.62 if floor_items else 0.48,
            reason_zh="根据可见门牌或楼层布局知识推断目标房间可能在相邻门序中。",
            source_ids=source_ids,
        )
        self.inferred_node_ids.append(candidate_id)

        if visible_doors:
            anchor_id = _observed_node_id(visible_doors[0].id)
            self._append_edge(
                candidate_id,
                anchor_id,
                "navigation_relation",
                "候选房间与当前可见门牌相邻",
                0.62,
                "由门牌观察和楼层门序先验推断。",
                source_ids,
            )
        self.recommended_verification_targets.append(room_label)
        self.uncertainty_notes.append("相邻房间候选依赖门牌识别和楼层布局先验，需要实地确认。")

    def _add_door_inspection_candidates(self) -> None:
        if self.task.task_type not in {"inspect_area", "check_door_state"}:
            return
        if not _task_mentions_door(self.task):
            return

        visible_doors = [obj for obj in self.scene.objects if _object_matches_any(obj, DOOR_ALIASES)]
        floor_items = [
            item
            for item in self.retrieved_knowledge
            if item.knowledge_type == "environment_layout"
        ]

        if visible_doors:
            for index, door in enumerate(visible_doors, start=1):
                candidate_id = f"psg_door_state_check_{index:03d}"
                self.nodes[candidate_id] = PredictiveSceneGraphNode(
                    id=candidate_id,
                    label=f"待检查门状态：{door.name_zh}",
                    node_type="place_node",
                    object_id=None,
                    visible=False,
                    confidence=0.66,
                    reason_zh="门状态任务需要从更合适角度确认开关状态。",
                    source_ids=[door.id, self.task.task_id],
                )
                self.inferred_node_ids.append(candidate_id)
                self._append_edge(
                    candidate_id,
                    _observed_node_id(door.id),
                    "verification_relation",
                    "靠近并观察门缝、门把手和门板角度",
                    0.7,
                    "当前单张图片可能不足以可靠判断门状态。",
                    [door.id, self.task.task_id],
                )
                self.recommended_verification_targets.append(door.name_zh)

        if floor_items and not visible_doors:
            candidate_id = "psg_floor_doors_to_inspect"
            self.nodes[candidate_id] = PredictiveSceneGraphNode(
                id=candidate_id,
                label="楼层待巡查门",
                node_type="place_node",
                object_id=None,
                visible=False,
                confidence=0.58,
                reason_zh="楼层知识显示存在多个门，需要按走廊拓扑逐一验证状态。",
                source_ids=[item.id for item in floor_items],
            )
            self.inferred_node_ids.append(candidate_id)
            self._append_edge(
                "psg_task_target",
                candidate_id,
                "verification_relation",
                "沿走廊逐一检查门状态",
                0.62,
                "当前视野没有可靠门状态，需要按楼层知识规划巡查。",
                [self.task.task_id, *[item.id for item in floor_items]],
            )
            self.recommended_verification_targets.append("楼层门状态")

        if visible_doors or floor_items:
            self.uncertainty_notes.append("门状态属于临时状态，应作为观察结果记录，不应直接覆盖长期布局知识。")

    def _append_edge(
        self,
        source_id: str,
        target_id: str,
        edge_type: str,
        label: str,
        confidence: float,
        reason_zh: str,
        source_ids: list[str],
    ) -> None:
        if source_id not in self.nodes or target_id not in self.nodes:
            return
        self.edges.append(
            PredictiveSceneGraphEdge(
                source_id=source_id,
                target_id=target_id,
                edge_type=edge_type,  # type: ignore[arg-type]
                label=label,
                confidence=confidence,
                reason_zh=reason_zh,
                source_ids=source_ids,
            )
        )

    def _target_present(self) -> bool:
        return self.scene.target_decision.is_present and bool(
            self.scene.target_decision.matched_object_ids
        )


def _observed_node_id(object_id: str) -> str:
    return f"psg_{object_id}"


def _object_matches_any(obj: SceneObject, candidates: set[str]) -> bool:
    values = {obj.name.lower(), obj.name_zh.lower(), obj.category.lower()}
    values.update(attribute.lower() for attribute in obj.attributes)
    return any(candidate in value for value in values for candidate in candidates)


def _prefer_context_object(objects: list[SceneObject]) -> SceneObject:
    priority = ["charger", "充电器", "keyboard", "键盘", "desk", "办公桌", "table", "桌子"]
    for name in priority:
        for obj in objects:
            if _object_matches_any(obj, {name}):
                return obj
    return max(objects, key=lambda item: item.confidence)


def _phone_candidate_location(context: SceneObject) -> str:
    if _object_matches_any(context, {"keyboard", "键盘"}):
        return "键盘旁"
    if _object_matches_any(context, {"charger", "充电器"}):
        return "充电器附近"
    if _object_matches_any(context, {"monitor", "显示器"}):
        return "显示器底座附近"
    return f"{context.name_zh}桌面区域"


def _task_mentions_door(task: RobotTask) -> bool:
    text = " ".join(
        value
        for value in [
            task.raw_text,
            task.target_object or "",
            task.target_location or "",
            task.target_room or "",
        ]
        if value
    ).lower()
    return any(alias in text for alias in DOOR_ALIASES)


def _bounded_confidence(value: float) -> float:
    return max(0.0, min(1.0, round(value, 3)))
