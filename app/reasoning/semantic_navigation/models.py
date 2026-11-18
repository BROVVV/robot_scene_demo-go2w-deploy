"""Data contracts for explainable semantic next-view reasoning."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any


@dataclass(frozen=True)
class GoalGraphNode:
    node_id: str
    label: str
    role: str
    aliases: list[str] = field(default_factory=list)
    attributes: list[str] = field(default_factory=list)
    source: str = "target_profile"
    evidence_level: str = "explicit"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class GoalGraphEdge:
    edge_id: str
    source_node_id: str
    target_node_id: str
    relation: str
    source: str = "target_profile"
    evidence_level: str = "explicit"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class GoalGraph:
    task_id: str
    raw_query: str
    target_node_id: str
    nodes: list[GoalGraphNode] = field(default_factory=list)
    edges: list[GoalGraphEdge] = field(default_factory=list)
    build_source: str = "target_profile"
    warnings: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            **asdict(self),
            "nodes": [item.to_dict() for item in self.nodes],
            "edges": [item.to_dict() for item in self.edges],
        }


class GraphMatchState(str, Enum):
    ZERO = "zero_match"
    PARTIAL = "partial_match"
    STRONG = "strong_match"


@dataclass(frozen=True)
class GraphMatchResult:
    state: GraphMatchState
    score: float
    matched_goal_node_ids: list[str] = field(default_factory=list)
    missing_goal_node_ids: list[str] = field(default_factory=list)
    matched_scene_node_ids: list[str] = field(default_factory=list)
    supporting_anchor_scene_node_ids: list[str] = field(default_factory=list)
    unmatched_relations: list[str] = field(default_factory=list)
    target_node_visually_present: bool = False
    reason_zh: str = ""
    evidence_refs: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    matched_relations: list[str] = field(default_factory=list)
    attribute_support: list[str] = field(default_factory=list)
    anchor_support: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["state"] = self.state.value
        return payload


class SearchDirectiveKind(str, Enum):
    LEGACY_SCAN = "legacy_scan"
    INSPECT_ANCHOR = "inspect_anchor"
    REOBSERVE_SECTOR = "reobserve_sector"
    EXPLORE_UNSEEN = "explore_unseen"
    HOLD_AND_REOBSERVE = "hold_and_reobserve"


@dataclass(frozen=True)
class SearchDirective:
    directive_id: str
    kind: SearchDirectiveKind
    source_backend: str
    match_state: str
    confidence: float
    preferred_heading_delta_deg: float | None = None
    preferred_distance_m: float | None = None
    anchor_scene_node_id: str | None = None
    anchor_label: str | None = None
    allow_forward: bool = False
    reason_zh: str = ""
    evidence_refs: list[str] = field(default_factory=list)
    memory_penalties: list[str] = field(default_factory=list)
    fallback_to_legacy: bool = False
    # Safety intent is informational for the executor's physical gate. A
    # directive must NEVER carry an authorizes_motion flag; the executor still
    # runs the full motion-boundary / rotation-lease / dual-LiDAR gate before
    # any arm or Action.
    safety_intent: str = "observe"
    requires_rotation_clearance: bool = False
    requires_front_clearance: bool = False
    # 任务 Agent 的结构化决策依据（计划书 §5.2）。冻结导航消费者只读
    # kind / heading / anchor 三个字段，多出来的字段它一概不看 —— 所以这个
    # 可选字段对下游是无损的。默认 None 保证旧调用者不受影响。
    decision_trace: dict[str, Any] | None = None

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["kind"] = self.kind.value
        return payload


@dataclass
class SearchReasoningContext:
    target_profile: Any = None
    goal_graph: GoalGraph | None = None
    scene_graph: Any = None
    graph_match: GraphMatchResult | None = None
    observation_memory: list[dict[str, Any]] = field(default_factory=list)
    negative_memory: Any = None
    auxiliary_hints: list[dict[str, Any]] = field(default_factory=list)
    auxiliary_status: dict[str, Any] = field(default_factory=dict)
    robot_pose: dict[str, Any] | None = None
    robot_yaw_deg: float = 0.0
    distance_from_origin_m: float = 0.0
    scan_index: int = 0
    safety_context: dict[str, Any] = field(default_factory=dict)
    legacy_scan_candidate: Any = None
    observed_heading_sectors: list[int] = field(default_factory=list)
    semantic_observation: Any = None
    # 计划书 §5.1：把已经存在、但此前没有送进推理器的输入显式带进来。
    # 四项都是可选的，默认值保证现有 4 个调用点一个字都不用改。
    #   task_context     —— SearchTaskContext.to_dict() 的结果（任务、子任务、约束）
    #   knowledge_items  —— 本地知识库检索结果。**缺失是 []，不是"负证据"**：
    #                       空列表只表示"本轮没检索到"，不能反推某方向不可能。
    #   topology_summary —— 只读拓扑摘要（当前地点/相邻地点/前沿数/图版本）。
    #                       **拿不到就是 None**，绝不用 0 或 {} 冒充"测量值为零"。
    #   run_mode         —— live / offline_replay / test。离线评估必须能自证离线。
    task_context: dict[str, Any] | None = None
    knowledge_items: list[dict[str, Any]] = field(default_factory=list)
    topology_summary: dict[str, Any] | None = None
    run_mode: str = "live"
    # The last coherent map/object/topology/candidate snapshot.  It is built
    # by the production goal generator after real path checks, then supplied
    # to the next reasoning cycle; None means no complete snapshot exists yet.
    world_state_snapshot: dict[str, Any] | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "goal_graph": self.goal_graph.to_dict() if self.goal_graph else None,
            "graph_match": self.graph_match.to_dict() if self.graph_match else None,
            "robot_pose": self.robot_pose,
            "robot_yaw_deg": self.robot_yaw_deg,
            "distance_from_origin_m": self.distance_from_origin_m,
            "scan_index": self.scan_index,
            "safety_context": self.safety_context,
            "observed_heading_sectors": self.observed_heading_sectors,
            "observation_memory_count": len(self.observation_memory),
            "observation_memory_ids": _memory_ids(self.observation_memory),
            "auxiliary_hint_count": len(self.auxiliary_hints),
            "auxiliary_status": self.auxiliary_status,
            "task_context": self.task_context,
            "knowledge_item_count": len(self.knowledge_items),
            # 缺失与"空"必须可分辨：None 原样透出，不折叠成 {}。
            "topology_summary": self.topology_summary,
            "run_mode": self.run_mode,
            "world_state_snapshot": self.world_state_snapshot,
        }


def _memory_ids(items: list[dict[str, Any]]) -> list[str]:
    """从观测记忆里取稳定 ID。

    这里原先引用了一个从未定义的 `observation_memory_ids`，`to_dict()` 一调用
    就抛 NameError。取值口径对齐既有写法 step_search_runner.py:300-304 的
    `memory_ids`：只认带 `memory_id` 的字典项，缺 ID 的条目跳过而不是补一个假 ID。
    """
    return [
        str(item.get("memory_id"))
        for item in items
        if isinstance(item, dict) and item.get("memory_id")
    ]
