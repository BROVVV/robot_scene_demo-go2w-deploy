"""Session-scoped spatial-semantic exploration graph.

The long-term ``ObservationMemoryStore`` keeps historical observations; this
module keeps the *current autonomous exploration session's* structured
topological memory: observation nodes, motion edges, visited / negative /
unreachable state and semantic interest.  A future metric backend can bind the
same nodes to true map poses; the current Go2-W uses relative + heading-sector
topology (see docs/HIGH_LEVEL_AUTONOMOUS_SEMANTIC_EXPLORATION.md).
"""

from __future__ import annotations

import json
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any

from .robot_backend import NavigationResult, RobotPose


class NodeState(str, Enum):
    UNSEEN = "UNSEEN"
    OBSERVED = "OBSERVED"
    VISITED = "VISITED"
    SEMANTIC_INTEREST = "SEMANTIC_INTEREST"
    NEGATIVE = "NEGATIVE"
    UNREACHABLE = "UNREACHABLE"
    TARGET_CANDIDATE = "TARGET_CANDIDATE"
    TARGET_CONFIRMED = "TARGET_CONFIRMED"


@dataclass
class ObservationNode:
    node_id: str
    timestamp: float

    pose: RobotPose | None = None
    pose_quality: str = "unavailable"
    heading: float | None = None
    heading_sector: int | None = None

    objects: list[str] = field(default_factory=list)
    relations: list[str] = field(default_factory=list)
    scene_graph: dict[str, Any] | None = None

    target_match_level: str = "none"
    target_score: float = 0.0

    semantic_relevance: float = 0.0
    information_gain: float = 0.0

    visited_count: int = 0
    negative_evidence_count: int = 0

    navigation_fail_count: int = 0
    reachable_state: str = NodeState.UNSEEN.value

    source_bundle_id: str | None = None
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        if self.pose is not None:
            payload["pose"] = self.pose.to_dict()
        return payload

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "ObservationNode":
        payload = dict(value)
        pose = payload.pop("pose", None)
        node = cls(
            node_id=str(payload.get("node_id") or "node"),
            timestamp=float(payload.get("timestamp", 0.0)),
            pose_quality=str(payload.get("pose_quality") or "unavailable"),
            heading=payload.get("heading"),
            heading_sector=payload.get("heading_sector"),
            objects=list(payload.get("objects") or []),
            relations=list(payload.get("relations") or []),
            scene_graph=payload.get("scene_graph"),
            target_match_level=str(payload.get("target_match_level") or "none"),
            target_score=float(payload.get("target_score", 0.0)),
            semantic_relevance=float(payload.get("semantic_relevance", 0.0)),
            information_gain=float(payload.get("information_gain", 0.0)),
            visited_count=int(payload.get("visited_count", 0)),
            negative_evidence_count=int(payload.get("negative_evidence_count", 0)),
            navigation_fail_count=int(payload.get("navigation_fail_count", 0)),
            reachable_state=str(payload.get("reachable_state") or NodeState.UNSEEN.value),
            source_bundle_id=payload.get("source_bundle_id"),
            provenance=dict(payload.get("provenance") or {}),
        )
        if isinstance(pose, dict):
            node.pose = RobotPose(
                x=float(pose.get("x", 0.0)),
                y=float(pose.get("y", 0.0)),
                yaw=float(pose.get("yaw", 0.0)),
                frame_id=str(pose.get("frame_id") or "odom"),
                source=str(pose.get("source") or "unknown"),
            )
        return node


@dataclass
class ExplorationEdge:
    source_node_id: str
    target_node_id: str
    action_type: str
    requested_motion: dict[str, Any] = field(default_factory=dict)
    observed_motion: dict[str, Any] = field(default_factory=dict)
    navigation_result: str = "succeeded"
    cost: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "ExplorationEdge":
        return cls(
            source_node_id=str(value.get("source_node_id") or ""),
            target_node_id=str(value.get("target_node_id") or ""),
            action_type=str(value.get("action_type") or ""),
            requested_motion=dict(value.get("requested_motion") or {}),
            observed_motion=dict(value.get("observed_motion") or {}),
            navigation_result=str(value.get("navigation_result") or "succeeded"),
            cost=float(value.get("cost", 0.0)),
        )


class ExplorationGraph:
    """Topological exploration memory for one live session."""

    def __init__(self, *, session_id: str = "exploration_session",
                 now: Any = time.time) -> None:
        self.session_id = session_id
        self._now = now
        self.nodes: dict[str, ObservationNode] = {}
        self.edges: list[ExplorationEdge] = []
        # Heading sectors that have at least one observation (coverage), kept
        # independent of node merging so revisits never un-observe a sector.
        self._observed_sectors: set[int] = set()
        # Failed goal counters keyed by (goal_type, heading_sector|None).
        self._failed_goals: dict[tuple[str, Any], int] = {}
        # Most recent goal decisions, used by the oscillation / tabu logic.
        self.recent_goals: deque[dict[str, Any]] = deque(maxlen=16)

    # ---- observation ------------------------------------------------------

    def add_observation(self, node: ObservationNode) -> ObservationNode:
        if node.heading_sector is not None:
            self._observed_sectors.add(node.heading_sector)
        if node.node_id in self.nodes:
            existing = self.nodes[node.node_id]
            existing.objects = _merge_unique(existing.objects, node.objects)
            existing.relations = _merge_unique(existing.relations, node.relations)
            existing.scene_graph = node.scene_graph or existing.scene_graph
            existing.information_gain = max(
                existing.information_gain, node.information_gain
            )
            existing.semantic_relevance = max(
                existing.semantic_relevance, node.semantic_relevance
            )
            if node.pose is not None and existing.pose is None:
                existing.pose = node.pose
            if existing.reachable_state == NodeState.UNSEEN.value:
                existing.reachable_state = NodeState.OBSERVED.value
            return existing
        self.nodes[node.node_id] = node
        return node

    def get_node(self, node_id: str) -> ObservationNode | None:
        return self.nodes.get(node_id)

    def node_or_create(self, node_id: str, *, timestamp: float | None = None,
                       **kwargs: Any) -> ObservationNode:
        node = self.nodes.get(node_id)
        if node is None:
            node = ObservationNode(
                node_id=node_id,
                timestamp=timestamp if timestamp is not None else float(self._now()),
                **kwargs,
            )
            self.nodes[node_id] = node
        return node

    # ---- motion / edges ---------------------------------------------------

    def connect_motion(self, edge: ExplorationEdge) -> None:
        self.edges.append(edge)

    def record_navigation(self, result: NavigationResult, *, goal_type: str,
                          requested_motion: dict[str, Any] | None = None,
                          observed_motion: dict[str, Any] | None = None,
                          target_node_id: str | None = None,
                          source_node_id: str | None = None,
                          heading_sector: int | None = None) -> None:
        """Record one executed goal and update node/sector failure counts."""
        if target_node_id is not None and target_node_id in self.nodes:
            node = self.nodes[target_node_id]
            if result.failed:
                node.navigation_fail_count += 1
                if node.navigation_fail_count >= 2:
                    node.reachable_state = NodeState.UNREACHABLE.value
            else:
                node.navigation_fail_count = 0
        if result.failed:
            key = (goal_type, heading_sector)
            self._failed_goals[key] = self._failed_goals.get(key, 0) + 1
        if source_node_id is not None and target_node_id is not None:
            self.connect_motion(
                ExplorationEdge(
                    source_node_id=source_node_id,
                    target_node_id=target_node_id,
                    action_type=goal_type,
                    requested_motion=requested_motion or {},
                    observed_motion=observed_motion or {},
                    navigation_result=result.status.value,
                    cost=float(result.elapsed_sec or 0.0),
                )
            )
        self.recent_goals.append(
            {
                "goal_id": result.goal_id,
                "goal_type": goal_type,
                "status": result.status.value,
                "target_node_id": target_node_id,
                "heading_sector": heading_sector,
                "host_s": round(float(self._now()), 6),
            }
        )

    def goal_failure_count(self, goal_type: str, heading_sector: int | None) -> int:
        return self._failed_goals.get((goal_type, heading_sector), 0)

    def sector_failure_count(self, heading_sector: int | None) -> int:
        if heading_sector is None:
            return 0
        return sum(
            count for (goal_type, sector), count in self._failed_goals.items()
            if sector == heading_sector
        )

    # ---- state transitions ------------------------------------------------

    def mark_visited(self, node_id: str) -> ObservationNode | None:
        node = self.nodes.get(node_id)
        if node is None:
            return None
        node.visited_count += 1
        if node.reachable_state != NodeState.UNREACHABLE.value:
            node.reachable_state = NodeState.VISITED.value
        return node

    def mark_negative(self, node_id: str, *, reason: str = "") -> ObservationNode | None:
        node = self.nodes.get(node_id)
        if node is None:
            return None
        node.negative_evidence_count += 1
        if node.reachable_state not in {
            NodeState.UNREACHABLE.value, NodeState.TARGET_CANDIDATE.value,
            NodeState.TARGET_CONFIRMED.value,
        }:
            node.reachable_state = NodeState.NEGATIVE.value
        node.provenance.setdefault("negative_reasons", []).append(
            {"reason": reason, "host_s": round(float(self._now()), 6)}
        )
        return node

    def mark_unreachable(self, node_id: str, *, reason: str = "") -> ObservationNode | None:
        node = self.nodes.get(node_id)
        if node is None:
            return None
        node.reachable_state = NodeState.UNREACHABLE.value
        node.provenance.setdefault("unreachable_reasons", []).append(
            {"reason": reason, "host_s": round(float(self._now()), 6)}
        )
        return node

    def mark_semantic_interest(self, node_id: str, *, anchor: str = "",
                               reason: str = "") -> ObservationNode | None:
        node = self.nodes.get(node_id)
        if node is None:
            return None
        if node.reachable_state not in {
            NodeState.TARGET_CANDIDATE.value, NodeState.TARGET_CONFIRMED.value,
            NodeState.UNREACHABLE.value,
        }:
            node.reachable_state = NodeState.SEMANTIC_INTEREST.value
        node.semantic_relevance = max(node.semantic_relevance, 0.5)
        node.provenance.setdefault("semantic_interest", []).append(
            {"anchor": anchor, "reason": reason, "host_s": round(float(self._now()), 6)}
        )
        return node

    def mark_target_candidate(self, node_id: str) -> ObservationNode | None:
        node = self.nodes.get(node_id)
        if node is None:
            return None
        node.reachable_state = NodeState.TARGET_CANDIDATE.value
        return node

    def mark_target_confirmed(self, node_id: str) -> ObservationNode | None:
        node = self.nodes.get(node_id)
        if node is None:
            return None
        node.reachable_state = NodeState.TARGET_CONFIRMED.value
        node.target_match_level = "confirmed"
        return node

    # ---- queries ----------------------------------------------------------

    def unvisited_nodes(self) -> list[ObservationNode]:
        return [
            node for node in self.nodes.values()
            if node.visited_count == 0
            and node.reachable_state != NodeState.UNREACHABLE.value
        ]

    def semantic_interest_nodes(self) -> list[ObservationNode]:
        return [
            node for node in self.nodes.values()
            if node.reachable_state == NodeState.SEMANTIC_INTEREST.value
        ]

    def target_candidate_nodes(self) -> list[ObservationNode]:
        return [
            node for node in self.nodes.values()
            if node.reachable_state == NodeState.TARGET_CANDIDATE.value
        ]

    def negative_nodes(self) -> list[ObservationNode]:
        return [
            node for node in self.nodes.values()
            if node.negative_evidence_count > 0
        ]

    def nearest_nodes(self, pose: RobotPose | None, k: int = 5) -> list[ObservationNode]:
        if pose is None or not self.nodes:
            return list(self.nodes.values())[:k]
        return sorted(
            self.nodes.values(),
            key=lambda node: _pose_distance(node.pose, pose),
        )[:k]

    def semantic_neighbors(self, anchor_label: str) -> list[ObservationNode]:
        anchor = anchor_label.strip().lower()
        if not anchor:
            return []
        matches: list[ObservationNode] = []
        for node in self.nodes.values():
            for label in node.objects:
                if anchor in str(label).strip().lower():
                    matches.append(node)
                    break
        return matches

    def nodes_in_sector(self, sector: int) -> list[ObservationNode]:
        return [
            node for node in self.nodes.values()
            if node.heading_sector == sector
        ]

    def sector_visited_count(self, sector: int) -> int:
        """Number of observation nodes in a heading sector (sector coverage)."""
        return 1 if sector in self._observed_sectors else 0

    def recent_goal_sequence(self) -> list[dict[str, Any]]:
        return list(self.recent_goals)

    def oscillation_penalty(self, candidate_sector: int | None,
                            candidate_node_id: str | None) -> float:
        """Penalty when the next goal would repeat a recent 2-cycle (A-B-A-B)."""
        recent = list(self.recent_goals)
        if len(recent) < 3:
            return 0.0
        penalty = 0.0
        if candidate_sector is not None:
            sector_seq = [item.get("heading_sector") for item in recent[-3:]]
            if (
                sector_seq[0] == sector_seq[2] is not None
                and sector_seq[1] != sector_seq[0]
                and candidate_sector == sector_seq[0]
            ):
                penalty = max(penalty, 0.8)
        if candidate_node_id is not None:
            node_seq = [item.get("target_node_id") for item in recent[-3:]]
            if (
                node_seq[0] == node_seq[2] is not None
                and node_seq[1] != node_seq[0]
                and candidate_node_id == node_seq[0]
            ):
                penalty = max(penalty, 0.8)
        return penalty

    # ---- serialization ----------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        return {
            "session_id": self.session_id,
            "node_count": len(self.nodes),
            "edge_count": len(self.edges),
            "observed_sectors": sorted(self._observed_sectors),
            "nodes": [node.to_dict() for node in self.nodes.values()],
            "edges": [edge.to_dict() for edge in self.edges],
            "recent_goals": list(self.recent_goals),
        }

    def save(self, path: str | Path) -> Path:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(
            json.dumps(self.to_dict(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        return target

    @classmethod
    def load(cls, path: str | Path, *, session_id: str | None = None) -> "ExplorationGraph":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        graph = cls(session_id=session_id or str(payload.get("session_id") or "exploration_session"))
        graph._observed_sectors = {
            int(item) for item in (payload.get("observed_sectors") or [])
        }
        for item in payload.get("nodes") or []:
            node = ObservationNode.from_dict(item)
            graph.nodes[node.node_id] = node
        graph.edges = [ExplorationEdge.from_dict(item) for item in payload.get("edges") or []]
        for item in payload.get("recent_goals") or []:
            graph.recent_goals.append(item)
        return graph


def _merge_unique(first: list[str], second: list[str]) -> list[str]:
    result = list(first)
    for item in second:
        if item not in result:
            result.append(item)
    return result


def _pose_distance(first: RobotPose | None, second: RobotPose) -> float:
    if first is None:
        return float("inf")
    return ((first.x - second.x) ** 2 + (first.y - second.y) ** 2) ** 0.5
