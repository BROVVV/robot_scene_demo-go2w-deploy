"""Shared persistent OBJECT->OBJECT relation store.

Used by both :class:`app.spatial.semantic_entity_graph.SemanticEntityGraph`
(CLI/offline runner) and :class:`app.spatial.semantic_navigation_graph.
SemanticNavigationGraph` (the graph the AutonomousExplorer feeds the WebUI),
so the WebUI semantic-topology view sees exactly the same persistent relations
on both paths.

Identity contract (plan §6/§9/§40):
  frame_object_id -> SemanticAssociation.source_object_id ->
  persistent_object_id (obj_xxx).  Relations are never keyed by label, so two
  chairs / two bins stay distinct.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any

OBJECT_TOPOLOGY_SCHEMA_VERSION = "semantic_object_topology_v1"

# Single relation vocabulary, identical to app/video/observed_scene_graph_builder.py
OBJECT_RELATIONS = {
    "near",
    "left_of",
    "right_of",
    "in_front_of",
    "behind",
    "on",
    "under",
    "above",
    "below",
    "in",
    "inside",
    "contains",
    "attached_to",
    "blocks",
    "adjacent_to",
}

RELATION_ALIASES = {
    "left": "left_of",
    "right": "right_of",
    "front": "in_front_of",
    "in_front": "in_front_of",
    "front_of": "in_front_of",
    "close": "near",
    "close_to": "near",
    "next_to": "adjacent_to",
    "beside": "adjacent_to",
    "within": "inside",
}

SYMMETRIC_RELATIONS = {
    "near",
    "adjacent_to",
    "attached_to",
}

VIEW_RELATIVE_RELATIONS = {
    "left_of",
    "right_of",
    "in_front_of",
    "behind",
    "above",
    "below",
    "under",
}

STRUCTURAL_RELATIONS = {
    "near",
    "adjacent_to",
    "inside",
    "in",
    "contains",
    "on",
    "attached_to",
    "blocks",
}

RELATION_TENTATIVE = "TENTATIVE"
RELATION_CONFIRMED = "CONFIRMED"
RELATION_STALE = "STALE"


def normalize_relation(value: Any) -> str | None:
    """Normalise a relation string to the canonical vocabulary."""
    if value is None:
        return None
    text = str(value).strip().lower()
    if not text:
        return None
    if text in OBJECT_RELATIONS:
        return text
    return RELATION_ALIASES.get(text)


def relation_scope_of(relation: str) -> str:
    return "STRUCTURAL" if relation in STRUCTURAL_RELATIONS else "VIEW_RELATIVE"


def relation_is_symmetric(relation: str) -> bool:
    return relation in SYMMETRIC_RELATIONS


def clamp01(value: Any, default: float = 0.5) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return float(default)
    return max(0.0, min(1.0, v)) if v == v else float(default)


@dataclass
class PersistentObjectRelation:
    edge_id: str
    source_object_id: str
    target_object_id: str
    relation: str

    confidence: float = 0.5
    observation_count: int = 1
    first_seen: float = 0.0
    last_seen: float = 0.0
    status: str = RELATION_TENTATIVE
    relation_scope: str = "STRUCTURAL"
    directed: bool = False
    source_observation_ids: list[str] = field(default_factory=list)
    descriptions_zh: list[str] = field(default_factory=list)
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "edge_id": self.edge_id,
            "from": self.source_object_id,
            "to": self.target_object_id,
            "relation": self.relation,
            "relation_scope": self.relation_scope,
            "directed": bool(self.directed),
            "confidence": round(self.confidence, 4),
            "observation_count": self.observation_count,
            "status": self.status,
            "first_seen": round(self.first_seen, 4),
            "last_seen": round(self.last_seen, 4),
            "description_zh": (self.descriptions_zh[-1] if self.descriptions_zh else ""),
            "descriptions_zh": list(self.descriptions_zh),
            "source_observation_ids": list(self.source_observation_ids),
            "provenance": dict(self.provenance),
        }


class ObjectRelationStore:
    """Deduplicated, evidence-merged persistent OBJECT->OBJECT relations.

    Keyed by ``(source_object_id, relation, target_object_id)``; symmetric
    relations are canonicalised so ``a near b`` and ``b near a`` stay one edge.
    """

    def __init__(
        self,
        *,
        relation_min_confidence: float = 0.45,
        relation_confirm_min_observations: int = 2,
        relation_stale_after_seconds: float = 180.0,
        max_relation_descriptions: int = 5,
        include_tentative_objects: bool = True,
        include_stale_objects: bool = True,
        include_view_relative_relations: bool = True,
    ) -> None:
        self.relation_min_confidence = clamp01(relation_min_confidence, 0.45)
        self.relation_confirm_min_observations = max(1, int(relation_confirm_min_observations))
        self.relation_stale_after_seconds = max(1.0, float(relation_stale_after_seconds))
        self.max_relation_descriptions = max(1, int(max_relation_descriptions))
        self.include_tentative_objects = bool(include_tentative_objects)
        self.include_stale_objects = bool(include_stale_objects)
        self.include_view_relative_relations = bool(include_view_relative_relations)

        self.relations: dict[tuple[str, str, str], PersistentObjectRelation] = {}
        self.debug: list[dict[str, Any]] = []

    # ------------------------------------------------------------------ #
    # ingest                                                             #
    # ------------------------------------------------------------------ #
    def sync(
        self,
        *,
        relations: list[dict[str, Any]],
        update_result: Any,
        observation_id: str,
        timestamp: float,
    ) -> None:
        source_to_persistent = self._association_mapping(update_result)
        for raw in relations or []:
            if not isinstance(raw, dict):
                continue
            source_frame_id = str(raw.get("subject_id") or "").strip()
            target_frame_id = str(raw.get("object_id") or "").strip()
            relation_text = raw.get("relation") or ""
            source_id = source_to_persistent.get(source_frame_id)
            target_id = source_to_persistent.get(target_frame_id)

            if not source_id or not target_id:
                self._debug(
                    observation_id=observation_id,
                    subject_id=source_frame_id,
                    object_id=target_frame_id,
                    relation=relation_text,
                    result="rejected",
                    reason="source_endpoint_unresolved" if not source_id else "target_endpoint_unresolved",
                )
                continue
            if source_id == target_id:
                self._debug(
                    observation_id=observation_id,
                    subject_id=source_frame_id,
                    object_id=target_frame_id,
                    relation=relation_text,
                    result="rejected",
                    reason="self_relation_rejected",
                )
                continue
            relation = normalize_relation(relation_text)
            if relation is None:
                self._debug(
                    observation_id=observation_id,
                    subject_id=source_frame_id,
                    object_id=target_frame_id,
                    relation=relation_text,
                    result="rejected",
                    reason="relation_not_allowed",
                )
                continue
            confidence = clamp01(raw.get("confidence"), 0.5)
            if confidence < self.relation_min_confidence:
                self._debug(
                    observation_id=observation_id,
                    subject_id=source_frame_id,
                    object_id=target_frame_id,
                    relation=relation,
                    result="rejected",
                    reason="confidence_below_min",
                    confidence=confidence,
                )
                continue

            if source_id > target_id and relation_is_symmetric(relation):
                source_id, target_id = target_id, source_id

            self._upsert(
                source_object_id=source_id,
                target_object_id=target_id,
                relation=relation,
                confidence=confidence,
                observation_id=observation_id,
                timestamp=timestamp,
                description_zh=raw.get("description_zh"),
            )
            self._debug(
                observation_id=observation_id,
                subject_id=source_frame_id,
                object_id=target_frame_id,
                relation=relation,
                result="accepted",
                persistent_source_id=source_id,
                persistent_target_id=target_id,
                confidence=confidence,
            )

    # ------------------------------------------------------------------ #
    # projection                                                         #
    # ------------------------------------------------------------------ #
    def object_topology_snapshot(
        self,
        object_entries: dict[str, Any],
        *,
        revision: int,
    ) -> dict[str, Any]:
        """Build the WebUI object-topology projection.

        ``object_entries`` maps persistent ``obj_xxx`` -> SemanticObjectEntry.
        Only OBJECT nodes and their relations are included (no Place / robot /
        frontier / metric coordinates).
        """
        nodes: list[dict[str, Any]] = []
        for object_id, obj in object_entries.items():
            status = str(getattr(obj, "status", "TENTATIVE") or "TENTATIVE").upper()
            if status == "STALE" and not self.include_stale_objects:
                continue
            if status == "TENTATIVE" and not self.include_tentative_objects:
                continue
            provenance = dict(getattr(obj, "provenance", None) or {})
            nodes.append(
                {
                    "node_id": object_id,
                    "node_type": "OBJECT",
                    "label": str(getattr(obj, "label", "object") or "object"),
                    "status": status,
                    "confidence": round(float(getattr(obj, "confidence", 0.0) or 0.0), 4),
                    "observation_count": int(getattr(obj, "observation_count", 0) or 0),
                    "spatial_quality": str(getattr(obj, "spatial_quality", "RGB_ONLY") or "RGB_ONLY"),
                    "is_target_candidate": bool(
                        provenance.get("target_candidate") or provenance.get("target_confirmed")
                    ),
                    "is_target_confirmed": bool(provenance.get("target_confirmed")),
                }
            )

        edges: list[dict[str, Any]] = []
        for rel in self.relations.values():
            if rel.status == "STALE" and not self.include_stale_objects:
                continue
            if rel.relation_scope == "VIEW_RELATIVE" and not self.include_view_relative_relations:
                continue
            edges.append(rel.to_dict())

        node_ids = {node["node_id"] for node in nodes}
        edges = [edge for edge in edges if edge["from"] in node_ids and edge["to"] in node_ids]
        confirmed_nodes = sum(1 for node in nodes if node["status"] == "CONFIRMED")
        confirmed_edges = sum(1 for edge in edges if edge["status"] == "CONFIRMED")
        return {
            "schema_version": OBJECT_TOPOLOGY_SCHEMA_VERSION,
            "revision": int(revision),
            # Deterministic timestamp for the projection: replay/compare of two
            # runs must yield identical dicts (real time.time() broke
            # test_exploration_replay determinism).
            "generated_at": float(revision),
            "nodes": nodes,
            "edges": edges,
            "stats": {
                "node_count": len(nodes),
                "edge_count": len(edges),
                "confirmed_nodes": confirmed_nodes,
                "confirmed_edges": confirmed_edges,
                "connected_components": _connected_components(node_ids, edges),
            },
        }

    # ------------------------------------------------------------------ #
    # internals                                                          #
    # ------------------------------------------------------------------ #
    @staticmethod
    def _association_mapping(update_result: Any) -> dict[str, str]:
        mapping: dict[str, str] = {}
        if update_result is None:
            return mapping
        for assoc in getattr(update_result, "associations", []):
            source = str(getattr(assoc, "source_object_id", "") or "").strip()
            persistent = str(getattr(assoc, "persistent_object_id", "") or "").strip()
            if source and persistent:
                mapping[source] = persistent
        return mapping

    def _upsert(
        self,
        *,
        source_object_id: str,
        target_object_id: str,
        relation: str,
        confidence: float,
        observation_id: str,
        timestamp: float,
        description_zh: Any,
    ) -> None:
        key = (source_object_id, relation, target_object_id)
        edge_id = f"{source_object_id}__{relation}__{target_object_id}"
        scope = relation_scope_of(relation)
        desc = str(description_zh or "").strip()
        existing = self.relations.get(key)
        if existing is None:
            self.relations[key] = PersistentObjectRelation(
                edge_id=edge_id,
                source_object_id=source_object_id,
                target_object_id=target_object_id,
                relation=relation,
                confidence=confidence,
                observation_count=1,
                first_seen=timestamp,
                last_seen=timestamp,
                status=(
                    RELATION_CONFIRMED
                    if self.relation_confirm_min_observations <= 1
                    else RELATION_TENTATIVE
                ),
                relation_scope=scope,
                directed=not relation_is_symmetric(relation),
                source_observation_ids=[observation_id] if observation_id else [],
                descriptions_zh=[desc] if desc else [],
                provenance={"source": "framed_object_relation", "last_observation_id": observation_id},
            )
            return
        prior_count = existing.observation_count
        existing.confidence = (
            existing.confidence * prior_count + confidence
        ) / (prior_count + 1)
        existing.observation_count += 1
        existing.last_seen = timestamp
        existing.provenance["last_observation_id"] = observation_id
        if observation_id and observation_id not in existing.source_observation_ids:
            existing.source_observation_ids.append(observation_id)
        if desc and desc not in existing.descriptions_zh:
            existing.descriptions_zh.append(desc)
            existing.descriptions_zh = existing.descriptions_zh[-self.max_relation_descriptions:]
        existing.status = (
            RELATION_CONFIRMED
            if existing.observation_count >= self.relation_confirm_min_observations
            else RELATION_TENTATIVE
        )

    def mark_stale(self, *, now: float) -> None:
        for rel in self.relations.values():
            if rel.last_seen and (now - rel.last_seen) > self.relation_stale_after_seconds:
                rel.status = RELATION_STALE

    def _debug(self, **fields: Any) -> None:
        entry: dict[str, Any] = {"type": "relation_association"}
        entry.update(fields)
        self.debug.append(entry)


def _connected_components(node_ids: set[str], edges: list[dict[str, Any]]) -> int:
    parent = {node_id: node_id for node_id in node_ids}

    def find(x: str) -> str:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a: str, b: str) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    for edge in edges:
        a, b = edge.get("from"), edge.get("to")
        if a in parent and b in parent:
            union(a, b)
    return len({find(node_id) for node_id in node_ids})
