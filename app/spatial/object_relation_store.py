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

import math
import time
from dataclasses import dataclass, field
from typing import Any

OBJECT_TOPOLOGY_SCHEMA_VERSION = "semantic_object_topology_v1"

# Single relation vocabulary, identical to app/video/observed_scene_graph_builder.py
OBJECT_RELATIONS = {
    "near",
    "spatial_neighbor",
    "co_visible",
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
    "spatial_neighbor",
    "co_visible",
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
    "spatial_neighbor",
    "co_visible",
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

    def observe_teleop_pairs(self, object_entries: dict[str, Any],
                             visible_ids: set[str], *, frame_id: str,
                             timestamp: float, near_distance_m: float = 3.0,
                             near_exit_distance_m: float = 3.4,
                             spatial_neighbor_distance_m: float = 4.5,
                             max_near_edges: int = 4,
                             bridge_extra_edges_per_node: int = 2,
                             max_co_visible_edges: int = 4,
                             pair_evidence: Any = None) -> None:
        """重建地图级空间骨架，并单独累计真实同帧共见证据。"""
        self.rebuild_spatial_adjacency(
            object_entries,
            timestamp=timestamp,
            near_distance_m=near_distance_m,
            near_exit_distance_m=near_exit_distance_m,
            spatial_neighbor_distance_m=spatial_neighbor_distance_m,
            max_near_edges=max_near_edges,
            bridge_extra_edges_per_node=bridge_extra_edges_per_node,
            pair_evidence=pair_evidence,
        )
        valid = {
            key: obj for key, obj in object_entries.items()
            if _spatial_object_is_active(obj)
        }
        positions = {
            key: getattr(obj, "map_xyz", None) for key, obj in valid.items()
            if _finite_xyz(getattr(obj, "map_xyz", None)) is not None
        }
        seen = sorted(set(visible_ids) & valid.keys())
        counts: dict[str, int] = {}
        visible_pairs = [(math.dist(positions[a], positions[b]) if a in positions and b in positions else float("inf"), a, b)
                         for index, a in enumerate(seen) for b in seen[index + 1:]]
        for _, a, b in sorted(visible_pairs):
            if counts.get(a, 0) >= max_co_visible_edges or counts.get(b, 0) >= max_co_visible_edges:
                continue
            counts[a] = counts.get(a, 0) + 1
            counts[b] = counts.get(b, 0) + 1
            self._upsert(source_object_id=a, target_object_id=b, relation="co_visible",
                         confidence=.75, observation_id=frame_id, timestamp=timestamp,
                         description_zh="同一采集帧共同出现")

    def rebuild_spatial_adjacency(
        self,
        object_entries: dict[str, Any],
        *,
        timestamp: float,
        near_distance_m: float = 3.0,
        near_exit_distance_m: float = 3.4,
        spatial_neighbor_distance_m: float = 4.5,
        max_near_edges: int = 4,
        bridge_extra_edges_per_node: int = 2,
        max_vertical_separation_m: float = 2.0,
        pair_evidence: Any = None,
    ) -> dict[str, Any]:
        """Rebuild the current map-space skeleton from all persistent objects.

        The callback ``pair_evidence(a, b, distance)`` may return ``bool`` or
        ``(bool, reason)``.  It is the seam used by a 2-D occupancy map to
        reject a known wall or unknown region; absent a callback, same-map
        finite coordinates are treated as local-space evidence.
        """
        near_distance_m = max(0.01, float(near_distance_m))
        near_exit_distance_m = max(near_distance_m, float(near_exit_distance_m))
        spatial_neighbor_distance_m = max(
            near_exit_distance_m, float(spatial_neighbor_distance_m)
        )
        max_near_edges = max(1, int(max_near_edges))
        bridge_extra_edges_per_node = max(0, int(bridge_extra_edges_per_node))

        valid = {
            object_id: obj for object_id, obj in object_entries.items()
            if _spatial_object_is_active(obj)
            and _finite_xyz(getattr(obj, "map_xyz", None)) is not None
        }
        candidates: list[dict[str, Any]] = []
        rejected: list[dict[str, Any]] = []
        ids = sorted(valid)
        for index, source_id in enumerate(ids):
            source = valid[source_id]
            source_xyz = _finite_xyz(getattr(source, "map_xyz", None))
            for target_id in ids[index + 1:]:
                target = valid[target_id]
                target_xyz = _finite_xyz(getattr(target, "map_xyz", None))
                if not _same_map_identity(source, target):
                    rejected.append({"from": source_id, "to": target_id, "reason": "map_identity_mismatch"})
                    continue
                if abs(source_xyz[2] - target_xyz[2]) > max_vertical_separation_m:
                    rejected.append({"from": source_id, "to": target_id, "reason": "vertical_layer_mismatch"})
                    continue
                distance = math.hypot(source_xyz[0] - target_xyz[0], source_xyz[1] - target_xyz[1])
                existing_near = (source_id, "near", target_id) in self.relations
                near_limit = near_exit_distance_m if existing_near else near_distance_m
                if distance <= near_limit:
                    relation = "near"
                elif distance <= spatial_neighbor_distance_m:
                    relation = "spatial_neighbor"
                else:
                    rejected.append({"from": source_id, "to": target_id, "reason": "distance_exceeded"})
                    continue
                accepted, reason = _pair_evidence_result(
                    pair_evidence, source, target, distance
                )
                if not accepted:
                    rejected.append({"from": source_id, "to": target_id, "reason": reason})
                    continue
                candidates.append({
                    "from": source_id,
                    "to": target_id,
                    "relation": relation,
                    "distance_m": distance,
                    "evidence": reason,
                    "existing": (source_id, relation, target_id) in self.relations,
                    "lifecycle_penalty": min(
                        _lifecycle_edge_weight(source),
                        _lifecycle_edge_weight(target),
                    ),
                })

        # First keep a stable, bounded nearest-neighbour set.  Existing valid
        # edges win ties so small coordinate changes do not make the web graph
        # flicker on every poll.
        selected: dict[tuple[str, str], dict[str, Any]] = {}
        degree: dict[str, int] = {}
        for item in sorted(
            candidates,
            key=lambda value: (
                not value["existing"], value["distance_m"],
                value["from"], value["to"],
            ),
        ):
            a, b = item["from"], item["to"]
            if degree.get(a, 0) >= max_near_edges or degree.get(b, 0) >= max_near_edges:
                continue
            selected[(a, b)] = item
            degree[a] = degree.get(a, 0) + 1
            degree[b] = degree.get(b, 0) + 1

        # Build components from all durable evidence, then spend a separate
        # bridge budget only on candidate edges that join two components.
        parent = {object_id: object_id for object_id in valid}

        def find(value: str) -> str:
            while parent[value] != value:
                parent[value] = parent[parent[value]]
                value = parent[value]
            return value

        def union(a: str, b: str) -> None:
            left, right = find(a), find(b)
            if left != right:
                parent[right] = left

        for rel in self.relations.values():
            # Previous map geometry is about to be replaced.  Using it here
            # could make two components look connected and suppress the very
            # bridge that the current rebuild must retain.
            if rel.relation != "co_visible":
                continue
            if rel.source_object_id in parent and rel.target_object_id in parent:
                union(rel.source_object_id, rel.target_object_id)
        for a, b in selected:
            union(a, b)

        bridge_degree: dict[str, int] = {}
        for item in sorted(candidates, key=lambda value: (
            value["distance_m"], value["from"], value["to"]
        )):
            a, b = item["from"], item["to"]
            if (a, b) in selected or find(a) == find(b):
                continue
            if (bridge_degree.get(a, 0) >= bridge_extra_edges_per_node
                    or bridge_degree.get(b, 0) >= bridge_extra_edges_per_node):
                continue
            item = {**item, "bridge": True}
            selected[(a, b)] = item
            bridge_degree[a] = bridge_degree.get(a, 0) + 1
            bridge_degree[b] = bridge_degree.get(b, 0) + 1
            union(a, b)

        old_spatial = {
            key: value for key, value in self.relations.items()
            if value.relation in {"near", "spatial_neighbor"}
        }
        for key in list(old_spatial):
            self.relations.pop(key, None)
        for item in selected.values():
            self._set_spatial_relation(item, old_spatial, timestamp=timestamp)

        components = len({find(object_id) for object_id in valid}) if valid else 0
        result = {
            "candidate_count": len(candidates),
            "selected_count": len(selected),
            "bridge_count": sum(bool(item.get("bridge")) for item in selected.values()),
            "connected_components": components,
            "rejected": rejected,
        }
        self.debug.append({"type": "spatial_adjacency_rebuild", **result})
        return result

    def _set_spatial_relation(
        self,
        item: dict[str, Any],
        previous: dict[tuple[str, str, str], PersistentObjectRelation],
        *,
        timestamp: float,
    ) -> None:
        source_id, target_id, relation = item["from"], item["to"], item["relation"]
        key = (source_id, relation, target_id)
        old = previous.get(key)
        distance = float(item["distance_m"])
        confidence = max(0.25, (
            1.0 - distance / max(9.0, distance + 0.01)
        ) * float(item.get("lifecycle_penalty", 1.0)))
        evidence = str(item.get("evidence") or "same_map_coordinates")
        description = f"二维地图距离 {distance:.2f} m"
        if item.get("bridge"):
            description += "；跨组关键连接"
        provenance = dict(old.provenance) if old else {}
        provenance.update({
            "source": "map_spatial_rebuild",
            "distance_m": round(distance, 4),
            "evidence": evidence,
            "bridge": bool(item.get("bridge")),
            "calculation_count": int(provenance.get("calculation_count", 0)) + 1,
        })
        descriptions = list(old.descriptions_zh) if old else []
        if description not in descriptions:
            descriptions.append(description)
        self.relations[key] = PersistentObjectRelation(
            edge_id=f"{source_id}__{relation}__{target_id}",
            source_object_id=source_id,
            target_object_id=target_id,
            relation=relation,
            confidence=confidence,
            # A geometry rebuild is a calculation, not a fresh detection.
            observation_count=old.observation_count if old else 0,
            first_seen=old.first_seen if old else timestamp,
            last_seen=timestamp,
            status=RELATION_CONFIRMED,
            relation_scope="STRUCTURAL",
            directed=False,
            source_observation_ids=list(old.source_observation_ids) if old else [],
            descriptions_zh=descriptions[-self.max_relation_descriptions:],
            provenance=provenance,
        )

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
        near_distance_m: float | None = None,
    ) -> dict[str, Any]:
        """Build the WebUI object-topology projection.

        ``object_entries`` maps persistent ``obj_xxx`` -> SemanticObjectEntry.
        Only OBJECT nodes and their relations are included (no Place / robot /
        frontier / metric coordinates).
        """
        nodes: list[dict[str, Any]] = []
        for object_id, obj in object_entries.items():
            if getattr(obj, "topology_eligible", None) is False or getattr(obj, "mobility_class", None) == "dynamic":
                continue
            lifecycle_state = str(getattr(obj, "lifecycle_state", "active") or "active")
            if lifecycle_state == "removed":
                continue
            status = str(getattr(obj, "status", "TENTATIVE") or "TENTATIVE").upper()
            if status == "STALE" and not self.include_stale_objects:
                continue
            if status == "TENTATIVE" and not self.include_tentative_objects:
                continue
            provenance = dict(getattr(obj, "provenance", None) or {})
            nodes.append(
                {
                    "node_id": object_id,
                    "mobility_class": getattr(obj, "mobility_class", None),
                    "mobility_state": getattr(obj, "mobility_state", None),
                    "mobility_reason": getattr(obj, "mobility_reason", "待复核"),
                    "mobility_confidence": getattr(obj, "mobility_confidence", 0.0),
                    "is_hazard": bool(getattr(obj, "is_hazard", False)),
                    "hazard_rule_id": getattr(obj, "hazard_rule_id", None),
                    "hazard_label": getattr(obj, "hazard_label", None),
                    "hazard_event_id": getattr(obj, "hazard_event_id", None),
                    "hazard_location_mode": getattr(obj, "hazard_location_mode", None),
                    "hazard_position_status": getattr(obj, "hazard_position_status", None),
                    "last_seen": getattr(obj, "last_seen", None),
                    "last_checked_at": getattr(obj, "last_checked_at", None),
                    "lifecycle_state": lifecycle_state,
                    "missing_observation_count": int(
                        getattr(obj, "missing_observation_count", 0) or 0
                    ),
                    "position_epoch": int(getattr(obj, "position_epoch", 0) or 0),
                    "position_changed_at": getattr(obj, "position_changed_at", None),
                    "node_type": "OBJECT",
                    "label": str(getattr(obj, "label_zh", None) or getattr(obj, "label", "object") or "object"),
                    "raw_label": str(getattr(obj, "label", "object") or "object"),
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
        # Spatial density selection happens exactly once in
        # rebuild_spatial_adjacency().  Snapshot projection must not silently
        # delete a bridge edge with a second hard-coded degree cap.
        confirmed_nodes = sum(1 for node in nodes if node["status"] == "CONFIRMED")
        confirmed_edges = sum(1 for edge in edges if edge["status"] == "CONFIRMED")
        lifecycle_counts = {
            state: sum(1 for node in nodes if node.get("lifecycle_state") == state)
            for state in ("active", "stale", "missing_candidate")
        }
        located_nodes = sum(
            1 for node in nodes
            if _finite_xyz(getattr(object_entries.get(node["node_id"]), "map_xyz", None)) is not None
        )
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
                "active_nodes": lifecycle_counts["active"],
                "stale_nodes": lifecycle_counts["stale"],
                "missing_candidate_nodes": lifecycle_counts["missing_candidate"],
                "located_nodes": located_nodes,
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
            if (rel.relation in {"near", "spatial_neighbor"}
                    and rel.provenance.get("source") == "map_spatial_rebuild"):
                continue
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


def _finite_xyz(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return None
    try:
        result = tuple(float(value[index]) for index in range(3))
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(part) for part in result) else None


def _spatial_object_is_active(obj: Any) -> bool:
    return (
        getattr(obj, "topology_eligible", None) is True
        and getattr(obj, "mobility_class", None) != "dynamic"
        and str(getattr(obj, "lifecycle_state", "active") or "active") != "removed"
    )


def _lifecycle_edge_weight(obj: Any) -> float:
    return {
        "missing_candidate": 0.65,
        "stale": 0.80,
    }.get(str(getattr(obj, "lifecycle_state", "active") or "active"), 1.0)


def _map_identity(obj: Any) -> tuple[str, str, str]:
    provenance = dict(getattr(obj, "provenance", None) or {})
    return (
        str(provenance.get("map_id") or ""),
        str(provenance.get("map_epoch") or ""),
        str(provenance.get("map_frame") or provenance.get("target_frame") or ""),
    )


def _same_map_identity(left: Any, right: Any) -> bool:
    a, b = _map_identity(left), _map_identity(right)
    return all(not x or not y or x == y for x, y in zip(a, b))


def _pair_evidence_result(callback: Any, source: Any, target: Any,
                          distance: float) -> tuple[bool, str]:
    if not callable(callback):
        return True, "same_map_coordinates"
    result = callback(source, target, distance)
    if isinstance(result, tuple):
        accepted, reason = result[0], result[1] if len(result) > 1 else ""
        return bool(accepted), str(reason or ("accepted" if accepted else "spatial_evidence_rejected"))
    return bool(result), "pair_evidence" if result else "spatial_evidence_rejected"
