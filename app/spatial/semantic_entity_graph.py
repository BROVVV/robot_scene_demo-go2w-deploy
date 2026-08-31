"""SemanticEntityGraph: persistent world-model graph for WebUI / planning.

The graph combines:

* PLACE nodes from :class:`PlaceGraph`
* OBJECT nodes from :class:`SemanticObjectMap`
* persistent OBJECT->OBJECT relations (delegated to
  :class:`app.spatial.object_relation_store.ObjectRelationStore`)
* MOVED_TO edges from PlaceGraph movement edges
* OBSERVED_FROM edges from Place -> persistent object observations

Unlike per-frame ``observed_scene_graph()``, this is the stable entity graph:
object ids are persistent entity ids (``obj_001``), not labels.  The WebUI
"semantic topology" view is a projection (``object_topology_snapshot``) whose
node layout is display-only and never feeds navigation.
"""

from __future__ import annotations

import time
from typing import Any

from app.spatial.object_relation_store import (
    OBJECT_RELATIONS,
    OBJECT_TOPOLOGY_SCHEMA_VERSION,
    RELATION_ALIASES,
    RELATION_CONFIRMED,
    RELATION_STALE,
    RELATION_TENTATIVE,
    STRUCTURAL_RELATIONS,
    SYMMETRIC_RELATIONS,
    VIEW_RELATIVE_RELATIONS,
    PersistentObjectRelation,
    ObjectRelationStore,
    normalize_relation,
    relation_is_symmetric,
    relation_scope_of,
)
from app.spatial.models import SpatialPose
from app.spatial.place_graph import PlaceGraph
from app.spatial.semantic_object_map import SemanticObjectMap

GRAPH_SCHEMA_VERSION = "semantic_entity_graph_v1"

__all__ = [
    "GRAPH_SCHEMA_VERSION",
    "OBJECT_TOPOLOGY_SCHEMA_VERSION",
    "OBJECT_RELATIONS",
    "RELATION_ALIASES",
    "RELATION_TENTATIVE",
    "RELATION_CONFIRMED",
    "RELATION_STALE",
    "SYMMETRIC_RELATIONS",
    "VIEW_RELATIVE_RELATIONS",
    "STRUCTURAL_RELATIONS",
    "PersistentObjectRelation",
    "ObjectRelationStore",
    "normalize_relation",
    "relation_is_symmetric",
    "relation_scope_of",
]


class SemanticEntityGraph:
    def __init__(
        self,
        *,
        place_graph: PlaceGraph | None = None,
        object_map: SemanticObjectMap | None = None,
        frame_id: str = "map",
        relation_min_confidence: float = 0.45,
        relation_confirm_min_observations: int = 2,
        relation_stale_after_seconds: float = 180.0,
        max_relation_descriptions: int = 5,
        include_tentative_objects: bool = True,
        include_stale_objects: bool = True,
        include_view_relative_relations: bool = True,
    ) -> None:
        self.place_graph = place_graph or PlaceGraph()
        self.object_map = object_map or SemanticObjectMap()
        self.frame_id = frame_id
        self.revision = 0
        self.route_plan: dict[str, Any] | None = None
        self.association_debug: list[dict[str, Any]] = []

        self.relation_min_confidence = float(relation_min_confidence)
        self.relation_confirm_min_observations = max(1, int(relation_confirm_min_observations))
        self.relation_stale_after_seconds = float(relation_stale_after_seconds)
        self.include_tentative_objects = bool(include_tentative_objects)
        self.include_stale_objects = bool(include_stale_objects)
        self.include_view_relative_relations = bool(include_view_relative_relations)

        # Persistent OBJECT->OBJECT relation store (shared implementation with
        # SemanticNavigationGraph so both WebUI/CLI paths agree).
        self._relation_store = ObjectRelationStore(
            relation_min_confidence=relation_min_confidence,
            relation_confirm_min_observations=relation_confirm_min_observations,
            relation_stale_after_seconds=relation_stale_after_seconds,
            max_relation_descriptions=max_relation_descriptions,
            include_tentative_objects=include_tentative_objects,
            include_stale_objects=include_stale_objects,
            include_view_relative_relations=include_view_relative_relations,
        )
        # Backwards-compatible alias: old code indexed object_relations by
        # (source, relation, target) tuple.
        self.object_relations: dict[tuple[str, str, str], PersistentObjectRelation] = (
            self._relation_store.relations
        )

    @property
    def current_place_id(self) -> str | None:
        return self.place_graph.current_place().place_id if self.place_graph.current_place() else None

    def sync_from_observation(
        self,
        *,
        observation_id: str,
        heading_sector: int | None,
        labels: list[str],
        spatial_objects: list[Any],
        pose: SpatialPose | None = None,
        timestamp: float | None = None,
        place_id: str | None = None,
        update_result: Any | None = None,
        relations: list[dict[str, Any]] | None = None,
    ) -> dict[str, Any]:
        """Refresh the graph after one observation + entity-map update.

        ``relations`` are per-frame scene relations; endpoints resolve strictly
        through ``update_result.associations`` (never by label).
        """
        now = timestamp if timestamp is not None else time.time()
        place = self.place_graph.places.get(place_id or "")
        if place is not None:
            persistent_ids = self._association_persistent_ids(update_result)
            if persistent_ids:
                self.place_graph.attach_objects(place.place_id, persistent_ids)
        if update_result is not None:
            self.association_debug.extend(
                item.to_dict() if hasattr(item, "to_dict") else item
                for item in getattr(update_result, "associations", [])
            )
            self.association_debug.extend(update_result.rejected_pairs)
        if relations:
            self._sync_object_relations(
                relations=relations,
                update_result=update_result,
                observation_id=observation_id,
                timestamp=now,
            )
        self._mark_stale_relations(now=now)
        self.revision += 1
        return self.snapshot()

    def set_route_plan(self, route_plan: dict[str, Any] | None) -> None:
        self.route_plan = route_plan
        self.revision += 1

    # ------------------------------------------------------------------ #
    # association helpers                                                #
    # ------------------------------------------------------------------ #
    @staticmethod
    def _association_persistent_ids(update_result: Any) -> list[str]:
        if update_result is None:
            return []
        return [
            str(assoc.persistent_object_id)
            for assoc in getattr(update_result, "associations", [])
            if getattr(assoc, "persistent_object_id", None)
        ]

    @staticmethod
    def _association_mapping(update_result: Any) -> dict[str, str]:
        return ObjectRelationStore._association_mapping(update_result)

    # ------------------------------------------------------------------ #
    # object relations (delegated)                                       #
    # ------------------------------------------------------------------ #
    def _sync_object_relations(
        self,
        *,
        relations: list[dict[str, Any]],
        update_result: Any,
        observation_id: str,
        timestamp: float,
    ) -> None:
        self._relation_store.sync(
            relations=relations,
            update_result=update_result,
            observation_id=observation_id,
            timestamp=timestamp,
        )
        self.association_debug.extend(self._relation_store.debug)
        self._relation_store.debug.clear()

    def _upsert_object_relation(self, **fields: Any) -> None:
        self._relation_store._upsert(**fields)

    def _mark_stale_relations(self, *, now: float) -> None:
        self._relation_store.mark_stale(now=now)

    def _relation_debug(self, **fields: Any) -> None:
        entry: dict[str, Any] = {"type": "relation_association"}
        entry.update(fields)
        self.association_debug.append(entry)

    # ------------------------------------------------------------------ #
    # projections                                                        #
    # ------------------------------------------------------------------ #
    def object_topology_snapshot(self) -> dict[str, Any]:
        return self._relation_store.object_topology_snapshot(
            self.object_map.objects,
            revision=self.revision,
        )

    def snapshot(self) -> dict[str, Any]:
        nodes: list[dict[str, Any]] = []
        edges: list[dict[str, Any]] = []

        for place in self.place_graph.places.values():
            node = place.to_dict()
            node.update(
                {
                    "node_id": place.place_id,
                    "node_type": "PLACE",
                    "label": place.place_id,
                    "current": place.place_id == self.current_place_id,
                }
            )
            nodes.append(node)

        for obj in self.object_map.objects.values():
            node = obj.to_dict()
            node.update(
                {
                    "node_id": obj.object_id,
                    "node_type": "OBJECT",
                    "label": obj.label,
                }
            )
            nodes.append(node)

        for edge in self.place_graph.edges:
            edges.append(
                {
                    "edge_id": edge.edge_id,
                    "from": edge.from_place,
                    "to": edge.to_place,
                    "relation": "MOVED_TO",
                    "observations": [],
                    "provenance": edge.provenance,
                }
            )

        # OBSERVED_FROM edges derived from association-based Place attaches
        # (``place.observed_object_ids``), never from label matching.
        for place_id, place in self.place_graph.places.items():
            for object_id in place.observed_object_ids:
                entry = self.object_map.objects.get(object_id)
                if entry is None:
                    continue
                edge_id = f"{place_id}__observed_from__{object_id}"
                count = 0
                for sid in getattr(entry, "source_observation_ids", []):
                    ob = self.place_graph.observations.get(sid)
                    if ob is not None and (ob.provenance or {}).get("place_id") == place_id:
                        count += 1
                edges.append(
                    {
                        "edge_id": edge_id,
                        "from": place_id,
                        "to": object_id,
                        "relation": "OBSERVED_FROM",
                        "observation_count": max(1, count),
                        "last_seen": entry.last_seen,
                        "provenance": "visual_observed_association",
                    }
                )

        # Deduplicate edges preserving stable ids.
        unique: dict[str, dict[str, Any]] = {}
        for edge in edges:
            unique[edge["edge_id"]] = edge

        return {
            "schema_version": GRAPH_SCHEMA_VERSION,
            "revision": self.revision,
            "frame_id": self.frame_id,
            "nodes": nodes,
            "edges": list(unique.values()),
            "current_place_id": self.current_place_id,
            "route_plan": self.route_plan,
            "object_topology": self.object_topology_snapshot(),
        }

    def to_dict(self) -> dict[str, Any]:
        return self.snapshot()

    def _entry_touches(
        self,
        entry: Any,
        observation_ids: str | list[str],
        labels: list[str],
        now: float,
    ) -> bool:
        """Legacy label/observation matching, kept only for non-identity uses.

        Persistent identity and Place->object attachment must go through
        ``update_result.associations``, not this helper.
        """
        ids = (
            list(observation_ids)
            if isinstance(observation_ids, list)
            else [observation_ids]
        )
        if any(observation_id in entry.source_observation_ids for observation_id in ids):
            return True
        return entry.label in (labels or [])

    def summary_stats(self) -> dict[str, Any]:
        return {
            "unique_places": len(self.place_graph.places),
            "places_revisited": sum(
                1 for place in self.place_graph.places.values() if place.revisited
            ),
            **self.object_map.summary_stats(),
            "persistent_object_relations": len(self.object_relations),
            "graph_revision": self.revision,
        }
