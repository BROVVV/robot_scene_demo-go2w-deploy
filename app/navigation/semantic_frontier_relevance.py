"""Evidence-backed semantic relevance for metric frontier candidates.

This module deliberately does not decide whether the robot may move.  It turns
measured object/place geometry and task terms into a bounded preference that is
consumed *after* reachability and safety checks.  Missing evidence remains
explicitly unknown instead of being manufactured as a positive score.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Iterable


@dataclass(frozen=True)
class FrontierSemanticEvidence:
    frontier_id: str
    score: float | None
    target_proximity: float | None = None
    anchor_proximity: float | None = None
    place_affinity: float | None = None
    uncertainty_factor: float | None = None
    route_factor: float | None = None
    object_ids: tuple[str, ...] = ()
    place_ids: tuple[str, ...] = ()
    evidence_refs: tuple[str, ...] = ()
    reasons: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {
            "frontier_id": self.frontier_id,
            "score": self.score,
            "components": {
                "target_proximity": self.target_proximity,
                "anchor_proximity": self.anchor_proximity,
                "place_affinity": self.place_affinity,
                "uncertainty_factor": self.uncertainty_factor,
                "route_factor": self.route_factor,
            },
            "object_ids": list(self.object_ids),
            "place_ids": list(self.place_ids),
            "evidence_refs": list(self.evidence_refs),
            "reasons": list(self.reasons),
        }


def score_frontier_semantics(
    frontiers: Iterable[Any],
    *,
    target_profile: Any,
    semantic_map: Any | None,
    place_graph: Any | None,
    route_costs: dict[str, dict[str, Any]] | None = None,
    current_position: tuple[float, float] | None = None,
    proximity_scale_m: float = 2.5,
) -> tuple[dict[str, float], dict[str, FrontierSemanticEvidence]]:
    """Return selector scores plus auditable component evidence.

    A score is emitted only when there is measured target/anchor/place
    evidence.  The compatibility mapping uses ``0.0`` for unknown because the
    existing selector accepts floats; the evidence record preserves ``None``
    so UI/trace consumers can distinguish unknown from measured zero.
    """
    route_costs = route_costs or {}
    target_terms, anchor_terms = _profile_terms(target_profile)
    objects = list(getattr(semantic_map, "objects", {}).values()) if semantic_map else []
    target_objects = [obj for obj in objects if _matches_object(obj, target_terms)]
    anchor_objects = [obj for obj in objects if _matches_object(obj, anchor_terms)]
    places = getattr(place_graph, "places", {}) if place_graph is not None else {}

    scores: dict[str, float] = {}
    audit: dict[str, FrontierSemanticEvidence] = {}
    for frontier in frontiers:
        frontier_id = str(getattr(frontier, "frontier_id", ""))
        position = _xy(getattr(frontier, "position", None))
        reasons: list[str] = []
        refs: list[str] = []
        object_ids: list[str] = []
        place_ids: list[str] = []

        target_score, target_ref, target_uncertainty = _nearest_object_score(
            position, target_objects, proximity_scale_m
        )
        if target_ref is not None:
            object_ids.append(target_ref)
            refs.append(f"object:{target_ref}")
            reasons.append("frontier near a measured target-class historical observation")

        anchor_score, anchor_ref, anchor_uncertainty = _nearest_object_score(
            position, anchor_objects, proximity_scale_m
        )
        if anchor_ref is not None:
            object_ids.append(anchor_ref)
            refs.append(f"object:{anchor_ref}")
            reasons.append("frontier near an explicit relation anchor")

        place_score, place_id = _place_affinity(
            position, places, target_terms | anchor_terms, proximity_scale_m
        )
        if place_id is not None:
            place_ids.append(place_id)
            refs.append(f"place:{place_id}")
            reasons.append("frontier associated with a place containing relevant observations")

        route = route_costs.get(frontier_id, {})
        route_factor: float | None = None
        if route:
            route_factor = 1.0 if bool(route.get("reachable", False)) else 0.0
            refs.append(f"route:{frontier_id}")
            if route_factor == 0.0:
                reasons.append("route planner marked candidate unreachable")
        uncertainty_values = [
            value for value in (target_uncertainty, anchor_uncertainty)
            if value is not None
        ]
        uncertainty_factor = (
            min(uncertainty_values) if uncertainty_values else None
        )

        measured = [value for value in (target_score, anchor_score, place_score)
                    if value is not None]
        if not measured:
            score = None
            reasons.append("no measured target, anchor, or relevant-place evidence")
        else:
            weighted = (
                0.50 * (target_score or 0.0)
                + 0.35 * (anchor_score or 0.0)
                + 0.15 * (place_score or 0.0)
            )
            if uncertainty_factor is not None:
                weighted *= uncertainty_factor
            if route_factor is not None:
                weighted *= route_factor
            score = _clamp(weighted)

        evidence = FrontierSemanticEvidence(
            frontier_id=frontier_id,
            score=score,
            target_proximity=target_score,
            anchor_proximity=anchor_score,
            place_affinity=place_score,
            uncertainty_factor=uncertainty_factor,
            route_factor=route_factor,
            object_ids=tuple(dict.fromkeys(object_ids)),
            place_ids=tuple(dict.fromkeys(place_ids)),
            evidence_refs=tuple(dict.fromkeys(refs)),
            reasons=tuple(reasons),
        )
        scores[frontier_id] = float(score) if score is not None else 0.0
        audit[frontier_id] = evidence
    return scores, audit


def _profile_terms(profile: Any) -> tuple[set[str], set[str]]:
    target_fields = ("canonical_name_zh", "raw_target", "primary_labels_en", "aliases_zh")
    anchor_fields = ("relation_anchor_labels_en", "relation_anchor_labels_zh")
    return _terms(profile, target_fields), _terms(profile, anchor_fields)


def _terms(value: Any, fields: tuple[str, ...]) -> set[str]:
    result: set[str] = set()
    for field_name in fields:
        raw = getattr(value, field_name, None)
        if raw is None and isinstance(value, dict):
            raw = value.get(field_name)
        items = raw if isinstance(raw, (list, tuple, set)) else [raw]
        for item in items:
            text = _normalise(item)
            if text:
                result.add(text)
    return result


def _normalise(value: Any) -> str:
    return "".join(str(value or "").casefold().split())


def _matches_object(obj: Any, terms: set[str]) -> bool:
    if not terms:
        return False
    labels = {
        _normalise(getattr(obj, name, ""))
        for name in ("label", "label_zh", "category")
    }
    labels.discard("")
    return any(term in label or label in term for term in terms for label in labels)


def _xy(value: Any) -> tuple[float, float] | None:
    if value is None:
        return None
    try:
        x, y = float(value[0]), float(value[1])
    except (TypeError, ValueError, IndexError):
        return None
    return (x, y) if math.isfinite(x) and math.isfinite(y) else None


def _nearest_object_score(
    position: tuple[float, float] | None,
    objects: list[Any],
    scale_m: float,
) -> tuple[float | None, str | None, float | None]:
    if position is None:
        return None, None, None
    best: tuple[float, str, float] | None = None
    for obj in objects:
        object_position = _xy(getattr(obj, "map_xyz", None))
        if object_position is None:
            continue
        distance = math.dist(position, object_position)
        proximity = math.exp(-distance / max(0.1, float(scale_m)))
        uncertainty = _uncertainty_factor(obj)
        value = proximity * uncertainty
        candidate = (value, str(getattr(obj, "object_id", "unknown")), uncertainty)
        if best is None or candidate[0] > best[0]:
            best = candidate
    return (None, None, None) if best is None else best


def _uncertainty_factor(obj: Any) -> float:
    variance = getattr(obj, "position_variance_xyz", None)
    if variance is None:
        return 0.55
    try:
        horizontal_sigma = math.sqrt(max(0.0, float(variance[0])) +
                                     max(0.0, float(variance[1])))
    except (TypeError, ValueError, IndexError):
        return 0.55
    return _clamp(1.0 / (1.0 + 2.0 * horizontal_sigma))


def _place_affinity(
    position: tuple[float, float] | None,
    places: dict[str, Any],
    terms: set[str],
    scale_m: float,
) -> tuple[float | None, str | None]:
    if position is None or not terms:
        return None, None
    best: tuple[float, str] | None = None
    for place_id, place in places.items():
        labels = {_normalise(item) for item in
                  getattr(place, "observed_object_labels", [])}
        if not any(term in label or label in term for term in terms for label in labels):
            continue
        pose = getattr(place, "pose", None)
        if pose is None:
            continue
        place_position = _xy((getattr(pose, "x", None), getattr(pose, "y", None)))
        if place_position is None:
            continue
        value = math.exp(-math.dist(position, place_position) / max(0.1, scale_m))
        candidate = (value, str(place_id))
        if best is None or candidate[0] > best[0]:
            best = candidate
    return (None, None) if best is None else best


def _clamp(value: float) -> float:
    return max(0.0, min(1.0, float(value)))
