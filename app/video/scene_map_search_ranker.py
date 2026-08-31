"""Rank navigation places as auxiliary search targets."""

from __future__ import annotations

import re
from typing import Any


NEGATIVE_SCENE_TERMS = {
    "bathroom",
    "toilet",
    "restroom",
    "corridor_only",
    "empty_corridor",
    "卫生间",
    "厕所",
    "走廊-only",
}


def rank_places_for_target_search(
    navigation_topology: dict[str, Any],
    target_profile: Any,
    target_search_result: dict[str, Any],
    config: Any,
) -> list[dict[str, Any]]:
    """Rank places for the next target observation.

    The ranker never confirms the target. It can only identify places worth
    searching again.
    """

    del config
    profile = _profile_dict(target_profile)
    places = _place_nodes(navigation_topology)
    children_by_place = _children_by_place(navigation_topology)
    ranked: list[dict[str, Any]] = []
    for place in places:
        children = children_by_place.get(str(place.get("node_id")), [])
        score, reasons = _score_place(place, children, profile)
        recommended = _recommended_observation(place, children, profile)
        ranked.append(
            {
                "place_id": place.get("node_id"),
                "scene_type": place.get("label"),
                "target_search_score": round(score, 3),
                "score": round(score, 3),
                "target_status_hint": (
                    "visual_candidate_needs_confirmation"
                    if _has_direct_target_child(children, profile)
                    else "likely_area_not_confirmed"
                    if score >= 0.7
                    else "low_priority_area"
                ),
                "reason": "; ".join(reasons) or "No strong target-related cues.",
                "recommended_observation": recommended,
                "can_confirm_target": False,
                "target_confirmed": False,
                "requires_visual_confirmation": True,
                "supporting_objects": [
                    item.get("label_zh") or item.get("label")
                    for item in children
                    if item.get("node_type") not in {"place", "free_space"}
                ],
            }
        )
    ranked.sort(key=lambda item: item["target_search_score"], reverse=True)
    return ranked


def annotate_topology_for_target_search(
    navigation_topology: dict[str, Any],
    ranked_places: list[dict[str, Any]],
    target_search_result: dict[str, Any],
) -> dict[str, Any]:
    """Add target-search annotations without converting them into confirmations."""

    ranked_by_place = {str(item.get("place_id")): item for item in ranked_places}
    for node in navigation_topology.get("nodes", []):
        place_id = str(node.get("node_id"))
        ranked = ranked_by_place.get(place_id)
        if not ranked:
            continue
        properties = node.setdefault("properties", {})
        properties["target_search_score"] = ranked["target_search_score"]
        properties["target_status_hint"] = ranked["target_status_hint"]
        properties["can_confirm_target"] = False
        label = node.get("label") or place_id
        node["label"] = f"{label} [search_score={ranked['target_search_score']:.2f}]"
        node["can_confirm_target"] = False

    navigation_topology.setdefault("metadata", {}).update(
        {
            "version": "navigation_topology_v1",
            "main_task": "target_search",
            "target": target_search_result.get("target")
            or target_search_result.get("task", {}).get("target"),
            "used_for_search": bool(ranked_places),
            "coordinate_mode": "topological_only",
            "has_metric_pose": False,
        }
    )
    navigation_topology["target_search_annotations"] = [
        {
            "place_id": item["place_id"],
            "target_search_score": item["target_search_score"],
            "target_confirmed_here": False,
            "reason": item["reason"],
            "requires_visual_confirmation": True,
        }
        for item in ranked_places
    ]
    navigation_topology["next_best_views"] = [
        {
            "place_id": item["place_id"],
            "action": "turn_and_observe",
            "observation_hint": item["recommended_observation"],
            "reason": item["reason"],
            "requires_visual_confirmation": True,
        }
        for item in ranked_places[:5]
        if item["target_search_score"] >= 0.45
    ]
    return navigation_topology


def _score_place(
    place: dict[str, Any],
    children: list[dict[str, Any]],
    profile: dict[str, Any],
) -> tuple[float, list[str]]:
    score = 0.0
    reasons: list[str] = []
    if _has_direct_target_child(children, profile):
        score += 0.40
        reasons.append("direct visual target candidate observed, still requires gating")
    context_hits = _context_hits(place, children, profile)
    if context_hits:
        score += 0.25
        reasons.append("context cues: " + ", ".join(context_hits[:6]))
    if len(context_hits) >= 3:
        score += 0.15
        reasons.append("multiple target-context cues co-occur in the same place")
    if _scene_type_matches(place, profile):
        score += 0.15
        reasons.append(f"scene type {place.get('label')} matches likely target context")
    if _has_helpful_spatial_relation(place, children):
        score += 0.10
        reasons.append("helpful spatial relation or viewpoint cue observed")
    if any(item.get("node_type") == "free_space" for item in children):
        score += 0.10
        reasons.append("passable free-space cue is available")
    if _is_negative_scene(place, profile):
        score -= 0.30
        reasons.append("scene type is weak or negative for this target")
    if sum(1 for item in children if item.get("node_type") == "obstacle") >= 3:
        score -= 0.20
        reasons.append("area appears heavily blocked")
    return max(0.0, min(1.0, score)), reasons


def _place_nodes(topology: dict[str, Any]) -> list[dict[str, Any]]:
    places = [item for item in topology.get("nodes", []) if item.get("node_type") == "place"]
    if places:
        return places
    return topology.get("places", [])


def _children_by_place(topology: dict[str, Any]) -> dict[str, list[dict[str, Any]]]:
    nodes = {str(item.get("node_id")): item for item in topology.get("nodes", [])}
    children: dict[str, list[dict[str, Any]]] = {}
    for edge in topology.get("edges", []):
        if edge.get("relation") not in {"contains", "observed_in", "passable_in"}:
            continue
        source = str(edge.get("from") or edge.get("source_node_id"))
        target = str(edge.get("to") or edge.get("target_node_id"))
        if target in nodes:
            children.setdefault(source, []).append(nodes[target])
    return children


def _profile_dict(profile: Any) -> dict[str, Any]:
    if hasattr(profile, "to_dict"):
        return profile.to_dict()
    return dict(profile or {})


def _direct_terms(profile: dict[str, Any]) -> set[str]:
    return {
        _normalize(item)
        for item in [
            profile.get("canonical_name_zh"),
            *profile.get("target_terms_zh", []),
            *profile.get("target_terms_en", []),
            *profile.get("zh_terms", []),
            *profile.get("en_terms", []),
            *profile.get("primary_labels_en", []),
            *profile.get("aliases_zh", []),
            *profile.get("aliases_en", []),
        ]
        if item
    }


def _context_terms(profile: dict[str, Any]) -> set[str]:
    return {
        _normalize(item)
        for item in [
            *profile.get("context_terms", []),
            *profile.get("context_objects", []),
            *profile.get("context_labels_en", []),
            *profile.get("context_labels_zh", []),
        ]
        if item
    }


def _has_direct_target_child(children: list[dict[str, Any]], profile: dict[str, Any]) -> bool:
    direct = _direct_terms(profile)
    for child in children:
        text = _node_text(child)
        if any(
            context_only in text
            for context_only in [
                "tv stand",
                "television stand",
                "entertainment cabinet",
                "电视柜",
                "影音柜",
            ]
        ):
            continue
        if _any_term_match(text, direct):
            return True
    return False


def _context_hits(
    place: dict[str, Any],
    children: list[dict[str, Any]],
    profile: dict[str, Any],
) -> list[str]:
    context = _context_terms(profile)
    hits = []
    for item in [place, *children]:
        text = _node_text(item)
        if _any_term_match(text, context):
            hits.append(item.get("label_zh") or item.get("label") or item.get("node_id"))
    return list(dict.fromkeys(str(item) for item in hits))


def _scene_type_matches(place: dict[str, Any], profile: dict[str, Any]) -> bool:
    text = _node_text(place)
    likely_regions = {
        _normalize(item).replace("_", " ")
        for item in [
            *profile.get("likely_regions_zh", []),
            *profile.get("likely_regions_en", []),
        ]
    }
    normalized_text = text.replace("_", " ")
    if _any_term_match(normalized_text, likely_regions):
        return True
    context = _context_terms(profile)
    return any("living room" in item for item in context) and "living" in normalized_text


def _has_helpful_spatial_relation(place: dict[str, Any], children: list[dict[str, Any]]) -> bool:
    text = " ".join([_node_text(place), *(_node_text(child) for child in children)])
    return any(term in text for term in ["facing", "front", "wall", "墙", "正对", "前方"])


def _is_negative_scene(place: dict[str, Any], profile: dict[str, Any]) -> bool:
    text = _node_text(place)
    negative = {_normalize(item) for item in profile.get("negative_terms", [])}
    return _any_term_match(text, negative | NEGATIVE_SCENE_TERMS)


def _recommended_observation(
    place: dict[str, Any],
    children: list[dict[str, Any]],
    profile: dict[str, Any],
) -> str:
    labels = [str(item.get("label_zh") or item.get("label") or "") for item in children]
    joined = " ".join(labels).lower()
    target = profile.get("canonical_name_zh") or profile.get("raw_target") or "target"
    if "tv stand" in joined or "电视柜" in joined:
        return "turn toward the wall above the TV stand and observe again"
    if "sofa" in joined or "沙发" in joined:
        return "move to the living-room-like area, face the wall opposite the sofa, and re-observe"
    return f"move to {place.get('node_id')} and perform visual confirmation for {target}"


def _node_text(node: dict[str, Any]) -> str:
    properties = node.get("properties", {}) if isinstance(node.get("properties"), dict) else {}
    parts = [
        node.get("node_id"),
        node.get("node_type"),
        node.get("label"),
        node.get("label_zh"),
        properties.get("category"),
        properties.get("navigation_role"),
        *(properties.get("dominant_labels") or []),
        *(properties.get("attributes") or []),
    ]
    return " ".join(str(item) for item in parts if item).lower()


def _any_term_match(text: str, terms: set[str]) -> bool:
    return any(term and (term in text or text in term) for term in terms)


def _normalize(value: Any) -> str:
    return " ".join(re.sub(r"[^a-z0-9\u4e00-\u9fff]+", " ", str(value).lower()).split())
