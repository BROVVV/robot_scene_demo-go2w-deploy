"""Target-state helpers for video target search."""

from __future__ import annotations

from typing import Any


TARGET_NOT_SEEN = "target_not_seen"
TARGET_CANDIDATE = "target_candidate"
TARGET_VISUAL_CONFIRMED = "target_visual_confirmed"
TARGET_LOST_AFTER_SEEN = "target_lost_after_seen"
TARGET_UNCONFIRMED_BUT_LIKELY_AREA_FOUND = "target_unconfirmed_but_likely_area_found"

TARGET_STATES = {
    TARGET_NOT_SEEN,
    TARGET_CANDIDATE,
    TARGET_VISUAL_CONFIRMED,
    TARGET_LOST_AFTER_SEEN,
    TARGET_UNCONFIRMED_BUT_LIKELY_AREA_FOUND,
}


def determine_target_state(
    target_search_result: dict[str, Any],
    ranked_places: list[dict[str, Any]] | None = None,
    high_score_threshold: float = 0.70,
) -> str:
    """Determine target status without allowing context-only confirmation."""

    if target_search_result.get("target_found") is True:
        return TARGET_VISUAL_CONFIRMED

    if _has_visual_candidate(target_search_result):
        return TARGET_CANDIDATE

    best_ranked_place = max(
        ranked_places or [],
        key=lambda item: float(item.get("target_search_score", item.get("score", 0.0)) or 0.0),
        default=None,
    )
    if best_ranked_place and _rank_score(best_ranked_place) >= high_score_threshold:
        return TARGET_UNCONFIRMED_BUT_LIKELY_AREA_FOUND

    return TARGET_NOT_SEEN


def apply_target_state(
    target_search_result: dict[str, Any],
    ranked_places: list[dict[str, Any]] | None = None,
    high_score_threshold: float = 0.70,
) -> dict[str, Any]:
    """Annotate a search result with the explicit target state contract."""

    status = determine_target_state(
        target_search_result,
        ranked_places=ranked_places,
        high_score_threshold=high_score_threshold,
    )
    target_search_result["target_status"] = status
    target_search_result["target_confirmed"] = status == TARGET_VISUAL_CONFIRMED
    if status != TARGET_VISUAL_CONFIRMED and target_search_result.get("target_found"):
        target_search_result["target_found"] = False
    return target_search_result


def _has_visual_candidate(target_search_result: dict[str, Any]) -> bool:
    if target_search_result.get("best_evidence"):
        return True
    if target_search_result.get("direct_candidates"):
        return True
    return any(
        item.get("type") == "direct_detection"
        for item in target_search_result.get("timeline", [])
    )


def _rank_score(place: dict[str, Any]) -> float:
    try:
        return float(place.get("target_search_score", place.get("score", 0.0)) or 0.0)
    except (TypeError, ValueError):
        return 0.0
