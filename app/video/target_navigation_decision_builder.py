"""Build target-conditioned navigation decisions."""

from __future__ import annotations

from typing import Any

from app.video.video_target_state import (
    TARGET_CANDIDATE,
    TARGET_NOT_SEEN,
    TARGET_UNCONFIRMED_BUT_LIKELY_AREA_FOUND,
    TARGET_VISUAL_CONFIRMED,
)


def build_target_navigation_decision(
    target_search_result: dict[str, Any],
    navigation_topology: dict[str, Any] | None,
    ranked_places: list[dict[str, Any]] | None,
    target_profile: dict[str, Any],
    config: Any,
) -> dict[str, Any]:
    del navigation_topology, config
    target = (
        target_search_result.get("target")
        or target_search_result.get("task", {}).get("target")
        or target_profile.get("raw_target")
    )
    status = target_search_result.get("target_status") or TARGET_NOT_SEEN
    best = target_search_result.get("best_evidence")
    if status == TARGET_VISUAL_CONFIRMED and best:
        return {
            "target": target,
            "target_status": TARGET_VISUAL_CONFIRMED,
            "target_confirmed": True,
            "best_frame": best.get("frame_id"),
            "best_timestamp": best.get("timestamp_sec"),
            "best_bbox": best.get("bbox"),
            "current_best_evidence": best,
            "next_action": "approach_safely_and_stop",
            "reason": "Target visually confirmed by frame evidence and evidence gating.",
            "requires_visual_confirmation": False,
        }

    best_place = (ranked_places or [None])[0]
    if best_place and float(best_place.get("target_search_score", 0.0)) >= 0.70:
        return {
            "target": target,
            "target_status": TARGET_UNCONFIRMED_BUT_LIKELY_AREA_FOUND,
            "target_confirmed": False,
            "current_best_evidence": None,
            "next_action": "navigate_to_place_and_observe",
            "next_place_id": best_place.get("place_id"),
            "next_observation_hint": best_place.get("recommended_observation"),
            "reason": best_place.get("reason"),
            "requires_visual_confirmation": True,
        }

    if status == TARGET_CANDIDATE:
        return {
            "target": target,
            "target_status": TARGET_CANDIDATE,
            "target_confirmed": False,
            "current_best_evidence": best,
            "next_action": "stop_and_reobserve_candidate",
            "reason": "A visual candidate exists, but evidence gating did not confirm the target.",
            "requires_visual_confirmation": True,
        }

    return {
        "target": target,
        "target_status": TARGET_NOT_SEEN,
        "target_confirmed": False,
        "current_best_evidence": None,
        "next_action": "continue_systematic_search",
        "reason": "No target candidate or high-confidence likely search area was found.",
        "requires_visual_confirmation": True,
    }
