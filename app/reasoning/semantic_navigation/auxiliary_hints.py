"""Safe low-priority PSG and precomputed situated-prior adapters."""

from __future__ import annotations

from typing import Any

from app.video.video_psg_predictor import VideoPSGPredictor


_ACTION_HEADINGS = {
    "turn_left_and_observe": 30.0,
    "turn_right_and_observe": -30.0,
    "turn_left": 30.0,
    "turn_right": -30.0,
}


def build_psg_auxiliary_hints(
    scene_graph: Any,
    *,
    enabled: bool,
    max_predicted_nodes: int = 30,
    confidence_threshold: float = 0.45,
) -> dict[str, Any]:
    """Convert observed-graph-grounded PSG views into turn-only tie breakers."""

    status = {
        "source": "psg",
        "enabled": bool(enabled),
        "available": False,
        "used_for_target_confirmation": False,
        "hint_count": 0,
    }
    if not enabled:
        status["reason"] = "disabled_by_config"
        return {"hints": [], "status": status, "psg_layer": None}
    if scene_graph is None or not hasattr(scene_graph, "nodes"):
        status["reason"] = "observed_scene_graph_unavailable"
        return {"hints": [], "status": status, "psg_layer": None}
    try:
        layer = VideoPSGPredictor(
            max_predicted_nodes=max_predicted_nodes,
            confidence_threshold=confidence_threshold,
        ).predict(scene_graph)
    except Exception as exc:
        status["reason"] = f"{type(exc).__name__}: {exc}"
        return {"hints": [], "status": status, "psg_layer": None}
    hints = []
    for view in layer.next_best_views:
        heading = _ACTION_HEADINGS.get(str(view.action))
        if heading is None:
            continue
        hints.append({
            "hint_id": f"psg:{view.view_id}",
            "source": "psg",
            "heading_delta_deg": heading,
            "confidence": max(0.0, min(1.0, float(view.priority))),
            "anchor_node_id": view.anchor_node_id,
            "reason_zh": view.reason_zh,
            "requires_visual_confirmation": True,
            "can_confirm_target": False,
            "allow_forward": False,
        })
    status.update({
        "available": bool(layer.predicted_nodes),
        "reason": "observed_graph_grounded_predictions",
        "hint_count": len(hints),
    })
    return {
        "hints": hints,
        "status": status,
        "psg_layer": layer.to_dict(),
    }


def build_precomputed_situated_prior_hints(
    payload: Any, *, enabled: bool
) -> dict[str, Any]:
    """Consume already-produced LLM hints without a new robot-loop API call."""

    status = {
        "source": "llm_situated_prior",
        "enabled": bool(enabled),
        "available": False,
        "used_for_target_confirmation": False,
        "hint_count": 0,
    }
    if not enabled:
        status["reason"] = "disabled_by_config"
        return {"hints": [], "status": status}
    value = payload if isinstance(payload, dict) else {}
    prior = value.get("situated_prior") or value.get("llm_situated_prior") or {}
    if not isinstance(prior, dict):
        prior = {}
    if prior.get("can_confirm_target") is not False:
        status["reason"] = "missing_safe_precomputed_prior"
        return {"hints": [], "status": status}
    hints = []
    for index, item in enumerate(prior.get("next_view_plan") or [], start=1):
        if not isinstance(item, dict):
            continue
        action = str(item.get("action") or "")
        heading = _ACTION_HEADINGS.get(action)
        if heading is None:
            region = str(item.get("image_region_hint") or "").lower()
            heading = 30.0 if "left" in region else -30.0 if "right" in region else None
        if heading is None:
            continue
        hints.append({
            "hint_id": str(item.get("hint_id") or f"situated_prior:{index:03d}"),
            "source": "llm_situated_prior",
            "heading_delta_deg": heading,
            "confidence": _score(
                item.get(
                    "expected_information_gain",
                    item.get("confidence", 0.0),
                )
            ),
            "reason_zh": str(item.get("reason_zh") or ""),
            "requires_visual_confirmation": True,
            "can_confirm_target": False,
            "allow_forward": False,
        })
    status.update({
        "available": bool(hints),
        "reason": "precomputed_prior_consumed" if hints else "no_safe_directional_hint",
        "hint_count": len(hints),
    })
    return {"hints": hints, "status": status}


def _score(value: Any) -> float:
    try:
        return max(0.0, min(1.0, float(value)))
    except (TypeError, ValueError):
        return 0.0
