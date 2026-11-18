"""Constrain a strategic VLM response to program-owned detection/candidate IDs."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Sequence


@dataclass(frozen=True)
class StrategicDecision:
    target_state: str
    target_detection_id: str | None
    selected_search_candidate_id: str | None
    reason: str
    fallback_used: bool = False

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def constrain_strategic_decision(
    payload: dict[str, Any] | None,
    *,
    detection_ids: Sequence[str],
    candidate_ids: Sequence[str],
) -> StrategicDecision:
    """Validate references and apply the single deterministic fallback rule."""
    raw = dict(payload or {})
    state = str(raw.get("target_state") or "unknown").strip().lower()
    if state not in {"present", "possible", "absent", "unknown"}:
        state = "unknown"
    valid_detections = {str(item) for item in detection_ids if str(item)}
    ordered_candidates = [str(item) for item in candidate_ids if str(item)]
    valid_candidates = set(ordered_candidates)
    detection_id = str(raw.get("target_detection_id") or "") or None
    candidate_id = str(raw.get("selected_search_candidate_id") or "") or None
    reason = str(raw.get("reason") or "").strip()[:300]
    fallback = False

    if detection_id not in valid_detections:
        detection_id = None
    if candidate_id not in valid_candidates:
        candidate_id = None

    if state in {"present", "possible"}:
        if detection_id is None:
            # The VLM may flag visible evidence, but only a local detection can
            # provide geometry.  Preserve uncertainty and request re-detection.
            state = "possible"
        return StrategicDecision(
            target_state=state,
            target_detection_id=detection_id,
            selected_search_candidate_id=None,
            reason=reason or "visual target evidence requires local detection",
            fallback_used=False,
        )

    if candidate_id is None and ordered_candidates:
        candidate_id = ordered_candidates[0]
        fallback = True
    return StrategicDecision(
        target_state=state,
        target_detection_id=None,
        selected_search_candidate_id=candidate_id,
        reason=reason or (
            "deterministic highest-ranked reachable candidate"
            if candidate_id is not None else "no reachable search candidate"
        ),
        fallback_used=fallback,
    )
