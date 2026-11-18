"""Tri-state target evidence shared by Quick VLM and live navigation.

``PRESENT`` is a positive visual candidate with a usable detection.  ``ABSENT``
is an explicit negative from the current frame.  ``POSSIBLE`` means the
category/region is plausible but an attribute, crop, score, or bounding box is
not strong enough to authorize a stop.  The latter state is intentionally
sticky at the navigation boundary: it must be re-observed before motion.

``UNKNOWN`` (计划书 §5.1.5) is **not a verdict**: nothing was decided.  It is
what a failed or timed-out verification produces, and it exists so that "we did
not find out" cannot be silently relabelled ``ABSENT`` -- which would turn a
verification that never ran into evidence that the target is not there.  It is
deliberately outside the positive set: ``target_state_is_present`` returns False
for it, exactly as it does for ``ABSENT``.
"""

from __future__ import annotations

from typing import Any

TARGET_ABSENT = "ABSENT"
TARGET_POSSIBLE = "POSSIBLE"
TARGET_PRESENT = "PRESENT"
TARGET_UNKNOWN = "UNKNOWN"
TARGET_STATES = {TARGET_ABSENT, TARGET_POSSIBLE, TARGET_PRESENT, TARGET_UNKNOWN}

#: The states a stop may be authorized from.  Everything else -- ``ABSENT``,
#: ``POSSIBLE``, ``UNKNOWN`` -- is not positive evidence.
TARGET_POSITIVE_STATES = frozenset({TARGET_PRESENT})


def normalize_target_state(value: Any, default: str = TARGET_ABSENT) -> str:
    """Normalize old boolean and new textual target decisions.

    Membership in ``TARGET_STATES`` is what makes ``"UNKNOWN"`` round-trip as
    ``TARGET_UNKNOWN`` instead of collapsing to the default.  That matters
    because observations carry the state as a string through
    ``target_match["target_state"]`` and read it back here.
    """
    if isinstance(value, bool):
        return TARGET_PRESENT if value else TARGET_ABSENT
    text = str(value or "").strip().upper().replace("-", "_")
    aliases = {
        "FOUND": TARGET_PRESENT,
        "TRUE": TARGET_PRESENT,
        "YES": TARGET_PRESENT,
        "NOT_FOUND": TARGET_ABSENT,
        "FALSE": TARGET_ABSENT,
        "NO": TARGET_ABSENT,
        "UNCERTAIN": TARGET_POSSIBLE,
        "MAYBE": TARGET_POSSIBLE,
        "CANDIDATE": TARGET_POSSIBLE,
    }
    state = aliases.get(text, text)
    return state if state in TARGET_STATES else default


def target_state_from_payload(payload: dict[str, Any] | None) -> str:
    """Read a Quick payload while remaining compatible with binary payloads."""
    if not isinstance(payload, dict):
        return TARGET_ABSENT
    decision = payload.get("target_decision")
    decision = decision if isinstance(decision, dict) else {}
    for candidate in (
        payload.get("target_state"),
        payload.get("state"),
        decision.get("target_state"),
        decision.get("state"),
    ):
        if candidate is not None:
            return normalize_target_state(candidate)
    if any(bool(decision.get(key)) for key in ("possible", "is_possible", "uncertain")):
        return TARGET_POSSIBLE
    if bool(payload.get("possible")) or bool(payload.get("uncertain")):
        return TARGET_POSSIBLE
    if decision.get("is_present") is not None:
        return normalize_target_state(decision.get("is_present"))
    if "found" in payload:
        return normalize_target_state(payload.get("found"))
    return TARGET_ABSENT


def target_state_is_present(payload: dict[str, Any] | None, min_score: float = 0.15) -> bool:
    """Return true only for PRESENT plus a sufficiently confident candidate.

    ``TARGET_UNKNOWN`` is not present evidence: a verification that timed out
    leaves the target exactly as unproven as an empty frame does.
    """
    if target_state_from_payload(payload) != TARGET_PRESENT:
        return False
    value = payload if isinstance(payload, dict) else {}
    candidates = list(value.get("target_objects") or value.get("objects") or [])
    if not candidates:
        return False
    scores: list[float] = []
    for item in candidates:
        if not isinstance(item, dict):
            continue
        try:
            score = float(item.get("score", item.get("confidence", 0.0)))
        except (TypeError, ValueError):
            continue
        if score == score:
            scores.append(score)
    return bool(scores) and max(scores) >= float(min_score)
