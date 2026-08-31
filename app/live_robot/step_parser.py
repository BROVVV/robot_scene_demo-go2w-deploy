"""Parsing and validation for ``f`` / ``b`` translation step primitives."""

from __future__ import annotations

import math


def forward_step_distance(step: str, default_distance_m: float) -> float:
    """Decode ``f`` or a distance-qualified primitive such as ``f0.300``."""
    if step == "f":
        return float(default_distance_m)
    if not step.startswith("f"):
        raise ValueError(f"not a forward step: {step}")
    try:
        distance = float(step[1:])
    except ValueError as exc:
        raise ValueError(f"invalid forward step distance: {step}") from exc
    if not math.isfinite(distance) or distance <= 0.0:
        raise ValueError(f"forward step distance must be positive: {step}")
    return distance


def backward_step_distance(step: str, default_distance_m: float) -> float:
    """Decode ``b`` or a distance-qualified backward recovery primitive."""
    if step == "b":
        return float(default_distance_m)
    if not step.startswith("b"):
        raise ValueError(f"not a backward recovery step: {step}")
    try:
        distance = float(step[1:])
    except ValueError as exc:
        raise ValueError(f"invalid backward step distance: {step}") from exc
    if not math.isfinite(distance) or distance <= 0.0:
        raise ValueError(f"backward step distance must be positive: {step}")
    # A blind long reverse is never allowed by the parser.  The backend still
    # clamps to its own 0.05-0.12 m recovery window, but anything clearly
    # beyond a small recovery distance is rejected at parse time.
    if distance > 0.5:
        raise ValueError(f"backward recovery step too large: {step}")
    return distance


def translation_step_distance(
    step: str,
    *,
    default_forward_m: float,
    default_backward_m: float,
) -> tuple[str, float]:
    """Return (direction, distance_m) for ``f``/``b`` translation steps."""
    if step.startswith("f"):
        return "forward", forward_step_distance(step, default_forward_m)
    if step.startswith("b"):
        return "backward", backward_step_distance(step, default_backward_m)
    raise ValueError(f"not a translation step: {step}")