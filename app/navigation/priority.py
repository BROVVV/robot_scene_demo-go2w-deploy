"""Priority normalization helpers shared by video navigation modules."""

from __future__ import annotations

from typing import Any


PRIORITY_SCORES = {
    "very_high": 0.95,
    "high": 0.8,
    "medium": 0.55,
    "normal": 0.5,
    "low": 0.3,
    "very_low": 0.15,
}


def priority_to_confidence(value: Any, default: float = 0.45) -> float:
    if value is None:
        return default
    if isinstance(value, str):
        normalized = value.strip().lower().replace("-", "_").replace(" ", "_")
        if normalized in PRIORITY_SCORES:
            return PRIORITY_SCORES[normalized]
        try:
            return float(normalized)
        except ValueError:
            return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default
