"""Deadline helpers for blocking vision-model calls."""

from __future__ import annotations

import time


def bounded_vlm_timeout(
    configured_seconds: float,
    session_deadline_monotonic: float | None,
    *,
    now_monotonic: float | None = None,
) -> float:
    """Cap one blocking VLM call by the remaining search-session budget."""
    configured = max(0.05, float(configured_seconds))
    if session_deadline_monotonic is None:
        return configured
    now = time.monotonic() if now_monotonic is None else float(now_monotonic)
    remaining = float(session_deadline_monotonic) - now
    if remaining <= 0.0:
        raise TimeoutError("search session budget expired before VLM request")
    return max(0.05, min(configured, remaining))
