"""Latency profiler for VLM-only low-latency semantic navigation.

This module intentionally has no external dependency and only records
``time.perf_counter()`` deltas plus API-call counters.  The real node writes a
JSONL ``latency_profile`` event from the returned profile dict.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Any


@dataclass
class LatencyProfile:
    planning_cycle: int = 0
    frame_id: str = ""
    timings_ms: dict[str, float] = field(default_factory=dict)
    api_calls: dict[str, int] = field(default_factory=dict)
    counters: dict[str, Any] = field(default_factory=dict)
    _marks: dict[str, float] = field(default_factory=dict, repr=False)

    def mark(self, name: str) -> None:
        self._marks[name] = time.perf_counter()

    def record(self, name: str, start_name: str | None = None,
               duration: float | None = None) -> None:
        if duration is None:
            end = self._marks.get(name + "_end")
            if end is None:
                end = time.perf_counter()
            start = self._marks.get(start_name or name + "_start")
            if start is None:
                # If only one mark exists, use it as the start.
                start = self._marks.get(name) or end
            duration = (end - start) * 1000.0
        self.timings_ms[name] = round(float(duration), 3)

    def snapshot(self) -> dict[str, Any]:
        return {
            "event": "latency_profile",
            "planning_cycle": self.planning_cycle,
            "frame_id": self.frame_id,
            "timings_ms": dict(self.timings_ms),
            "api_calls": dict(self.api_calls),
            "counters": dict(self.counters),
        }


class LatencyProfiler:
    """Compact wrapper used by the high-level runner."""

    def __init__(self, planning_cycle: int = 0, frame_id: str = "") -> None:
        self.profile = LatencyProfile(
            planning_cycle=planning_cycle,
            frame_id=frame_id,
        )

    def mark(self, name: str) -> None:
        self.profile.mark(name)

    def record(self, name: str, start_name: str | None = None,
               duration: float | None = None) -> None:
        self.profile.record(name, start_name=start_name, duration=duration)

    def incr(self, counter: str, count: int = 1) -> None:
        self.profile.api_calls[counter] = self.profile.api_calls.get(counter, 0) + count

    def set_counter(self, key: str, value: Any) -> None:
        self.profile.counters[key] = value

    def snapshot(self) -> dict[str, Any]:
        return self.profile.snapshot()
