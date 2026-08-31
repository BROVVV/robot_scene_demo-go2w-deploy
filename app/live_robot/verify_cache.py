"""Same-frame verification cache for VLM-only navigation.

The key is ``(bundle_id, canonical_target, quantized_bbox)``.  A Verify VLM
result is cached per frame so the state machine cannot accidentally issue
multiple API calls for the same image/box/target combination.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Any


@dataclass
class VerificationCacheEntry:
    confirmed: bool
    confidence: float
    reason: str = ""
    timestamp: float = field(default_factory=time.time)
    details: dict[str, Any] = field(default_factory=dict)


class VerificationCache:
    def __init__(self) -> None:
        self._entries: dict[tuple[str, str, str], VerificationCacheEntry] = {}

    def make_key(self, bundle_id: str, target: str, bbox: list[float]) -> tuple[str, str, str]:
        quantized = ",".join(f"{float(v):.2f}" for v in bbox)
        return (str(bundle_id), str(target), quantized)

    def get(self, key: tuple[str, str, str]) -> VerificationCacheEntry | None:
        return self._entries.get(key)

    def put(self, key: tuple[str, str, str], entry: VerificationCacheEntry) -> None:
        self._entries[key] = entry

    def __len__(self) -> int:
        return len(self._entries)
