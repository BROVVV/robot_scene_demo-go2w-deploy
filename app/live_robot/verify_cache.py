"""Same-frame verification cache for VLM-only navigation.

Legacy keys use ``(bundle_id, canonical_target, quantized_bbox)``. Measured
targets use their exact entity/capture/crop identity instead of a quantized
box. Repeated requests for the same identity still share one result.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
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

    def make_key(self, bundle_id: str, target: str, bbox: list[float], *,
                 candidate_identity: dict[str, Any] | None = None) -> tuple[str, str, str]:
        if candidate_identity is not None:
            # A measured target is bound to its exact entity, acquisition and
            # crop. Two-decimal box quantization can alias distinct candidates.
            identity = json.dumps(candidate_identity, sort_keys=True,
                                  separators=(",", ":"), allow_nan=False)
            return (str(bundle_id), str(target), "capture_v1:" + identity)
        quantized = ",".join(f"{float(v):.2f}" for v in bbox)
        return (str(bundle_id), str(target), quantized)

    def get(self, key: tuple[str, str, str]) -> VerificationCacheEntry | None:
        return self._entries.get(key)

    def put(self, key: tuple[str, str, str], entry: VerificationCacheEntry) -> None:
        self._entries[key] = entry

    def __len__(self) -> int:
        return len(self._entries)
