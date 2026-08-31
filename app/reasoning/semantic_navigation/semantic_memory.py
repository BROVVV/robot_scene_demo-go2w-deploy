"""Session-scoped negative search memory with provenance and expiry."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from app.memory.observation_memory_store import ObservationMemoryStore


@dataclass(frozen=True)
class SearchNegativeEvidence:
    target_key: str
    anchor_key: str | None
    observation_pose: dict[str, Any] | None
    heading_sector: int | None
    reason: str
    confidence: float
    created_at: float
    ttl_sec: float | None
    source_event_id: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class SemanticSearchMemory:
    """Keep transient failed-search evidence separate from long-term memory."""

    def __init__(self, *, default_ttl_sec: float = 300.0,
                 observation_store: ObservationMemoryStore | None = None,
                 now: Any = time.time) -> None:
        self.default_ttl_sec = float(default_ttl_sec)
        self.observation_store = observation_store
        self._now = now
        self._negative: list[SearchNegativeEvidence] = []

    def add_negative(self, *, target_key: str, heading_sector: int | None,
                     reason: str, source_event_id: str,
                     anchor_key: str | None = None,
                     observation_pose: dict[str, Any] | None = None,
                     confidence: float = 0.7,
                     ttl_sec: float | None = None) -> SearchNegativeEvidence:
        if not source_event_id:
            raise ValueError("negative evidence requires source_event_id provenance")
        item = SearchNegativeEvidence(
            target_key=target_key,
            anchor_key=anchor_key,
            observation_pose=observation_pose,
            heading_sector=heading_sector,
            reason=reason,
            confidence=max(0.0, min(1.0, float(confidence))),
            created_at=float(self._now()),
            ttl_sec=self.default_ttl_sec if ttl_sec is None else ttl_sec,
            source_event_id=source_event_id,
        )
        self._negative.append(item)
        return item

    def active(self, *, target_key: str | None = None) -> list[SearchNegativeEvidence]:
        now = float(self._now())
        self._negative = [
            item for item in self._negative
            if item.ttl_sec is None or now - item.created_at < item.ttl_sec
        ]
        if target_key is None:
            return list(self._negative)
        return [item for item in self._negative if item.target_key == target_key]

    def sector_penalty(self, target_key: str, heading_sector: int) -> tuple[float, list[str]]:
        matches = [
            item for item in self.active(target_key=target_key)
            if item.heading_sector == heading_sector
        ]
        penalty = min(0.9, sum(item.confidence * 0.35 for item in matches))
        return penalty, [item.source_event_id for item in matches]

    def release(self, *, target_key: str, heading_sector: int | None = None,
                anchor_key: str | None = None) -> int:
        before = len(self._negative)
        self._negative = [
            item for item in self._negative
            if not (
                item.target_key == target_key
                and (heading_sector is None or item.heading_sector == heading_sector)
                and (anchor_key is None or item.anchor_key == anchor_key)
            )
        ]
        return before - len(self._negative)

    def reset_for_task_change(self) -> None:
        self._negative.clear()

    def retrieve_long_term(self, target: str, top_k: int = 10) -> list[dict[str, Any]]:
        if self.observation_store is None:
            return []
        return self.observation_store.retrieve(target, top_k=top_k)

    def to_dict(self) -> dict[str, Any]:
        return {
            "scope": "session",
            "negative_evidence": [item.to_dict() for item in self.active()],
            "persistent_write_attempted": False,
        }
