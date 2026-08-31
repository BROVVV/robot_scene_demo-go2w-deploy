"""Rule-based retrieval of prior video memories."""

from __future__ import annotations

from typing import Any

from app.memory.video_memory_store import VideoMemoryStore
from app.video.target_context_rules import TARGET_CONTEXT_RULES
from app.video.target_search import canonical_target


class MemoryRetriever:
    def __init__(self, store: VideoMemoryStore) -> None:
        self.store = store

    def retrieve(self, target: str, top_k: int = 10) -> dict[str, Any]:
        memories = self.store.search_by_target(target, top_k=max(top_k * 2, 10))
        positive = [
            item for item in memories if item.get("target_context", {}).get("found") is True
        ][:top_k]
        negative = [
            item for item in memories if item.get("target_context", {}).get("found") is False
        ][:top_k]
        rule = TARGET_CONTEXT_RULES.get(canonical_target(target), {})
        return {
            "positive_memories": positive,
            "negative_memories": negative,
            "target_prior": {
                "likely_places": rule.get("likely_regions", []),
                "likely_objects": rule.get("likely_objects", []),
                "unlikely_places": ["empty_corridor", "wall_only_region"],
            },
        }
