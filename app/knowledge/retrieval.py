"""Knowledge retrieval helpers for task and scene reasoning modules."""

from __future__ import annotations

from pathlib import Path

from app.knowledge.scene_kb import SceneKnowledgeBase
from app.schemas import KnowledgeItem


def retrieve_relevant_knowledge(
    target_text: str | None = None,
    room_type: str | None = None,
    location_hint: str | None = None,
    kb_dir: str | Path | None = None,
) -> list[KnowledgeItem]:
    kb = SceneKnowledgeBase(kb_dir=kb_dir) if kb_dir is not None else SceneKnowledgeBase()
    items: list[KnowledgeItem] = []

    if target_text:
        items.extend(kb.search_by_target(target_text))
    if room_type:
        items.extend(kb.search_by_room_type(room_type))
    if location_hint:
        items.extend(kb.search_by_location(location_hint))

    return _dedupe_items(items)


def _dedupe_items(items: list[KnowledgeItem]) -> list[KnowledgeItem]:
    seen: set[str] = set()
    deduped: list[KnowledgeItem] = []
    for item in items:
        if item.id in seen:
            continue
        seen.add(item.id)
        deduped.append(item)
    return deduped
