"""Lightweight JSONL storage and retrieval for situated-search experience."""

from __future__ import annotations

import json
import re
from pathlib import Path

from app.schemas import SpatialExperienceMemory


class LLMExperienceMemory:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)

    def append(self, memory: SpatialExperienceMemory) -> Path:
        self.append_if_novel(memory)
        return self.path

    def append_if_novel(self, memory: SpatialExperienceMemory) -> bool:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if any(_same_experience(item, memory) for item in self._read_all()[-200:]):
            return False
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(memory.model_dump_json() + "\n")
        return True

    def retrieve(
        self,
        *,
        target_text: str,
        scene_type: str | None,
        visible_anchor_labels: list[str],
        top_k: int = 5,
    ) -> list[dict]:
        target_tokens = _tokens(target_text)
        anchor_tokens = {_normalize(item) for item in visible_anchor_labels if item}
        scored: list[tuple[float, SpatialExperienceMemory]] = []
        for memory in self._read_all():
            score = _jaccard(target_tokens, _tokens(memory.target_text)) * 0.5
            if scene_type and memory.scene_type == scene_type:
                score += 0.2
            memory_anchors = {
                _normalize(item) for item in memory.visible_anchor_labels if item
            }
            score += _jaccard(anchor_tokens, memory_anchors) * 0.2
            if memory.outcome == "found":
                score += 0.1
            if score > 0:
                scored.append((score, memory))
        scored.sort(key=lambda item: item[0], reverse=True)
        return [
            {
                **memory.model_dump(mode="json"),
                "retrieval_score": round(score, 4),
            }
            for score, memory in scored[:top_k]
        ]

    def _read_all(self) -> list[SpatialExperienceMemory]:
        if not self.path.is_file():
            return []
        result: list[SpatialExperienceMemory] = []
        for line in self.path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            try:
                result.append(SpatialExperienceMemory.model_validate(json.loads(line)))
            except (ValueError, json.JSONDecodeError):
                continue
        return result


def _normalize(value: str) -> str:
    return re.sub(r"\s+", "", value.strip().lower())


def _tokens(value: str) -> set[str]:
    normalized = _normalize(value)
    ascii_words = set(re.findall(r"[a-z0-9_]+", normalized))
    han = set(re.findall(r"[\u4e00-\u9fff]", normalized))
    return ascii_words | han


def _jaccard(left: set[str], right: set[str]) -> float:
    if not left or not right:
        return 0.0
    return len(left & right) / len(left | right)


def _same_experience(
    left: SpatialExperienceMemory,
    right: SpatialExperienceMemory,
) -> bool:
    return (
        _normalize(left.target_normalized) == _normalize(right.target_normalized)
        and left.scene_type == right.scene_type
        and _normalize(left.hypothesis_region_zh)
        == _normalize(right.hypothesis_region_zh)
        and left.outcome == right.outcome
        and left.action_taken == right.action_taken
        and set(left.visible_anchor_labels) == set(right.visible_anchor_labels)
    )
