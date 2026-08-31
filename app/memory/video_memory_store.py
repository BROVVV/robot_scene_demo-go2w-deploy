"""JSONL-backed long-term spatial memory for first-person videos."""

from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
from typing import Any


_IMPORTANCE_RANK = {"high": 3, "medium": 2, "low": 1}


class VideoMemoryStore:
    def __init__(self, path: str | Path, dedup_similarity: float = 0.86) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.dedup_similarity = dedup_similarity

    def append(self, memory: dict[str, Any]) -> None:
        self.append_many([memory])

    def append_many(self, memories: list[dict[str, Any]]) -> int:
        if not memories:
            return 0
        existing = self.load_all()
        accepted: list[dict[str, Any]] = []
        for memory in memories:
            if any(self._is_duplicate(memory, item) for item in [*existing, *accepted]):
                continue
            accepted.append(memory)
        if not accepted:
            return 0
        with self.path.open("a", encoding="utf-8") as handle:
            for memory in accepted:
                handle.write(json.dumps(memory, ensure_ascii=False) + "\n")
        return len(accepted)

    def load_all(self) -> list[dict[str, Any]]:
        if not self.path.is_file():
            return []
        items: list[dict[str, Any]] = []
        for line in self.path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(payload, dict):
                items.append(payload)
        return items

    def search_by_target(self, target: str, top_k: int = 10) -> list[dict[str, Any]]:
        matches = [
            item
            for item in self.load_all()
            if str(item.get("target_context", {}).get("target", "")).strip() == target.strip()
            or target.strip() in [str(tag) for tag in item.get("tags", [])]
        ]
        return self._rank(matches)[:top_k]

    def search_by_room_type(
        self, room_type: str, top_k: int = 10
    ) -> list[dict[str, Any]]:
        matches = [
            item
            for item in self.load_all()
            if str(item.get("room_type", "")).strip() == room_type.strip()
        ]
        return self._rank(matches)[:top_k]

    def search_negative_evidence(
        self, target: str, top_k: int = 10
    ) -> list[dict[str, Any]]:
        matches = [
            item
            for item in self.search_by_target(target, top_k=max(top_k * 4, 20))
            if item.get("target_context", {}).get("found") is False
        ]
        return matches[:top_k]

    def _rank(self, items: list[dict[str, Any]]) -> list[dict[str, Any]]:
        return sorted(
            items,
            key=lambda item: (
                _IMPORTANCE_RANK.get(str(item.get("importance", "low")), 0),
                _parse_time(item.get("created_at")),
            ),
            reverse=True,
        )

    def _is_duplicate(
        self, left: dict[str, Any], right: dict[str, Any]
    ) -> bool:
        if left.get("video_id") != right.get("video_id"):
            return False
        if left.get("room_type") != right.get("room_type"):
            return False
        left_target = left.get("target_context", {})
        right_target = right.get("target_context", {})
        if left_target.get("target") != right_target.get("target"):
            return False
        if left_target.get("found") != right_target.get("found"):
            return False
        left_tokens = _signature_tokens(left)
        right_tokens = _signature_tokens(right)
        if not left_tokens or not right_tokens:
            return left.get("scene_summary") == right.get("scene_summary")
        similarity = len(left_tokens & right_tokens) / len(left_tokens | right_tokens)
        return similarity >= self.dedup_similarity


def _signature_tokens(memory: dict[str, Any]) -> set[str]:
    signature = memory.get("place_signature", {})
    values = [
        *signature.get("stable_landmarks", []),
        *[region.get("name", "") for region in memory.get("regions", [])],
        str(memory.get("room_type", "")),
    ]
    return {str(value).strip().lower() for value in values if str(value).strip()}


def _parse_time(value: Any) -> float:
    try:
        return datetime.fromisoformat(str(value)).timestamp()
    except ValueError:
        return 0.0
