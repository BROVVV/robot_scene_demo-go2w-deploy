"""JSONL store for observation-grounded memories."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import TYPE_CHECKING, Any
from uuid import uuid4

from app.config import Settings, get_settings

if TYPE_CHECKING:
    from app.schemas import SceneAnalysisResult


class ObservationMemoryStore:
    def __init__(self, path: str | Path | None = None, settings: Settings | None = None) -> None:
        self.settings = settings or get_settings()
        self.path = Path(path or self.settings.observation_memory_store_path)

    def load_all(self) -> list[dict[str, Any]]:
        if not self.path.exists():
            return []
        records: list[dict[str, Any]] = []
        for line in self.path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                continue
        return records

    def retrieve(self, target: str, top_k: int | None = None) -> list[dict[str, Any]]:
        limit = top_k or self.settings.observation_memory_retrieval_top_k
        target_norm = target.strip().lower()
        memories = []
        for item in self.load_all():
            text = json.dumps(item, ensure_ascii=False).lower()
            if target_norm in text or any(token and token in text for token in target_norm.split()):
                memories.append(item)
        memories.sort(key=lambda item: str(item.get("created_at", "")), reverse=True)
        return memories[:limit]

    def append(self, record: dict[str, Any]) -> dict[str, Any]:
        _validate_memory(record, self.settings)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")
        return record

    def append_many(self, records: list[dict[str, Any]]) -> list[dict[str, Any]]:
        written = []
        for record in records:
            written.append(self.append(record))
        return written


def build_scene_memory_updates(
    scene: SceneAnalysisResult,
    gate_report: dict[str, Any],
    image_path: str | Path | None = None,
) -> list[dict[str, Any]]:
    updates: list[dict[str, Any]] = []
    confirmed = gate_report.get("target_found") is True
    for candidate_report in gate_report.get("candidates") or []:
        evidence = candidate_report.get("evidence") or {}
        if not evidence.get("bbox"):
            continue
        updates.append(
            {
                "memory_id": f"mem_{uuid4().hex[:12]}",
                "memory_type": "object_observation",
                "label": evidence.get("label_zh") or evidence.get("label") or "object",
                "target_related": evidence.get("candidate_id")
                in scene.target_decision.matched_object_ids
                or candidate_report.get("target_found") is True,
                "evidence": {
                    "frame_id": evidence.get("frame_id") or "single_image",
                    "image_path": str(image_path) if image_path is not None else None,
                    "crop_path": evidence.get("crop_path"),
                    "bbox": evidence.get("bbox"),
                    "mask_area_ratio": evidence.get("mask_area_ratio"),
                    "detector_score": evidence.get("detector_score"),
                    "crop_verify_score": evidence.get("crop_verify_score"),
                    "source_detector": evidence.get("source_detector"),
                },
                "spatial_context": {"near": [], "on": [], "under": []},
                "confirmed_by_user": False,
                "confirmed_by_visual_gate": bool(
                    confirmed and candidate_report.get("target_found")
                ),
                "created_at": datetime.now(timezone.utc).isoformat(),
            }
        )
    return updates


def _validate_memory(record: dict[str, Any], settings: Settings) -> None:
    memory_type = record.get("memory_type")
    if memory_type == "llm_summarized_observation_pattern":
        if not record.get("supporting_observations"):
            raise ValueError("LLM memory summaries require supporting_observations.")
        if record.get("can_confirm_target") is not False:
            raise ValueError("LLM memory summaries cannot confirm targets.")
        return
    if memory_type != "object_observation":
        raise ValueError(f"Unsupported observation memory_type: {memory_type}")
    evidence = record.get("evidence") or {}
    if settings.observation_memory_require_provenance and not evidence.get("bbox"):
        raise ValueError("Observation memories require bbox provenance.")
    if settings.observation_memory_write_visual_only and not (
        evidence.get("image_path") or evidence.get("crop_path") or evidence.get("frame_id")
    ):
        raise ValueError("Observation memories must come from visual evidence.")
