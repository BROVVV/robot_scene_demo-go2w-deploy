"""Data structures for detector candidates and verification evidence."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass
class CandidateObject:
    object_id: str
    label: str
    label_zh: str | None = None
    bbox: list[float] | None = None
    score: float = 0.0
    detector_score: float | None = None
    text_score: float | None = None
    mask_area_ratio: float | None = None
    source: str = "unknown"
    source_prompt_term: str | None = None
    frame_index: int | None = None
    timestamp_sec: float | None = None
    crop_path: str | None = None
    crop_verify: dict[str, Any] | None = None
    attributes: list[str] = field(default_factory=list)
    spatial_relations: list[dict[str, Any]] = field(default_factory=list)
    final_score: float | None = None
    decision: str = "candidate"
    rejection_reason: str | None = None
    explanation: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "CandidateObject":
        fields = cls.__dataclass_fields__
        return cls(**{key: value for key, value in payload.items() if key in fields})
