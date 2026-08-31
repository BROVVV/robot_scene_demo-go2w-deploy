"""JSON/JSONL storage for the local scene knowledge base."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from app.knowledge.kb_schema import (
    FloorLayoutFile,
    ObjectLocationPriorFile,
    ObservationLogRecord,
    RoomPriorFile,
    SceneKBData,
)


DEFAULT_KB_DIR = Path("data/scene_kb")


def _read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def _read_observations(path: Path) -> list[ObservationLogRecord]:
    if not path.exists():
        return []

    records: list[ObservationLogRecord] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            records.append(ObservationLogRecord.model_validate_json(line))
    return records


def load_kb(kb_dir: str | Path = DEFAULT_KB_DIR) -> SceneKBData:
    root = Path(kb_dir)
    floor_layouts = FloorLayoutFile.model_validate(
        _read_json(root / "floor_layout.json")
    ).floor_layouts
    room_priors = RoomPriorFile.model_validate(
        _read_json(root / "room_type_priors.json")
    ).room_type_priors
    object_priors = ObjectLocationPriorFile.model_validate(
        _read_json(root / "object_location_priors.json")
    ).object_location_priors
    observations = _read_observations(root / "observations.jsonl")

    return SceneKBData(
        floor_layouts=floor_layouts,
        room_type_priors=room_priors,
        object_location_priors=object_priors,
        observations=observations,
    )


def save_kb(data: SceneKBData, kb_dir: str | Path = DEFAULT_KB_DIR) -> None:
    root = Path(kb_dir)
    _write_json(
        root / "floor_layout.json",
        {
            "floor_layouts": [
                item.model_dump(mode="json") for item in data.floor_layouts
            ]
        },
    )
    _write_json(
        root / "room_type_priors.json",
        {
            "room_type_priors": [
                item.model_dump(mode="json") for item in data.room_type_priors
            ]
        },
    )
    _write_json(
        root / "object_location_priors.json",
        {
            "object_location_priors": [
                item.model_dump(mode="json") for item in data.object_location_priors
            ]
        },
    )

    observation_path = root / "observations.jsonl"
    observation_path.parent.mkdir(parents=True, exist_ok=True)
    observation_path.write_text(
        "".join(
            json.dumps(item.model_dump(mode="json"), ensure_ascii=False) + "\n"
            for item in data.observations
        ),
        encoding="utf-8",
    )


def append_observation(
    observation: ObservationLogRecord | dict[str, Any],
    kb_dir: str | Path = DEFAULT_KB_DIR,
) -> ObservationLogRecord:
    record = ObservationLogRecord.model_validate(observation)
    path = Path(kb_dir) / "observations.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(record.model_dump(mode="json"), ensure_ascii=False) + "\n")
    return record


def update_confidence(
    item_id: str,
    confidence: float,
    kb_dir: str | Path = DEFAULT_KB_DIR,
) -> bool:
    data = load_kb(kb_dir)
    updated = False

    for layout in data.floor_layouts:
        if layout.floor_id == item_id:
            layout.confidence = confidence
            updated = True
        for door in layout.doors:
            if door.door_id == item_id:
                door.confidence = confidence
                updated = True
        for room in layout.rooms:
            if room.room_id == item_id:
                room.confidence = confidence
                updated = True

    for prior in data.room_type_priors:
        if prior.room_type == item_id:
            prior.confidence = confidence
            updated = True

    for prior in data.object_location_priors:
        if prior.object_name == item_id:
            prior.confidence = confidence
            updated = True

    if updated:
        save_kb(data, kb_dir)
    return updated
