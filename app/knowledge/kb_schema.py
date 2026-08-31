"""Schemas for the local scene knowledge base files."""

from __future__ import annotations

from pydantic import Field

from app.schemas import Confidence, KnowledgeItem, StrictBaseModel


class KBDoor(StrictBaseModel):
    door_id: str
    label: str
    side: str | None = None
    order: int | None = None
    connected_room_id: str | None = None
    location_hint: str | None = None
    confidence: Confidence = Field(ge=0.0, le=1.0)


class KBRoom(StrictBaseModel):
    room_id: str
    label: str | None = None
    room_type: str | None = None
    location_hint: str | None = None
    confidence: Confidence = Field(ge=0.0, le=1.0)


class FloorLayout(StrictBaseModel):
    floor_id: str
    building_id: str | None = None
    corridor_direction: str | None = None
    description_zh: str
    doors: list[KBDoor] = Field(default_factory=list)
    rooms: list[KBRoom] = Field(default_factory=list)
    confidence: Confidence = Field(ge=0.0, le=1.0)


class FloorLayoutFile(StrictBaseModel):
    floor_layouts: list[FloorLayout] = Field(default_factory=list)


class RoomPrior(StrictBaseModel):
    room_type: str
    name_zh: str | None = None
    common_objects: list[str] = Field(default_factory=list)
    likely_layout: list[str] = Field(default_factory=list)
    confidence: Confidence = Field(ge=0.0, le=1.0)


class RoomPriorFile(StrictBaseModel):
    room_type_priors: list[RoomPrior] = Field(default_factory=list)


class ObjectLocationPriorRecord(StrictBaseModel):
    object_name: str
    name_zh: str | None = None
    likely_locations: list[str] = Field(default_factory=list)
    unlikely_locations: list[str] = Field(default_factory=list)
    related_objects: list[str] = Field(default_factory=list)
    confidence: Confidence = Field(ge=0.0, le=1.0)


class ObjectLocationPriorFile(StrictBaseModel):
    object_location_priors: list[ObjectLocationPriorRecord] = Field(default_factory=list)


class ObservationLogRecord(StrictBaseModel):
    observation_id: str
    timestamp: str
    location_hint: str | None = None
    summary_zh: str
    confidence: Confidence = Field(ge=0.0, le=1.0)


class SceneKBData(StrictBaseModel):
    floor_layouts: list[FloorLayout] = Field(default_factory=list)
    room_type_priors: list[RoomPrior] = Field(default_factory=list)
    object_location_priors: list[ObjectLocationPriorRecord] = Field(default_factory=list)
    observations: list[ObservationLogRecord] = Field(default_factory=list)


__all__ = [
    "FloorLayout",
    "FloorLayoutFile",
    "KBDoor",
    "KBRoom",
    "KnowledgeItem",
    "ObjectLocationPriorFile",
    "ObjectLocationPriorRecord",
    "ObservationLogRecord",
    "RoomPrior",
    "RoomPriorFile",
    "SceneKBData",
]
