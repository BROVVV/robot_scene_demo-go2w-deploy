"""High-level query interface for the local scene knowledge base."""

from __future__ import annotations

from pathlib import Path

from app.knowledge import kb_store
from app.knowledge.kb_schema import FloorLayout, ObjectLocationPriorRecord, RoomPrior
from app.schemas import KnowledgeItem


class SceneKnowledgeBase:
    def __init__(self, kb_dir: str | Path = kb_store.DEFAULT_KB_DIR) -> None:
        self.kb_dir = Path(kb_dir)
        self.data = kb_store.load_kb(self.kb_dir)

    def reload(self) -> None:
        self.data = kb_store.load_kb(self.kb_dir)

    def get_object_location_prior(
        self, object_name: str
    ) -> ObjectLocationPriorRecord | None:
        normalized = object_name.lower()
        for prior in self.data.object_location_priors:
            aliases = {prior.object_name.lower()}
            if prior.name_zh:
                aliases.add(prior.name_zh.lower())
            if normalized in aliases:
                return prior
        return None

    def get_floor_layout(self, floor_id: str) -> FloorLayout | None:
        for layout in self.data.floor_layouts:
            if layout.floor_id == floor_id:
                return layout
        return None

    def search_by_target(self, target_text: str) -> list[KnowledgeItem]:
        normalized = target_text.lower()
        items: list[KnowledgeItem] = []
        for prior in self.data.object_location_priors:
            names = [prior.object_name]
            if prior.name_zh:
                names.append(prior.name_zh)
            if any(name.lower() in normalized for name in names):
                items.append(
                    KnowledgeItem(
                        id=f"object_prior:{prior.object_name}",
                        knowledge_type="object_location_prior",
                        content_zh=_object_prior_to_zh(prior),
                        source=str(self.kb_dir / "object_location_priors.json"),
                        confidence=prior.confidence,
                        metadata={"object_name": prior.object_name},
                    )
                )
        return items

    def search_by_room_type(self, room_type: str) -> list[KnowledgeItem]:
        prior = self._get_room_prior(room_type)
        if prior is None:
            return []
        return [
            KnowledgeItem(
                id=f"room_prior:{prior.room_type}",
                knowledge_type="room_type_prior",
                content_zh=_room_prior_to_zh(prior),
                source=str(self.kb_dir / "room_type_priors.json"),
                confidence=prior.confidence,
                metadata={"room_type": prior.room_type},
            )
        ]

    def search_by_location(self, location_hint: str) -> list[KnowledgeItem]:
        normalized = location_hint.lower()
        items: list[KnowledgeItem] = []
        for layout in self.data.floor_layouts:
            if _matches_floor_scope(normalized, layout):
                items.append(_floor_layout_item(layout, self.kb_dir))
                continue
            if layout.floor_id.lower() in normalized or normalized in layout.description_zh.lower():
                items.append(_floor_layout_item(layout, self.kb_dir))
                continue
            for door in layout.doors:
                door_values = [door.door_id, door.label, door.location_hint or ""]
                if any(value.lower() in normalized for value in door_values):
                    items.append(_floor_layout_item(layout, self.kb_dir))
                    break
        return items

    def _get_room_prior(self, room_type: str) -> RoomPrior | None:
        normalized = room_type.lower()
        for prior in self.data.room_type_priors:
            aliases = {prior.room_type.lower()}
            if prior.name_zh:
                aliases.add(prior.name_zh.lower())
            if normalized in aliases:
                return prior
        return None


def _object_prior_to_zh(prior: ObjectLocationPriorRecord) -> str:
    name = prior.name_zh or prior.object_name
    likely = "、".join(prior.likely_locations) or "未知位置"
    unlikely = "、".join(prior.unlikely_locations) or "暂无"
    return f"{name}常见位置：{likely}；不常见位置：{unlikely}。"


def _room_prior_to_zh(prior: RoomPrior) -> str:
    name = prior.name_zh or prior.room_type
    objects = "、".join(prior.common_objects) or "暂无"
    layout = "；".join(prior.likely_layout) or "暂无"
    return f"{name}常见物体：{objects}；常见布局：{layout}。"


def _floor_layout_item(layout: FloorLayout, kb_dir: Path) -> KnowledgeItem:
    return KnowledgeItem(
        id=f"floor_layout:{layout.floor_id}",
        knowledge_type="environment_layout",
        content_zh=layout.description_zh,
        source=str(kb_dir / "floor_layout.json"),
        confidence=layout.confidence,
        metadata={
            "floor_id": layout.floor_id,
            "building_id": layout.building_id,
            "corridor_direction": layout.corridor_direction,
        },
    )


def _matches_floor_scope(normalized: str, layout: FloorLayout) -> bool:
    if normalized in {"current_floor", "floor", "this_floor", "这层楼", "本层"}:
        return True
    if "这层" in normalized or "本层" in normalized:
        return True
    if normalized in {"corridor", "走廊"} and layout.corridor_direction:
        return True
    if "走廊" in normalized and layout.corridor_direction:
        return True
    return False
