"""SiliconFlow scene-object JSON parser tests (plan book §46)."""

from __future__ import annotations

import pytest

from app.manual_web_demo.models import SceneObject
from app.manual_web_demo.scene_object_analyzer import (
    normalize_confidence,
    parse_scene_objects_payload,
)
from app.utils.json_utils import extract_json_from_text


def test_valid_payload() -> None:
    payload = {
        "scene_summary": "室内办公区域",
        "objects": [
            {
                "name_zh": "椅子",
                "name_en": "chair",
                "count": 2,
                "position": "左侧和中间",
                "confidence": "high",
            },
            {"name_zh": "桌子", "name_en": "table", "count": 1,
             "position": "中间", "confidence": "high"},
        ],
    }
    objects, summary = parse_scene_objects_payload(payload)
    assert summary == "室内办公区域"
    assert len(objects) == 2
    assert objects[0].name_zh == "椅子"
    assert objects[0].count == 2
    assert objects[0].confidence == "high"


def test_markdown_fenced_json() -> None:
    raw = """```json
    {"scene_summary": "厨房", "objects": [{"name_zh": "冰箱", "count": 1, "confidence": "high"}]}
    ```"""
    data = extract_json_from_text(raw)
    objects, summary = parse_scene_objects_payload(data)
    assert summary == "厨房"
    assert objects[0].name_zh == "冰箱"


def test_missing_count_becomes_none() -> None:
    payload = {"objects": [{"name_zh": "门", "count": "无法确认", "confidence": "medium"}]}
    objects, _ = parse_scene_objects_payload(payload)
    assert objects[0].count is None


def test_missing_name_en_and_position() -> None:
    payload = {"objects": [{"name_zh": "垃圾桶", "confidence": "low"}]}
    objects, _ = parse_scene_objects_payload(payload)
    assert objects[0].name_en is None
    assert objects[0].position is None


def test_unknown_confidence_normalized_to_medium() -> None:
    assert normalize_confidence("high") == "high"
    assert normalize_confidence("LOW") == "low"
    assert normalize_confidence("maybe") == "medium"
    assert normalize_confidence(None) == "medium"


def test_duplicate_objects_merged() -> None:
    payload = {
        "objects": [
            {"name_zh": "椅子", "count": 1},
            {"name_zh": "椅子", "count": 3},
            {"name_zh": "chair", "count": 2},
        ]
    }
    objects, _ = parse_scene_objects_payload(payload)
    assert len(objects) == 2  # zh + en both appear but each once


def test_malformed_response_raises() -> None:
    with pytest.raises(ValueError):
        parse_scene_objects_payload("not a dict")
    with pytest.raises(ValueError):
        parse_scene_objects_payload({"objects": "nope"})
    with pytest.raises(ValueError):
        parse_scene_objects_payload({})


def test_empty_response() -> None:
    objects, summary = parse_scene_objects_payload({"objects": [], "scene_summary": ""})
    assert objects == []
    assert summary is None


def test_object_without_names_is_skipped() -> None:
    payload = {"objects": [{"count": 4, "confidence": "high"}, {"name_zh": "沙发"}]}
    objects, _ = parse_scene_objects_payload(payload)
    assert [obj.name_zh for obj in objects] == ["沙发"]
