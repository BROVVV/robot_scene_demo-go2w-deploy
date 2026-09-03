"""Tests for the VLM-only Quick target contract (P0-1)."""

from __future__ import annotations

from app.detectors.siliconflow_vision_worker import quick_target_present
from app.live_robot.semantic_observer import semantic_payload_from_quick_target_absence


def test_no_target_with_ordinary_objects_is_not_target_present():
    payload = {
        "objects": [],
        "target_objects": [],
        "scene_objects": [
            {"label": "办公椅", "bbox_2d": [0.1, 0.2, 0.3, 0.8], "score": 0.9},
            {"label": "办公桌", "bbox_2d": [0.3, 0.3, 0.8, 0.8], "score": 0.9},
        ],
        "target_decision": {"is_present": False, "confidence": 0.96},
    }
    assert quick_target_present(payload) is False
    # compatibility objects must be target-only
    assert payload["objects"] == []


def test_true_target_present_passes_gate():
    payload = {
        "objects": [{"label": "蓝色垃圾桶", "bbox_2d": [0.5, 0.2, 0.7, 0.8], "score": 0.91}],
        "target_objects": [{"label": "蓝色垃圾桶", "bbox_2d": [0.5, 0.2, 0.7, 0.8], "score": 0.91}],
        "scene_objects": [],
        "target_decision": {"is_present": True, "confidence": 0.91},
    }
    assert quick_target_present(payload) is True
    assert quick_target_present(payload, min_score=0.95) is False


def test_low_score_target_does_not_pass_gate():
    payload = {
        "target_objects": [{"label": "x", "score": 0.10}],
        "target_decision": {"is_present": True},
    }
    assert quick_target_present(payload, min_score=0.15) is False


def test_absent_with_objects_key_still_reuses_explicit_absence():
    result = semantic_payload_from_quick_target_absence(
        {
            "objects": [],
            "scene_summary_zh": "没有目标",
            "target_decision": {"is_present": False},
        },
        image_path="i",
        frame_id="f",
    )
    assert result is not None
    assert result["scene_objects"] == []


def test_confirmed_target_is_also_a_scene_object(tmp_path):
    """PRESENT 的目标必须同时进 scene_objects，否则对象拓扑里没有目标节点。"""
    from types import SimpleNamespace

    from PIL import Image

    from app.detectors.siliconflow_vision_worker import _quick_detect

    image_path = tmp_path / "frame.jpg"
    Image.new("RGB", (64, 48), (10, 120, 10)).save(image_path)

    content = (
        '{"target_state": "PRESENT", "found": true, "bbox_2d": [0.4, 0.3, 0.6, 0.7],'
        ' "confidence": 0.88, "name_zh": "绿色网格状垃圾桶", "reason_zh": "绿色网格桶",'
        ' "scene_objects_light": [{"name_zh": "办公椅", "confidence": 0.9,'
        ' "bbox_2d": [0.1, 0.2, 0.3, 0.8], "category": "furniture"}]}'
    )

    def _create(**_kwargs):
        message = SimpleNamespace(content=content)
        return SimpleNamespace(choices=[SimpleNamespace(message=message)])

    client = SimpleNamespace(
        chat=SimpleNamespace(completions=SimpleNamespace(create=_create))
    )
    settings = SimpleNamespace(
        siliconflow_api_key="k",
        siliconflow_base_url="http://local",
        siliconflow_timeout_seconds=1,
        vision_model="m",
        image_max_side=64,
        image_detail="low",
        vlm_runtime_quick_max_tokens=256,
    )

    result = _quick_detect(settings, str(image_path), "绿色垃圾桶", "", client=client)

    assert result["target_state"] == "PRESENT"
    target_entry = result["scene_objects"][0]
    assert "绿色垃圾桶" in target_entry["label"]
    assert target_entry["bbox_2d"] == [0.4, 0.3, 0.6, 0.7]
    assert target_entry["category"] == "target"
    assert "办公椅" in [item["label"] for item in result["scene_objects"]]
    # 兼容契约不变：objects 仍然只含目标
    assert [item["label"] for item in result["objects"]] == [target_entry["label"]]


def _quick_settings():
    from types import SimpleNamespace

    return SimpleNamespace(
        siliconflow_api_key="k",
        siliconflow_base_url="http://local",
        siliconflow_timeout_seconds=1,
        vision_model="m",
        image_max_side=64,
        image_detail="low",
        vlm_runtime_quick_max_tokens=256,
    )


def _quick_client(content: str):
    from types import SimpleNamespace

    def _create(**_kwargs):
        message = SimpleNamespace(content=content)
        return SimpleNamespace(choices=[SimpleNamespace(message=message)])

    return SimpleNamespace(
        chat=SimpleNamespace(completions=SimpleNamespace(create=_create))
    )


def test_possible_target_with_box_is_also_a_scene_object(tmp_path):
    """POSSIBLE 也要进 scene_objects：verify 确认的目标常常来自 POSSIBLE 帧。

    实测 search_20260902_213913 quick 判 POSSIBLE、verification 确认成
    TARGET_FOUND，只覆盖 PRESENT 时 target_found.object_id 仍是 null。
    """
    from PIL import Image

    from app.detectors.siliconflow_vision_worker import _quick_detect

    image_path = tmp_path / "frame.jpg"
    Image.new("RGB", (64, 48), (10, 120, 10)).save(image_path)
    content = (
        '{"target_state": "POSSIBLE", "found": false, "bbox_2d": [0.4, 0.3, 0.6, 0.7],'
        ' "confidence": 0.55, "name_zh": "浅绿色网格垃圾桶", "reason_zh": "疑似",'
        ' "scene_objects_light": [{"name_zh": "办公椅", "confidence": 0.9,'
        ' "bbox_2d": [0.1, 0.2, 0.3, 0.8], "category": "furniture"}]}'
    )

    result = _quick_detect(
        _quick_settings(), str(image_path), "绿色垃圾桶", "",
        client=_quick_client(content))

    assert result["target_state"] == "POSSIBLE"
    target_entry = result["scene_objects"][0]
    assert target_entry["category"] == "target"
    assert target_entry["frame_object_id"] == "target_obj_001"
    assert "绿色垃圾桶" in target_entry["label"]


def test_possible_target_without_box_stays_out_of_topology(tmp_path):
    """无框候选没有几何位置，不进对象拓扑。"""
    from PIL import Image

    from app.detectors.siliconflow_vision_worker import _quick_detect

    image_path = tmp_path / "frame.jpg"
    Image.new("RGB", (64, 48), (10, 120, 10)).save(image_path)
    content = (
        '{"target_state": "POSSIBLE", "found": false, "confidence": 0.4,'
        ' "name_zh": "疑似垃圾桶", "reason_zh": "看不清",'
        ' "scene_objects_light": [{"name_zh": "办公椅", "confidence": 0.9,'
        ' "bbox_2d": [0.1, 0.2, 0.3, 0.8], "category": "furniture"}]}'
    )

    result = _quick_detect(
        _quick_settings(), str(image_path), "绿色垃圾桶", "",
        client=_quick_client(content))

    assert result["target_state"] == "POSSIBLE"
    assert [item["category"] for item in result["scene_objects"]] == ["furniture"]
