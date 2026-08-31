"""quick 目标否定不再丢物体：保证“没找到目标”的帧也能持续建物体列表/拓扑。

语义观察器之前只要目标检测说“不存在”就直接返回 scene_objects=[]，导致
搜白色垃圾桶时整场 0 物体、WebUI 始终不建图。修复后：
  * quick 否定带物体 -> 透传物体（继续建图）
  * quick 否定无物体 -> 返回 None，落到全场景分析列出所有可见物体
"""

from __future__ import annotations

from app.live_robot.semantic_observer import semantic_payload_from_quick_target_absence


def test_absent_with_objects_keeps_them():
    payload = semantic_payload_from_quick_target_absence(
        {
            "scene_summary_zh": "有办公椅和背包，没有白色垃圾桶",
            "objects": [{"label": "办公椅", "bbox_2d": [0.1, 0.1, 0.3, 0.4]}],
            "target_decision": {"is_present": False, "confidence": 0.9},
        },
        image_path="img", frame_id="f",
    )
    assert payload is not None
    assert len(payload["scene_objects"]) == 1
    assert payload["scene_objects"][0]["label"] == "办公椅"


def test_absent_without_objects_falls_back_to_full_scene():
    payload = semantic_payload_from_quick_target_absence(
        {
            "scene_summary_zh": "没有白色垃圾桶",
            "target_decision": {"is_present": False},
        },
        image_path="img", frame_id="f",
    )
    assert payload is None  # -> full scene 分析列出所有可见物体


def test_ambiguous_or_present_never_short_circuits():
    assert semantic_payload_from_quick_target_absence(
        {"target_decision": {"is_present": True}}, image_path="i", frame_id="f"
    ) is None
    assert semantic_payload_from_quick_target_absence(
        {}, image_path="i", frame_id="f"
    ) is None
