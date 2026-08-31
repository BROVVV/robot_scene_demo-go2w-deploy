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
