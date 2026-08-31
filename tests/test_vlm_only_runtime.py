"""Regression: VLM-only runtime must not launch GroundingDINO/SAM2."""

from __future__ import annotations

from app.detectors.siliconflow_vision_worker import quick_target_present


def test_quick_target_present_ignores_scene_objects():
    payload = {
        "objects": [],
        "scene_objects": [{"label": "chair", "score": 0.99}],
        "target_decision": {"is_present": False},
    }
    assert quick_target_present(payload) is False


def test_quick_target_present_requires_decision_true():
    payload = {
        "objects": [{"label": "bin", "score": 0.9}],
        "target_decision": {"is_present": False},
    }
    assert quick_target_present(payload) is False


def test_no_grounded_sam_argv_in_vlm_only_path():
    # The production path is VLM-only; guard against accidental Grounded-SAM
    # worker references in the main LLM detector method.
    import inspect
    from app.detectors.siliconflow_vision_worker import _QUICK_SYSTEM_PROMPT

    assert "grounded_sam_worker" not in _QUICK_SYSTEM_PROMPT
    assert "GroundingDINO" not in _QUICK_SYSTEM_PROMPT
    assert "SAM2" not in _QUICK_SYSTEM_PROMPT
