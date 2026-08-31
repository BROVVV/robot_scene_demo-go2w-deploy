"""Tests for AsyncSemanticObservationManager coalescing / stale protection."""

from __future__ import annotations

import threading
import time

from app.live_robot.async_semantic_observer import AsyncSemanticObservationManager
from app.live_robot.semantic_observer import SemanticObservation


def _semantic(frame_id: str, timestamp: float, x: float = 0.0) -> SemanticObservation:
    return SemanticObservation(
        frame_id=frame_id,
        timestamp_sec=timestamp,
        robot_pose={"x": x, "y": 0.0, "yaw_deg": 0.0},
        objects=[],
        relations=[],
        source="test",
        stale=False,
        heading_sector=0,
    )


def test_latest_wins_when_old_result_arrives_late():
    manager = AsyncSemanticObservationManager(lambda **kwargs: {}, enabled=False)
    manager.seed(_semantic("frame_10", 10.0, x=0.0))
    # Simulate applying results manually: newer first, then old should be ignored.
    manager._latest = _semantic("frame_11", 11.0, x=1.0)
    manager._latest_sequence = 11
    # Creating a request with sequence 10 (older).
    from app.live_robot.async_semantic_observer import AsyncSemanticRequest

    old_request = AsyncSemanticRequest(
        sequence=10, image_path="img", frame_id="frame_10",
        capture_timestamp=10.0, robot_pose=None, target_profile=None,
    )
    manager._on_result(old_request, {
        "frame_id": "frame_10",
        "scene_objects": [],
        "scene_relations": [],
        "scene_summary_zh": "old",
    })
    assert manager.get_latest_completed().frame_id == "frame_11"


def test_coalescing_keeps_only_latest_pending():
    manager = AsyncSemanticObservationManager(lambda **kwargs: {"ok": True}, enabled=True)
    manager.seed(_semantic("frame_1", 1.0))
    # Force inflight and check pending coalescing.
    manager._inflight = 1
    submitted = manager.submit_if_needed(
        image_path="f1",
        frame_id="frame_2",
        capture_timestamp=2.0,
        robot_pose={"x": 0.1, "y": 0.0, "yaw_deg": 0.0},
        target_profile=None,
    )
    assert submitted is False
    assert manager._pending is not None
    assert manager._pending.frame_id == "frame_2"


def test_pose_at_capture_is_preserved():
    manager = AsyncSemanticObservationManager(lambda **kwargs: {}, enabled=False)
    # A result must use the request capture pose, not a later pose.
    request_pose = {"x": 1.0, "y": 2.0, "yaw_deg": 30.0}
    from app.live_robot.async_semantic_observer import AsyncSemanticRequest

    request = AsyncSemanticRequest(
        sequence=1, image_path="img", frame_id="f",
        capture_timestamp=5.0, robot_pose=request_pose, target_profile=None,
    )
    manager._on_result(request, {
        "frame_id": "f",
        "scene_objects": [],
        "scene_relations": [],
        "scene_summary_zh": "ok",
    })
    latest = manager.get_latest_completed()
    assert latest is not None
    assert latest.robot_pose == request_pose
