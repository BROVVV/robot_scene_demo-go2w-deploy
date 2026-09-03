"""计划书 §17.1/§17.2/§17.3：语义 timeout 不得缓存为成功、轻量场景 fallback、
frame binding 元信息与 depth-frame 解析。

不变量 1（状态不能撒谎）：timeout ≠ empty scene，stale ≠ current frame，
pending ≠ no objects。
"""

from __future__ import annotations

import time

import pytest

from app.live_robot.async_semantic_observer import (
    AsyncSemanticObservationManager,
)
from app.live_robot.autonomous_explorer import PerceptionFailure
from app.live_robot.semantic_observer import (
    SEMANTIC_STATUS_FRESH_FULL,
    SEMANTIC_STATUS_FRESH_QUICK,
    SEMANTIC_STATUS_TIMEOUT,
    SemanticObservation,
    _from_payload,
    semantic_observation_to_live,
    semantic_payload_from_quick_target_absence,
)
from app.perception.depth_object_localizer import resolve_depth_frame
from app.perception.rgbd_source import RGBDFrame


def _frame(frame_id: str = "100") -> RGBDFrame:
    return RGBDFrame(
        frame_id=frame_id,
        timestamp=time.time(),
        color_ref=f"color_{frame_id}.jpg",
        depth_ref=f"depth_{frame_id}.png",
        width=640, height=480, fx=600.0, fy=600.0, cx=320.0, cy=240.0,
    )


def test_full_semantic_timeout_never_caches_as_success():
    """§17.1：Full Semantic 永久超时 -> latest_success 不被覆盖。"""
    events: list[dict] = []

    def analyze(*args, **kwargs):
        raise PerceptionFailure(
            "FULL_SEMANTIC_TIMEOUT: exceeded",
            code="FULL_SEMANTIC_TIMEOUT",
            recoverable=True,
        )

    manager = AsyncSemanticObservationManager(
        analyze, enabled=True, event_sink=events.append, now=time.time,
    )
    # 首帧也允许后台提交（计划书 §3.4：不再首帧同步阻塞）。
    submitted = manager.submit_if_needed(
        image_path="frame.jpg", frame_id="42",
        capture_timestamp=time.time(), robot_pose=None,
        target_profile=object(),
    )
    assert submitted is True
    deadline = time.time() + 3.0
    while manager.has_inflight() and time.time() < deadline:
        time.sleep(0.02)
    assert manager.get_latest_completed() is None
    assert manager.last_error_code() == "FULL_SEMANTIC_TIMEOUT"
    assert any(event.get("event") == "semantic_timeout" for event in events)
    # 没有成功结果，就不能把空场景当“确认无物体”使用。
    assert not [e for e in events if e.get("event") == "semantic_result_applied"]


def test_first_frame_submits_to_background_single_flight():
    """§3.4/§3.5：第一个 Full Semantic 也在后台跑，且最多一个 in-flight。"""
    inflight: list[int] = []

    def analyze(*args, **kwargs):
        inflight.append(1)
        time.sleep(0.15)
        inflight.pop()
        return {
            "scene_objects": [{"id": "o1", "name": "chair", "name_zh": "办公椅",
                               "confidence": 0.9, "bbox_2d": [0.1, 0.1, 0.3, 0.3]}],
            "scene_relations": [],
        }

    manager = AsyncSemanticObservationManager(
        analyze, enabled=True, now=time.time,
    )
    assert manager.submit_if_needed(
        image_path="f1.jpg", frame_id="1", capture_timestamp=time.time(),
        robot_pose=None, target_profile=object(),
    ) is True
    # 第二个请求在第一个完成前到来 -> 只 coalesce，不并行。
    assert manager.submit_if_needed(
        image_path="f2.jpg", frame_id="2", capture_timestamp=time.time(),
        robot_pose=None, target_profile=object(),
    ) is False
    assert max(inflight) <= 1
    deadline = time.time() + 3.0
    while manager.has_inflight() and time.time() < deadline:
        time.sleep(0.02)
    latest = manager.get_latest_completed()
    assert latest is not None
    assert latest.semantic_status == SEMANTIC_STATUS_FRESH_FULL
    assert latest.semantic_source_frame_id == "1"


def test_quick_scene_fallback_payload_is_fresh_quick():
    """§17.2：Quick 显式无目标 + 有物体 -> fresh_quick_scene 轻量语义。"""
    payload = semantic_payload_from_quick_target_absence(
        {
            "scene_objects": [
                {"name_zh": "办公椅", "confidence": 0.91, "bbox_2d": [0.1, 0.1, 0.3, 0.3]},
                {"name_zh": "纸箱", "confidence": 0.8, "bbox_2d": [0.5, 0.1, 0.8, 0.4]},
            ],
            "scene_summary_zh": "办公室",
            "target_decision": {"is_present": False},
        },
        image_path="frame.jpg",
        frame_id="77",
    )
    assert payload is not None
    assert payload["semantic_status"] == SEMANTIC_STATUS_FRESH_QUICK
    assert payload["semantic_source_frame_id"] == "77"
    semantic = _from_payload(
        payload, robot_pose=None, sector=2, now=time.time()
    )
    assert semantic.semantic_status == SEMANTIC_STATUS_FRESH_QUICK
    assert len(semantic.objects) == 2
    live = semantic_observation_to_live(
        semantic,
        bundle_id="bundle_77",
        detections=[],
        target_present=False,
        pose={"x": 0.0, "y": 0.0, "yaw_rad": 0.0},
        navigation_heading_sector=2,
    )
    assert live.semantic_status == SEMANTIC_STATUS_FRESH_QUICK
    assert live.semantic_source_frame_id == "77"
    assert live.navigation_heading_sector == 2
    assert len(live.scene_objects) == 2


def test_resolve_depth_frame_only_uses_matching_frame():
    """§17.3：old semantic bbox 绝不配 current depth。"""
    current = _frame("200")
    cached = _frame("100")
    cache = {"100": cached}
    # 语义来自当前帧 -> 用当前帧深度。
    assert resolve_depth_frame("200", current, cache) is current
    # 语义来自缓存中的旧帧 -> 用该旧帧自己的深度。
    assert resolve_depth_frame("100", current, cache) is cached
    # 语义帧既不是当前帧也不在缓存 -> None（必须降级 SEMANTIC_2D_ONLY）。
    assert resolve_depth_frame("999", current, cache) is None
    # 语义无 frame 信息时保守返回当前帧（同帧约定）。
    assert resolve_depth_frame(None, current, cache) is current


def test_timeout_status_is_not_fresh():
    """不变量 1：timeout 状态绝不能解释为“确认空场景”。"""
    semantic = SemanticObservation(
        frame_id="9",
        timestamp_sec=time.time(),
        robot_pose=None,
        objects=[],
        relations=[],
        source="siliconflow_full_scene",
        stale=False,
        semantic_status=SEMANTIC_STATUS_TIMEOUT,
        semantic_error_code="FULL_SEMANTIC_TIMEOUT",
    )
    assert semantic.semantic_status not in {SEMANTIC_STATUS_FRESH_FULL, SEMANTIC_STATUS_FRESH_QUICK}
    assert semantic.semantic_error_code == "FULL_SEMANTIC_TIMEOUT"
