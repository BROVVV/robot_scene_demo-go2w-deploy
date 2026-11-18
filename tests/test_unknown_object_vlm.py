from __future__ import annotations

import time
from types import SimpleNamespace

from app.perception.unknown_object_vlm import (
    AsyncUnknownObjectLabeler,
    UnknownObjectVLMConfig,
    accepted_chinese_name,
    UnknownLabelResult,
)


def test_accept_name_requires_confirmed_specific_chinese_label() -> None:
    assert accepted_chinese_name(
        {"is_target": True, "object_name_zh": "工业风扇", "confidence": 0.88},
        min_confidence=0.65,
    )[0] == "工业风扇"
    assert accepted_chinese_name(
        {"is_target": True, "object_name_zh": "物体", "confidence": 0.99},
        min_confidence=0.65,
    )[0] is None
    assert accepted_chinese_name(
        {"is_target": True, "object_name_zh": "工业风扇", "confidence": 0.4},
        min_confidence=0.65,
    )[0] is None
    assert accepted_chinese_name(
        {"is_target": False, "object_name_zh": "工业风扇", "confidence": 0.99},
        min_confidence=0.65,
    )[0] is None


def test_async_labeler_deduplicates_and_records_current_instance(tmp_path) -> None:
    calls = []

    def verify(request):
        calls.append(request.raw_label)
        return {
            "is_target": True,
            "object_name_zh": "工业风扇",
            "confidence": 0.91,
            "reason_zh": "框内是带叶片的风扇",
            "provider": "test",
        }

    config = UnknownObjectVLMConfig(
        max_pending=4,
        max_calls_per_minute=60,
        retry_cooldown_s=0,
    )
    cache = tmp_path / "labels.json"
    labeler = AsyncUnknownObjectLabeler(verify, config, cache_path=cache)
    try:
        assert labeler.submit(
            review_key="teleop_0:epoch_a:obj_001",
            raw_label="Industrial_Fan",
            frame_id="f1",
            object_id="obj_001",
            image_path=str(tmp_path / "frame.jpg"),
            bbox=[0.1, 0.2, 0.6, 0.8],
        )
        assert not labeler.submit(
            review_key="teleop_0:epoch_a:obj_001",
            raw_label="industrial fan",
            frame_id="f2",
            object_id="obj_002",
            image_path=str(tmp_path / "frame2.jpg"),
            bbox=[0.1, 0.2, 0.6, 0.8],
        )
        deadline = time.monotonic() + 2.0
        completed = []
        while time.monotonic() < deadline and not completed:
            completed = labeler.poll_completed()
            time.sleep(0.01)
        assert len(completed) == 1
        assert completed[0].accepted
        assert completed[0].label_zh == "工业风扇"
        assert calls == ["industrial fan"]
        assert labeler.cached_object_label(
            "teleop_0:epoch_a:obj_001"
        ) == "工业风扇"
        assert cache.is_file()
    finally:
        labeler.close()

    restored = AsyncUnknownObjectLabeler(verify, config, cache_path=cache)
    try:
        # A prior visual correction is not a global class dictionary.  A new
        # mapper process/rebuilt object identity must be reviewed independently.
        assert restored.cached_object_label("teleop_0:epoch_a:obj_001") is None
        assert restored.needs_review("teleop_0:epoch_a:obj_001")
    finally:
        restored.close()


def test_unresolved_answer_is_not_cached(tmp_path) -> None:
    def verify(_request):
        return {"is_target": True, "object_name_zh": "无法判断", "confidence": 0.95}

    config = UnknownObjectVLMConfig(
        max_calls_per_minute=60,
        retry_cooldown_s=0,
    )
    labeler = AsyncUnknownObjectLabeler(
        verify, config, cache_path=tmp_path / "labels.json"
    )
    try:
        assert labeler.submit(
            review_key="teleop_0:epoch_a:obj_001",
            raw_label="mystery fixture",
            frame_id="f1",
            object_id="obj_001",
            image_path=str(tmp_path / "frame.jpg"),
            bbox=[0.1, 0.1, 0.9, 0.9],
        )
        deadline = time.monotonic() + 2.0
        completed = []
        while time.monotonic() < deadline and not completed:
            completed = labeler.poll_completed()
            time.sleep(0.01)
        assert len(completed) == 1
        assert not completed[0].accepted
        assert labeler.cached_object_label("teleop_0:epoch_a:obj_001") is None
    finally:
        labeler.close()


def test_completed_result_relabels_existing_semantic_node() -> None:
    from app.detectors.vocabulary import LABEL_ZH_FALLBACK
    from scripts.go2w.remote_control_semantic_mapping_worker import (
        RemoteControlSemanticMapper,
    )

    result = UnknownLabelResult(
        review_key="teleop_0:epoch_a:obj_001",
        raw_label="industrial fan",
        frame_id="f10",
        object_id="obj_001",
        label_zh="工业风扇",
        confidence=0.9,
        reason_zh="可见风扇叶片",
        provider="test",
        latency_ms=10.0,
    )

    class FakeLabeler:
        def poll_completed(self):
            return [result]

        def stats(self):
            return {"pending": 0, "queued": 0, "cache_entries": 1}

    entry = SimpleNamespace(
        object_id="obj_001",
        label="industrial fan",
        label_zh=LABEL_ZH_FALLBACK,
        provenance={},
    )
    mapper = RemoteControlSemanticMapper.__new__(RemoteControlSemanticMapper)
    mapper._unknown_labeler = FakeLabeler()
    mapper._unknown_label_reviewed = 0
    mapper._unknown_label_relabeled = 0
    mapper._unknown_label_unresolved = 0
    mapper._semantic_map = SimpleNamespace(objects={"obj_001": entry}, revision=0)
    mapper._object_revision = 4
    mapper._session_id = 0
    mapper._map_epoch = "epoch_a"
    mapper._current_detections = [{
        "source_object_id": "gsam_obj_001",
        "persistent_object_id": "obj_001",
        "label": "industrial fan",
        "label_zh": "待VLM复核 · industrial fan",
        "label_review_state": "pending",
    }]
    mapper._last_current_latency = 0.0
    events = []
    published = []
    mapper._write_event = events.append
    mapper._host_s = lambda: 1.0
    mapper._publish_display_objects = lambda revision: published.append(("objects", revision))
    mapper._publish_topology = lambda detections, latency: published.append(("topology", latency))
    mapper._publish_status = lambda **updates: published.append(("status", updates))

    mapper._apply_unknown_label_results()

    assert entry.label == "industrial fan"  # association identity remains stable
    assert entry.label_zh == "工业风扇"
    assert entry.provenance["label_source"] == "vlm_background_review"
    assert mapper._current_detections[0]["label_zh"] == "工业风扇"
    assert mapper._current_detections[0]["label_review_state"] == "resolved"
    assert mapper._current_detections[0]["label_source"] == "vlm_background_review"
    assert mapper._semantic_map.revision == 1
    assert mapper._object_revision == 5
    assert events[0]["event"] == "semantic_object_vlm_relabeled"
    assert ("objects", 5) in published


def test_live_box_binds_persistent_id_and_reuses_reviewed_name() -> None:
    from scripts.go2w.remote_control_semantic_mapping_worker import (
        RemoteControlSemanticMapper,
    )

    entry = SimpleNamespace(
        object_id="obj_007",
        label="rottweiler",
        label_zh="纸箱",
        provenance={
            "label_source": "vlm_background_review",
            "label_vlm_confidence": 0.95,
        },
    )
    mapper = RemoteControlSemanticMapper.__new__(RemoteControlSemanticMapper)
    mapper._unknown_labeler = object()
    mapper._semantic_map = SimpleNamespace(objects={"obj_007": entry})
    current = [{
        "source_object_id": "gsam_obj_001",
        "label": "rottweiler",
        "label_zh": "未分类物体",
    }]
    associations = {
        "gsam_obj_001": SimpleNamespace(persistent_object_id="obj_007")
    }

    mapper._bind_current_detection_labels(current, associations)

    assert current[0]["persistent_object_id"] == "obj_007"
    assert current[0]["label_zh"] == "纸箱"
    assert current[0]["label_review_state"] == "resolved"
    assert current[0]["label_source"] == "vlm_background_review"
    assert current[0]["label_vlm_confidence"] == 0.95


def test_live_unknown_box_exposes_pending_review_instead_of_generic_fallback() -> None:
    from app.detectors.vocabulary import LABEL_ZH_FALLBACK
    from scripts.go2w.remote_control_semantic_mapping_worker import (
        RemoteControlSemanticMapper,
    )

    entry = SimpleNamespace(
        object_id="obj_008",
        label="veterinarians office",
        label_zh=LABEL_ZH_FALLBACK,
        provenance={},
    )
    mapper = RemoteControlSemanticMapper.__new__(RemoteControlSemanticMapper)
    mapper._unknown_labeler = object()
    mapper._semantic_map = SimpleNamespace(objects={"obj_008": entry})
    current = [{
        "source_object_id": "gsam_obj_002",
        "label": "veterinarians office",
        "label_zh": LABEL_ZH_FALLBACK,
    }]
    associations = {
        "gsam_obj_002": SimpleNamespace(persistent_object_id="obj_008")
    }

    mapper._bind_current_detection_labels(current, associations)

    assert current[0]["persistent_object_id"] == "obj_008"
    assert current[0]["label_zh"] == "待VLM复核 · veterinarians office"
    assert current[0]["raw_label"] == "veterinarians office"
    assert current[0]["label_review_state"] == "pending"
