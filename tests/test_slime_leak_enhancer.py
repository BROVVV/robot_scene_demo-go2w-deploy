from pathlib import Path

import cv2
import numpy as np

from app.detectors.base import DetectedObject
from app.perception.depth_object_localizer import decode_mask_runs
from app.perception.slime_leak_enhancer import (
    _encode_mask_runs,
    enhance_slime_leak_detections,
    select_low_confidence_pipe_contexts,
)
from app.spatial.hazard_rules import HazardMonitor


def _write_demo_frame(path: Path) -> None:
    image = np.full((200, 300, 3), 55, dtype=np.uint8)
    # Neutral pipe with a fluorescent green patch crossing its surface.
    cv2.rectangle(image, (35, 85), (265, 120), (145, 145, 145), thickness=-1)
    cv2.rectangle(image, (130, 78), (175, 126), (20, 240, 30), thickness=-1)
    assert cv2.imwrite(str(path), image)


def _pipe() -> DetectedObject:
    mask = np.zeros((200, 300), dtype=bool)
    mask[85:121, 35:266] = True
    return DetectedObject(
        label="pipe",
        label_zh="管道",
        category="structure",
        bbox_2d=(0.10, 0.40, 0.90, 0.63),
        score=0.82,
        mask_rle=_encode_mask_runs(mask),
        mask_size=(200, 300),
        mask_source="sam2_mask",
        source_prompt_term="pipe",
    )


def test_fluorescent_patch_on_pipe_becomes_mask_backed_hazard(tmp_path: Path) -> None:
    image_path = tmp_path / "slime.jpg"
    _write_demo_frame(image_path)

    result = enhance_slime_leak_detections(
        str(image_path), [_pipe()], {"enabled": True}
    )

    assert len(result) == 2
    leak = result[1]
    assert leak.label == "simulated liquid leak"
    assert leak.label_zh == "模拟泄漏危险信号"
    assert leak.source == "hsv_slime_cue"
    assert "semantic_context=pipe_surface_mask" in leak.attributes
    assert leak.mask_source == "hsv_slime_mask"
    assert leak.context_mask_rle
    mask = decode_mask_runs(leak.mask_rle or [], *(leak.mask_size or (0, 0)))
    assert mask.shape == (200, 300)
    assert int(mask.sum()) > 1_000

    monitor = HazardMonitor({
        "enabled": True,
        "confirm_observations": 2,
        "rules": [{
            "id": "demo_slime_leak",
            "labels": ["simulated liquid leak"],
            "display_name": "疑似管道液体泄漏",
        }],
    })
    event, is_new = monitor.observe(
        leak, track_id="track_1", frame_id="frame_1", timestamp=1.0,
        has_position=True,
    )
    assert is_new is True
    assert event is not None and event["status"] == "suspected"
    event, is_new = monitor.observe(
        leak, track_id="track_1", frame_id="frame_2", timestamp=2.0,
        has_position=True,
    )
    assert is_new is False
    assert event is not None and event["status"] == "confirmed"


def test_yoloe_route_uses_english_hazard_display_name(tmp_path: Path) -> None:
    image_path = tmp_path / "slime_en.jpg"
    _write_demo_frame(image_path)

    result = enhance_slime_leak_detections(
        str(image_path), [_pipe()], {"enabled": True}, display_language="en"
    )

    assert result[1].label == "simulated liquid leak"
    assert result[1].label_zh == "simulated liquid leak"


def test_pipe_context_rejects_separate_green_object(tmp_path: Path) -> None:
    image_path = tmp_path / "separate.jpg"
    image = np.full((200, 300, 3), 55, dtype=np.uint8)
    cv2.rectangle(image, (35, 85), (265, 120), (145, 145, 145), thickness=-1)
    cv2.rectangle(image, (5, 5), (45, 45), (20, 240, 30), thickness=-1)
    assert cv2.imwrite(str(image_path), image)

    result = enhance_slime_leak_detections(
        str(image_path), [_pipe()], {"enabled": True}
    )

    assert [item.label for item in result] == ["pipe"]


def test_colour_without_pipe_or_leak_prompt_does_not_trigger(tmp_path: Path) -> None:
    image_path = tmp_path / "green_object.jpg"
    image = np.full((200, 300, 3), 55, dtype=np.uint8)
    cv2.rectangle(image, (110, 70), (180, 135), (20, 240, 30), thickness=-1)
    assert cv2.imwrite(str(image_path), image)

    result = enhance_slime_leak_detections(
        str(image_path), [], {"enabled": True}
    )

    assert len(result) == 1
    assert result[0].label == "fluorescent colour review region"
    assert result[0].category == "review"
    assert result[0].hazard_state == "suspected"


def test_grounded_leak_without_pipe_does_not_trigger(tmp_path: Path) -> None:
    image_path = tmp_path / "model_false_positive.jpg"
    _write_demo_frame(image_path)
    grounded = DetectedObject(
        label="fluid spill",
        label_zh="液体泄漏",
        category="hazard",
        bbox_2d=(0.40, 0.35, 0.62, 0.68),
        score=0.77,
        source_prompt_term="fluid spill",
    )

    result = enhance_slime_leak_detections(
        str(image_path), [grounded], {"enabled": True}
    )

    assert [item.label for item in result] == ["simulated liquid leak candidate"]
    assert result[0].hazard_state == "suspected"


def test_overlapping_grounded_slime_is_verified_without_duplicate(tmp_path: Path) -> None:
    image_path = tmp_path / "slime.jpg"
    _write_demo_frame(image_path)
    grounded = DetectedObject(
        label="green slime",
        label_zh="绿色史莱姆泥",
        category="hazard",
        bbox_2d=(0.42, 0.37, 0.60, 0.65),
        score=0.78,
        source_prompt_term="green slime",
    )

    result = enhance_slime_leak_detections(
        str(image_path), [_pipe(), grounded], {"enabled": True}
    )

    assert len(result) == 2
    verified = result[1]
    assert verified.label == "simulated liquid leak"
    assert verified.source == "grounded_sam+hsv_slime_cue"
    assert verified.mask_rle


def test_green_inside_pipe_bbox_but_outside_surface_stays_suspected(tmp_path: Path) -> None:
    image_path = tmp_path / "plant_in_box.jpg"
    image = np.full((200, 300, 3), 55, dtype=np.uint8)
    cv2.rectangle(image, (35, 85), (265, 120), (145, 145, 145), thickness=-1)
    cv2.rectangle(image, (60, 55), (90, 80), (20, 240, 30), thickness=-1)
    assert cv2.imwrite(str(image_path), image)

    result = enhance_slime_leak_detections(str(image_path), [_pipe()], {"enabled": True})

    assert len(result) == 2
    assert result[1].hazard_state == "suspected"
    assert result[1].context_quality == "pipe_bbox_weak"


def test_small_surface_patch_survives_without_fixed_five_pixel_open(tmp_path: Path) -> None:
    image_path = tmp_path / "small.jpg"
    image = np.full((200, 300, 3), 55, dtype=np.uint8)
    cv2.rectangle(image, (35, 85), (265, 120), (145, 145, 145), thickness=-1)
    cv2.rectangle(image, (140, 96), (145, 101), (20, 240, 30), thickness=-1)
    assert cv2.imwrite(str(image_path), image)

    result = enhance_slime_leak_detections(
        str(image_path), [_pipe()],
        {"enabled": True, "min_valid_pixels": 8, "min_roi_area_ratio": 0.0005},
    )

    assert len(result) == 2
    assert result[1].hazard_state == "confirmed_demo_candidate"


def test_low_score_yoloe_pipe_alias_is_context_only_but_can_confirm_cue(
    tmp_path: Path,
) -> None:
    image_path = tmp_path / "prompt_free_pipe_bowl.jpg"
    _write_demo_frame(image_path)
    pipe = _pipe()
    pipe = DetectedObject(
        **{
            **pipe.__dict__,
            "label": "pipe bowl",
            "label_zh": "pipe bowl",
            "source_prompt_term": "pipe bowl",
            "score": 0.134,
        }
    )
    config = {
        "enabled": True,
        "pipe_labels": ["pipe", "pipe bowl"],
        "min_pipe_context_confidence": 0.10,
        "max_pipe_context_bbox_area_ratio": 0.25,
    }

    context = select_low_confidence_pipe_contexts(
        [pipe], config, persistent_min_confidence=0.35
    )
    assert context == [pipe]

    result = enhance_slime_leak_detections(
        str(image_path), [], config, context_detections=context
    )

    # The low-confidence alias is not returned/persisted, while its measured
    # mask can still verify the fluorescent cue on the physical pipe surface.
    assert [item.label for item in result] == ["simulated liquid leak"]
    assert result[0].hazard_state == "confirmed_demo_candidate"
    assert result[0].host_detection_index is None


def test_confirmation_uses_recent_unique_source_frames() -> None:
    monitor = HazardMonitor({
        "enabled": True,
        "confirm_observations": 2,
        "evidence_window_frames": 3,
        "evidence_window_s": 12,
        "rules": [{"id": "demo", "labels": ["simulated liquid leak"]}],
    })
    leak = DetectedObject(
        label="simulated liquid leak", label_zh="模拟泄漏危险信号",
        category="hazard", bbox_2d=(0.1, 0.1, 0.2, 0.2), score=0.8,
        hazard_state="confirmed_demo_candidate",
    )
    first, _ = monitor.observe(
        leak, track_id="t", frame_id="f1", timestamp=1.0, has_position=False
    )
    duplicate, _ = monitor.observe(
        leak, track_id="t", frame_id="f1", timestamp=1.0, has_position=False
    )
    assert first is duplicate and duplicate["status"] == "suspected"
    late, _ = monitor.observe(
        leak, track_id="t", frame_id="f2", timestamp=30.0, has_position=False
    )
    assert late["status"] == "suspected"
    confirmed, _ = monitor.observe(
        leak, track_id="t", frame_id="f3", timestamp=31.0, has_position=False
    )
    assert confirmed["status"] == "confirmed"


def test_same_pipe_separated_anchors_do_not_merge() -> None:
    monitor = HazardMonitor({
        "enabled": True, "host_anchor_distance": 0.15,
        "rules": [{"id": "demo", "labels": ["simulated liquid leak"]}],
    })
    leak = DetectedObject(
        label="simulated liquid leak", label_zh="模拟泄漏危险信号",
        category="hazard", bbox_2d=(0.1, 0.1, 0.2, 0.2), score=0.8,
        hazard_state="confirmed_demo_candidate",
    )
    monitor.observe(leak, track_id="a", frame_id="f1", timestamp=1.0,
                    has_position=False, host_local_anchor=[0.2, 0.5])
    monitor.observe(leak, track_id="b", frame_id="f1", timestamp=1.0,
                    has_position=False, host_local_anchor=[0.8, 0.5])
    monitor.bind_host(track_id="a", host_persistent_object_id="pipe_1")
    monitor.bind_host(track_id="b", host_persistent_object_id="pipe_1")
    assert len({event["hazard_event_id"] for event in monitor.events.values()}) == 2
