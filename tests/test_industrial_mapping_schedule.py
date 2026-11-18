from app.perception.term_rotation import RotatingTermScheduler
from app.detectors.base import DetectedObject
from app.perception.pipe_context_tracker import PipeContextTracker
from app.perception.slime_leak_enhancer import _encode_mask_runs
from scripts.go2w.remote_control_semantic_mapping_worker import (
    _merge_discovered_scene_terms,
)

import cv2
import numpy as np


def test_priority_terms_are_resident_and_scene_groups_do_not_starve() -> None:
    scheduler = RotatingTermScheduler({
        "max_terms_per_round": 5,
        "priority_terms": ["pipe", "green slime"],
        "profiles": {
            "industrial_park": {
                "groups": [
                    ["lamp post", "fire hydrant", "sign"],
                    ["storage tank", "valve", "pump"],
                    ["truck", "pallet", "container"],
                ]
            }
        },
    })

    rounds = [scheduler.next_terms("industrial_park") for _ in range(3)]

    assert all("pipe" in terms and "green slime" in terms for terms in rounds)
    ordinary = {term for terms in rounds for term in terms} - {"pipe", "green slime"}
    assert {"lamp post", "storage tank", "truck"}.issubset(ordinary)
    assert all(len(terms) <= 5 for terms in rounds)


def test_ram_scene_terms_fill_budget_before_static_fallback() -> None:
    merged = _merge_discovered_scene_terms(
        ["pipe", "green slime", "lamp post", "fire hydrant"],
        ["chair", "office chair", "pipe"],
        ["pipe", "green slime"],
        5,
    )

    assert merged == ["pipe", "green slime", "chair", "office chair", "lamp post"]


def test_pipe_context_requires_current_image_tracking_quality(tmp_path) -> None:
    first = np.zeros((120, 180, 3), dtype=np.uint8)
    for x in range(35, 145, 12):
        cv2.circle(first, (x, 60 + ((x // 12) % 3) * 4), 3, (220, 220, 220), -1)
    second = np.roll(first, 4, axis=1)
    first_path, second_path = tmp_path / "first.png", tmp_path / "second.png"
    cv2.imwrite(str(first_path), first)
    cv2.imwrite(str(second_path), second)
    mask = np.zeros((120, 180), dtype=bool)
    mask[48:78, 30:150] = True
    pipe = DetectedObject(
        label="pipe", label_zh="管道", category="structure",
        bbox_2d=(30 / 180, 48 / 120, 150 / 180, 78 / 120), score=0.9,
        mask_rle=_encode_mask_runs(mask), mask_size=(120, 180),
    )
    tracker = PipeContextTracker(max_age_s=8.0, min_quality=0.2)

    assert tracker.contexts(str(first_path), [pipe], timestamp=1.0, pipe_terms={"pipe"}) == []
    tracked = tracker.contexts(str(second_path), [], timestamp=2.0, pipe_terms={"pipe"})

    assert tracked
    assert tracked[0]["tracking_quality"] >= 0.2
    assert int(tracked[0]["surface"].sum()) > 100
