from pathlib import Path
from types import SimpleNamespace
import threading

import cv2
import numpy as np

from app.detectors.base import DetectedObject
from app.perception.passive_target_watch import TargetWatchConfig, deterministic_spec
from app.perception.pipe_context_tracker import PipeContextTracker
from app.perception.slime_leak_enhancer import _encode_mask_runs
from scripts.go2w.remote_control_semantic_mapping_worker import (
    RemoteControlSemanticMapper,
)


ROOT = Path(__file__).resolve().parents[1]
WORKER = ROOT / "scripts/go2w/remote_control_semantic_mapping_worker.py"


def test_mapping_worker_keeps_auto_hazard_parallel_to_operator_target():
    source = WORKER.read_text(encoding="utf-8")

    assert "active_verification_specs(" in source
    assert '"verification_spec": candidate_spec.to_dict()' in source
    assert '"automatic_hazard": bool(automatic_hazard)' in source
    assert 'env.get("automatic_hazard")' in source
    assert "not automatic_hazard and \"target\" in slots" in source


def test_auto_hazard_candidate_uses_existing_bounded_verifier():
    source = WORKER.read_text(encoding="utf-8")

    assert "self._target_watch_cooldown.permits(" in source
    assert "self._target_watch_budget.permits()" in source
    assert "self._target_watch_verifier.submit(" in source
    assert '"profile": spec.to_dict()' in source


class _Detector:
    prompt_free = True
    last_diagnostics = {"prompt_mode": "prompt_free"}

    def __init__(self, detections):
        self.detections = detections

    def detect_with_dynamic_terms(self, *_args, **_kwargs):
        return list(self.detections)


class _Scheduler:
    priority = ["pipe", "green slime"]
    max_terms = 8
    last_group_index = 0

    def next_terms(self, _profile, *, extra_priority):
        return [*extra_priority, "pipe", "green slime"]


class _Hazards:
    @staticmethod
    def prompt_terms():
        return ["green slime", "slime", "liquid leak", "fluid spill"]


def _detect_slime_batch(tmp_path, *, operator_spec=None):
    image_path = tmp_path / "slime_pipe.jpg"
    image = np.full((200, 300, 3), 55, dtype=np.uint8)
    cv2.rectangle(image, (35, 85), (265, 120), (145, 145, 145), -1)
    cv2.rectangle(image, (130, 78), (175, 126), (20, 240, 30), -1)
    assert cv2.imwrite(str(image_path), image)
    mask = np.zeros((200, 300), dtype=bool)
    mask[85:121, 35:266] = True
    pipe = DetectedObject(
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

    mapper = RemoteControlSemanticMapper.__new__(RemoteControlSemanticMapper)
    mapper.mapping_config = {
        "scene_profile": "industrial_park",
        "perception": {
            "min_detection_confidence": 0.35,
            "max_bbox_area_ratio": 0.65,
        },
        "hazard": {"visual_enhancement": {"enabled": True}},
    }
    mapper._target_watch_config = TargetWatchConfig.from_mapping(
        {"auto_hazard": {"enabled": True}}
    )
    mapper._target_watch_spec = operator_spec
    mapper._target_watch_generation = 7 if operator_spec is not None else 0
    mapper._session_id = 0
    mapper._target_watch_lock = threading.RLock()
    mapper._detect_lock = threading.Lock()
    mapper._detector = _Detector([pipe])
    mapper._term_scheduler = _Scheduler()
    mapper._hazards = _Hazards()
    mapper._pipe_tracker = PipeContextTracker(max_age_s=8.0, min_quality=0.65)
    mapper.detector_backend = "yoloe_seg"
    mapper._sync_target_watch_request = lambda: None
    mapper._ram_scene_terms = lambda _path: []
    mapper._publish_status = lambda **_updates: None
    mapper._write_event = lambda _event: None
    mapper._target_stat = lambda _key, _delta=1: None
    mapper._publish_target_watch_state = lambda **_updates: None

    return mapper._detect_batch(SimpleNamespace(
        color_ref=str(image_path), frame_id="frame_001", timestamp=1.0
    ))


def test_detected_slime_starts_auto_candidate_with_empty_target(tmp_path):
    batch = _detect_slime_batch(tmp_path)

    automatic = [
        item for item in batch.target_candidates if item["automatic_hazard"]
    ]
    assert automatic
    assert automatic[0]["detection"].label == "simulated liquid leak"
    assert automatic[0]["verification_spec"]["verification_mode"] == "diffuse_hazard"
    assert automatic[0]["target_generation"] == 0


def test_unrelated_operator_target_does_not_disable_auto_slime_candidate(tmp_path):
    batch = _detect_slime_batch(
        tmp_path, operator_spec=deterministic_spec("黑椅子")
    )

    assert batch.target_candidates[0]["automatic_hazard"] is True
    assert batch.target_candidates[0]["detection"].label == "simulated liquid leak"
    assert batch.target_candidates[0]["target_generation"] == 7
