from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

from app.config import Settings
from app.detectors.yoloe_seg_subprocess import (
    YOLOESegSubprocessDetector,
    _prompt_terms_and_slots,
    _to_detected_object,
)
from app.detectors.vocabulary import BASE_TERMS
from app.reasoning.observed_facts_builder import infer_object_source
from app.reasoning.target_profile import TargetProfileResolver
from app.schemas import EvidenceSource


def test_terms_keep_target_anchor_scene_roles_and_are_stable() -> None:
    profile = TargetProfileResolver().resolve("饮水机旁边的蓝色垃圾桶", use_llm=False)
    terms, slots = _prompt_terms_and_slots(
        target_text="蓝色垃圾桶",
        dynamic_terms=["trash bin", "blue trash bin"],
        scene_terms=["chair", "trash bin"],
        target_profile=profile,
        slots=None,
        include_static_scene=True,
    )
    assert terms == sorted(set(terms))
    assert "trash bin" in slots["target"]
    assert "trash bin" in slots["scene"]
    assert set(slots) >= {"target", "scene"}


def test_static_scene_terms_are_kept_when_target_terms_exist() -> None:
    terms, slots = _prompt_terms_and_slots(
        target_text="trash bin",
        dynamic_terms=["trash bin"],
        scene_terms=[],
        target_profile=None,
        slots=None,
        include_static_scene=True,
    )
    assert "trash bin" in slots["target"]
    assert set(BASE_TERMS).issubset(slots["scene"])
    assert terms == sorted(set(terms))


def test_empty_slot_set_does_not_start_worker(tmp_path) -> None:
    image = tmp_path / "frame.jpg"
    image.write_bytes(b"not decoded because no request is sent")
    detector = YOLOESegSubprocessDetector(Settings(siliconflow_api_key=""))
    assert detector.detect_with_dynamic_terms(
        str(image), "chair", ["chair"], slots=frozenset()
    ) == []
    assert detector._worker is None


def test_conversion_preserves_prompt_slot_and_bbox_only_degradation() -> None:
    obj = _to_detected_object(
        {
            "label": "chair",
            "source_prompt_term": "office chair",
            "bbox_2d": [-0.2, 0.1, 1.2, 0.9],
            "score": 0.8,
            "detection_slots": ["anchor", "scene"],
        },
        None,
        model_id="yoloe-26s-seg.pt",
    )
    assert obj.label == "office chair"
    assert obj.source_prompt_term == "office chair"
    assert obj.label_zh == "办公椅"
    assert obj.bbox_2d == (0.0, 0.1, 1.0, 0.9)
    assert obj.detection_slots == ("anchor", "scene")
    assert obj.mask_rle is None
    assert obj.mask_source is None
    assert obj.source == "yoloe_seg"


def test_prompt_free_label_is_displayed_in_chinese() -> None:
    obj = _to_detected_object(
        {
            "label": "movie poster",
            "bbox_2d": [0.1, 0.1, 0.4, 0.5],
            "score": 0.8,
        },
        None,
        model_id="yoloe-26s-seg-pf.pt",
    )
    assert obj.label == "movie poster"
    assert obj.label_zh == "电影海报"
    assert obj.category == "sign"


def test_yoloe_mask_keeps_its_own_reasoning_evidence_source() -> None:
    obj = SimpleNamespace(
        crop_verify=None,
        attributes=["detected_by_yoloe", "segmented_by_yoloe"],
        mask_area_ratio=0.2,
        detector_score=0.8,
    )
    assert infer_object_source(obj) is EvidenceSource.YOLOE_SEG


def test_client_command_contains_all_effective_runtime_values(tmp_path) -> None:
    python = tmp_path / "python"
    python.write_text("", encoding="utf-8")
    python.chmod(0o755)
    model = tmp_path / "yoloe-26s-seg.pt"
    model.write_bytes(b"weights")
    encoder = tmp_path / "mobileclip2_b.ts"
    encoder.write_bytes(b"encoder")
    settings = replace(
        Settings(siliconflow_api_key=""),
        yoloe_python=str(python), yoloe_model=str(model), yoloe_text_encoder=str(encoder),
        yoloe_device="cuda:1", yoloe_precision="fp32",
        yoloe_confidence=0.31, yoloe_iou_threshold=0.44,
        yoloe_image_size=512, yoloe_max_detections=27,
    )
    command = YOLOESegSubprocessDetector(settings)._command()
    assert command[0] == str(python)
    for flag, expected in {
        "--model": str(model), "--text-encoder": str(encoder), "--device": "cuda:1", "--precision": "fp32",
        "--confidence": "0.31", "--iou-threshold": "0.44",
        "--image-size": "512", "--max-detections": "27",
    }.items():
        assert command[command.index(flag) + 1] == expected


def test_prompt_free_client_omits_text_encoder_and_ignores_scene_terms(tmp_path) -> None:
    python = tmp_path / "python"
    python.write_text("", encoding="utf-8")
    python.chmod(0o755)
    model = tmp_path / "yoloe-26s-seg-pf.pt"
    model.write_bytes(b"weights")
    settings = replace(
        Settings(siliconflow_api_key=""),
        yoloe_python=str(python), yoloe_model=str(model),
        yoloe_text_encoder=str(tmp_path / "does-not-exist.ts"),
    )
    detector = YOLOESegSubprocessDetector(settings)
    requests = []
    detector._request = lambda **payload: requests.append(payload) or {"objects": []}
    image = tmp_path / "frame.jpg"
    image.write_bytes(b"worker is mocked")

    assert detector.detect_with_dynamic_terms(
        str(image), "chair", ["chair"], ["industrial pipe"], frame_id="f1"
    ) == []

    assert "--text-encoder" not in detector._command()
    assert requests[0]["terms"] == []
    assert requests[0]["slot_terms"] == {}

