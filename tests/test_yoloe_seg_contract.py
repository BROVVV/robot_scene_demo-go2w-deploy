from __future__ import annotations

import numpy as np
import pytest

from app.detectors.grounded_sam_subprocess import DetectorRuntimeError
from app.detectors.yoloe_seg_subprocess import _to_detected_object
from app.detectors.yoloe_seg_worker import (
    YOLOERuntime,
    _prediction_precision,
    encode_row_runs,
    prompt_signature,
)


def _decode(runs: list[list[int]], shape: tuple[int, int]) -> np.ndarray:
    mask = np.zeros(shape, dtype=bool)
    for row, start, length in runs:
        mask[row, start:start + length] = True
    return mask


def test_project_row_run_round_trip_is_pixel_exact() -> None:
    mask = np.zeros((7, 9), dtype=bool)
    mask[1, 2:5] = True
    mask[2, 0:2] = True
    mask[2, 7:9] = True
    mask[6, 4] = True
    runs = encode_row_runs(mask)
    assert np.array_equal(_decode(runs, mask.shape), mask)


def test_full_mask_contract_converts_to_detected_object() -> None:
    mask = np.zeros((4, 5), dtype=bool)
    mask[1:3, 2:5] = True
    obj = _to_detected_object(
        {
            "label": "pipe",
            "source_prompt_term": "industrial pipe",
            "bbox_2d": [0.4, 0.25, 1.0, 0.75],
            "score": 0.91,
            "mask_rle": encode_row_runs(mask),
            "mask_size": [4, 5],
            "mask_area_ratio": float(mask.mean()),
            "detection_slots": ["scene"],
        },
        None,
        model_id="yoloe-26s-seg.pt",
    )
    assert obj.mask_size == (4, 5)
    assert obj.mask_source == "yoloe_seg"
    assert obj.source == "yoloe_seg"
    assert np.array_equal(_decode(obj.mask_rle or [], obj.mask_size), mask)


@pytest.mark.parametrize(
    ("mask_rle", "mask_size"),
    [
        ([], [4, 5]),
        ([[4, 0, 1]], [4, 5]),
        ([[0, 4, 2]], [4, 5]),
        ([[0, 0, 0]], [4, 5]),
        ([[0, 0]], [4, 5]),
        ([[0, 0, 1]], [0, 5]),
        ([["bad", 0, 1]], [4, 5]),
    ],
)
def test_invalid_masks_degrade_to_bbox_only(mask_rle, mask_size) -> None:
    obj = _to_detected_object(
        {
            "label": "chair",
            "bbox_2d": [0.1, 0.1, 0.8, 0.8],
            "score": 0.7,
            "mask_rle": mask_rle,
            "mask_size": mask_size,
            "mask_area_ratio": 2.0,
        },
        None,
        model_id="yoloe-26s-seg.pt",
    )
    assert obj.mask_rle is None
    assert obj.mask_size is None
    assert obj.mask_area_ratio is None
    assert obj.mask_source is None


def test_zero_area_bbox_is_rejected() -> None:
    with pytest.raises(DetectorRuntimeError, match="zero-area bbox"):
        _to_detected_object(
            {"label": "chair", "bbox_2d": [0.5, 0.1, 0.5, 0.9], "score": 0.7},
            None,
            model_id="yoloe-26s-seg.pt",
        )


def test_prompt_signature_is_order_independent_but_model_and_slots_are_not() -> None:
    first = prompt_signature(["chair", "person", "chair"], ["scene", "target"], "a.pt")
    second = prompt_signature(["person", "chair"], ["target", "scene"], "a.pt")
    assert first == second
    assert first != prompt_signature(["chair", "person"], ["scene"], "a.pt")
    assert first != prompt_signature(["chair", "person"], ["scene", "target"], "b.pt")


def test_prediction_precision_uses_current_ultralytics_quantize_values() -> None:
    assert _prediction_precision("fp16") == 16
    assert _prediction_precision("fp32") == 32
    with pytest.raises(ValueError, match="unsupported YOLOE precision"):
        _prediction_precision("int8")


def test_prompt_library_output_is_redirected_away_from_jsonl_stdout(capsys) -> None:
    class NoisyModel:
        def get_text_pe(self, terms):
            print("third-party-noise")
            return terms

        def set_classes(self, terms, embeddings):
            print("more-third-party-noise")

    runtime = object.__new__(YOLOERuntime)
    runtime.model = NoisyModel()
    runtime.model_path = type("ModelPath", (), {"name": "test-seg.pt"})()
    runtime.current_prompt_hash = ""
    runtime.current_terms = []

    runtime._set_terms(["chair"], ["scene"])

    captured = capsys.readouterr()
    assert captured.out == ""
    assert "third-party-noise" in captured.err


def test_predict_uses_quantize_and_redirects_library_output(capsys) -> None:
    class NoisyModel:
        def __init__(self) -> None:
            self.kwargs = {}

        def predict(self, source, **kwargs):
            print("predict-library-noise")
            self.kwargs = kwargs
            return [source]

    runtime = object.__new__(YOLOERuntime)
    runtime.model = NoisyModel()
    runtime.precision = "fp16"

    assert runtime._predict("frame.jpg", verbose=False) == ["frame.jpg"]

    captured = capsys.readouterr()
    assert captured.out == ""
    assert "predict-library-noise" in captured.err
    assert runtime.model.kwargs["quantize"] == 16
    assert "half" not in runtime.model.kwargs


def test_prompt_free_runtime_rejects_per_frame_term_switching() -> None:
    runtime = object.__new__(YOLOERuntime)
    runtime.prompt_free = True

    with pytest.raises(RuntimeError, match="does not accept per-frame terms"):
        runtime._set_terms(["chair"], ["scene"])

