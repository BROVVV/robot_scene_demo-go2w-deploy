"""Detector backend names and explicit capability declarations."""

from __future__ import annotations

from dataclasses import dataclass


LLM_BACKEND = "llm"
GROUNDED_SAM_BACKEND = "grounded_sam"
YOLOE_SEG_BACKEND = "yoloe_seg"

SEARCH_DETECTOR_BACKENDS = (LLM_BACKEND, GROUNDED_SAM_BACKEND, YOLOE_SEG_BACKEND)
LOCAL_DETECTOR_BACKENDS = (GROUNDED_SAM_BACKEND, YOLOE_SEG_BACKEND)
REMOTE_MAPPING_DETECTOR_BACKENDS = LOCAL_DETECTOR_BACKENDS


@dataclass(frozen=True)
class DetectorCapabilities:
    backend: str
    open_vocabulary: bool
    structured_boxes: bool
    instance_masks: bool
    local_inference: bool
    ram_scene_terms: bool
    detection_slots: bool


_CAPABILITIES = {
    LLM_BACKEND: DetectorCapabilities(
        backend=LLM_BACKEND,
        open_vocabulary=True,
        structured_boxes=False,
        instance_masks=False,
        local_inference=False,
        ram_scene_terms=False,
        detection_slots=False,
    ),
    GROUNDED_SAM_BACKEND: DetectorCapabilities(
        backend=GROUNDED_SAM_BACKEND,
        open_vocabulary=True,
        structured_boxes=True,
        instance_masks=True,
        local_inference=True,
        ram_scene_terms=True,
        detection_slots=True,
    ),
    YOLOE_SEG_BACKEND: DetectorCapabilities(
        backend=YOLOE_SEG_BACKEND,
        open_vocabulary=True,
        structured_boxes=True,
        instance_masks=True,
        local_inference=True,
        ram_scene_terms=False,
        detection_slots=True,
    ),
}


def detector_capabilities(backend: str) -> DetectorCapabilities:
    try:
        return _CAPABILITIES[str(backend).strip().lower()]
    except KeyError as exc:
        raise ValueError(f"unsupported detector backend: {backend}") from exc


def is_local_structured_backend(backend: str) -> bool:
    capabilities = detector_capabilities(backend)
    return capabilities.local_inference and capabilities.structured_boxes

