from __future__ import annotations

import pytest

from app.config import Settings
from app.detectors.backend import (
    LOCAL_DETECTOR_BACKENDS,
    SEARCH_DETECTOR_BACKENDS,
    detector_capabilities,
)
from app.detectors.factory import create_local_detector
from app.detectors.grounded_sam_subprocess import GroundedSAMSubprocessDetector
from app.detectors.yoloe_seg_subprocess import YOLOESegSubprocessDetector


def _settings() -> Settings:
    return Settings(siliconflow_api_key="")


def test_capability_table_covers_all_search_backends() -> None:
    assert SEARCH_DETECTOR_BACKENDS == ("llm", "grounded_sam", "yoloe_seg")
    assert LOCAL_DETECTOR_BACKENDS == ("grounded_sam", "yoloe_seg")
    assert detector_capabilities("yoloe_seg").instance_masks is True
    assert detector_capabilities("yoloe_seg").ram_scene_terms is False


def test_factory_constructs_both_local_backends_without_loading_models() -> None:
    assert isinstance(create_local_detector("grounded_sam", _settings()), GroundedSAMSubprocessDetector)
    assert isinstance(create_local_detector("yoloe_seg", _settings()), YOLOESegSubprocessDetector)


def test_factory_rejects_llm_and_unknown_names() -> None:
    with pytest.raises(ValueError, match="local detector"):
        create_local_detector("llm", _settings())
    with pytest.raises(ValueError, match="local detector"):
        create_local_detector("made_up", _settings())

