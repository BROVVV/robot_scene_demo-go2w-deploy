"""Single construction point for local structured detector backends."""

from __future__ import annotations

from typing import Any

from app.config import Settings
from app.detectors.backend import GROUNDED_SAM_BACKEND, YOLOE_SEG_BACKEND


def create_local_detector(
    backend: str,
    settings: Settings,
    *,
    target_profile: Any | None = None,
    parsed_task: Any | None = None,
    navigation_task: Any | None = None,
    llm_client: Any | None = None,
):
    name = str(backend).strip().lower()
    if name == GROUNDED_SAM_BACKEND:
        from app.detectors.grounded_sam_subprocess import GroundedSAMSubprocessDetector

        return GroundedSAMSubprocessDetector(
            settings=settings,
            target_profile=target_profile,
            parsed_task=parsed_task,
            navigation_task=navigation_task,
            llm_client=llm_client,
        )
    if name == YOLOE_SEG_BACKEND:
        from app.detectors.yoloe_seg_subprocess import YOLOESegSubprocessDetector

        return YOLOESegSubprocessDetector(
            settings=settings,
            target_profile=target_profile,
        )
    raise ValueError(f"local detector backend is not supported: {backend}")

