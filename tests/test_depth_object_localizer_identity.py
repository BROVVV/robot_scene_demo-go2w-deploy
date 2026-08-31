"""Plan §4 tests: DepthObjectLocalizer must preserve frame_object_id.

The real-time semantic observer emits ``frame_object_id`` (``semantic_obj_001``);
the localizer must keep it on ``ObjectSpatialObservation.object_id`` so the
entity association and the object-relation graph can remap it to a persistent
``obj_xxx`` id.  Without this, two same-label objects would collapse.
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from app.perception.depth_object_localizer import (
    SPATIAL_QUALITY_RGB_ONLY,
    DepthObjectLocalizer,
    _source_object_id,
)
from app.perception.rgbd_source import RGBDFrame


def _depth_frame(tmp_path: Path) -> RGBDFrame:
    depth_path = tmp_path / "depth.png"
    depth_mm = np.full((64, 64), 0, dtype=np.uint16)
    depth_mm[20:44, 20:44] = 2000
    cv2.imwrite(str(depth_path), depth_mm)
    return RGBDFrame(
        frame_id="f_id", timestamp=0.0, color_ref="", depth_ref=str(depth_path),
        width=64, height=64, fx=50.0, fy=50.0, cx=32.0, cy=32.0, depth_unit_m=0.001,
    )


def test_frame_object_id_preserved_in_rgbd_path(tmp_path: Path):
    """Plan §4.6 case 1: frame_object_id flows into object_id."""
    frame = _depth_frame(tmp_path)
    localizer = DepthObjectLocalizer()
    out = localizer.localize(
        [{"frame_object_id": "semantic_obj_001", "label": "chair", "bbox_2d": [0.3, 0.3, 0.7, 0.7]}],
        frame,
    )
    assert len(out) == 1
    assert out[0].object_id == "semantic_obj_001"
    assert out[0].provenance.get("source_object_id") == "semantic_obj_001"


def test_frame_object_id_preserved_in_rgb_only_path():
    """Plan §4.6 case 2: RGB-only fallback keeps the same identity."""
    frame = RGBDFrame(
        frame_id="f_rgb", timestamp=0.0, color_ref="", depth_ref="", width=64, height=64,
        fx=50.0, fy=50.0, cx=32.0, cy=32.0,
    )
    localizer = DepthObjectLocalizer()
    out = localizer.localize(
        [{"frame_object_id": "semantic_obj_002", "label_zh": "绿色垃圾桶", "bbox_2d": [0.1, 0.1, 0.3, 0.3]}],
        frame,
    )
    assert len(out) == 1
    assert out[0].spatial_quality == SPATIAL_QUALITY_RGB_ONLY
    assert out[0].object_id == "semantic_obj_002"
    assert out[0].provenance.get("source_object_id") == "semantic_obj_002"


def test_id_precedence_is_compatible():
    """Plan §4.6 case 3: id/object_id/frame_object_id precedence is stable.

    When all are present the legacy ``id`` / ``object_id`` keys keep priority
    so existing payloads do not regress; frame_object_id is always supported.
    """
    assert _source_object_id({"frame_object_id": "semantic_obj_001"}) == "semantic_obj_001"
    assert _source_object_id({"object_id": "obj_A"}) == "obj_A"
    assert _source_object_id({"id": "obj_B"}) == "obj_B"
    assert _source_object_id({"id": "obj_B", "object_id": "obj_C", "frame_object_id": "semantic_001"}) == "obj_B"
    assert _source_object_id({"label": "chair"}) is None
    assert _source_object_id({}) is None
    assert _source_object_id({"frame_object_id": "   "}) is None
