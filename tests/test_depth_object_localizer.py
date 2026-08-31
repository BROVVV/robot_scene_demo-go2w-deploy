"""Tests for DepthObjectLocalizer using synthetic depth images."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import pytest

from app.perception.depth_object_localizer import (
    SPATIAL_QUALITY_CAMERA_LOCAL,
    SPATIAL_QUALITY_RGB_ONLY,
    DepthObjectLocalizer,
)
from app.perception.rgbd_source import RGBDFrame


def _write_depth(path: Path, depth_mm: np.ndarray) -> None:
    cv2.imwrite(str(path), depth_mm.astype(np.uint16))


def _frame(tmp_path: Path, depth_mm: np.ndarray) -> RGBDFrame:
    depth_path = tmp_path / "depth.png"
    _write_depth(depth_path, depth_mm)
    return RGBDFrame(
        frame_id="f1",
        timestamp=0.0,
        color_ref=str(tmp_path / "color.jpg"),
        depth_ref=str(depth_path),
        width=64,
        height=64,
        fx=50.0,
        fy=50.0,
        cx=32.0,
        cy=32.0,
        depth_unit_m=0.001,
    )


def test_localize_constant_depth_box(tmp_path: Path):
    depth_mm = np.full((64, 64), 0, dtype=np.uint16)
    depth_mm[20:44, 20:44] = 2000
    frame = _frame(tmp_path, depth_mm)
    localizer = DepthObjectLocalizer()
    objects = [{"label": "box", "label_zh": "箱子", "bbox_2d": [0.3, 0.3, 0.7, 0.7], "confidence": 0.9}]
    out = localizer.localize(objects, frame)
    assert len(out) == 1
    assert out[0].spatial_quality == SPATIAL_QUALITY_CAMERA_LOCAL
    assert out[0].depth_m == pytest.approx(2.0, abs=0.02)
    assert out[0].camera_xyz is not None
    assert out[0].bearing_deg is not None


def test_localize_mixed_invalid_depth_uses_median(tmp_path: Path):
    depth_mm = np.full((64, 64), 0, dtype=np.uint16)
    depth_mm[20:44, 20:44] = 1500
    depth_mm[22:24, 22:24] = 0
    depth_mm[26:28, 26:28] = 9999
    frame = _frame(tmp_path, depth_mm)
    localizer = DepthObjectLocalizer()
    objects = [{"label": "box", "bbox_2d": [0.3, 0.3, 0.7, 0.7]}]
    out = localizer.localize(objects, frame)
    assert out[0].depth_m == pytest.approx(1.5, abs=0.02)


def test_localize_rgb_only_when_depth_missing(tmp_path: Path):
    frame = RGBDFrame(
        frame_id="f2", timestamp=0.0, color_ref="", depth_ref="", width=64, height=64,
        fx=50.0, fy=50.0, cx=32.0, cy=32.0,
    )
    localizer = DepthObjectLocalizer()
    out = localizer.localize([{"label": "box", "bbox_2d": [0.3, 0.3, 0.7, 0.7]}], frame)
    assert out[0].spatial_quality == SPATIAL_QUALITY_RGB_ONLY
    assert out[0].depth_m is None
