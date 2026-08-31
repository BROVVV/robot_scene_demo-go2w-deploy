"""Tests for LightweightDepthBEVMapper fallback spatial map."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from app.perception.rgbd_source import RGBDFrame
from app.spatial.frontier_extractor import FrontierExtractor
from app.spatial.lightweight_depth_bev import LightweightDepthBEVMapper
from app.spatial.models import SpatialPose


def _frame(tmp_path: Path) -> RGBDFrame:
    depth_path = tmp_path / "depth.png"
    cv2.imwrite(str(depth_path), np.full((64, 64), 2000, dtype=np.uint16))
    return RGBDFrame(
        frame_id="f1", timestamp=0.0, color_ref="", depth_ref=str(depth_path),
        width=64, height=64, fx=50.0, fy=50.0, cx=32.0, cy=32.0,
        depth_unit_m=0.001,
    )


def test_bev_mapper_creates_free_and_occupied(tmp_path: Path):
    mapper = LightweightDepthBEVMapper()
    snap = mapper.update(_frame(tmp_path), SpatialPose(x=0.0, y=0.0, yaw=0.0))
    assert snap.revision >= 1
    assert len(snap.free) > 0
    assert len(snap.occupied) > 0
    assert snap.quality == "RELATIVE_RGBD"


def test_bev_mapper_feeds_frontier_extractor(tmp_path: Path):
    mapper = LightweightDepthBEVMapper()
    snap = mapper.update(_frame(tmp_path), SpatialPose(x=0.0, y=0.0, yaw=0.0))
    frontiers = FrontierExtractor(min_component_size=1).extract(
        snap, SpatialPose(x=0.0, y=0.0, yaw=0.0)
    )
    assert len(frontiers) > 0
