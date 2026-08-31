"""FOV diagnostics for the D435 stream service.

Validates the intrinsic -> HFOV/VFOV helper in realsense_stream.py.  Locks the
key fact used to answer the "取景范围太小" question: 640x480 is a centre crop
(~55° H) while 848x480 / 1280x720 use the full-width ~69° H sensor area, so
switching resolutions genuinely widens the visible area.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

STREAM = Path(__file__).resolve().parents[1] / "scripts/go2w/realsense_stream.py"

pytest.importorskip("pyrealsense2")


def _load_stream_module():
    spec = importlib.util.spec_from_file_location("rs_stream_test", STREAM)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_lowres_crop_is_narrower_than_full_width():
    mod = _load_stream_module()
    fov = mod.fov_deg_from_intrinsics
    low = fov({"fx": 615.0, "fy": 615.0, "width": 640, "height": 480})
    wide = fov({"fx": 615.0, "fy": 615.0, "width": 848, "height": 480})
    assert low["h_deg"] < 60.0, f"640x480 should be a narrow crop, got {low['h_deg']}"
    assert wide["h_deg"] > 65.0, f"848x480 should reach full width, got {wide['h_deg']}"
    assert wide["h_deg"] > low["h_deg"]


def test_fov_returns_none_for_missing_intrinsics():
    mod = _load_stream_module()
    assert mod.fov_deg_from_intrinsics({}) is None
    assert mod.fov_deg_from_intrinsics(None) is None
