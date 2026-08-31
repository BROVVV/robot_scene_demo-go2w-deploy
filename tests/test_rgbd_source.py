"""Tests for RGBDFrame and RealSense HTTP source parsing/materialization."""

from __future__ import annotations

import json
import urllib.request
from pathlib import Path

import pytest

from app.perception.realsense_http_rgbd_source import RealSenseHTTPRGBDSource
from app.perception.rgbd_source import RGBDFrame, RGBDFrameUnavailable


def test_rgbd_frame_roundtrip():
    frame = RGBDFrame(
        frame_id="42",
        timestamp=1.0,
        color_ref="/tmp/color.jpg",
        depth_ref="/tmp/depth.png",
        width=640,
        height=480,
        fx=600.0,
        fy=600.0,
        cx=320.0,
        cy=240.0,
    )
    restored = RGBDFrame.from_dict(frame.to_dict())
    assert restored.frame_id == "42"
    assert restored.fx == 600.0
    assert restored.depth_aligned_to_color is True


class _FakeResponse:
    def __init__(self, data: bytes) -> None:
        self._data = data

    def read(self) -> bytes:
        return self._data

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


def test_real_sense_source_materialize_local(monkeypatch, tmp_path: Path):
    source = RealSenseHTTPRGBDSource(
        "http://example.invalid", cache_dir=tmp_path, request_timeout=0.2
    )
    meta = {
        "frame_id": "7",
        "host_timestamp": 1.0,
        "color_url": "/rgbd/frame/7/color.jpg",
        "depth_url": "/rgbd/frame/7/depth.png",
        "width": 2,
        "height": 2,
        "intrinsics": {"fx": 1, "fy": 1, "cx": 0, "cy": 0},
        "depth_unit_m": 0.001,
        "health": {"age_s": 0.0},
    }
    calls: list[str] = []

    def fake_urlopen(url, timeout=0.0):
        calls.append(str(url))
        path = str(url).split("/")[-1]
        if path.endswith(".jpg"):
            return _FakeResponse(b"JPGDATA")
        if path.endswith(".png"):
            return _FakeResponse(b"PNGDATA")
        if url.endswith("/rgbd/latest.json"):
            return _FakeResponse(json.dumps(meta).encode())
        raise AssertionError(f"unexpected url {url}")

    monkeypatch.setattr(urllib.request, "urlopen", fake_urlopen)
    frame = source.get_latest(timeout_seconds=0.5)
    assert frame.frame_id == "7"
    assert frame.color_ref.endswith("color_7.jpg")
    assert frame.depth_ref.endswith("depth_7.png")
    assert Path(frame.color_ref).read_bytes() == b"JPGDATA"
    assert Path(frame.depth_ref).read_bytes() == b"PNGDATA"
    assert "latest.json" in calls[0]


def test_real_sense_source_stale(monkeypatch, tmp_path: Path):
    source = RealSenseHTTPRGBDSource(
        "http://example.invalid", cache_dir=tmp_path, request_timeout=0.2, max_age_seconds=0.1
    )
    meta = {
        "frame_id": "7",
        "host_timestamp": 1.0,
        "color_url": "/c.jpg",
        "depth_url": "/d.png",
        "width": 2,
        "height": 2,
        "intrinsics": {},
        "health": {"age_s": 99.0},
    }

    def fake_urlopen(url, timeout=0.0):
        return _FakeResponse(json.dumps(meta).encode())

    monkeypatch.setattr(urllib.request, "urlopen", fake_urlopen)
    with pytest.raises(RGBDFrameUnavailable):
        source.get_latest(timeout_seconds=0.05)
