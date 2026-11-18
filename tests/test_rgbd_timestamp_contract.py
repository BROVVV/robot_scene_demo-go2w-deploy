"""计划书 §17.11 / §11.1：D435 采集时间优先 host_timestamp，非法回退
receive_time 并显式标记 timestamp_quality。"""

from __future__ import annotations

import time
from pathlib import Path

from app.perception.realsense_http_rgbd_source import (
    RealSenseHTTPRGBDSource,
    _resolve_capture_timestamp,
)
from app.perception.rgbd_source import RGBDFrame


def test_valid_host_timestamp_used():
    now = time.time()
    stamp, quality = _resolve_capture_timestamp(now, default=now - 10.0)
    assert abs(stamp - now) < 1e-6
    assert quality == "host_timestamp"


def test_invalid_host_timestamp_falls_back_to_receive_time():
    default = time.time()
    for bad in (None, "", "abc", 0.0, 1e12, -5.0):
        stamp, quality = _resolve_capture_timestamp(bad, default=default)
        assert stamp == default
        assert quality == "receive_time"


def test_receive_time_fallback_is_explicit():
    default = 12345.678
    stamp, quality = _resolve_capture_timestamp(None, default=default)
    assert stamp == default
    assert quality == "receive_time"


def test_materialize_preserves_timestamp_quality(tmp_path, monkeypatch):
    source = RealSenseHTTPRGBDSource(cache_dir=tmp_path)
    frame = RGBDFrame(
        frame_id="fallback",
        timestamp=12345.678,
        color_ref="unused-color",
        depth_ref="unused-depth",
        width=2,
        height=2,
        fx=1.0,
        fy=1.0,
        cx=0.0,
        cy=0.0,
        timestamp_quality="receive_time",
    )
    (tmp_path / "color_fallback.jpg").write_bytes(b"jpg")
    (tmp_path / "depth_fallback.png").write_bytes(b"png")
    # Legacy flat cache files no longer establish capture identity. Exercise
    # a real materialization while retaining the timestamp-quality contract.
    monkeypatch.setattr(source, '_download', lambda url, path:
        path.write_bytes(b'fresh jpg' if url == 'unused-color' else b'fresh png'))

    materialized = source.materialize(frame)

    assert materialized.timestamp == frame.timestamp
    assert materialized.timestamp_quality == "receive_time"
    assert Path(materialized.color_ref).read_bytes() == b'fresh jpg'
