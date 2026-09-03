"""计划书 §17.11 / §11.1：D435 采集时间优先 host_timestamp，非法回退
receive_time 并显式标记 timestamp_quality。"""

from __future__ import annotations

import time

from app.perception.realsense_http_rgbd_source import _resolve_capture_timestamp


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
