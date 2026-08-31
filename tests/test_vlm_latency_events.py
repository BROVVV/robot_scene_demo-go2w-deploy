"""Tests for latency profiler event serialisation."""

from __future__ import annotations

from app.live_robot.latency_profiler import LatencyProfiler


def test_profiler_snapshot_shape():
    profiler = LatencyProfiler(planning_cycle=3, frame_id="f1")
    profiler.mark("capture_start")
    profiler.mark("capture_end")
    profiler.record("capture_ms", "capture_start")
    profiler.incr("quick_api_calls")
    snapshot = profiler.snapshot()
    assert snapshot["event"] == "latency_profile"
    assert snapshot["planning_cycle"] == 3
    assert snapshot["frame_id"] == "f1"
    assert "capture_ms" in snapshot["timings_ms"]
    assert snapshot["api_calls"]["quick_api_calls"] == 1


def test_profiler_manual_duration():
    profiler = LatencyProfiler()
    profiler.record("quick", duration=123.4)
    assert profiler.profile.timings_ms["quick"] == 123.4
