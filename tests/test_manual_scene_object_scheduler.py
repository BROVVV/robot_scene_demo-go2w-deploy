"""Scene-object scheduler tests (plan book §45)."""

from __future__ import annotations

import threading
from dataclasses import replace
from pathlib import Path
from typing import Any

import pytest

from app.manual_web_demo.config import ManualDemoSettings
from app.manual_web_demo.models import SceneObject
from app.manual_web_demo.scene_object_analyzer import SceneObjectAnalyzer


def config_with(**overrides) -> ManualDemoSettings:
    overrides.setdefault("llm_enabled", True)
    overrides.setdefault("llm_interval_seconds", 5.0)
    return replace(ManualDemoSettings(), **overrides)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def make_fake_analyzer() -> tuple[Any, threading.Event, list[str]]:
    calls: list[str] = []
    done = threading.Event()

    def analyzer(image_path: str) -> dict[str, Any]:
        calls.append(image_path)
        done.set()
        return {
            "objects": [SceneObject(name_zh="椅子", count=2, confidence="high")],
            "scene_summary": "办公室",
            "model": "fake",
        }

    return analyzer, done, calls


def make_analyzer(tmp_path: Path, *, camera_fresh=True, config=None, analyzer=None):
    frame = tmp_path / "latest.jpg"
    frame.write_bytes(b"\xff\xd8\xff\xd9")  # tiny fake jpeg
    analyzer_fn = analyzer or make_fake_analyzer()[0]
    clock = FakeClock()
    instance = SceneObjectAnalyzer(
        config=config or config_with(),
        frame_provider=lambda: str(frame),
        camera_fresh_provider=lambda: camera_fresh,
        analyzer_fn=analyzer_fn,
        clock=clock,
    )
    return instance, clock


def test_interval_skips_while_running(tmp_path: Path) -> None:
    analyzer_fn, done, calls = make_fake_analyzer()
    analyzer, clock = make_analyzer(tmp_path, analyzer=analyzer_fn)
    assert analyzer.run_cycle(now=0.0) == "started"
    assert analyzer.run_cycle(now=2.0) == "skip_running"
    done.wait(1.0)
    assert len(calls) == 1
    assert analyzer.state_dict()["status"] == "ok"


def test_next_cycle_starts_after_interval(tmp_path: Path) -> None:
    analyzer_fn, done, calls = make_fake_analyzer()
    analyzer, clock = make_analyzer(tmp_path, analyzer=analyzer_fn)
    assert analyzer.run_cycle(now=0.0) == "started"
    done.wait(1.0)
    assert len(calls) == 1
    assert analyzer.run_cycle(now=4.0) == "skip_interval"
    assert analyzer.run_cycle(now=6.0) == "started"


def test_camera_stale_skips_llm(tmp_path: Path) -> None:
    analyzer, _ = make_analyzer(tmp_path, camera_fresh=False)
    assert analyzer.run_cycle(now=0.0) == "skip_camera_stale"


def test_no_frame_skips(tmp_path: Path) -> None:
    analyzer, _ = make_analyzer(tmp_path)
    # Remove the frame file after construction.
    (tmp_path / "latest.jpg").unlink()
    assert analyzer.run_cycle(now=0.0) == "skip_no_frame"


def test_disabled_by_default_and_toggle(tmp_path: Path) -> None:
    analyzer_fn, done, _ = make_fake_analyzer()
    analyzer, _ = make_analyzer(
        tmp_path, config=config_with(llm_enabled=False), analyzer=analyzer_fn
    )
    assert analyzer.run_cycle(now=0.0) == "skip_disabled"
    analyzer.set_enabled(True)
    assert analyzer.run_cycle(now=0.0) == "started"
    done.wait(1.0)  # let the inference finish so status is not "running"
    analyzer.set_enabled(False)
    assert analyzer.run_cycle(now=5.0) == "skip_disabled"


def test_llm_failure_keeps_last_success(tmp_path: Path) -> None:
    def failing_analyzer(image_path: str) -> dict[str, Any]:
        raise RuntimeError("network down")

    analyzer, _ = make_analyzer(tmp_path, analyzer=failing_analyzer)
    assert analyzer.run_cycle(now=0.0) == "started"
    # Wait for the inference thread to finish.
    for _ in range(50):
        if analyzer.state_dict()["status"] == "error":
            break
        import time

        time.sleep(0.02)
    state = analyzer.state_dict()
    assert state["status"] == "error"
    assert "network down" in (state["error"] or "")
