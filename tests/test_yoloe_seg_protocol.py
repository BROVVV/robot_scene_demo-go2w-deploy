from __future__ import annotations

import subprocess
import sys
import tempfile
from dataclasses import replace

import pytest

from app.config import Settings
from app.detectors.grounded_sam_subprocess import DetectorRuntimeError
from app.detectors.yoloe_seg_subprocess import YOLOESegSubprocessDetector
from app.live_robot.explorer_search_adapter import ExplorerSearchAdapter
from app.live_robot.search_event_bus import SearchEventBus
from app.live_robot.search_state_store import SearchStateStore


def _client_for_script(script: str, *, timeout: float = 0.5) -> tuple[YOLOESegSubprocessDetector, subprocess.Popen]:
    process = subprocess.Popen(
        [sys.executable, "-u", "-c", script],
        text=True,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    detector = YOLOESegSubprocessDetector(
        replace(Settings(siliconflow_api_key=""), yoloe_timeout_seconds=timeout)
    )
    detector._worker = process
    detector._worker_stderr = tempfile.TemporaryFile(mode="w+b")
    detector._ensure_worker = lambda: None  # type: ignore[method-assign]
    return detector, process


def test_non_json_worker_output_is_rejected() -> None:
    detector, _ = _client_for_script(
        "import sys; sys.stdin.readline(); print('worker noise', flush=True)"
    )
    with pytest.raises(DetectorRuntimeError, match="non-JSON"):
        detector.health()


def test_request_id_mismatch_is_rejected() -> None:
    detector, _ = _client_for_script(
        "import json,sys; sys.stdin.readline(); print(json.dumps({'request_id':'wrong'}), flush=True)"
    )
    with pytest.raises(DetectorRuntimeError, match="request_id mismatch"):
        detector.health()


def test_worker_timeout_is_explicit() -> None:
    detector, _ = _client_for_script(
        "import sys,time; sys.stdin.readline(); time.sleep(0.3)", timeout=0.05
    )
    with pytest.raises(DetectorRuntimeError, match="timed out"):
        detector.health()


def test_close_sends_orderly_shutdown() -> None:
    script = """
import json, sys
for line in sys.stdin:
    request = json.loads(line)
    if request.get('operation') == 'shutdown':
        print(json.dumps({'ok': True, 'request_id': request.get('request_id')}), flush=True)
        raise SystemExit(0)
    print(json.dumps({'ok': True, 'request_id': request.get('request_id')}), flush=True)
"""
    detector, process = _client_for_script(script)
    assert detector.health()["ok"] is True
    detector.close()
    assert process.wait(timeout=1) == 0
    assert detector._worker is None
    assert process.stdin.closed and process.stdout.closed and process.stderr.closed


def test_close_reaps_a_worker_that_ignores_shutdown_and_sigterm(monkeypatch) -> None:
    detector, process = _client_for_script(
        "import signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); "
        "print('ready',flush=True); time.sleep(60)"
    )
    original_wait = process.wait
    waits = []
    def bounded_wait(timeout=None):
        waits.append(timeout)
        # Exercise real TERM/KILL/wait semantics without spending production
        # 5+2 second grace windows in a small process-lifecycle regression.
        return original_wait(timeout=.05 if len(waits)<3 else 1.)
    monkeypatch.setattr(process,"wait",bounded_wait)
    try:
        assert process.stdout.readline().strip()=='ready'
        detector.close()
        assert process.returncode == -9, 'forced worker must be reaped before close returns'
        assert waits == [5,2,2]
        assert detector._worker is None
        assert process.stdin.closed and process.stdout.closed and process.stderr.closed
        detector.close()  # Repeated owner teardown remains harmless.
    finally:
        if process.poll() is None:process.kill()
        original_wait(timeout=2)
        for stream in (process.stdin,process.stdout,process.stderr):
            if stream is not None:stream.close()
        detector.close()


def test_worker_reported_backend_becomes_web_state_authority() -> None:
    store = SearchStateStore()
    adapter = ExplorerSearchAdapter(
        SearchEventBus(), store, session_id="yoloe-health-test"
    )
    adapter.on_explorer_event({
        "event": "detector_startup_health",
        "detector_backend": "yoloe_seg",
        "health": {"backend": "yoloe_seg", "fallback_used": False},
    })
    snapshot = store.snapshot()
    assert snapshot["detector_backend"] == "yoloe_seg"
    assert snapshot["health"]["backend"] == "yoloe_seg"
    assert snapshot["health"]["fallback_used"] is False
