"""「一搜就 FAILED」worker 解释器/失败原因回归测试。

1) SubprocessSearchExecutor 自动探测能 import rclpy + run_semantic_exploration
   的解释器（优先 /usr/bin/python3 = ROS），避免“用了无 rclpy 的 Web/Conda
   python 导致真机搜索立刻 FAILED”。
2) worker 端 session_result 携带 error/reason，SearchSessionService 把它传入
   SEARCH_FINISHED，WebUI 能显示具体原因而不是笼统的「搜索结束: FAILED」。
"""

from __future__ import annotations

import subprocess

import pytest

from app.manual_web_demo.search_executor import (
    SubprocessSearchExecutor,
    _resolve_worker_python,
)
from app.manual_web_demo.control_ownership import ControlOwner
from app.manual_web_demo.search_session_service import SearchSessionService


def _ok_result(*args, **kwargs):
    return subprocess.CompletedProcess(args[0], 0, stdout="", stderr="")


def _fail_result(*args, **kwargs):
    return subprocess.CompletedProcess(args[0], 1, stdout="", stderr="oops")


def test_prefers_ros_python_when_probe_passes(monkeypatch):
    monkeypatch.delenv("GO2W_WORKER_PYTHON", raising=False)
    monkeypatch.setattr("os.path.isfile", lambda p: True)
    monkeypatch.setattr("app.manual_web_demo.search_executor.subprocess.run", _ok_result)
    assert _resolve_worker_python() == "/usr/bin/python3"


def test_falls_back_to_ros_when_no_probe_passes(monkeypatch):
    """开发机没有 rclpy：探测全失败，但必须回退到 ROS python 而不是 conda。"""
    monkeypatch.delenv("GO2W_WORKER_PYTHON", raising=False)
    monkeypatch.setattr("os.path.isfile", lambda p: True)
    monkeypatch.setattr("app.manual_web_demo.search_executor.subprocess.run", _fail_result)
    assert _resolve_worker_python() == "/usr/bin/python3"


def test_env_override_wins_when_probe_passes(monkeypatch):
    monkeypatch.setenv("GO2W_WORKER_PYTHON", "/custom/python")
    monkeypatch.setattr("os.path.isfile", lambda p: True)
    monkeypatch.setattr("app.manual_web_demo.search_executor.subprocess.run", _ok_result)
    assert _resolve_worker_python() == "/custom/python"


def test_subprocess_executor_uses_resolved_python(monkeypatch):
    monkeypatch.setattr(
        "app.manual_web_demo.search_executor._resolve_worker_python",
        lambda: "/usr/bin/python3",
    )
    exe = SubprocessSearchExecutor()
    assert exe._cmd == ["/usr/bin/python3", "scripts/go2w/autonomous_search_worker.py"]


def test_session_result_failure_reason_surfaces_to_webui(monkeypatch):
    """worker 返回 exit_code=4 + error -> SEARCH_FINISHED 携带 reason。"""
    service = SearchSessionService(
        owner=ControlOwner(),
        executor_factory=lambda: None if False else object(),
        session_dir="outputs/live_runs",
    )
    service._session_id = "s_fail"
    service._info = type("I", (), {
        "session_id": "s_fail", "target": "x", "status": "STARTING", "task_text": "x",
        "task_context": {}, "result": None, "started_at": 0.0, "finished_at": None,
        "backend": "go2w_experimental", "reasoner": "semantic_navigation",
    })()
    service._apply_session_result({
        "exit_code": 4,
        "finish_reason": "FAILED",
        "error": "ModuleNotFoundError: No module named 'openai'",
        "reason": "ModuleNotFoundError: No module named 'openai'",
    })
    state = service.state_snapshot()
    assert state["status"] == "FAILED"
    recent = service.recent_events(5)
    finished = [e for e in recent if e.get("event_type") == "SEARCH_FINISHED"]
    assert finished, "应发出 SEARCH_FINISHED 事件"
    payload = finished[0].get("payload") or {}
    assert payload.get("result") == "FAILED"
    assert "ModuleNotFoundError" in (payload.get("error") or "")
