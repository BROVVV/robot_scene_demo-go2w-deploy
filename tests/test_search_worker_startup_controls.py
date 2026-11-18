from __future__ import annotations

import io
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from app.manual_web_demo.search_executor import SubprocessSearchExecutor
from scripts.go2w import autonomous_search_worker as worker


@pytest.fixture(autouse=True)
def isolated_worker(monkeypatch):
    messages = []
    monkeypatch.setattr(worker, 'emit', messages.append)
    for key, value in dict(holder=None, controlled_explorer=None,
                           stop_requested=False, pause_requested=False,
                           worker_state='starting', worker_stage='LOAD_PIPELINE',
                           finish_reason='', last_graph_revision=None).items():
        monkeypatch.setitem(worker._STATE, key, value)
    return messages


def explorer():
    return SimpleNamespace(session_id='test', request_stop=Mock(),
                           request_pause=Mock(), request_resume=Mock())


@pytest.mark.parametrize('command', ['stop', 'estop'])
def test_startup_stop_is_applied_when_shared_holder_gets_explorer(command):
    holder = {'explorer': None}
    hook = worker.make_event_hook()
    hook({'event': 'startup'}, holder)
    worker.handle_command({'cmd': command})
    current = explorer()
    holder['explorer'] = current
    hook({'event': 'session_start'}, holder)
    hook({'event': 'backend_health'}, holder)
    current.request_stop.assert_called_once()
    current.request_pause.assert_not_called()
    worker.handle_command({'cmd': 'resume'})
    current.request_resume.assert_not_called()


def test_pause_received_before_holder_is_not_lost():
    worker.handle_command({'cmd': 'pause'})
    current = explorer()
    worker.make_event_hook()({'event': 'session_start'}, {'explorer': current})
    current.request_pause.assert_called_once()
    worker.handle_command({'cmd': 'resume'})
    current.request_resume.assert_called_once()


def test_resume_during_startup_clears_pending_pause():
    worker.handle_command({'cmd': 'pause'})
    worker.handle_command({'cmd': 'resume'})
    current = explorer()
    worker.make_event_hook()({'event': 'session_start'}, {'explorer': current})
    current.request_pause.assert_not_called()


def test_startup_status_is_not_reported_as_running(monkeypatch, isolated_worker):
    monkeypatch.setattr(worker, '_running', lambda: True)
    worker.handle_command({'cmd': 'status'})
    assert isolated_worker[-1]['status']['state'] == 'starting'
    worker.make_event_hook()({'event': 'session_start'}, {'explorer': explorer()})
    worker.handle_command({'cmd': 'status'})
    assert isolated_worker[-1]['status']['state'] == 'running'


def test_stop_during_terminal_pipe_close_does_not_raise():
    executor = SubprocessSearchExecutor(cmd=('unused',))
    pipe = io.StringIO()
    pipe.close()
    executor._proc = SimpleNamespace(poll=lambda: None, stdin=pipe)
    executor.stop()
    assert executor.status()['state'] == 'broken'


def test_command_serialization_errors_are_not_hidden_as_pipe_failures():
    executor = SubprocessSearchExecutor(cmd=('unused',))
    executor._proc = SimpleNamespace(poll=lambda: None, stdin=io.StringIO())
    circular = {}; circular['self'] = circular
    with pytest.raises(ValueError, match='Circular'):
        executor._send(circular)
