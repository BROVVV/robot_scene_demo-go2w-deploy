"""Stop intent stays visible while a live worker finishes queued work."""
import time

import pytest

from app.live_robot.search_event import (
    ERROR, OBSERVATION_UPDATED, PAUSED, RESUMED, SEARCH_FINISHED, SESSION_STARTED,
    SEARCH_STATE_CHANGED, STOP_REQUESTED,
    TARGET_CONFIRMED, SearchEvent,
)
from app.live_robot.search_state_store import SearchStateStore
from app.manual_web_demo.control_ownership import ControlOwner
from app.manual_web_demo.search_models import SearchStartRequest
from app.manual_web_demo.search_session_service import SearchSessionService
from app.task_understanding.search_task_context import SearchTaskContext


class HoldingExecutor:
    def __init__(self):
        self.running = False
        self.stops = self.pauses = self.resumes = 0
        self.on_stop = None

    def set_on_message(self, callback):
        self.callback = callback

    def start(self, params):
        self.running = True

    def emit(self, event):
        self.callback({'type': 'event', 'event': event})

    def alive(self):
        return self.running

    def status(self):
        return {'state': 'running' if self.running else 'stopped'}

    def stop(self):
        self.stops += 1
        if self.on_stop:
            self.on_stop()

    def pause(self):
        self.pauses += 1

    def resume(self):
        self.resumes += 1

    def shutdown(self):
        self.running = False

    def estop(self):
        self.stop()


def service(tmp_path):
    executor = HoldingExecutor()
    svc = SearchSessionService(owner=ControlOwner(), executor_factory=lambda: executor,
        session_dir=str(tmp_path/'sessions'),
        task_understanding_runner=lambda text: SearchTaskContext.mock_fallback(text))
    result = svc.start_search(SearchStartRequest.from_dict(
        {'task_text': '寻找椅子', 'backend': 'nav2', 'dry_run_motion': True}))
    assert result['ok']
    executor.emit({'event': 'session_start', 'host_s': time.time()})
    svc._mark_status('RUNNING')
    assert svc.state_snapshot()['status'] == 'RUNNING'
    return svc, executor


def test_accepted_stop_is_visible_and_blocks_pause_resume_while_worker_alive(tmp_path):
    svc, executor = service(tmp_path)
    try:
        assert svc.stop_search()['status'] == 'STOPPING'
        assert executor.alive() and executor.stops == 1
        assert svc.current_session().status == 'STOPPING'
        assert svc.state_snapshot()['status'] == 'STOPPING'
        assert svc.pause_search()['ok'] is False
        assert svc.resume_search()['ok'] is False
        assert executor.pauses == executor.resumes == 0
        assert svc._store.snapshot()['status'] == 'STOPPING'
        assert any(e['event_type'] == 'STOP_REQUESTED' for e in svc.recent_events())
    finally:
        svc.shutdown()


def test_terminal_ack_during_stop_send_is_not_replaced_by_stop_request(tmp_path):
    svc, executor = service(tmp_path)
    executor.on_stop = lambda: executor.emit(
        {'event': 'session_finish', 'result': 'OPERATOR_STOP', 'host_s': time.time()})
    try:
        assert svc.stop_search()['status'] == 'OPERATOR_STOP'
        assert svc.state_snapshot()['status'] == 'OPERATOR_STOP'
    finally:
        svc.shutdown()


def event(kind, payload=None, number=1):
    return SearchEvent(event_id=number, session_id='same-session', timestamp=100.+number,
                       event_type=kind, payload=payload or {})


@pytest.mark.parametrize('late', [SESSION_STARTED, PAUSED, RESUMED, TARGET_CONFIRMED,
                                 SEARCH_STATE_CHANGED])
def test_late_worker_status_cannot_undo_stop_but_raw_events_are_retained(late):
    store = SearchStateStore()
    store.reset(session_id='same-session', target='chair')
    store.apply(event(SESSION_STARTED))
    store.apply(event(STOP_REQUESTED, number=2))
    store.apply(event(late, number=3))
    store.apply(event(OBSERVATION_UPDATED, {'bundle_id': 'last-frame'}, number=4))
    snapshot = store.snapshot()
    assert snapshot['status'] == snapshot['phase'] == 'STOPPING'
    assert snapshot.get('result') != 'TARGET_FOUND'
    assert (snapshot.get('summary') or {}).get('result') != 'TARGET_FOUND'
    assert snapshot['observation']['bundle_id'] == 'last-frame'
    assert late in [x['event_type'] for x in snapshot['timeline']]
    store.apply(event(SEARCH_FINISHED, {'result': 'OPERATOR_STOP'}, number=5))
    assert store.snapshot()['status'] == store.snapshot()['phase'] == 'OPERATOR_STOP'
    store.apply(event(RESUMED, number=6))
    assert store.snapshot()['status'] == 'OPERATOR_STOP'
    store.reset(session_id='new-session', target='cabinet')
    assert store.snapshot()['status'] == 'STARTING'


def test_repeated_stop_does_not_restart_stop_timeout(tmp_path):
    svc, _ = service(tmp_path)
    try:
        svc.stop_search()
        first = svc._stopping_at
        svc.stop_search()
        assert svc._stopping_at == first
        assert svc.state_snapshot()['status'] == 'STOPPING'
    finally:
        svc.shutdown()


def test_stop_command_precedes_persistence_and_gates_during_send(tmp_path, monkeypatch):
    svc, executor = service(tmp_path)
    writes = []
    record = svc._archive.record

    def record_after_command(snapshot, event=None):
        if event and event.event_type == STOP_REQUESTED:
            assert executor.stops == 1
            writes.append(snapshot['status'])
        return record(snapshot, event)

    monkeypatch.setattr(svc._archive, 'record', record_after_command)

    def during_send():
        assert svc.current_session().status == 'STOPPING'
        assert svc.pause_search()['ok'] is False
        assert svc.resume_search()['ok'] is False

    executor.on_stop = during_send
    try:
        svc.stop_search()
        assert writes == ['STOPPING']
        session_id = svc.current_session().session_id
        saved = svc.history_session(session_id)
        assert saved['state']['status'] == 'STOPPING'
    finally:
        svc.shutdown()


def test_restored_stop_intent_keeps_failure_and_late_telemetry():
    store = SearchStateStore()
    store.reset(session_id='same-session', target='chair')
    store.apply(event(SESSION_STARTED))
    store.apply(event(STOP_REQUESTED, number=2))
    restored = SearchStateStore()
    restored.restore(store.snapshot())
    restored.apply(event(RESUMED, number=3))
    assert restored.snapshot()['status'] == 'STOPPING'
    restored.apply(event(ERROR, {'message': 'worker failed during stopping'}, number=4))
    assert restored.snapshot()['status'] == restored.snapshot()['phase'] == 'FAILED'
    restored.apply(event(SESSION_STARTED, number=5))
    assert restored.snapshot()['status'] == 'FAILED'
    restored.apply(event(OBSERVATION_UPDATED, {'bundle_id': 'final-frame'}, number=6))
    assert restored.snapshot()['observation']['bundle_id'] == 'final-frame'


def test_status_and_command_gate_do_not_copy_scene_but_full_state_still_does(tmp_path, monkeypatch):
    svc, _ = service(tmp_path)
    calls = []
    original = svc._store.snapshot

    def full_state():
        calls.append('full')
        return original()

    monkeypatch.setattr(svc._store, 'snapshot', full_state)
    try:
        assert svc.current_session().status == svc._active_status() == 'RUNNING'
        assert calls == []
        assert svc.state_snapshot()['status'] == 'RUNNING'
        assert calls == ['full']
        calls.clear()
        svc.stop_search()
        assert svc.current_session().status == svc._active_status() == 'STOPPING'
        assert calls == []
    finally:
        svc.shutdown()


def test_lifecycle_snapshot_matches_full_state_without_copying_graph(monkeypatch):
    import app.live_robot.search_state_store as module
    store = SearchStateStore()
    store.reset(session_id='same-session', target='chair')
    store.apply(event(SESSION_STARTED))
    store.apply(event(OBSERVATION_UPDATED, {'bundle_id': 'retained-frame'}, number=2))
    before = store.snapshot()
    original = module.copy.deepcopy

    def only_lifecycle(value):
        assert set(value) == {'status', 'result', 'phase'}
        return original(value)

    with monkeypatch.context() as patch:
        patch.setattr(module.copy, 'deepcopy', only_lifecycle)
        result = store.lifecycle_snapshot()
    assert result == {key: before[key] for key in ('status', 'result', 'phase')}
    result['status'] = 'incorrect'
    assert store.snapshot() == before
