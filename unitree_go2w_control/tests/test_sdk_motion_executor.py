"""Exercise actual Unix sockets and watchdog threads without vendor SDK/DDS."""
from __future__ import annotations
import json
import socket
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from sdk_motion_executor import SdkMotionExecutor, executor_timeouts


class FakeClient:
    def __init__(self):
        self.calls = []
        self.velocity = (0., 0., 0.)
        self.lease = 42
        self.move_code = 0
        self.stop_code = 0
        self.stopped = threading.Event()

    def GetLeaseId(self):
        return self.lease

    def Move(self, x, y, z):
        self.calls.append(('move', time.monotonic(), x, y, z))
        if self.move_code == 0:
            self.velocity = (x, y, z)
        return self.move_code

    def StopMove(self):
        self.calls.append(('stop', time.monotonic()))
        if self.stop_code == 0:
            self.velocity = (0., 0., 0.)
            self.stopped.set()
        return self.stop_code


@pytest.fixture
def executor(tmp_path):
    running = []
    def start(timeout=.12):
        client = FakeClient()
        path = tmp_path/('sdk_%d.sock' % len(running))
        stop = threading.Event()
        owner = SdkMotionExecutor(client, threading.Lock(), path, stop,
                                  command_timeout_seconds=timeout)
        thread = threading.Thread(target=owner.serve)
        thread.start()
        running.append((owner, thread, stop))
        end = time.monotonic()+2
        while time.monotonic()<end:
            try:
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as probe:
                    probe.settimeout(.5)
                    probe.connect(str(path))
                    # A rejected request proves listen/dispatch are ready;
                    # socket path existence alone precedes listen().
                    probe.sendall(b'{}\n')
                    assert json.loads(probe.recv(4096))['status_code'] == -9996
                break
            except (FileNotFoundError, ConnectionRefusedError):
                time.sleep(.005)
        else:
            raise AssertionError('executor did not become ready')
        return client, owner, path, stop
    yield start
    for owner, thread, stop in running:
        stop.set()
        owner.close()
        thread.join(timeout=4)
        assert not thread.is_alive()
        assert owner._watchdog_thread is None or not owner._watchdog_thread.is_alive()


def request(path, *, x=.1, api=1008):
    body = dict(request_id=time.time_ns(), api_id=api,
                parameter={'x': x, 'y': 0., 'z': 0.} if api == 1008 else {})
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as stream:
        stream.settimeout(1)
        stream.connect(str(path))
        stream.sendall(json.dumps(body).encode()+b'\n')
        with stream.makefile('rb') as reader:
            result = json.loads(reader.readline())
    assert result['request_id'] == body['request_id']
    return result


def test_legacy_default_has_no_watchdog_or_extra_stop(executor):
    client, owner, path, _ = executor(0)
    assert request(path)['status_code'] == 0
    assert not client.stopped.wait(.18)
    assert owner._watchdog_thread is None
    assert client.velocity == (.1, 0., 0.)


def test_silent_client_stops_and_does_not_repeat_successful_stop(executor):
    client, _, path, _ = executor()
    assert request(path)['status_code'] == 0
    assert client.stopped.wait(1)
    assert client.velocity == (0., 0., 0.)
    before = len(client.calls)
    time.sleep(.18)
    assert len(client.calls) == before


def test_fresh_commands_renew_deadline_over_many_intervals(executor):
    client, _, path, _ = executor()
    for _ in range(10):
        assert request(path)['status_code'] == 0
        assert not client.stopped.wait(.025)
    assert client.stopped.wait(1)


@pytest.mark.parametrize('api', [1003, 1008])
def test_explicit_stop_or_zero_disarms_watchdog(executor, api):
    client, _, path, _ = executor()
    request(path)
    assert request(path, api=api, x=0.)['status_code'] == 0
    before = len(client.calls)
    time.sleep(.2)
    assert len(client.calls) == before
    assert client.velocity == (0., 0., 0.)


def test_rejected_move_does_not_refresh_previous_success(executor):
    client, _, path, _ = executor()
    request(path)
    client.move_code = 123
    for _ in range(3):
        assert request(path)['status_code'] == 123
        time.sleep(.02)
    assert client.stopped.wait(1)


def test_stop_failure_retries_until_success(executor):
    client, _, path, _ = executor()
    client.stop_code = 11
    request(path)
    end = time.monotonic()+1
    while not any(row[0] == 'stop' for row in client.calls) and time.monotonic()<end:
        time.sleep(.005)
    assert client.velocity == (.1, 0., 0.)
    client.stop_code = 0
    assert client.stopped.wait(1)
    assert sum(row[0] == 'stop' for row in client.calls) >= 2


def test_slow_socket_reader_does_not_block_independent_watchdog(executor):
    client, _, path, _ = executor()
    request(path)
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as stalled:
        stalled.connect(str(path))
        stalled.sendall(b'{')  # Server waits for the rest, outside the SDK lock.
        assert client.stopped.wait(1)


def test_close_stops_active_command_and_joins_watchdog(executor):
    client, owner, path, _ = executor()
    request(path)
    owner.close()
    assert client.stopped.is_set()
    assert not owner._watchdog_thread.is_alive()


def test_idle_executor_does_not_send_unsolicited_stop(executor):
    client, _, _, _ = executor()
    assert not client.stopped.wait(.18)
    assert not client.calls


@pytest.mark.parametrize('command,rpc', [(float('nan'),.2),(-1,.2),(.3,float('inf')),
    (.3,0),(.3,.4),(True,.2),(.3,False)])
def test_bad_or_unbounded_timeout_contract_is_rejected(command,rpc):
    with pytest.raises(ValueError):
        executor_timeouts(command,rpc)


def test_only_explicit_deployment_enables_watchdog_contract():
    assert executor_timeouts() == (0., 5.)
    assert executor_timeouts(.3,.2) == (.3,.2)


def lease_holder_without_vendor(monkeypatch):
    import importlib.util
    import types
    calls = []
    for name in ('unitree_sdk2py', 'unitree_sdk2py.comm', 'unitree_sdk2py.comm.motion_switcher',
                 'unitree_sdk2py.core', 'unitree_sdk2py.go2', 'unitree_sdk2py.go2.sport'):
        package = types.ModuleType(name); package.__path__ = []
        monkeypatch.setitem(sys.modules, name, package)
    channel = types.ModuleType('unitree_sdk2py.core.channel')
    channel.ChannelFactoryInitialize = lambda *args: calls.append(('channel', args))
    monkeypatch.setitem(sys.modules, channel.__name__, channel)
    class Client(FakeClient):
        def __init__(self, enableLease):
            super().__init__();calls.append(('client',enableLease))
        def SetTimeout(self, timeout):calls.append(('timeout',timeout))
        def Init(self):calls.append(('init',))
    sport = types.ModuleType('unitree_sdk2py.go2.sport.sport_client');sport.SportClient = Client
    monkeypatch.setitem(sys.modules, sport.__name__, sport)
    switcher = types.ModuleType('unitree_sdk2py.comm.motion_switcher.motion_switcher_client')
    event = []
    class Switcher:
        def SetTimeout(self, timeout):pass
        def Init(self):pass
        def CheckMode(self):
            event[0].set()
            return 0, {'name':'ai-w','form':'w'}
    switcher.MotionSwitcherClient = Switcher
    monkeypatch.setitem(sys.modules, switcher.__name__, switcher)
    path = Path(__file__).resolve().parents[1]/'scripts/hold_sport_lease.py'
    spec = importlib.util.spec_from_file_location('isolated_holder_test',path)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    actual = module.SdkMotionExecutor
    def create(client,lock,path,stop,**kwargs):
        event.append(stop)
        return actual(client,lock,path,stop,**kwargs)
    monkeypatch.setattr(module,'SdkMotionExecutor',create)
    return module,calls


def test_invalid_timeouts_fail_before_creating_a_channel_or_client(monkeypatch):
    module,calls = lease_holder_without_vendor(monkeypatch)
    monkeypatch.setattr(sys,'argv',['holder','--interface','lo','--command-timeout-seconds','.3',
                                   '--rpc-timeout-seconds','2'])
    with pytest.raises(SystemExit) as error:
        module.main()
    assert error.value.code == 2
    assert not calls


def test_watchdog_commands_use_short_timeout_after_lease_acquisition(monkeypatch,tmp_path):
    import signal
    module,calls = lease_holder_without_vendor(monkeypatch)
    monkeypatch.setattr(sys,'argv',['holder','--interface','lo','--command-timeout-seconds','.3',
        '--rpc-timeout-seconds','.2','--socket-path',str(tmp_path/'holder.sock'),
        '--status-dir',str(tmp_path/'status')])
    previous = {s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
    try:
        assert module.main() == 0
    finally:
        for sig,handler in previous.items():
            signal.signal(sig,handler)
    assert [row[1] for row in calls if row[0]=='timeout'] == [5., .2]
    assert calls.index(('init',)) < calls.index(('timeout',.2))
    assert (tmp_path/'status/alive').read_text().strip() == '0'
    assert not (tmp_path/'holder.sock').exists()
