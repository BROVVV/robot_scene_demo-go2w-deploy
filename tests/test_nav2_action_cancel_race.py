from __future__ import annotations

from concurrent.futures import Future
from types import SimpleNamespace
import threading
import sys
import types
import pytest

from app.navigation.nav2_search_backend import (
    RosSpinActionClientPort, SpinGoalRequest, ACTION_UNKNOWN,
    ACTION_ACCEPTED, ACTION_CANCELING, ACTION_CANCELED, ACTION_ABORTED,
    ACTION_SUCCEEDED)


class GoalHandle:
    accepted = True
    def __init__(self):
        self.cancel_calls = 0
        self.result = Future()
    def cancel_goal_async(self):
        self.cancel_calls += 1
        return Future()
    def get_result_async(self): return self.result


class Client:
    def __init__(self): self.sends = []
    def send_goal_async(self, goal):
        future = Future(); self.sends.append(future); return future


def port():
    p = object.__new__(RosSpinActionClientPort)
    p._action_type = SimpleNamespace(Goal=lambda: SimpleNamespace(
        time_allowance=SimpleNamespace(sec=0,nanosec=0)))
    p._client = Client(); p._action_name = '/spin'
    p._goal_handle = None; p._status = ACTION_UNKNOWN; p._path = (); p._error = ''
    p._goal_lock = threading.RLock(); p._goal_generation = 0
    p._goal_send_pending = False; p._cancel_requested = False
    p._cancel_sent = False
    return p


def send(p):
    p.send(SpinGoalRequest(target_yaw=.1, time_allowance_s=2.))


def test_cancel_before_acceptance_is_delivered_to_the_late_goal_handle():
    p = port(); send(p)
    assert p.cancel() is True
    assert p.poll().status == ACTION_CANCELING
    goal = GoalHandle(); p._client.sends[0].set_result(goal)
    assert goal.cancel_calls == 1
    goal.result.set_result(SimpleNamespace(status=5,result=None))
    assert p.poll().status == ACTION_CANCELED


def test_late_acceptance_cannot_replace_the_new_goals_state():
    p = port(); send(p); p.cancel(); send(p)
    current = GoalHandle(); p._client.sends[1].set_result(current)
    retired = GoalHandle(); p._client.sends[0].set_result(retired)
    assert retired.cancel_calls == 1
    assert p._goal_handle is current
    assert p.poll().status == ACTION_ACCEPTED


def test_late_result_cannot_complete_a_new_goal():
    p = port(); send(p)
    retired = GoalHandle(); p._client.sends[0].set_result(retired)
    p.cancel(); send(p)
    current = GoalHandle(); p._client.sends[1].set_result(current)
    retired.result.set_result(SimpleNamespace(status=4,result=None))
    assert p._goal_handle is current
    assert p.poll().status == ACTION_ACCEPTED


def test_idle_cancel_does_not_claim_an_action_was_canceled():
    assert port().cancel() is False


@pytest.mark.parametrize('response', [None, SimpleNamespace(accepted=False)])
def test_rejection_after_pending_cancel_is_terminal(response):
    p = port(); send(p); p.cancel()
    p._client.sends[0].set_result(response)
    assert p.poll().status == ACTION_ABORTED
    assert p.cancel() is False


def test_late_failed_send_does_not_abort_the_current_goal():
    p = port(); send(p); send(p)
    current = GoalHandle(); p._client.sends[1].set_result(current)
    p._client.sends[0].set_exception(RuntimeError('old send failure'))
    assert p.poll().status == ACTION_ACCEPTED
    assert p.poll().error == ''


def test_cancel_rpc_failure_keeps_result_subscription_and_can_be_retried():
    class FailedOnce(GoalHandle):
        def cancel_goal_async(self):
            self.cancel_calls += 1
            if self.cancel_calls == 1:
                raise RuntimeError('transport unavailable')
            return Future()
    p = port(); send(p); p.cancel()
    goal = FailedOnce(); p._client.sends[0].set_result(goal)
    assert p.poll().status == ACTION_CANCELING
    assert 'transport unavailable' in p.poll().error
    assert p.cancel() is True
    assert goal.cancel_calls == 2
    goal.result.set_result(SimpleNamespace(status=5,result=None))
    assert p.poll().status == ACTION_CANCELED


def test_terminal_goal_is_not_recanceled_or_reported_as_canceling():
    p = port(); send(p)
    goal = GoalHandle(); p._client.sends[0].set_result(goal)
    goal.result.set_result(SimpleNamespace(status=4,result=None))
    assert p.cancel() is False
    assert p.poll().status == ACTION_SUCCEEDED
    assert goal.cancel_calls == 0


def test_cancel_is_ordered_with_a_concurrent_send():
    p = port(); entered = threading.Event(); release = threading.Event()
    original = p._client.send_goal_async
    def blocked(goal):
        entered.set()
        assert release.wait(5)
        return original(goal)
    p._client.send_goal_async = blocked
    sender = threading.Thread(target=send,args=(p,))
    canceled = []
    sender.start(); assert entered.wait(5)
    canceler = threading.Thread(target=lambda: canceled.append(p.cancel()))
    canceler.start(); release.set()
    sender.join(5); canceler.join(5)
    assert not sender.is_alive() and not canceler.is_alive()
    assert canceled == [True]
    goal = GoalHandle(); p._client.sends[0].set_result(goal)
    assert goal.cancel_calls == 1


@pytest.mark.parametrize('distro,enabled,expected', [
    ('humble','0','native'),('foxy','0','native'),('foxy','1','bounded')])
def test_factory_selects_the_explicit_duration_capable_foxy_action(monkeypatch,distro,enabled,expected):
    from app.navigation import nav2_search_backend as backend
    native=object(); bounded=object()
    nav=types.ModuleType('nav2_msgs.action')
    nav.ComputePathToPose=object();nav.NavigateToPose=object();nav.Spin=native
    compat=types.ModuleType('go2w_nav2_compat_msgs.action');compat.Spin=bounded
    monkeypatch.setitem(sys.modules,'nav2_msgs.action',nav)
    monkeypatch.setitem(sys.modules,'go2w_nav2_compat_msgs.action',compat)
    monkeypatch.setenv('ROS_DISTRO',distro);monkeypatch.setenv('GO2W_FOXY_SPIN_ACTION',enabled)
    monkeypatch.setattr(backend,'RosActionClientPort',lambda node,name,action:action)
    monkeypatch.setattr(backend,'RosSpinActionClientPort',lambda node,name,action:action)
    ports=backend.create_nav2_action_clients(None)
    assert ports[-1] is (bounded if expected=='bounded' else native)
