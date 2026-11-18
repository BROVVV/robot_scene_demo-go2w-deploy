"""Normal ROS interruption closes the node without hiding unrelated failures."""
from types import SimpleNamespace

import pytest
from go2w_sensor_time_bridge import time_bridge_node as module


@pytest.mark.parametrize('interruption', ['keyboard', 'external'])
@pytest.mark.parametrize('context_active', [False, True])
def test_normal_interrupt_cleans_up_and_returns(monkeypatch, interruption, context_active):
    calls=[]
    node=SimpleNamespace(destroy_node=lambda:calls.append('destroy'))
    error=KeyboardInterrupt() if interruption=='keyboard' else module.ExternalShutdownException()
    def spin(actual):
        assert actual is node
        raise error
    monkeypatch.setattr(module,'TimeBridge',lambda:node)
    monkeypatch.setattr(module.rclpy,'init',lambda **kw:calls.append(('init',kw)))
    monkeypatch.setattr(module.rclpy,'spin',spin)
    monkeypatch.setattr(module.rclpy,'ok',lambda:context_active)
    monkeypatch.setattr(module.rclpy,'shutdown',lambda:calls.append('shutdown'))
    escaped=None
    try:module.main(args=['--ros-args'])
    except BaseException as exc:escaped=exc
    assert escaped is None, 'normal interruption escaped: '+repr(escaped)
    assert calls==[('init',{'args':['--ros-args']}),'destroy']+(['shutdown'] if context_active else [])


def test_unrelated_spin_failure_still_propagates_after_cleanup(monkeypatch):
    calls=[]
    def fail(_):raise ValueError('unexpected callback failure')
    monkeypatch.setattr(module,'TimeBridge',lambda:SimpleNamespace(destroy_node=lambda:calls.append('destroy')))
    monkeypatch.setattr(module.rclpy,'init',lambda **kw:None)
    monkeypatch.setattr(module.rclpy,'spin',fail)
    monkeypatch.setattr(module.rclpy,'ok',lambda:True)
    monkeypatch.setattr(module.rclpy,'shutdown',lambda:calls.append('shutdown'))
    with pytest.raises(ValueError,match='unexpected callback failure'):module.main()
    assert calls==['destroy','shutdown']
