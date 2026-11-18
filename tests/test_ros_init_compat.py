import sys
from types import ModuleType, SimpleNamespace

import pytest

from app.ros_compat import init_preserving_python_signals


@pytest.mark.parametrize('modern', [False, True])
def test_ros_distro_uses_supported_signal_initialization(monkeypatch, modern):
    calls = []
    module, signals = ModuleType('rclpy'), ModuleType('rclpy.signals')
    module.init = lambda **kw: calls.append(kw)
    no_handlers = object()
    if modern:
        signals.SignalHandlerOptions = SimpleNamespace(NO=no_handlers)
    monkeypatch.setitem(sys.modules, 'rclpy', module)
    monkeypatch.setitem(sys.modules, 'rclpy.signals', signals)
    init_preserving_python_signals()
    assert calls == ([{'signal_handler_options': no_handlers}] if modern else [{}])


def test_initialization_failure_is_not_retried_with_other_options(monkeypatch):
    module, signals = ModuleType('rclpy'), ModuleType('rclpy.signals')
    calls = []

    def init(**kw):
        calls.append(kw)
        raise RuntimeError('context initialization failed')

    module.init = init
    signals.SignalHandlerOptions = SimpleNamespace(NO=0)
    monkeypatch.setitem(sys.modules, 'rclpy', module)
    monkeypatch.setitem(sys.modules, 'rclpy.signals', signals)
    with pytest.raises(RuntimeError, match='context initialization failed'):
        init_preserving_python_signals()
    assert calls == [{'signal_handler_options': 0}]
