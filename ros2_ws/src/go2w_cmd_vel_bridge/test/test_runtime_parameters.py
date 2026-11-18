from types import SimpleNamespace

from go2w_cmd_vel_bridge.bridge_core import Velocity
from go2w_cmd_vel_bridge.bridge_node import CmdVelBridge


def _parameter(name, value):
    return SimpleNamespace(name=name, value=value)


def test_runtime_disable_updates_live_state_sends_zero_and_cancels():
    bridge = CmdVelBridge.__new__(CmdVelBridge)
    bridge._execution_enabled = True
    bridge._requested = Velocity(0.1, 0.2)
    bridge._previous = Velocity(0.1, 0.2)
    bridge._dispatch_permitted = True
    bridge._stream_token = "active-token"
    published = []
    canceled = []
    bridge._publish_stream = published.append
    bridge._cancel_active = lambda: canceled.append(True)

    result = bridge._parameters_update([_parameter("execution_enabled", False)])

    assert result.successful
    assert bridge._execution_enabled is False
    assert bridge._requested.is_zero and bridge._previous.is_zero
    assert bridge._dispatch_permitted is False
    assert published == [Velocity()]
    assert canceled == [True]


def test_runtime_enable_is_rejected_and_cannot_change_live_state():
    bridge = CmdVelBridge.__new__(CmdVelBridge)
    bridge._execution_enabled = False

    result = bridge._parameters_update([_parameter("execution_enabled", True)])

    assert not result.successful
    assert "restart" in result.reason
    assert bridge._execution_enabled is False


def test_guard_and_geometry_bindings_are_startup_only():
    for name in (
        "trial_ledger",
        "geometry_execution_scope",
        "geometry_profile_sha256",
        "operator_supervised_rotation",
    ):
        bridge = CmdVelBridge.__new__(CmdVelBridge)
        bridge._execution_enabled = False
        result = bridge._parameters_update([_parameter(name, "changed")])
        assert not result.successful, name
        assert "startup-only" in result.reason
