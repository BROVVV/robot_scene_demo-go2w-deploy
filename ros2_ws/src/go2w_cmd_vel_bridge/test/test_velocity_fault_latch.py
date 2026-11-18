"""Replay loss/recovery without ROS or a physical command publisher."""
import ast
from pathlib import Path

import pytest


path=Path(__file__).parents[1]/'go2w_cmd_vel_bridge/velocity_guard.py'
tree=ast.parse(path.read_text())
definition=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='FaultStopLatch')
namespace={}
exec(compile(ast.Module(body=[definition],type_ignores=[]),str(path),'exec'),namespace)
FaultStopLatch=namespace['FaultStopLatch']


@pytest.mark.parametrize('fault',['scan_stale','localization_stale','smoothed_command_stale',
    'controller_command_stale','nonfinite_command'])
def test_running_fault_stays_stopped_when_input_recovers(fault):
    gate=FaultStopLatch();gate.allowed_output(.12,0)
    assert gate.apply(fault)==fault
    for _ in range(10):
        assert gate.apply('')==fault+'_waiting_controller_stop'
    # Another nonzero controller command is not the reset handshake.
    gate.allowed_output(.12,0)
    assert gate.apply('')==fault+'_waiting_controller_stop'
    gate.controller_stop()
    assert gate.apply('controller_stop')=='controller_stop'
    assert gate.apply('')==''


def test_missing_startup_inputs_do_not_latch_before_motion():
    gate=FaultStopLatch()
    assert gate.apply('scan_stale')=='scan_stale'
    assert gate.apply('')==''
    gate.allowed_output(0,0)
    assert gate.apply('localization_stale')=='localization_stale'
    assert gate.apply('')==''


def test_turning_also_requires_zero_after_a_fault():
    gate=FaultStopLatch();gate.allowed_output(0,.1)
    assert gate.apply('scan_stale')=='scan_stale'
    assert gate.apply('')=='scan_stale_waiting_controller_stop'


def test_zero_ends_old_motion_and_new_fault_does_not_reuse_old_running_state():
    gate=FaultStopLatch();gate.allowed_output(.12,0);gate.controller_stop()
    assert gate.apply('scan_stale')=='scan_stale'
    assert gate.apply('')==''
    gate.allowed_output(.12,0)
    assert gate.apply('localization_stale')=='localization_stale'
    assert gate.apply('')=='localization_stale_waiting_controller_stop'


def test_shadow04_observed_fault_recovery_sequence_has_no_second_allowed_output():
    gate=FaultStopLatch();gate.allowed_output(.1675,-.07456)
    # +12.8ms fault, +108.7ms recovered inputs, +147.1ms controller stop.
    assert gate.apply('scan_stale')=='scan_stale'
    assert gate.apply('')=='scan_stale_waiting_controller_stop'
    gate.controller_stop()
    assert gate.apply('controller_stop')=='controller_stop'
