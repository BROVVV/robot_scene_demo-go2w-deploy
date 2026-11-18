import pytest
from go2w_cmd_vel_bridge.velocity_guard import guard_reason


def healthy():
    return dict(raw_age=.02,command_age=.01,scan_age=.03,scan_receive_age=.02,
                localization_ok=True,linear=.1,angular=0.)


def test_current_commands_and_scan_are_allowed():
    assert guard_reason(**healthy())==''


def test_explicit_controller_stop_overrides_a_still_moving_smoother_tail():
    assert guard_reason(**healthy(),raw_stopped=True)=='controller_stop'


@pytest.mark.parametrize('key,value', [('raw_age',.31),('command_age',.31),('scan_age',.401),
    ('scan_age',-.11),('scan_receive_age',.301),('localization_ok',False),('linear',float('nan'))])
def test_stale_sensors_and_commands_are_stopped(key,value):
    inputs=healthy();inputs[key]=value
    assert guard_reason(**inputs)


def test_release08_observed_scan_jitter_remains_inside_collision_monitor_timeout():
    inputs=healthy();inputs.update(scan_age=.361,scan_receive_age=.254)
    assert guard_reason(**inputs)==''
