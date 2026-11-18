import pytest
from go2w_control_arbiter.emergency_reset import reset_blocker


def ready():
    return dict(emergency_stop=(False,10.),software_arm=(True,10.),remote_override=(False,10.))


def test_explicit_reset_needs_three_actual_fresh_conditions():
    assert reset_blocker(ready(),10.1)==''
    assert reset_blocker({},10.1)


@pytest.mark.parametrize('name', ['emergency_stop','software_arm','remote_override'])
def test_stale_false_and_reversed_flags_never_reset(name):
    samples=ready();samples[name]=(not samples[name][0],10.)
    assert reset_blocker(samples,10.1)
    samples=ready();samples[name]=(samples[name][0],9.)
    assert reset_blocker(samples,10.1)
    assert reset_blocker(ready(),9.9)
