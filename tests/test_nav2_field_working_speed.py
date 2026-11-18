"""Native field commands after the recorded Go2-W actuator experiment."""
import pytest

from tests.test_nav2_search_backend import (
    _backend, _corridor_grid, _goal, _straight_path, SpeedRecorder, _drive_to_navigating,
)
from app.navigation.nav2_search_backend import ActionPoll, ACTION_SUCCEEDED


@pytest.mark.parametrize('half_width', [1.25, .90])
def test_field_working_speed_does_not_add_an_eighty_cm_clearance_gate(half_width):
    speed=SpeedRecorder()
    backend,planner,navigator=_backend(grid=_corridor_grid(half_width),
        speed_setter=speed,translation_speed_mps=.30)
    handle=_drive_to_navigating(backend,planner,navigator)
    assert speed.calls==[pytest.approx(.30)]
    navigator.queue(ActionPoll(status=ACTION_SUCCEEDED))
    result=backend.get_navigation_status(handle)
    decision=result.provenance['candidate_decision']
    assert decision['linear_speed_policy']=='recorded_working_command'
    assert decision['min_side_gap_m']>=.40


def test_field_working_speed_cannot_pass_a_blocked_corridor():
    speed=SpeedRecorder();arms=[]
    backend,planner,navigator=_backend(grid=_corridor_grid(.55),
        speed_setter=speed,translation_speed_mps=.30,
        motion_authorizer=lambda:arms.append(True) or True)
    planner.queue(ActionPoll(status=ACTION_SUCCEEDED,path=_straight_path()))
    result=backend.get_navigation_status(backend.execute_goal(_goal()))
    assert result.status.value=='rejected'
    assert speed.calls==[] and arms==[] and navigator.sent==[]


def test_readonly_field_reports_working_speed_without_setting_or_sending_it():
    speed=SpeedRecorder()
    backend,planner,navigator=_backend(grid=_corridor_grid(.90),
        speed_setter=speed,translation_speed_mps=.30,dry_run=True)
    planner.queue(ActionPoll(status=ACTION_SUCCEEDED,path=_straight_path()))
    result=backend.get_navigation_status(backend.execute_goal(_goal()))
    decision=result.provenance['candidate_decision']
    assert decision['speed_limit_mps']==.30
    assert decision['geometric_reference_speed_mps']<.1
    assert speed.calls==[] and navigator.sent==[]


@pytest.mark.parametrize('speed',[.05,.10,.12,.15,float('nan')])
def test_field_speed_cannot_be_silently_invented(speed):
    with pytest.raises(ValueError,match='measured working command'):
        _backend(translation_speed_mps=speed)
