"""Action success must agree with the current map pose after a SLAM correction."""
import pytest
from tests.test_nav2_search_backend import _backend,_corridor_grid,_drive_to_navigating,_pose
from app.navigation.nav2_search_backend import ActionPoll,ACTION_SUCCEEDED


@pytest.mark.parametrize('x,yaw,expected',[(1.94,0.,'failed'),(1.99,8.,'failed'),(1.98,0.,'succeeded')])
def test_action_result_does_not_override_final_pose_tolerance(x,yaw,expected):
    backend,planner,navigator=_backend(grid=_corridor_grid(1.25))
    backend._goal_xy_tolerance_m=.03;backend._goal_yaw_tolerance_rad=.05
    handle=_drive_to_navigating(backend,planner,navigator)
    backend._pose_provider=lambda:_pose(x=x,yaw_deg=yaw)
    navigator.queue(ActionPoll(status=ACTION_SUCCEEDED))
    result=backend.get_navigation_status(handle)
    assert result.status.value==expected
    assert result.observed_motion['final_position_error_m']==pytest.approx(abs(2.-x))
    if expected=='failed':
        assert 'outside current goal tolerance' in result.message
        assert not result.provenance['candidate_decision']['accepted']


def test_success_without_final_pose_is_rejected():
    backend,planner,navigator=_backend(grid=_corridor_grid(1.25))
    backend._goal_xy_tolerance_m=.03
    handle=_drive_to_navigating(backend,planner,navigator)
    backend._pose_provider=lambda:None
    navigator.queue(ActionPoll(status=ACTION_SUCCEEDED))
    result=backend.get_navigation_status(handle)
    assert result.status.value=='rejected' and 'fresh final pose' in result.message
