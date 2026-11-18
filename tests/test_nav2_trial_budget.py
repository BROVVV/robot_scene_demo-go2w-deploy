import json
import math
from pathlib import Path

import pytest

from app.navigation.robot_backend import RobotPose
from app.navigation.trial_budget import NavigationTrialBudget, TrialBudgetError
from app.navigation.models import ExplorationGoal
from app.navigation.nav2_search_backend import Nav2SearchBackend, _ActiveGoal, Nav2GoalRequest
from app.navigation.robot_backend import NavigationHandle


def pose(x=0, y=0, yaw=0):
    return RobotPose(x=x, y=y, yaw=yaw, frame_id='map')


def budget(tmp_path):
    return NavigationTrialBudget.initialize(tmp_path/'trial.json', window_id='test',
        reviewed_robot_boot_id='test-robot-boot')


def reserve(b, **overrides):
    values = dict(goal_id='g', path_length_m=.12, goal_x=.12, goal_y=0,
        goal_yaw=0, pose=pose())
    values.update(overrides)
    b.reserve(**values)


def test_forward_window_is_single_use_and_independent_of_old_budget(tmp_path):
    old=budget(tmp_path)
    reserve(old); old.finish(); original=old.path.read_bytes()
    forward=NavigationTrialBudget.initialize_forward(tmp_path/'forward.json',
        window_id='operator-forward-3m',reviewed_robot_boot_id='new-boot')
    assert forward.MAX_STEPS==1 and forward.MAX_TRANSLATION_M==3.
    assert forward.STOP_RESERVE_M==.15 and forward.forward_only
    forward.require_reviewed_boot('new-boot')
    reserve(forward,path_length_m=2.,goal_x=2.)
    assert forward.observe(pose(.5))==''
    assert 'travel' in forward.observe(pose(2.86))
    forward.finish()
    with pytest.raises(TrialBudgetError,match='exhausted'):
        NavigationTrialBudget(forward.path).require_reviewed_boot('new-boot')
    assert old.path.read_bytes()==original
    with pytest.raises(FileExistsError):
        NavigationTrialBudget.initialize_forward(forward.path,window_id='reset',reviewed_robot_boot_id='new-boot')


def test_collision_diagnostic_is_single_use_and_stops_at_authorized_twenty_cm(tmp_path):
    diagnostic=NavigationTrialBudget.initialize_collision_diagnostic(
        tmp_path/'collision.json',window_id='operator-0.20m',reviewed_robot_boot_id='boot')
    assert isinstance(json.loads(diagnostic.path.read_text())['created_wall'],float)
    assert diagnostic.MAX_STEPS==1 and diagnostic.MAX_TRANSLATION_M==.25
    assert diagnostic.STOP_RESERVE_M==.05 and diagnostic.forward_only
    reserve(diagnostic,path_length_m=.20,goal_x=.20)
    assert diagnostic.observe(pose(.19))==''
    assert 'travel' in diagnostic.observe(pose(.20))
    diagnostic.finish()
    with pytest.raises(TrialBudgetError,match='exhausted'):
        NavigationTrialBudget(diagnostic.path).require_reviewed_boot('boot')
    with pytest.raises(FileExistsError):
        NavigationTrialBudget.initialize_collision_diagnostic(
            diagnostic.path,window_id='reset',reviewed_robot_boot_id='boot')


def test_collision_diagnostic_rejects_goal_beyond_authorized_twenty_cm(tmp_path):
    diagnostic=NavigationTrialBudget.initialize_collision_diagnostic(
        tmp_path/'collision.json',window_id='operator-0.20m',reviewed_robot_boot_id='boot')
    with pytest.raises(TrialBudgetError,match='bounded step envelope'):
        reserve(diagnostic,path_length_m=.201,goal_x=.201)
    assert json.loads(diagnostic.path.read_text())['steps']==[]


def test_speed_calibration_030_is_single_use_straight_and_nonrenewable(tmp_path):
    calibration=NavigationTrialBudget.initialize_speed_calibration_030(
        tmp_path/'speed030.json',window_id='operator-speed-030',reviewed_robot_boot_id='boot')
    assert calibration.MAX_STEPS==1 and calibration.MAX_SPEED_MPS==.30
    assert calibration.MAX_TRANSLATION_M==.60 and calibration.STOP_RESERVE_M==.15
    assert calibration.forward_only and calibration.speed_calibration_0_30
    reserve(calibration,path_length_m=.30,goal_x=.30)
    calibration.finish()
    with pytest.raises(TrialBudgetError,match='exhausted'):
        NavigationTrialBudget(calibration.path).require_reviewed_boot('boot')
    with pytest.raises(FileExistsError):
        NavigationTrialBudget.initialize_speed_calibration_030(
            calibration.path,window_id='reset',reviewed_robot_boot_id='boot')


def test_speed_calibration_030_rejects_wider_goal_and_policy_edit(tmp_path):
    calibration=NavigationTrialBudget.initialize_speed_calibration_030(
        tmp_path/'speed030.json',window_id='operator-speed-030',reviewed_robot_boot_id='boot')
    with pytest.raises(TrialBudgetError,match='bounded step envelope'):
        reserve(calibration,path_length_m=.451,goal_x=.451)
    data=json.loads(calibration.path.read_text());data['maximum_speed_mps']=.31
    calibration.path.write_text(json.dumps(data))
    with pytest.raises(TrialBudgetError,match='policy mismatch'):
        NavigationTrialBudget(calibration.path)


def test_five_goal_field_window_is_durable_and_cannot_refund_old_window(tmp_path):
    old=NavigationTrialBudget.initialize_search(tmp_path/'old.json',window_id='old',
        reviewed_robot_boot_id='boot',prior_translation_charge_m=.66)
    for _ in range(3):
        reserve(old);old.finish()
    old_bytes=old.path.read_bytes()
    field=NavigationTrialBudget.initialize_field_search(tmp_path/'field.json',
        window_id='operator-five-goal-field',reviewed_robot_boot_id='boot')
    assert (field.MAX_STEPS,field.MAX_TRANSLATION_M,field.STOP_RESERVE_M)==(5,3.,.15)
    assert field.field_search and field.bounded_search
    for remaining in range(5,0,-1):
        assert field.require_reviewed_boot('boot')==remaining
        reserve(field,path_length_m=2.8,goal_x=2.8)
        field.finish()  # A failed goal still consumes its whole slot.
        field=NavigationTrialBudget(field.path)
    with pytest.raises(TrialBudgetError,match='exhausted'):
        field.require_reviewed_boot('boot')
    with pytest.raises(FileExistsError):
        NavigationTrialBudget.initialize_field_search(field.path,window_id='reset',
            reviewed_robot_boot_id='boot')
    assert old.path.read_bytes()==old_bytes


@pytest.mark.parametrize('change',[dict(path_length_m=2.86),dict(goal_x=2.86),
    dict(goal_yaw=math.radians(21))])
def test_field_window_rejects_goal_outside_three_meter_envelope(tmp_path,change):
    field=NavigationTrialBudget.initialize_field_search(tmp_path/'field.json',
        window_id='five',reviewed_robot_boot_id='boot')
    with pytest.raises(TrialBudgetError):reserve(field,**change)
    assert json.loads(field.path.read_text())['steps']==[]


def test_field_window_records_max_heading_excursion_not_correction_sum(tmp_path):
    field=NavigationTrialBudget.initialize_field_search(tmp_path/'field.json',
        window_id='five',reviewed_robot_boot_id='boot')
    reserve(field)
    assert field.observe(pose(.02,yaw=.10))==''
    assert field.observe(pose(.04,yaw=-.10))==''
    assert field.observe(pose(.06,yaw=.05))==''
    step=json.loads(field.path.read_text())['steps'][-1]
    assert step['turned_rad']==pytest.approx(.10)
    assert field.observe(pose(.08,yaw=math.radians(21)))=='trial observed turn limit'


@pytest.mark.parametrize('changes',[dict(goal_x=-.1),dict(goal_y=.1),dict(path_length_m=2.9),
                                   dict(goal_yaw=.16)])
def test_forward_window_rejects_other_directions_before_reserving(tmp_path,changes):
    b=NavigationTrialBudget.initialize_forward(tmp_path/'forward.json',window_id='test',
        reviewed_robot_boot_id='boot')
    with pytest.raises(TrialBudgetError):reserve(b,**changes)
    assert json.loads(b.path.read_text())['steps']==[]


def test_forward_window_stops_lateral_deviation(tmp_path):
    b=NavigationTrialBudget.initialize_forward(tmp_path/'forward.json',window_id='test',
        reviewed_robot_boot_id='boot')
    reserve(b)
    assert 'direction' in b.observe(pose(.1,.16))


def test_three_reservations_survive_process_restarts_and_retries(tmp_path):
    b = budget(tmp_path)
    for _ in range(3):
        reserve(b)
        b.finish()  # Even failed arm/send consumes its reservation.
        b = NavigationTrialBudget(b.path)
    with pytest.raises(TrialBudgetError, match='exhausted'):
        reserve(b)
    with pytest.raises(FileExistsError):
        NavigationTrialBudget.initialize(b.path, window_id='reset', reviewed_robot_boot_id='boot')


def test_separately_authorized_calibration_preserves_three_step_and_distance_limits(tmp_path):
    old=budget(tmp_path)
    reserve(old);old.finish();original=old.path.read_bytes()
    new=NavigationTrialBudget.initialize(tmp_path/'calibration.json',window_id='additional-authorized-window',
        reviewed_robot_boot_id='new-boot',maximum_speed_mps=.12)
    assert new.MAX_SPEED_MPS==.12 and old.MAX_SPEED_MPS==.05
    for _ in range(3):reserve(new);new.finish();new=NavigationTrialBudget(new.path)
    with pytest.raises(TrialBudgetError,match='exhausted'):reserve(new)
    assert old.path.read_bytes()==original
    assert new.MAX_TRANSLATION_M==.2 and new.STOP_RESERVE_M==.05


def test_calibration_speed_policy_cannot_be_changed_in_a_live_ledger(tmp_path):
    b=budget(tmp_path);data=json.loads(b.path.read_text())
    data.update(version=2,purpose='supervised_velocity_calibration',maximum_speed_mps=.12)
    b.path.write_text(json.dumps(data))
    with pytest.raises(TrialBudgetError,match='policy mismatch'):reserve(b)


def test_one_observed_recheck_cannot_turn_into_three_steps_after_restart(tmp_path):
    b=NavigationTrialBudget.initialize(tmp_path/'one.json',window_id='one-observed-recheck',
        reviewed_robot_boot_id='boot',maximum_speed_mps=.12,maximum_steps=1)
    assert b.require_reviewed_boot('boot')==1
    reserve(b);b.finish();b=NavigationTrialBudget(b.path)
    assert b.MAX_STEPS==1 and b.MAX_SPEED_MPS==.12
    with pytest.raises(TrialBudgetError,match='exhausted'):reserve(b)
    with pytest.raises(TrialBudgetError,match='exhausted'):b.require_reviewed_boot('boot')
    data=json.loads(b.path.read_text());data['maximum_steps']=3;b.path.write_text(json.dumps(data))
    with pytest.raises(TrialBudgetError,match='policy mismatch'):NavigationTrialBudget(b.path)


def test_crash_or_concurrent_owner_cannot_reuse_an_active_reservation(tmp_path):
    b = budget(tmp_path)
    reserve(b)
    other = NavigationTrialBudget(b.path)
    with pytest.raises(TrialBudgetError, match='unfinished'):
        reserve(other)
    b.finish()
    reserve(other)
    assert len(json.loads(b.path.read_text())['steps']) == 2


@pytest.mark.parametrize('changes', [dict(path_length_m=.16), dict(goal_x=.16),
    dict(path_length_m=float('nan')), dict(goal_yaw=math.radians(21)), dict(pose=None)])
def test_oversized_or_invalid_plan_is_rejected_before_a_slot_is_used(tmp_path, changes):
    b = budget(tmp_path)
    with pytest.raises(TrialBudgetError):
        reserve(b, **changes)
    assert json.loads(b.path.read_text())['steps'] == []


def test_cumulative_travel_catches_backtracking_and_latches_stop(tmp_path):
    b = budget(tmp_path)
    reserve(b)
    assert not b.observe(pose(.08))
    assert 'travel limit' in b.observe(pose())  # Net displacement zero; total .16m.
    assert 'travel limit' in b.observe(pose())


def test_tracking_pose_uses_continuous_odometry_not_corrected_map_pose(tmp_path):
    b = budget(tmp_path)
    odom = RobotPose(x=10.0, y=4.0, yaw=0.2, frame_id='odom_fused')
    reserve(b, tracking_pose=odom)
    data = json.loads(b.path.read_text())
    assert data['steps'][-1]['planning_pose']['frame'] == 'map'
    assert data['steps'][-1]['initial_pose']['frame'] == 'odom_fused'
    assert not b.observe(RobotPose(
        x=10.08, y=4.0, yaw=0.2, frame_id='odom_fused'
    ))
    step = json.loads(b.path.read_text())['steps'][-1]
    assert step['traveled_m'] == pytest.approx(0.08)


def test_translation_during_rotation_counts_and_pose_loss_stops(tmp_path):
    b = budget(tmp_path)
    reserve(b, path_length_m=0., goal_x=0., goal_yaw=.1)
    assert not b.observe(pose(.099, yaw=.1))
    assert 'rotation translation limit' in b.observe(pose(.101, yaw=.1))
    step = json.loads(b.path.read_text())['steps'][-1]
    assert step['requested_translation_m'] == pytest.approx(0.)
    assert step['requested_turn_rad'] == pytest.approx(.1)
    b.finish()
    reserve(b)
    assert 'finite current pose' in b.observe(None)
    assert 'finite current pose' in b.observe(pose())


def test_cumulative_turn_is_bounded_even_with_small_net_angle(tmp_path):
    b = budget(tmp_path)
    reserve(b)
    assert not b.observe(pose(yaw=.20))
    assert 'turn limit' in b.observe(pose(yaw=0.))


def test_corrupt_or_missing_ledger_fails_closed(tmp_path):
    with pytest.raises(TrialBudgetError):
        NavigationTrialBudget(tmp_path/'missing.json')
    p=tmp_path/'corrupt.json';p.write_text('{}')
    with pytest.raises(TrialBudgetError):
        NavigationTrialBudget(p)


def test_boot_review_cannot_renew_or_refund_three_step_allowance(tmp_path):
    b=budget(tmp_path)
    assert b.require_reviewed_boot('test-robot-boot')==3
    with pytest.raises(TrialBudgetError,match='rebooted'):
        b.require_reviewed_boot('new-boot')
    reserve(b)
    with pytest.raises(TrialBudgetError,match='unfinished'):
        b.require_reviewed_boot('test-robot-boot')
    b.finish()
    assert b.require_reviewed_boot('test-robot-boot')==2


def test_review_after_reboot_keeps_all_consumed_slots(tmp_path):
    b=budget(tmp_path)
    for _ in range(2):reserve(b);b.finish()
    previous=json.loads(b.path.read_text())['steps']
    b.record_boot_review(boot_id='new-boot',audit_sha256='a'*64,sensor_profile_sha256='b'*64)
    assert b.require_reviewed_boot('new-boot')==1
    assert json.loads(b.path.read_text())['steps']==previous
    reserve(b)
    with pytest.raises(TrialBudgetError,match='unfinished'):
        b.record_boot_review(boot_id='other-boot',audit_sha256='a'*64,sensor_profile_sha256='b'*64)


class Client:
    def __init__(self):
        self.sent=[];self.canceled=0
    def send(self, request):
        self.sent.append(request)
    def cancel(self):
        self.canceled+=1
        return True


def active():
    goal=ExplorationGoal(goal_id='g',goal_type='NAVIGATE_POSE',position=(.12,0),yaw=0)
    return _ActiveGoal(goal=goal,handle=NavigationHandle('g'),phase='APPLYING_SPEED',
        request=Nav2GoalRequest(x=.12,y=0,yaw=0,frame_id='map'),
        started_at=0.,plan_sent_at=0.,path_length_m=.12,speed_limit_mps=.1)


def test_backend_reserves_before_arm_caps_speed_and_disarms_success(tmp_path):
    b=budget(tmp_path);client=Client();events=[]
    def arm():
        assert len(json.loads(b.path.read_text())['steps'])==1
        events.append('arm');return True
    backend=Nav2SearchBackend(path_client=Client(),navigate_client=client,trial_budget=b,
        pose_provider=pose, speed_limit_setter=lambda v: events.append(v) or True,
        motion_authorizer=arm, motion_releaser=lambda: events.append('disarm'),
        stop_publisher=lambda: events.append('stop'))
    a=active()
    assert backend._advance_speed_limit(a) is None
    assert events==[.05,'arm'] and len(client.sent)==1
    backend._release_motion(a,succeeded=True)
    assert events[-2:]==['stop','disarm']
    assert json.loads(b.path.read_text())['active_token'] is None


def test_backend_travel_limit_cancels_and_stops_before_next_action_poll(tmp_path):
    b=budget(tmp_path);reserve(b);client=Client();stops=[]
    backend=Nav2SearchBackend(path_client=Client(),navigate_client=client,trial_budget=b,
        pose_provider=lambda:pose(.16),stop_publisher=lambda:stops.append(True))
    result=backend._advance_navigating(active())
    assert 'travel limit' in result.message
    assert client.canceled==1 and stops==[True]


def test_readonly_trial_settlement_does_not_send_a_stop_without_owned_motion(tmp_path):
    b=budget(tmp_path);calls=[]
    backend=Nav2SearchBackend(path_client=Client(),navigate_client=Client(),trial_budget=b,
        stop_publisher=lambda:calls.append('stop'))
    a=active();a.armed=False
    backend._release_motion(a)
    assert calls==[] and json.loads(b.path.read_text())['steps']==[]


def test_broken_stop_transport_cannot_skip_owned_disarm(tmp_path):
    b=budget(tmp_path);reserve(b);calls=[]
    def broken_stop():raise RuntimeError('publisher context closed')
    backend=Nav2SearchBackend(path_client=Client(),navigate_client=Client(),trial_budget=b,
        stop_publisher=broken_stop,motion_releaser=lambda:calls.append('disarm'))
    a=active();a.armed=True
    backend._release_motion(a,succeeded=True)
    assert calls==['disarm'] and not a.armed
    assert json.loads(b.path.read_text())['active_token'] is None


def test_trial_controller_tolerances_do_not_swallow_a_twelve_cm_step():
    import yaml
    from pathlib import Path
    from scripts.go2w.prepare_nav2_trial import trial_parameters
    base=yaml.safe_load(Path('ros2_ws/src/robot_scene_nav_bringup/config/nav2_params_humble.yaml').read_text())
    trial=trial_parameters(base)['controller_server']['ros__parameters']
    assert trial['general_goal_checker']['xy_goal_tolerance']==.03
    assert trial['general_goal_checker']['yaw_goal_tolerance']==.05
    assert trial['progress_checker']['required_movement_radius']==.02
    assert trial['progress_checker']['movement_time_allowance']==15.0
    assert trial_parameters(base)['planner_server']['ros__parameters']['GridBased']['tolerance']==.01
    assert trial['FollowPath']['desired_linear_vel']==.05
    assert base['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance']==.20
    assert base['controller_server']['ros__parameters']['progress_checker']['movement_time_allowance']==8.0


def test_v2_calibration_approach_floor_does_not_modify_v1_or_normal_profile():
    import copy,yaml
    from pathlib import Path
    from scripts.go2w.prepare_nav2_trial import trial_parameters
    base=yaml.safe_load(Path('ros2_ws/src/robot_scene_nav_bringup/config/nav2_params_humble.yaml').read_text())
    original=copy.deepcopy(base)
    v2=trial_parameters(base,.12)['controller_server']['ros__parameters']['FollowPath']
    v1=trial_parameters(base,.05)['controller_server']['ros__parameters']['FollowPath']
    assert v2['min_approach_linear_velocity']==.12
    assert v2['desired_linear_vel']==.12
    assert v2['regulated_linear_scaling_min_speed']==.12
    assert v1.get('min_approach_linear_velocity',.05)==.05
    assert base==original


def test_trial_smoother_tracks_authorized_cap_without_changing_acceleration():
    import copy,yaml
    from pathlib import Path
    from scripts.go2w.prepare_nav2_trial import trial_parameters
    root=Path('ros2_ws/src/robot_scene_nav_bringup/config')
    base=yaml.safe_load((root/'nav2_params_humble.yaml').read_text())
    smoother=yaml.safe_load((root/'velocity_smoother.yaml').read_text());original=copy.deepcopy(smoother)
    for maximum in (.05,.12):
        params=trial_parameters(base,maximum,velocity_smoother=smoother)['velocity_smoother']['ros__parameters']
        assert params['max_velocity'][0]==maximum
        assert params['max_accel']==original['velocity_smoother']['ros__parameters']['max_accel']
        assert params['max_decel']==original['velocity_smoother']['ros__parameters']['max_decel']
    assert smoother==original


def test_controller_inverts_the_active_costmap_inflation_model():
    """RPP's distance estimate must use the costmaps' actual exponential scale."""
    import yaml
    from pathlib import Path
    from scripts.go2w.prepare_nav2_trial import trial_parameters
    base=yaml.safe_load(Path('ros2_ws/src/robot_scene_nav_bringup/config/nav2_params_humble.yaml').read_text())
    for parameters in (base,trial_parameters(base,.05),trial_parameters(base,.12)):
        inverse=parameters['controller_server']['ros__parameters']['FollowPath']['inflation_cost_scaling_factor']
        for name in ('local_costmap','global_costmap'):
            inflation=parameters[name][name]['ros__parameters']['inflation_layer']
            assert inverse==inflation['cost_scaling_factor']


@pytest.mark.parametrize('position,yaw,path,wide,expected',[
    ((.105,0.),0.,((0.,0.),(.105,0.)),True,.12),
    ((0.,.105),0.,((0.,0.),(0.,.105)),True,.10),
    ((.105,0.),30.,((0.,0.),(.105,0.)),True,.10),
    ((.20,0.),0.,((0.,0.),(.20,0.)),True,.10),
    ((.105,0.),0.,((0.,0.),(.1,.1),(.105,0.)),True,.10),
    ((.105,0.),0.,((0.,0.),(.105,0.)),False,None),
])
def test_v2_higher_speed_requires_wide_short_forward_path(tmp_path,position,yaw,path,wide,expected):
    from tests.test_nav2_search_backend import _backend,_pose,_goal,_free_grid,_corridor_grid
    from app.navigation.nav2_search_backend import ActionPoll,ACTION_SUCCEEDED
    b=NavigationTrialBudget.initialize(tmp_path/'calibration.json',window_id='approved_new_window',
        reviewed_robot_boot_id='test-boot',maximum_speed_mps=.12)
    calls=[]
    def pending(value):calls.append(value);return None
    backend,planner,navigator=_backend(grid=_free_grid() if wide else _corridor_grid(.95),
        pose=_pose(),speed_setter=pending)
    backend._trial_budget=b
    planner.queue(ActionPoll(status=ACTION_SUCCEEDED,path=path))
    backend.get_navigation_status(backend.execute_goal(_goal(position=position,yaw_deg=yaw)))
    assert calls
    if expected is None:assert calls[-1]<.10
    else:assert calls[-1]==pytest.approx(expected)
    assert navigator.sent==[]
    assert json.loads(b.path.read_text())['steps']==[]


def test_pending_calibration_speed_reverts_when_forward_pose_no_longer_matches(tmp_path):
    from tests.test_nav2_search_backend import _backend,_pose,_goal,_free_grid
    from app.navigation.nav2_search_backend import ActionPoll,ACTION_SUCCEEDED
    b=NavigationTrialBudget.initialize(tmp_path/'calibration.json',window_id='approved_new_window',
        reviewed_robot_boot_id='test-boot',maximum_speed_mps=.12)
    calls=[]
    def pending(value):calls.append(value);return None
    backend,planner,navigator=_backend(grid=_free_grid(),pose=_pose(),speed_setter=pending)
    backend._trial_budget=b
    planner.queue(ActionPoll(status=ACTION_SUCCEEDED,path=((0.,0.),(.105,0.))))
    h=backend.execute_goal(_goal(position=(.105,0.)))
    backend.get_navigation_status(h)
    assert calls[-1]==.12
    backend._pose_provider=lambda:_pose(yaw_deg=90.)
    backend.get_navigation_status(h)
    assert calls[-1]==.10
    assert navigator.sent==[]


@pytest.mark.parametrize('path', [((0.,0.),(-.03,0.),(1.,0.)),
                                ((0.,0.),(.5,.11),(1.,0.)),
                                ((0.,0.),(.5,0.),(.4,0.),(1.,0.)),
                                ((-.06,0.),(1.,0.))])
def test_forward_window_rejects_reverse_or_lateral_plan_before_any_side_effect(tmp_path,path):
    from tests.test_nav2_search_backend import _backend,_pose,_goal,_free_grid
    from app.navigation.nav2_search_backend import ActionPoll,ACTION_SUCCEEDED
    b=NavigationTrialBudget.initialize_forward(tmp_path/'forward.json',window_id='approved',
        reviewed_robot_boot_id='test-boot')
    arms=[];speeds=[]
    backend,planner,navigator=_backend(grid=_free_grid(),pose=_pose(),
        speed_setter=lambda s:speeds.append(s) or True,
        motion_authorizer=lambda:arms.append(True) or True)
    backend._trial_budget=b
    planner.queue(ActionPoll(status=ACTION_SUCCEEDED,path=path))
    result=backend.get_navigation_status(backend.execute_goal(_goal(position=(1.,0.))))
    assert result.status.value=='rejected'
    assert 'detour' in result.message
    assert arms==speeds==navigator.sent==[]
    assert json.loads(b.path.read_text())['steps']==[]


def test_forward_dry_run_applies_same_speed_and_geometry_without_reserving(tmp_path):
    from tests.test_nav2_search_backend import _backend,_pose,_goal,_corridor_grid
    from app.navigation.nav2_search_backend import ActionPoll,ACTION_SUCCEEDED
    b=NavigationTrialBudget.initialize_forward(tmp_path/'forward.json',window_id='approved',
        reviewed_robot_boot_id='test-boot')
    backend,planner,navigator=_backend(grid=_corridor_grid(.90),pose=_pose(),dry_run=True)
    backend._trial_budget=b
    planner.queue(ActionPoll(status=ACTION_SUCCEEDED,path=((-0.025,0.),(.1,0.),(1.,0.))))
    result=backend.get_navigation_status(backend.execute_goal(_goal(position=(1.,0.))))
    decision=result.provenance['candidate_decision']
    assert decision['accepted'] is True and decision['speed_limit_mps']==.12
    assert decision['trial_speed_policy']=='bounded_forward_working_command'
    assert navigator.sent==[] and json.loads(b.path.read_text())['steps']==[]


def test_forward_profile_never_emits_commands_above_independent_direction_gate():
    import yaml
    from scripts.go2w.prepare_nav2_trial import trial_parameters
    root=Path(__file__).resolve().parents[1]/'ros2_ws/src/robot_scene_nav_bringup/config'
    base=yaml.safe_load((root/'nav2_params_humble.yaml').read_text())
    smooth=yaml.safe_load((root/'velocity_smoother.yaml').read_text())
    params=trial_parameters(base,.12,velocity_smoother=smooth,forward_only=True)
    assert params['controller_server']['ros__parameters']['FollowPath']['rotate_to_heading_angular_vel']==.05
    v=params['velocity_smoother']['ros__parameters']
    assert v['max_velocity']==[.12,0.,.05]
    assert v['min_velocity']==[0.,0.,-.05]
    # Old calibrated ledgers retain their existing steering envelope.
    assert trial_parameters(base,.12,velocity_smoother=smooth)['velocity_smoother']['ros__parameters']['max_velocity'][2]==.5


def test_speed_calibration_030_params_are_bounded_to_straight_point_three():
    import yaml
    from scripts.go2w.prepare_nav2_trial import trial_parameters
    root=Path(__file__).resolve().parents[1]/'ros2_ws/src/robot_scene_nav_bringup/config'
    base=yaml.safe_load((root/'nav2_params_humble.yaml').read_text())
    smooth=yaml.safe_load((root/'velocity_smoother.yaml').read_text())
    params=trial_parameters(base,.30,velocity_smoother=smooth,forward_only=True)
    follow=params['controller_server']['ros__parameters']['FollowPath']
    assert follow['desired_linear_vel']==.30
    assert follow['min_approach_linear_velocity']==.30
    assert follow['regulated_linear_scaling_min_speed']==.30
    velocities=params['velocity_smoother']['ros__parameters']
    assert velocities['max_velocity']==[.30,0.,.05]
    assert velocities['min_velocity']==[0.,0.,-.05]


def test_speed_calibration_030_dry_run_selects_new_speed_only_after_geometry(tmp_path):
    from tests.test_nav2_search_backend import _backend,_pose,_goal,_corridor_grid
    from app.navigation.nav2_search_backend import ActionPoll,ACTION_SUCCEEDED
    calibration=NavigationTrialBudget.initialize_speed_calibration_030(
        tmp_path/'speed030.json',window_id='operator-speed-030',reviewed_robot_boot_id='boot')
    backend,planner,navigator=_backend(grid=_corridor_grid(.90),pose=_pose(),dry_run=True)
    backend._trial_budget=calibration
    planner.queue(ActionPoll(status=ACTION_SUCCEEDED,path=((-0.025,0.),(.1,0.),(.30,0.))))
    result=backend.get_navigation_status(backend.execute_goal(_goal(position=(.30,0.))))
    decision=result.provenance['candidate_decision']
    assert decision['accepted'] is True and decision['speed_limit_mps']==.30
    assert decision['trial_speed_policy']=='supervised_velocity_calibration_0_30'
    assert navigator.sent==[] and json.loads(calibration.path.read_text())['steps']==[]


def test_search_trial_conservatively_fits_all_actions_in_cumulative_scope(tmp_path):
    b=NavigationTrialBudget.initialize_search(tmp_path/'search.json',window_id='approved',
        reviewed_robot_boot_id='boot',prior_translation_charge_m=.66)
    assert b.bounded_search and b.MAX_STEPS==3 and b.MAX_TRANSLATION_M==.6
    assert .66+b.MAX_STEPS*b.MAX_TRANSLATION_M<=3.
    for _ in range(3):
        reserve(b,path_length_m=.45,goal_x=.45)
        b.finish()  # Failure and zero observed motion do not refund a slot.
        b=NavigationTrialBudget(b.path)
    with pytest.raises(TrialBudgetError,match='exhausted'):reserve(b)
    with pytest.raises(FileExistsError):
        NavigationTrialBudget.initialize_search(b.path,window_id='reset',reviewed_robot_boot_id='boot',prior_translation_charge_m=0.)


@pytest.mark.parametrize('prior',[1.200001,-.01,float('inf'),float('nan')])
def test_search_scope_cannot_omit_prior_travel_or_exceed_three_meters(tmp_path,prior):
    with pytest.raises(TrialBudgetError):
        NavigationTrialBudget.initialize_search(tmp_path/'bad.json',window_id='x',reviewed_robot_boot_id='boot',prior_translation_charge_m=prior)


def test_search_trial_stops_at_per_goal_reserve_and_cannot_grow(tmp_path):
    b=NavigationTrialBudget.initialize_search(tmp_path/'search.json',window_id='approved',reviewed_robot_boot_id='boot',prior_translation_charge_m=.66)
    reserve(b,path_length_m=.45,goal_x=.45)
    assert 'travel' in b.observe(pose(.51))
    data=json.loads(b.path.read_text());data['maximum_steps']=4;b.path.write_text(json.dumps(data))
    with pytest.raises(TrialBudgetError):NavigationTrialBudget(b.path)


def test_short_search_goal_cannot_consume_the_whole_policy_envelope(tmp_path):
    b=NavigationTrialBudget.initialize_search(tmp_path/'search.json',window_id='approved',
        reviewed_robot_boot_id='boot',prior_translation_charge_m=0.)
    reserve(b,path_length_m=.15,goal_x=.15)
    step=json.loads(b.path.read_text())['steps'][-1]
    assert step['planned_path_length_m']==pytest.approx(.15)
    assert step['observed_travel_limit_m']==pytest.approx(.25)
    assert not b.observe(pose(.24))
    assert 'goal-specific observed travel limit' in b.observe(pose(.251))


def test_long_search_goal_keeps_existing_half_metre_stop_limit(tmp_path):
    b=NavigationTrialBudget.initialize_search(tmp_path/'search.json',window_id='approved',
        reviewed_robot_boot_id='boot',prior_translation_charge_m=0.)
    reserve(b,path_length_m=.45,goal_x=.45)
    step=json.loads(b.path.read_text())['steps'][-1]
    assert step['observed_travel_limit_m']==pytest.approx(.5)
    assert not b.observe(pose(.49))
    assert 'goal-specific observed travel limit' in b.observe(pose(.501))


def test_search_trial_bounds_heading_excursion_not_controller_hunting(tmp_path):
    b=NavigationTrialBudget.initialize_search(tmp_path/'search.json',window_id='approved',
        reviewed_robot_boot_id='boot',prior_translation_charge_m=0.)
    reserve(b,path_length_m=.45,goal_x=.45)
    assert not b.observe(pose(yaw=.10))
    assert not b.observe(pose(yaw=-.10))
    assert json.loads(b.path.read_text())['steps'][-1]['turned_rad']==pytest.approx(.10)
    assert 'turn limit' in b.observe(pose(yaw=math.radians(21)))


def test_search_trial_uses_working_speed_in_safe_nonpreferred_corridor(tmp_path):
    from tests.test_nav2_search_backend import _backend,_pose,_goal,_corridor_grid
    from app.navigation.nav2_search_backend import ActionPoll,ACTION_SUCCEEDED
    b=NavigationTrialBudget.initialize_search(tmp_path/'search.json',window_id='approved',reviewed_robot_boot_id='boot',prior_translation_charge_m=.66)
    backend,planner,navigator=_backend(grid=_corridor_grid(.9),pose=_pose(),dry_run=True)
    backend._trial_budget=b
    planner.queue(ActionPoll(status=ACTION_SUCCEEDED,path=((0.,0.),(.3,0.))))
    result=backend.get_navigation_status(backend.execute_goal(_goal(position=(.3,0.))))
    assert result.provenance['candidate_decision']['speed_limit_mps']==.12
    assert navigator.sent==[] and json.loads(b.path.read_text())['steps']==[]
