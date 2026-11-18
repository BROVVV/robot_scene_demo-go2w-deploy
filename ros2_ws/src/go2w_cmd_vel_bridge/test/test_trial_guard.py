import json
import math

import pytest

from go2w_cmd_vel_bridge.trial_guard import TrialVelocityGuard


def ledger(path, *, count=1, active=True, reserved_at=101.):
    steps=[dict(token=str(i),state='finished',reserved_at=reserved_at) for i in range(count)]
    if active and steps:
        steps[-1]['state']='reserved'
    data=dict(version=1,maximum_steps=3,maximum_translation_m=.2,stop_reserve_m=.05,
        steps=steps,active_token=str(count-1) if active else None)
    path.write_text(json.dumps(data))
    return data


def odom(g,x=0.,y=0.,yaw=0.,stamp=102.):
    g.odometry(x=x,y=y,yaw=yaw,frame='odom_fused',stamp=stamp,received=stamp)


def check(g, now=102., **kwargs):
    values=dict(now=now,received_now=now,linear=.05,angular=0.,duration=.25)
    values.update(kwargs)
    return g.check(**values)


def make_guard(tmp_path):
    path=tmp_path/'trial.json';ledger(path)
    guard=TrialVelocityGuard(path,started_at=100.)
    odom(guard)
    assert check(guard)==''
    return path,guard


def forward_guard(tmp_path):
    path=tmp_path/'forward.json';data=ledger(path)
    data.update(version=4,maximum_steps=1,maximum_translation_m=3.,stop_reserve_m=.15,
        maximum_speed_mps=.12,purpose='forward_observed_recheck',forward_only=True,
        maximum_turn_rad=.15,maximum_lateral_m=.15,maximum_duration_s=60.)
    path.write_text(json.dumps(data));g=TrialVelocityGuard(path,started_at=100.);odom(g)
    return path,g


def collision_guard(tmp_path):
    path=tmp_path/'collision.json';data=ledger(path)
    data.update(version=7,maximum_steps=1,maximum_translation_m=.25,stop_reserve_m=.05,
        maximum_speed_mps=.12,purpose='supervised_collision_diagnostic',forward_only=True,
        maximum_turn_rad=.15,maximum_lateral_m=.10,maximum_duration_s=15.)
    path.write_text(json.dumps(data));g=TrialVelocityGuard(path,started_at=100.);odom(g)
    return path,g


def speed_030_guard(tmp_path):
    path=tmp_path/'speed030.json';data=ledger(path)
    data.update(version=8,maximum_steps=1,maximum_translation_m=.6,stop_reserve_m=.15,
        maximum_speed_mps=.30,purpose='supervised_velocity_calibration_0_30',
        forward_only=True,maximum_turn_rad=.15,maximum_lateral_m=.10,
        maximum_duration_s=15.)
    path.write_text(json.dumps(data));g=TrialVelocityGuard(path,started_at=100.);odom(g)
    return path,g


def test_speed_030_guard_accepts_only_bounded_straight_point_three(tmp_path):
    _,g=speed_030_guard(tmp_path)
    assert g.maximum_speed_mps==.30 and check(g,linear=.30)==''
    assert check(g,linear=.30001)=='trial_speed_exceeded'


def test_speed_030_guard_accounts_for_full_outstanding_horizon(tmp_path):
    _,g=speed_030_guard(tmp_path)
    assert check(g,linear=.30)==''
    odom(g,x=.34,stamp=102.1)
    assert check(g,now=102.1,linear=.30)=='trial_travel_limit'


@pytest.mark.parametrize('change',[{'maximum_speed_mps':.31},{'maximum_translation_m':.61},
    {'stop_reserve_m':.14},{'maximum_duration_s':16.},{'maximum_lateral_m':.11}])
def test_speed_030_guard_rejects_widened_policy(tmp_path,change):
    path,g=speed_030_guard(tmp_path);data=json.loads(path.read_text())
    data.update(change);path.write_text(json.dumps(data))
    assert check(g,linear=.30)=='trial_ledger_invalid'


def test_collision_guard_stops_within_authorized_twenty_cm(tmp_path):
    _,g=collision_guard(tmp_path)
    assert check(g,linear=.12)==''
    odom(g,x=.14,stamp=102.1)
    assert check(g,now=102.1,linear=.12)=='trial_travel_limit'


@pytest.mark.parametrize('change',[{'maximum_translation_m':.26},{'stop_reserve_m':.04},
    {'maximum_duration_s':16.},{'maximum_lateral_m':.11}])
def test_collision_guard_rejects_widened_policy(tmp_path,change):
    path,g=collision_guard(tmp_path);data=json.loads(path.read_text())
    data.update(change);path.write_text(json.dumps(data))
    assert check(g,linear=.12)=='trial_ledger_invalid'


def search_guard(tmp_path):
    path=tmp_path/'search.json';data=ledger(path)
    data.update(version=5,maximum_speed_mps=.12,purpose='bounded_search_session',
        maximum_translation_m=.6,stop_reserve_m=.1,maximum_scope_translation_m=3.,
        prior_translation_charge_m=.66)
    path.write_text(json.dumps(data));g=TrialVelocityGuard(path,started_at=100.);odom(g)
    return path,g


def field_guard(tmp_path):
    path=tmp_path/'field.json';data=ledger(path)
    data.update(version=6,maximum_steps=5,maximum_translation_m=3.,
        stop_reserve_m=.15,maximum_speed_mps=.12,
        purpose='supervised_five_step_field_search',maximum_scope_translation_m=15.)
    path.write_text(json.dumps(data));g=TrialVelocityGuard(path,started_at=100.);odom(g)
    return path,g


def test_field_guard_stops_before_three_meters_on_independent_odometry(tmp_path):
    _,g=field_guard(tmp_path)
    assert check(g,linear=.12)==''
    odom(g,x=2.9,stamp=102.1)
    assert check(g,now=102.1,linear=.12)=='trial_travel_limit'


def test_field_guard_bounds_heading_excursion_not_alternating_steering_sum(tmp_path):
    _,g=field_guard(tmp_path)
    assert check(g)==''
    odom(g,yaw=.10,stamp=102.1)
    odom(g,yaw=-.10,stamp=102.2)
    assert g.turned==pytest.approx(.30)
    assert check(g,now=102.2,linear=.05,angular=0.)==''
    odom(g,yaw=math.radians(19.5),stamp=102.3)
    assert check(g,now=102.3,linear=0.,angular=0.)=='trial_turn_limit'


@pytest.mark.parametrize('change',[{'maximum_steps':6},{'maximum_translation_m':3.1},
    {'stop_reserve_m':.1},{'maximum_scope_translation_m':16.}])
def test_field_guard_rejects_widened_policy(tmp_path,change):
    path,g=field_guard(tmp_path);data=json.loads(path.read_text())
    data.update(change);path.write_text(json.dumps(data))
    assert check(g,linear=.12)=='trial_ledger_invalid'


def test_search_guard_independently_stops_before_each_six_tenths_meter(tmp_path):
    _,g=search_guard(tmp_path);assert check(g,linear=.12)==''
    odom(g,x=.5,stamp=102.1)
    assert check(g,now=102.1,linear=.12)=='trial_travel_limit'


def test_search_guard_bounds_heading_excursion_not_controller_hunting(tmp_path):
    _,g=search_guard(tmp_path)
    assert check(g)==''
    odom(g,yaw=.10,stamp=102.1)
    odom(g,yaw=-.10,stamp=102.2)
    assert g.turned==pytest.approx(.30)
    assert check(g,now=102.2,linear=.05,angular=0.)==''
    odom(g,yaw=math.radians(19.5),stamp=102.3)
    assert check(g,now=102.3,linear=0.,angular=0.)=='trial_turn_limit'


@pytest.mark.parametrize('change',[{'maximum_steps':4},{'maximum_translation_m':.7},
    {'prior_translation_charge_m':1.21},{'maximum_scope_translation_m':4.}])
def test_search_guard_refuses_widened_cumulative_scope(tmp_path,change):
    path,g=search_guard(tmp_path);data=json.loads(path.read_text());data.update(change);path.write_text(json.dumps(data))
    assert check(g,linear=.12)=='trial_ledger_invalid'


def test_forward_guard_observes_three_meter_ceiling(tmp_path):
    _,g=forward_guard(tmp_path)
    assert check(g,linear=.12)==''
    odom(g,x=2.86,stamp=102.1)
    assert check(g,now=102.1,linear=.12)=='trial_travel_limit'


@pytest.mark.parametrize('linear,angular',[(-.01,0.),(.12,.06)])
def test_forward_guard_refuses_reverse_or_turn_commands(tmp_path,linear,angular):
    _,g=forward_guard(tmp_path)
    assert check(g,linear=linear,angular=angular)=='trial_forward_only'


def test_forward_guard_rejects_lateral_excursion(tmp_path):
    _,g=forward_guard(tmp_path);assert check(g,linear=.12)==''
    odom(g,x=.1,y=.16,stamp=102.1)
    assert check(g,now=102.1,linear=.12)=='trial_forward_deviation'


def test_forward_guard_cannot_extend_window_or_duration(tmp_path):
    path,g=forward_guard(tmp_path)
    assert check(g,linear=.12)==''
    odom(g,stamp=162.)
    assert check(g,now=162.,linear=.12)=='trial_duration_limit'
    data=json.loads(path.read_text());data['maximum_translation_m']=4.
    path.write_text(json.dumps(data))
    assert check(g,now=162.)=='trial_ledger_invalid'


def test_explicit_calibration_speed_retains_independent_twenty_centimeter_stop(tmp_path):
    path=tmp_path/'calibration.json';data=ledger(path)
    data.update(version=2,purpose='supervised_velocity_calibration',maximum_speed_mps=.12)
    path.write_text(json.dumps(data));g=TrialVelocityGuard(path,started_at=100.)
    odom(g)
    assert g.maximum_speed_mps==.12 and check(g,linear=.12)==''
    odom(g,x=.09,stamp=102.1)
    assert check(g,now=102.1,linear=.12)=='trial_travel_limit'


def test_legacy_window_cannot_be_upgraded_while_guard_is_watching(tmp_path):
    path,g=make_guard(tmp_path);data=json.loads(path.read_text())
    data.update(version=2,purpose='supervised_velocity_calibration',maximum_speed_mps=.12)
    path.write_text(json.dumps(data))
    assert check(g,linear=.12)=='trial_ledger_invalid'


def test_one_observed_recheck_rejects_a_second_reservation_and_widening(tmp_path):
    path=tmp_path/'single.json';data=ledger(path)
    data.update(version=3,maximum_steps=1,maximum_speed_mps=.12,purpose='single_observed_recheck')
    path.write_text(json.dumps(data));g=TrialVelocityGuard(path,started_at=100.);odom(g)
    assert check(g,linear=.12)==''
    data['steps'].append(dict(token='second',state='reserved',reserved_at=101.5))
    data['active_token']='second';path.write_text(json.dumps(data))
    assert check(g,linear=.12)=='trial_ledger_invalid'
    data['steps']=data['steps'][:1];data['active_token']='0';data['maximum_steps']=3
    path.write_text(json.dumps(data));g=TrialVelocityGuard(path,started_at=100.);odom(g)
    assert check(g,linear=.12)=='trial_ledger_invalid'


def test_independent_odometry_catches_backtracking_and_latches_stop(tmp_path):
    _,g=make_guard(tmp_path)
    odom(g,x=.08,stamp=102.01)
    odom(g,x=0.,stamp=102.02)
    assert check(g,now=102.02)=='trial_travel_limit'
    odom(g,stamp=102.03)
    assert check(g,now=102.03,linear=0.)=='trial_travel_limit'


def test_turn_budget_is_independent_of_map_and_net_yaw(tmp_path):
    _,g=make_guard(tmp_path)
    odom(g,yaw=.15,stamp=102.01)
    odom(g,yaw=-.15,stamp=102.02)
    assert check(g,now=102.02,linear=0.,angular=.1)=='trial_turn_limit'


@pytest.mark.parametrize('reserved_at',[99.,float('nan'),103.])
def test_restarted_bridge_or_invalid_timestamp_refuses_old_reservation(tmp_path,reserved_at):
    path=tmp_path/'trial.json';ledger(path,reserved_at=reserved_at)
    g=TrialVelocityGuard(path,started_at=100.);odom(g)
    assert check(g)=='trial_ledger_invalid'


def test_missing_or_malformed_ledger_cannot_authorize_a_slice(tmp_path):
    path=tmp_path/'trial.json';g=TrialVelocityGuard(path,started_at=100.);odom(g)
    assert check(g)=='trial_ledger_invalid'
    for content in ('{}','{broken','null'):
        path.write_text(content)
        assert check(g)=='trial_ledger_invalid'
    ledger(path,count=4)
    assert check(g)=='trial_ledger_invalid'


def test_no_step_and_finished_step_are_closed(tmp_path):
    path=tmp_path/'trial.json';ledger(path,count=0,active=False)
    g=TrialVelocityGuard(path,started_at=100.);odom(g)
    assert check(g)=='trial_step_not_reserved'
    ledger(path);assert check(g)==''
    ledger(path,active=False)
    assert check(g)=='trial_step_not_reserved'
    ledger(path)
    assert check(g)=='trial_ledger_invalid'


def test_switch_requires_a_durable_finished_predecessor(tmp_path):
    path,g=make_guard(tmp_path)
    data=ledger(path,count=2)
    data['steps'][0]['state']='reserved';path.write_text(json.dumps(data))
    assert check(g)=='trial_ledger_invalid'


def test_next_finished_step_gets_a_new_bounded_counter(tmp_path):
    path,g=make_guard(tmp_path)
    odom(g,x=.08,stamp=102.01)
    ledger(path,count=2)
    assert check(g,now=102.01)==''
    assert g.traveled==0.
    odom(g,x=.24,stamp=102.02)
    assert check(g,now=102.02)=='trial_travel_limit'


@pytest.mark.parametrize('problem',['stale','missing','nonfinite','regressed','wrong_frame'])
def test_bad_odometry_latches_a_stop(tmp_path,problem):
    _,g=make_guard(tmp_path)
    if problem=='stale':
        assert check(g,now=102.21)=='trial_odometry_stale'
    elif problem=='missing':
        g.pose=None
    elif problem=='nonfinite':
        odom(g,x=float('nan'),stamp=102.01)
    elif problem=='regressed':
        odom(g,stamp=101.)
    else:
        g.odometry(x=0,y=0,yaw=0,frame='map',stamp=102.01,received=102.01)
    assert check(g,now=102.22).startswith('trial_odometry_')
    odom(g,stamp=102.3)
    assert check(g,now=102.3).startswith('trial_odometry_')


@pytest.mark.parametrize('changes',[dict(linear=.06),dict(angular=.11),
    dict(duration=.3),dict(linear=float('nan'))])
def test_overspeed_or_unbounded_slice_is_refused(tmp_path,changes):
    _,g=make_guard(tmp_path)
    assert check(g,**changes) in {'trial_speed_exceeded','trial_command_invalid'}


def test_ledger_deletion_mid_step_cancels_even_after_file_is_restored(tmp_path):
    path,g=make_guard(tmp_path);path.unlink()
    assert check(g)=='trial_ledger_invalid'
    ledger(path)
    assert check(g)=='trial_ledger_invalid'
