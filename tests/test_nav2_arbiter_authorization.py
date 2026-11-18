from app.navigation.motion_authorization import arm_with_arbiter_reset,motion_preflight_blockers


def _ready_state(now=10.):
    return {
        'sport':(1,0,now),
        'lease_alive':(True,now),
        'lidar_fresh':(True,now),
        'localization_valid':(True,now),
        'robot_error_zero':(True,now),
        'emergency_stop':(False,now),
        'remote_override':(False,now),
        'software_arm':(False,now),
    }


def test_motion_preflight_requires_every_fresh_gate_before_trial_reservation():
    state=_ready_state()
    assert motion_preflight_blockers(state,now=10.)==[]
    del state['sport']
    assert motion_preflight_blockers(state,now=10.)==['sport_state_missing']


def test_motion_preflight_reports_stale_and_rejecting_inputs():
    state=_ready_state()
    state['sport']=(1,0,9.)
    state['emergency_stop']=(True,10.)
    state['lease_alive']=(True,9.)
    assert motion_preflight_blockers(state,now=10.)==[
        'sport_state_stale','lease_alive_stale','emergency_stop_rejecting'
    ]


def test_arm_reset_and_navigation_admission_order():
    calls=[];armed=[False]
    def arm():armed[0]=True;calls.append('arm')
    def reset():assert armed[0];calls.append('reset');return True
    assert arm_with_arbiter_reset(arm=arm,is_armed=lambda:armed[0],reset=reset,disarm=lambda:calls.append('disarm'))
    assert calls==['arm','reset']


def test_reset_failure_or_exception_releases_our_arm():
    for throws in (False,True):
        calls=[];armed=[False]
        def arm():armed[0]=True
        def reset():
            if throws:raise RuntimeError('reset unavailable')
            return False
        def disarm():armed[0]=False;calls.append('disarm')
        assert not arm_with_arbiter_reset(arm=arm,is_armed=lambda:armed[0],reset=reset,disarm=disarm)
        assert not armed[0] and calls==['disarm']


def test_failed_arm_never_resets_or_disarms_an_unowned_arm():
    calls=[]
    assert not arm_with_arbiter_reset(arm=lambda:None,is_armed=lambda:False,
        reset=lambda:calls.append('reset'),disarm=lambda:calls.append('disarm'))
    assert calls==[]
