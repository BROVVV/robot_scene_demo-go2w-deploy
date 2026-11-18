"""Wheel traces are independent of the planner: detours and reverse add up."""
import math

import pytest

from go2w_cmd_vel_bridge.goal_travel_guard import GoalTravelGuard


class Trace:
    def __init__(self):
        self.guard = GoalTravelGuard()
        self.now = 100.
        self.x = self.y = 0.
        self.request = None
        for _ in range(5):
            self.feed()

    def feed(self, dx=0., dy=0., **changes):
        self.now += .1
        self.x += dx
        self.y += dy
        values = dict(x=self.x,y=self.y,yaw=0.,frame='odom_fused',child='base_link',
            stamp=self.now,received=self.now)
        values.update(changes)
        self.guard.odometry(**values)

    def send(self, phase='run', **changes):
        if self.request is None:
            self.request = dict(version=1,epoch=self.guard.epoch,token='goal-1',
                limit_m=3.,created=self.now)
            phase = 'prepare'
        values = dict(self.request,phase=phase,stamp=self.now)
        values.update(changes)
        return self.guard.request(values,now=self.now,received_now=self.now)

    def start(self):
        assert self.send() == ''
        assert self.guard.phase == 'prepared'
        assert self.check() == 'goal_travel_not_running'
        assert self.send() == ''

    def check(self, **changes):
        values = dict(now=self.now,received_now=self.now,linear=.1,angular=0.,duration=.25)
        values.update(changes)
        return self.guard.check(**values)


def test_detour_reverse_and_lateral_motion_count_even_if_net_displacement_is_zero():
    t=Trace();t.start()
    # Equivalent to Nav2 doing a 1.4m outbound detour and backing to its start.
    for direction in (1.,-1.):
        for _ in range(140):
            t.feed(dy=.01*direction);t.send()
    assert t.y == pytest.approx(0.)
    assert t.guard.traveled == pytest.approx(2.8)
    assert not t.check()  # Positive control: ordinary movement is permitted.
    for _ in range(3):
        t.feed(dx=.01);t.send()
    assert t.check() == 'goal_travel_limit'  # Reserve + outstanding slice.
    t.feed();t.send()
    assert t.check(linear=0.) == 'goal_travel_limit'


def test_repeated_heartbeat_cannot_reset_travel_or_swap_goal_mid_move():
    t=Trace();t.start()
    for _ in range(60):
        t.feed(dx=.01);t.send()
    assert t.guard.traveled == pytest.approx(.6)
    assert t.send(token='replacement',created=t.now) == 'goal_travel_overlapping_goal'
    assert t.guard.traveled == pytest.approx(.6)
    assert t.check() == 'goal_travel_overlapping_goal'


@pytest.mark.parametrize('problem',['stale','regression','gap','jump','frame','child','nan','clock_back'])
def test_bad_wheel_data_or_clock_latches_and_cannot_be_cleared_by_new_data(problem):
    t=Trace();t.start()
    if problem=='stale':
        t.now += .21
    elif problem=='regression':
        t.feed(stamp=t.now-.1)
    elif problem=='gap':
        t.now += .3;t.feed()
    elif problem=='jump':
        t.feed(dx=1.)
    elif problem=='frame':
        t.feed(frame='map')
    elif problem=='child':
        t.feed(child='laser')
    elif problem=='nan':
        t.feed(x=math.nan)
    else:
        t.now -= 1.
    assert t.check().startswith('goal_travel_')
    t.feed();t.send()
    assert t.check().startswith('goal_travel_')


def test_lost_search_heartbeat_stops_even_with_fresh_odometry():
    t=Trace();t.start()
    for _ in range(6):t.feed()
    assert t.check() == 'goal_travel_heartbeat_stale'
    assert t.send() == 'goal_travel_heartbeat_stale'


def test_prepare_is_nonmoving_and_allows_bounded_arm_handshake_delay():
    t=Trace();assert t.send()==''
    for _ in range(15):t.feed()
    assert t.check() == 'goal_travel_not_running'
    assert t.send()==''
    assert not t.check()


def test_calibrated_field_speed_is_allowed_but_higher_commands_latch():
    t=Trace();t.start()
    assert t.check(linear=.30)==''
    assert t.check(linear=.301)=='goal_travel_command_invalid'
    assert t.check(linear=.30)=='goal_travel_command_invalid'


def test_close_requires_stationary_new_goal_and_never_replays_old_reservation():
    t=Trace();t.start();t.feed(dx=.01);t.send()
    old=dict(t.request)
    assert t.send('close')==''
    t.request=None
    assert t.send()=='goal_travel_not_ready'
    for _ in range(5):t.feed()
    t.request=None
    assert t.send()==''
    assert t.guard.traveled==0.
    t.send('close')
    assert t.guard.request(dict(old,phase='prepare',stamp=t.now),now=t.now,received_now=t.now)=='goal_travel_not_ready'


def test_bridge_restart_changes_epoch_and_refuses_old_live_request():
    t=Trace();t.start()
    previous=dict(t.request,phase='run',stamp=t.now)
    t.guard=GoalTravelGuard()
    assert t.guard.request(previous,now=t.now,received_now=t.now)=='goal_travel_request_invalid'
    assert t.check()=='goal_travel_not_reserved'


@pytest.mark.parametrize('changes',[dict(limit_m=3.01),dict(limit_m=math.nan),
    dict(stamp=1010),dict(version=2),dict(epoch='other-bridge')])
def test_malformed_request_never_grants_a_counter(changes):
    t=Trace()
    assert t.send(**changes)=='goal_travel_request_invalid'
    assert t.check()=='goal_travel_not_reserved'


def test_normal_goal_count_and_duration_have_no_session_cap():
    t=Trace()
    for i in range(100):
        t.now += 86400.
        for _ in range(6):t.feed()
        t.request=None
        assert t.send(token=f'goal-{i}')==''
        t.request['token']=f'goal-{i}'
        assert t.send()==''
        assert not t.check()
        assert t.send('close')==''


def test_finite_but_overflowing_odometry_cannot_crash_the_status_publisher():
    import json
    t=Trace();t.start()
    t.feed(x=1e308)
    t.feed(x=-1e308)
    status=t.guard.status(now=t.now,received_now=t.now)
    assert status['reason'].startswith('goal_travel_odometry_')
    json.dumps(status,allow_nan=False)  # Must still reach the bridge cancel path.
