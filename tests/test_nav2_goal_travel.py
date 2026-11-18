"""Exercise both sides of the goal handshake with real wheel traces, no ROS."""
import importlib.util
from pathlib import Path

import pytest

from app.navigation.goal_travel_budget import GoalTravelBudget, GoalTravelError
from app.navigation.nav2_search_backend import (
    Nav2SearchBackend, ActionPoll, ACTION_EXECUTING, ACTION_SUCCEEDED,
)
from app.navigation.robot_backend import NavigationStatus
from tests.test_nav2_search_backend import FakeActionClient, _free_grid, _goal, _straight_path, _pose


@pytest.fixture
def harness():
    source=Path('ros2_ws/src/go2w_cmd_vel_bridge/go2w_cmd_vel_bridge/goal_travel_guard.py')
    spec=importlib.util.spec_from_file_location('isolated_goal_travel_guard',source)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    class Link:
        def __init__(self):
            self.now=100.
            self.guard=module.GoalTravelGuard()
            self.requests=[]
            self.client=GoalTravelBudget(self.send,now=lambda:self.now,monotonic=lambda:self.now)
            self.x=0.
            for _ in range(5):self.tick()
        def send(self,request):
            self.requests.append(request)
            self.guard.request(request,now=self.now,received_now=self.now)
        def tick(self,dx=0.,deliver=True):
            self.now+=.1;self.x+=dx
            self.guard.odometry(x=self.x,y=0.,yaw=0.,frame='odom_fused',child='base_link',
                stamp=self.now,received=self.now)
            if deliver:self.status()
        def status(self):
            self.client.update(self.guard.status(now=self.now,received_now=self.now))
        def start(self):
            assert self.client.prepare('goal',3.) is None
            self.status()
            assert self.client.prepare('goal',3.) is True
            assert self.client.activate() is None
            self.status()
            assert self.client.activate() is True
    return Link()


def test_client_waits_for_both_acknowledgements(harness):
    h=harness;h.start()
    assert h.client.snapshot()['traveled_m']==0.
    assert [r['phase'] for r in h.requests]==['prepare','prepare','run','run']


def test_no_status_or_missing_odometry_never_prepares_or_arms():
    client=GoalTravelBudget(lambda _:pytest.fail('must not request without a ready bridge'))
    assert client.prepare('goal',3.) is None


@pytest.mark.parametrize('mode',['readonly','trial','field'])
def test_ros_factory_wires_the_guard_only_for_normal_execution(mode):
    from tests.test_nav2_backend_pose_provider import _fake_ros,FakeNode
    from app.navigation.nav2_search_backend import create_nav2_search_backend
    class Node(FakeNode):
        def __init__(self):super().__init__();self.publishers=[];self.subscriptions=[]
        def create_publisher(self,kind,topic,*args,**kwargs):
            self.publishers.append(topic)
            return super().create_publisher(kind,topic,*args,**kwargs)
        def create_subscription(self,kind,topic,*args,**kwargs):
            self.subscriptions.append(topic)
            return super().create_subscription(kind,topic,*args,**kwargs)
    with _fake_ros():
        node=Node()
        backend=create_nav2_search_backend(node,dry_run=mode=='readonly',
            trial_budget=object() if mode=='trial' else None)
    assert ('/go2w/nav/goal_travel/request' in node.publishers)==(mode=='field')
    assert ('/go2w/nav/goal_travel/status' in node.subscriptions)==(mode=='field')
    assert (backend._travel_budget is not None)==(mode=='field')


@pytest.mark.parametrize('problem',['bridge_restart','status_lost','counter_fault','reservation_lost'])
def test_bad_or_absent_ack_stops_an_active_client(harness,problem):
    h=harness;h.start()
    if problem=='bridge_restart':h.guard.epoch='restarted'
    elif problem=='counter_fault':h.guard.reason='goal_travel_limit'
    elif problem=='reservation_lost':h.guard.token=None
    else:
        for _ in range(6):h.tick(deliver=False)
    if problem!='status_lost':h.status()
    with pytest.raises(GoalTravelError):h.client.check()


def test_backend_ack_before_arm_then_ack_before_navigate_and_counts_recovery(harness):
    h=harness;path=FakeActionClient();nav=FakeActionClient();events=[]
    backend=Nav2SearchBackend(path_client=path,navigate_client=nav,
        occupancy_grid_provider=lambda:_free_grid(12),pose_provider=_pose,
        travel_budget=h.client,max_path_length_m=3.,now=lambda:h.now,
        motion_authorizer=lambda:events.append('arm') or True,
        motion_releaser=lambda:events.append('disarm'),
        stop_publisher=lambda:events.append('stop'))
    path.queue(ActionPoll(status=ACTION_SUCCEEDED,path=_straight_path(1.)))
    nav.queue(ActionPoll(status=ACTION_EXECUTING))
    handle=backend.execute_goal(_goal(position=(1.,0)))
    backend.get_navigation_status(handle)
    assert events==[] and nav.sent==[]
    h.tick();backend.get_navigation_status(handle)
    assert events==['arm'] and nav.sent==[]
    h.tick();backend.get_navigation_status(handle)
    assert len(nav.sent)==1
    # Internally replanned out-and-back motion, while map pose stays at zero.
    for sign in (1,-1):
        for _ in range(140):
            h.tick(.01*sign)
            result=backend.get_navigation_status(handle)
            assert result.status==NavigationStatus.RUNNING
    assert h.x==pytest.approx(0.)
    for _ in range(11):
        h.tick(.01)
        result=backend.get_navigation_status(handle)
        if result.status!=NavigationStatus.RUNNING:break
    assert result.status==NavigationStatus.REJECTED
    assert result.message=='goal_travel_limit'
    assert result.provenance['wheel_travel']['traveled_m'] >= 2.9
    assert nav.cancel_calls==1
    assert events==['arm','stop','disarm']
    assert h.guard.phase=='idle'


@pytest.mark.parametrize('outcome',['success','cancel','stop_error','send_error'])
def test_guarded_goal_closes_and_disarms_for_every_owned_exit(harness,outcome):
    h=harness;path=FakeActionClient();nav=FakeActionClient();events=[]
    def stop():
        events.append('stop')
        if outcome=='stop_error':raise RuntimeError('transport closed')
    backend=Nav2SearchBackend(path_client=path,navigate_client=nav,
        occupancy_grid_provider=lambda:_free_grid(),pose_provider=_pose,
        travel_budget=h.client,max_path_length_m=3.,now=lambda:h.now,
        motion_authorizer=lambda:events.append('arm') or True,
        motion_releaser=lambda:events.append('disarm'),stop_publisher=stop)
    path.queue(ActionPoll(status=ACTION_SUCCEEDED,path=_straight_path(1.)))
    handle=backend.execute_goal(_goal(position=(1.,0)))
    backend.get_navigation_status(handle)
    h.tick();backend.get_navigation_status(handle)
    h.tick()
    if outcome=='send_error':
        nav.send=lambda _:(_ for _ in ()).throw(RuntimeError('send failed'))
        with pytest.raises(RuntimeError):backend.get_navigation_status(handle)
        backend.cancel(handle)  # Production runner's exception cleanup.
    else:
        backend.get_navigation_status(handle)
        if outcome=='cancel':backend.cancel(handle)
        else:
            nav.queue(ActionPoll(status=ACTION_SUCCEEDED))
            result=backend.get_navigation_status(handle)
            assert result.status==NavigationStatus.SUCCEEDED
    assert events==['arm','stop','disarm']
    assert h.guard.phase=='idle' and h.client.request is None


def test_3m_timeout_includes_slow_corridor_and_is_per_goal():
    backend=Nav2SearchBackend(path_client=FakeActionClient(),navigate_client=FakeActionClient(),
        max_path_length_m=3.)
    assert backend.navigation_timeout_sec() >= 3./.05+10.+30.


def test_explorer_can_finish_a_40_second_goal_and_still_bounds_a_stuck_goal():
    from types import SimpleNamespace
    from app.live_robot.autonomous_explorer import AutonomousExplorer
    from app.navigation.robot_backend import NavigationResult,NavigationHandle
    from scripts.go2w.run_semantic_exploration import _Nav2MotionEvidenceRecorder
    backend=Nav2SearchBackend(path_client=FakeActionClient(),navigate_client=FakeActionClient(),
        max_path_length_m=3.)
    # Exercise the production evidence wrapper, not just the inner backend.
    wrapper=_Nav2MotionEvidenceRecorder(backend,node=None,motion_state={})
    clock=[0.]
    explorer=AutonomousExplorer.__new__(AutonomousExplorer)
    explorer._backend=SimpleNamespace(navigation_timeout_sec=wrapper.navigation_timeout_sec,
        get_navigation_status=lambda h:NavigationResult(goal_id=h.goal_id,
            status=NavigationStatus.SUCCEEDED if clock[0]>=40 else NavigationStatus.RUNNING))
    explorer._now=lambda:clock[0]
    explorer._operator_stop_requested=explorer._pause_requested=False
    explorer._navigation_target_monitor=None
    explorer._pump_backend=lambda duration:clock.__setitem__(0,clock[0]+duration)
    handle=NavigationHandle('goal')
    assert explorer._wait_result(handle,timeout_sec=15.).status==NavigationStatus.SUCCEEDED
    assert clock[0]>=40.
    explorer._backend.get_navigation_status=lambda h:NavigationResult(
        goal_id=h.goal_id,status=NavigationStatus.RUNNING)
    clock[0]=1000000.  # Session age does not consume the next goal's allowance.
    assert explorer._wait_result(handle,timeout_sec=15.).status==NavigationStatus.TIMEOUT
    assert 1000000.+110. <= clock[0] <= 1000000.+120.
