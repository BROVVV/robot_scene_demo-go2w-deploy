from app.live_robot.autonomous_explorer import AutonomousExplorer
from app.live_robot.mock_observation_scene import scenario_no_target
from app.navigation.backend_factory import MockBackend
from app.navigation.exploration_config import ExplorationBudget,load_exploration_policy
from app.navigation.robot_backend import NavigationHandle,NavigationResult,NavigationStatus
from app.live_robot.search_state_store import SearchStateStore
from app.live_robot.search_event import SearchEvent,OBSERVATION_UPDATED
from app.live_robot.search_event_bus import SearchEventBus
from app.live_robot.explorer_search_adapter import ExplorerSearchAdapter


class CheckedPlanBackend(MockBackend):
    def execute_goal(self,goal):
        return NavigationHandle(goal.goal_id,NavigationResult(goal.goal_id,
            NavigationStatus.CANCELLED,'dry run: path checked; NavigateToPose not sent',
            provenance={'dry_run':True,'candidate_decision':{'accepted':True}}))


def test_checked_readonly_plan_neither_recovers_nor_counts_as_physical_success():
    scene=scenario_no_target(empty_scenes=8);policy=load_exploration_policy()
    policy.budget=ExplorationBudget(max_motion_steps=3,max_planning_cycles=2)
    explorer=AutonomousExplorer(target='蓝色垃圾桶',observer=scene.observer(),
        matcher=scene.matcher(),verifier=scene.verifier(),backend=CheckedPlanBackend(),
        policy=policy,dry_run=True)
    result=explorer.run()
    events=[e for e in explorer.events if e.get('event')=='navigation_result']
    assert events and all(e['dry_run'] and e['path_checked'] for e in events)
    assert result.motion_steps==0 and result.replans==0 and result.navigation_failures==0
    assert all(d.execution_status=='PLANNED' for d in explorer.decision_records)


def test_readonly_result_passes_adapter_and_does_not_claim_arrival():
    store=SearchStateStore();bus=SearchEventBus()
    adapter=ExplorerSearchAdapter(bus,store,session_id='readonly')
    adapter.on_explorer_event({'event':'navigation_result','status':'cancelled',
        'dry_run':True,'path_checked':True,'candidate_decision':{'accepted':True}})
    robot=store.snapshot()['robot']
    assert robot['motion_status']=='PLANNED'
    assert robot['last_motion_result']['status']=='cancelled'
    adapter.on_explorer_event({'event':'navigation_result','status':'rejected',
        'dry_run':True,'path_checked':False,'candidate_decision':{'accepted':False}})
    assert store.snapshot()['robot']['motion_status']=='FAILED'


def test_observation_keeps_metric_pose_quality():
    store=SearchStateStore()
    store.apply(SearchEvent(event_id=1,event_type=OBSERVATION_UPDATED,
        session_id='readonly',timestamp=100,payload={'pose':{'x':1,'y':2,'quality':'metric'}}))
    assert store.snapshot()['robot']['pose_quality']=='metric'
