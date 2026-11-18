from types import SimpleNamespace

import pytest

from app.navigation.models import LiveObservation
from app.live_robot.autonomous_explorer import AutonomousExplorer, VerificationOutcome
from app.spatial.semantic_object_map import SemanticObjectMap,SemanticObjectEntry,SemanticMapUpdateResult,SemanticAssociation
from app.spatial.target_candidate_map import TargetCandidateMapper,merge_updates,target_source_id
from app.perception.target_spatial import target_identity,validated_target_binding


def target(frame, stamp, *, entity='cup', x=1.2, label='cup', score=.9, epoch='epoch-a'):
    return dict(entity_id=entity,entity_source_frame_id=frame,label=label,label_zh=label,
        bbox_2d=[.3,.3,.6,.8],score=score,depth_m=1.2,camera_xyz=[0.,0.,1.2],
        map_xyz=[x,0.,.5],bearing_deg=0.,spatial_quality='METRIC_RGBD',
        spatial_provenance={'frame_id':frame,'map_frame':'map','image_timestamp':stamp,
            'timestamp_quality':'shared_host_capture',
            'image_time_transform':{'target_frame':'map','transform_source':'tf2',
                'tf_available':True,'transform_quality':'METRIC_LIDAR','map_epoch':epoch,'map_id':'map-a'}})


def feed(mapper, objects, items):
    return mapper.ingest(items,objects,place_id='P1',min_confidence=.35)


def stabilized(mapper, objects, *, entity='cup', x=1.2, label='cup', score=.9):
    output=result=diagnostics=None
    for i in range(3):
        output,result,diagnostics=feed(mapper,objects,[target(f'capture-{i}',100.+i,entity=entity,x=x,label=label,score=score)])
    return output[0],result,diagnostics


def test_target_needs_distinct_captures_and_original_two_second_span():
    mapper=TargetCandidateMapper();objects=SemanticObjectMap()
    for _ in range(5):
        got,result,_=feed(mapper,objects,[target('one-capture',100.)])
        assert 'persistent_object_id' not in got[0] and not objects.objects
    feed(mapper,objects,[target('capture-b',101.)]);assert not objects.objects
    got,result,_=feed(mapper,objects,[target('capture-c',102.)])
    oid=got[0]['persistent_object_id'];entry=objects.objects[oid]
    assert entry.mobility_state=='stationary' and entry.topology_eligible
    assert entry.observation_count==1
    assert entry.source_observation_ids==['capture-c']
    assert result.associations[0].source_object_id==target_source_id('cup')
    count=entry.observation_count
    feed(mapper,objects,[target('capture-c',102.)])
    assert entry.observation_count==count


def test_three_frames_in_less_than_two_seconds_are_still_pending():
    mapper=TargetCandidateMapper();objects=SemanticObjectMap()
    for i in range(3):feed(mapper,objects,[target(str(i),100.+i*.1)])
    assert not objects.objects


@pytest.mark.parametrize('kind',['low_score','person','missing_depth','wrong_capture','nominal_tf'])
def test_target_cannot_bypass_existing_quality_or_dynamic_gates(kind):
    mapper=TargetCandidateMapper();objects=SemanticObjectMap()
    for i in range(4):
        item=target(str(i),100.+i)
        if kind=='low_score':item['score']=.2
        elif kind=='person':item['label']=item['label_zh']='person'
        elif kind=='missing_depth':item['depth_m']=None
        elif kind=='wrong_capture':item['spatial_provenance']['frame_id']='someone-else'
        else:item['spatial_provenance']['image_time_transform']['transform_source']='nominal_extrinsic'
        feed(mapper,objects,[item])
    assert not objects.objects


def test_mixed_target_frames_preserve_each_source_and_time():
    mapper=TargetCandidateMapper();objects=SemanticObjectMap()
    for i in range(3):
        got,_,groups=feed(mapper,objects,[target(f'left-{i}',100.+i,entity='left',x=1.2),
            target(f'right-{i}',100.2+i,entity='right',x=3.2)])
    assert len(objects.objects)==2
    assert got[0]['persistent_object_id']!=got[1]['persistent_object_id']
    assert [g['frame_id'] for g in groups]==['left-2','right-2']
    assert objects.objects[got[0]['persistent_object_id']].last_seen==102.
    assert objects.objects[got[1]['persistent_object_id']].last_seen==102.2


def test_conflicting_timestamps_for_same_capture_do_not_create_stability():
    mapper=TargetCandidateMapper();objects=SemanticObjectMap()
    for i in range(3):feed(mapper,objects,[target(str(i),100.+i),target(str(i),101.+i,entity='other',x=3.)])
    assert not objects.objects and not mapper.tracker.tracks


def test_dynamic_reclassification_retires_the_previously_stable_target():
    mapper=TargetCandidateMapper();objects=SemanticObjectMap();item,_,_=stabilized(mapper,objects)
    oid=item['persistent_object_id']
    feed(mapper,objects,[target('moving-1',103.,x=1.6)])
    got,_,_=feed(mapper,objects,[target('moving-2',104.,x=2.)])
    assert 'persistent_object_id' not in got[0]
    assert objects.objects[oid].mobility_class=='dynamic'
    assert not objects.objects[oid].topology_eligible


def test_merge_preserves_target_source_id_without_aliasing_scene_associations():
    a=SemanticAssociation(0,'scene-1','scene-persistent','CREATED',1.,None)
    b=SemanticAssociation(0,target_source_id('cup'),'target-persistent','CREATED',1.,None)
    scene=SemanticMapUpdateResult(created_ids=['scene-persistent'],associations=[a])
    target_result=SemanticMapUpdateResult(created_ids=['target-persistent'],associations=[b])
    merged=merge_updates(scene,target_result)
    assert [x.observation_index for x in merged.associations]==[0,1]
    assert [x.source_object_id for x in merged.associations]==['scene-1','target:cup']
    assert b.observation_index==0


def explorer_and_observation():
    mapper=TargetCandidateMapper();objects=SemanticObjectMap();item,result,_=stabilized(mapper,objects)
    # A more confident unrelated object must never become the confirmed target.
    objects.objects['unrelated-chair']=SemanticObjectEntry('unrelated-chair','chair',confidence=.99)
    explorer=object.__new__(AutonomousExplorer);explorer.target='cup'
    explorer.semantic_graph=SimpleNamespace(object_map=objects)
    from app.navigation.search_policy_config import load_search_policy
    explorer.search_policy=load_search_policy()
    observation=LiveObservation(bundle_id='bundle',timestamp=5.,target_candidates=[item],
        pose={'x':0.,'y':0.,'yaw':0.,'frame_id':'map','quality':'metric'},
        scene_objects=[{'category':'target','label':'chair','depth_m':9.}],
        provenance={'target_spatial_contract':'capture_v1','current_capture_id':'capture-2'})
    mapping={'frame_object_ids':{a.source_object_id:a.persistent_object_id for a in result.associations}}
    return explorer,observation,mapping


def test_confirmed_identity_and_context_use_selected_target_capture_not_scene():
    explorer,observation,mapping=explorer_and_observation();candidate=observation.target_candidates[0]
    verdict=VerificationOutcome(True,1,details={'target_identity':target_identity(candidate)})
    oid=explorer._confirmed_target_object_id(observation,mapping,verification=verdict)
    assert oid==candidate['persistent_object_id'] and oid!='unrelated-chair'
    context=explorer._target_approach_context(observation,object_id=oid)
    assert context.depth_m==1.2 and context.map_xyz==(1.2,0.,.5)
    assert context.last_seen_timestamp==102.  # not the old scene timestamp=5


def test_metric_target_bearing_uses_current_pose_not_old_camera_angle():
    import math
    explorer,observation,_=explorer_and_observation()
    observation.pose['yaw']=math.pi/2
    oid=observation.target_candidates[0]['persistent_object_id']
    context=explorer._target_approach_context(observation,object_id=oid)
    assert context.bearing_deg==pytest.approx(-90.)
    observation.pose['frame_id']='odom'
    assert explorer._target_approach_context(observation,object_id=oid) is None


@pytest.mark.parametrize('change',['wrong_verdict','no_verdict','wrong_capture','wrong_persistent','moving','epoch','stale'])
def test_current_confirmation_never_falls_back_to_historical_label(change):
    explorer,observation,mapping=explorer_and_observation();candidate=observation.target_candidates[0]
    verdict=VerificationOutcome(True,1,details={'target_identity':target_identity(candidate)})
    if change=='wrong_verdict':verdict.details['target_identity']['entity_id']='different-cup'
    elif change=='no_verdict':verdict=None
    elif change=='wrong_capture':observation.provenance['current_capture_id']='new-image'
    elif change=='wrong_persistent':mapping['frame_object_ids']['target:cup']='unrelated-chair'
    elif change=='moving':explorer.semantic_graph.object_map.objects[candidate['persistent_object_id']].mobility_state='motion_candidate'
    elif change=='epoch':candidate['spatial_provenance']['image_time_transform']['map_epoch']='epoch-b'
    else:explorer.semantic_graph.object_map.objects[candidate['persistent_object_id']].status='STALE'
    assert explorer._confirmed_target_object_id(observation,mapping,verification=verdict) is None


def actual_verifier_wrapper(unbound):
    import ast
    from pathlib import Path
    from app.live_robot.autonomous_explorer import SemanticMatch
    from app.perception.entity_roles import select_candidate
    path=Path(__file__).resolve().parents[1]/'scripts/go2w/run_semantic_exploration.py'
    source=path.read_text();tree=ast.parse(source)
    fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='verifier'
        and 'target_spatial_contract' in (ast.get_source_segment(source,n) or ''))
    namespace=dict(LiveObservation=LiveObservation,SemanticMatch=SemanticMatch,
        VerificationOutcome=VerificationOutcome,VERIFY_STATUS_UNKNOWN='UNKNOWN',
        _verify_candidate_unbound=unbound,select_candidate=select_candidate,
        settings=SimpleNamespace(go2w_verify_candidate_scope='target_candidates',go2w_result_role_separation=True))
    namespace['node']=SimpleNamespace(_write=lambda value:None,_host_s=lambda:0.)
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(path),'exec'),namespace)
    return namespace['verifier']


def test_actual_verifier_binds_cached_result_to_the_current_target_identity():
    _,observation,_=explorer_and_observation();called=[]
    def inner(obs,match):called.append(1);return VerificationOutcome(True,1,details={'cached':True})
    result=actual_verifier_wrapper(inner)(observation,None)
    assert result.confirmed and result.details['cached'] is True and called==[1]
    assert result.details['target_identity']==target_identity(observation.target_candidates[0])


def test_actual_verifier_never_sends_old_box_to_current_image_or_relabels_wrong_result():
    _,observation,_=explorer_and_observation();called=[]
    def inner(obs,match):called.append(1);return VerificationOutcome(True,1,details={'target_identity':{'entity_id':'wrong'}})
    wrapper=actual_verifier_wrapper(inner)
    result=wrapper(observation,None)
    assert not result.confirmed and result.status=='UNKNOWN'
    observation.provenance['current_capture_id']='different-image'
    result=wrapper(observation,None)
    assert not result.confirmed and called==[1]


def test_state_machine_stays_stopped_until_three_captures_bind_the_verified_target():
    from app.navigation.backend_factory import MockBackend
    from app.navigation.exploration_config import load_exploration_policy,ExplorationBudget
    from app.live_robot.autonomous_explorer import SemanticMatch
    from app.spatial.semantic_navigation_graph import SemanticNavigationGraph
    mapper=TargetCandidateMapper();objects=SemanticObjectMap();seen=[];approach=[]
    class NoMotion(MockBackend):
        def execute_goal(self,*args,**kwargs):raise AssertionError('pending target must never command motion')
        def get_pose(self):
            from app.navigation.robot_backend import RobotPose,PoseQuality
            return RobotPose(0.,0.,0.,frame_id='map',quality=PoseQuality.METRIC,source='synthetic-test')
    backend=NoMotion()
    def observe():
        i=len(seen);assert i<3
        frame=f'capture-{i}';items,result,_=feed(mapper,objects,[target(frame,100.+i)])
        observation=LiveObservation(bundle_id='bundle-'+frame,timestamp=100.+i,
            target_candidates=items,object_update_result=result,
            target_match={'target_present':True,'score':.9},
            provenance={'target_spatial_contract':'capture_v1','current_capture_id':frame})
        seen.append(observation);return observation
    def verify(obs,match):return VerificationOutcome(True,1,details={'target_identity':target_identity(obs.target_candidates[0])})
    policy=load_exploration_policy();policy.budget=ExplorationBudget(max_search_seconds=5.,max_planning_cycles=10)
    explorer=AutonomousExplorer(target='cup',observer=observe,matcher=lambda obs:SemanticMatch(
        has_candidate=True,target_score=.9,target_match_level='candidate'),verifier=verify,
        backend=backend,policy=policy,finish_on_visual_confirmation=False,
        semantic_graph=SemanticNavigationGraph(object_map=objects))
    def ready(observation,match,object_id):
        assert len(seen)==3
        context=explorer._target_approach_context(observation,object_id=object_id)
        assert context.map_xyz==(1.2,0.,.5);approach.append(object_id)
        return True,{'scope':'no-motion unit fixture, approach controller separately tested'}
    explorer._run_target_approach=ready
    result=explorer.run()
    assert result.result=='TARGET_FOUND' and len(approach)==1
    assert len([e for e in explorer.events if e['event']=='target_position_pending'])==2
