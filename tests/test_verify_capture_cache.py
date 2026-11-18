"""Run actual live verifier closures; distinct capture/box identities cannot share verdicts."""
from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from app.live_robot.async_verifier import VerificationResult, result_matches_submission
from app.live_robot.autonomous_explorer import SemanticMatch, VerificationOutcome
from app.live_robot.verify_cache import VerificationCache, VerificationCacheEntry
from app.navigation.models import LiveObservation
from app.perception.entity_roles import entity_id_for, select_candidate
from app.perception.target_spatial import target_identity


def live_verifier(async_mode=False):
    path=Path(__file__).resolve().parents[1]/'scripts/go2w/run_semantic_exploration.py'
    source=path.read_text();tree=ast.parse(source);names={'_async_verify','_verify_candidate_unbound'}
    functions=[n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and
        (n.name in names or (n.name=='verifier' and 'target_spatial_contract' in (ast.get_source_segment(source,n) or '')))]
    assert len(functions)==3
    calls=[]
    def verify(image,bbox,env,**kwargs):
        calls.append(dict(image=image,bbox=bbox,**kwargs))
        return {'is_target':True,'confidence':.9}
    class Immediate:
        task_generation=7
        def submit(self,**kwargs):
            verify(kwargs['image_path'],kwargs['bbox'],kwargs['env'],frame_id=kwargs['frame_id'])
            self.request=kwargs;return len(calls)
        def await_submission(self,seq):
            return VerificationResult(sequence=seq,task_generation=7,
                frame_id=self.request['frame_id'],candidate_id=self.request['candidate_id'],
                status='CONFIRMED',details={'is_target':True,'confidence':.9})
        def queue_depth(self):return {}
    ns=dict(LiveObservation=LiveObservation,SemanticMatch=SemanticMatch,
        VerificationOutcome=VerificationOutcome,VerificationCacheEntry=VerificationCacheEntry,
        VERIFY_STATUS_UNKNOWN='UNKNOWN',VERIFY_STATUS_CONFIRMED='CONFIRMED',
        entity_id_for=entity_id_for,target_identity=target_identity,
        select_candidate=select_candidate,result_matches_submission=result_matches_submission,
        node=SimpleNamespace(_verify_target=verify,_write=lambda value:None,_host_s=lambda:0.),
        args=SimpleNamespace(target='cup',verify_min_confidence=.5),env={},
        settings=SimpleNamespace(go2w_verify_candidate_scope='target_candidates',go2w_result_role_separation=True),
        verify_cache=VerificationCache(),async_verifier=Immediate() if async_mode else None)
    exec(compile(ast.Module(body=functions,type_ignores=[]),str(path),'exec'),ns)
    return ns['verifier'],calls


def observation(*,box=None,entity='cup-a',source='capture-a',contract=True):
    item=dict(entity_id=entity,entity_source_frame_id=source,label='cup',score=.9,
              bbox_2d=box or [.101,.20,.301,.60])
    return LiveObservation(bundle_id='same-observation-bundle',timestamp=1.,
        target_candidates=[item],image_ref='/immutable/'+source+'.jpg',
        provenance=dict(target_spatial_contract='capture_v1',current_capture_id=source) if contract else {})


@pytest.mark.parametrize('async_mode',[False,True])
@pytest.mark.parametrize('change',['nearby_box','different_entity','different_capture'])
def test_distinct_target_identity_never_inherits_cached_verdict(async_mode,change):
    verify,calls=live_verifier(async_mode)
    first=observation();assert verify(first,None).confirmed
    kwargs={'box':[.104,.20,.304,.60]} if change=='nearby_box' else (
        {'entity':'cup-b'} if change=='different_entity' else {'source':'capture-b'})
    second=observation(**kwargs);outcome=verify(second,None)
    assert outcome.confirmed
    assert len(calls)==2, 'new identity was relabelled with another request’s cached verdict'
    assert outcome.details['target_identity']==target_identity(second.target_candidates[0])


@pytest.mark.parametrize('async_mode',[False,True])
def test_identical_target_identity_still_uses_one_verification(async_mode):
    verify,calls=live_verifier(async_mode)
    verify(observation(),None);verify(observation(),None)
    assert len(calls)==1


def test_legacy_cache_key_and_quantization_remain_unchanged():
    cache=VerificationCache()
    assert cache.make_key('f','cup',[.101,.20,.301,.60])==('f','cup','0.10,0.20,0.30,0.60')
