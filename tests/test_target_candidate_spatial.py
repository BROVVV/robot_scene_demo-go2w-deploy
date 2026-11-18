from dataclasses import replace
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from app.perception.rgbd_source import RGBDFrame
from app.perception.target_spatial import localize_target_candidates, project_target_candidates
from app.live_robot.semantic_observer import local_detection_scene_objects
from app.perception.entity_roles import classify_roles
from app.reasoning.target_profile import TargetProfile


def frame(tmp_path, epoch, depth=1200):
    path=tmp_path/(epoch+'.png')
    assert cv2.imwrite(str(path),np.full((24,32),depth,dtype=np.uint16))
    return RGBDFrame(frame_id='7',timestamp=100.,color_ref=epoch+'.jpg',depth_ref=str(path),
        width=32,height=24,fx=20.,fy=20.,cx=16.,cy=12.,provenance={'capture_id':epoch})


def candidate(f, **extra):
    return dict(label='electric chair',score=.9,bbox_2d=[.25,.25,.75,.75],
                entity_id='selected-chair',entity_source_frame_id=f.evidence_id,**extra)


def test_target_excluded_from_scene_still_gets_its_own_depth(tmp_path):
    f=frame(tmp_path,'a');profile=TargetProfile(raw_query='椅子',canonical_name_zh='椅子',primary_labels_en=['chair'])
    objects=[dict(candidate(f),source_prompt_term='chair')]
    assert local_detection_scene_objects(objects,profile,source_frame_id=f.evidence_id,current_frame_id=f.evidence_id)==[]
    targets=classify_roles(objects,profile,source_frame_id=f.evidence_id).target_candidates
    got=localize_target_candidates(targets,current_frame=f)
    assert got[0]['depth_m']==pytest.approx(1.2)
    assert got[0]['camera_xyz']==pytest.approx([0.,0.,1.2])
    assert got[0]['entity_id']=='selected-chair'
    assert 'depth_m' not in targets[0]
    assert got[0]['spatial_provenance']['frame_id']==f.evidence_id


def test_mixed_views_use_each_capture_not_the_current_depth(tmp_path):
    old,current=frame(tmp_path,'before',1200),frame(tmp_path,'after',4400)
    got=localize_target_candidates([candidate(old),candidate(current)],current_frame=current,
        cache={old.evidence_id:old})
    assert [x['depth_m'] for x in got]==pytest.approx([1.2,4.4])
    assert old.frame_id==current.frame_id


@pytest.mark.parametrize('mode',['evicted','wrong_cache_value','unknown','counter_only'])
def test_missing_or_unproven_acquisition_clears_old_geometry(tmp_path,mode):
    old,current=frame(tmp_path,'old'),frame(tmp_path,'new',4400);item=candidate(old,map_xyz=[9.,0.,0.],depth_m=9.)
    cache={}
    if mode=='wrong_cache_value':cache[old.evidence_id]=current
    elif mode=='unknown':item.pop('entity_source_frame_id')
    elif mode=='counter_only':item['entity_source_frame_id']='7'
    got=localize_target_candidates([item],current_frame=current,cache=cache)[0]
    assert got['camera_xyz'] is None and got['map_xyz'] is None and got['depth_m'] is None


def test_duplicate_identity_in_one_capture_does_not_join_different_boxes(tmp_path):
    f=frame(tmp_path,'a');a=candidate(f);b=dict(a,bbox_2d=[0.,0.,.2,.2])
    got=localize_target_candidates([a,b],current_frame=f)
    assert all(x['camera_xyz'] is None for x in got)


def test_unaligned_depth_is_not_target_geometry(tmp_path):
    f=replace(frame(tmp_path,'a'),depth_aligned_to_color=False)
    assert localize_target_candidates([candidate(f)],current_frame=f)[0]['camera_xyz'] is None


@pytest.mark.parametrize('value',[None,float('nan'),0.])
def test_invalid_intrinsics_fail_closed_without_throwing(tmp_path,value):
    f=replace(frame(tmp_path,'a'),fx=value)
    assert localize_target_candidates([candidate(f)],current_frame=f)[0]['camera_xyz'] is None


def provider(calls, **overrides):
    evidence=dict(target_frame='map',transform_source='tf2',tf_available=True,transform_quality='METRIC_LIDAR')
    evidence.update(overrides)
    def project(xyz, stamp):calls.append((xyz,stamp));return (xyz[2],-xyz[0],.5-xyz[1])
    return SimpleNamespace(quality=lambda:'METRIC_LIDAR',camera_point_to_map=project,
        transform_provenance=lambda:evidence)


def test_projection_uses_target_capture_time_and_keeps_candidates_separate(tmp_path):
    f=frame(tmp_path,'a');items=localize_target_candidates([candidate(f)],current_frame=f);calls=[]
    got=project_target_candidates(items,provider(calls),pose_accepted=True)[0]
    assert calls==[((0.,0.,1.2),100.)]
    assert got['map_xyz']==pytest.approx([1.2,0.,.5])
    assert got['spatial_quality']=='METRIC_RGBD'
    assert got['spatial_provenance']['motion_authorized'] is False
    assert items[0]['map_xyz'] is None


@pytest.mark.parametrize('change',['pose_rejected','nominal','no_tf','odom_frame','bad_quality','wrong_frame','bad_point','no_stamp'])
def test_unproven_transform_never_promotes_target_to_map(tmp_path,change):
    f=frame(tmp_path,'a');items=localize_target_candidates([candidate(f)],current_frame=f);calls=[]
    overrides={}
    if change=='nominal':overrides['transform_source']='nominal_extrinsic'
    elif change=='no_tf':overrides['tf_available']=False
    elif change=='odom_frame':overrides['target_frame']='odom_fused'
    elif change=='bad_quality':overrides['transform_quality']='CAMERA_LOCAL'
    elif change=='wrong_frame':items[0]['spatial_provenance']['frame_id']='unrelated'
    elif change=='bad_point':items[0]['camera_xyz']=[float('nan'),0.,1.]
    elif change=='no_stamp':items[0]['spatial_provenance'].pop('image_timestamp')
    got=project_target_candidates(items,provider(calls,**overrides),pose_accepted=change!='pose_rejected')[0]
    assert got['map_xyz'] is None and got['spatial_quality']!='METRIC_RGBD'


def test_rejected_new_projection_removes_old_map_provenance(tmp_path):
    f=frame(tmp_path,'a');items=localize_target_candidates([candidate(f)],current_frame=f)
    old=project_target_candidates(items,provider([]),pose_accepted=True)
    got=project_target_candidates(old,provider([]),pose_accepted=False)[0]
    assert got['map_xyz'] is None
    assert 'map_frame' not in got['spatial_provenance']
    assert 'image_time_transform' not in got['spatial_provenance']


def test_enrichment_preserves_shared_detection_target_anchor_dictionaries(tmp_path):
    from app.perception.target_spatial import bind_target_measurements
    f=frame(tmp_path,'a')
    profile=TargetProfile(raw_query='椅子',canonical_name_zh='椅子',primary_labels_en=['chair'],
        relation_anchor_labels_en=['chair'])
    partition=classify_roles([dict(candidate(f),source_prompt_term='chair')],profile,source_frame_id=f.evidence_id)
    measured=localize_target_candidates(partition.target_candidates,current_frame=f)
    bind_target_measurements(partition.target_candidates,measured,source_frame_id=f.evidence_id)
    first=partition.target_candidates[0]
    assert first is partition.detections[0] and first is partition.relation_anchors[0]
    assert first['depth_m']==1.2 and partition.scene_objects==[]
    projected=project_target_candidates(partition.target_candidates,provider([]),pose_accepted=True)
    bind_target_measurements(partition.target_candidates,projected)
    assert first['map_xyz']==pytest.approx([1.2,0.,.5])


def test_shared_binding_refuses_a_box_from_another_entity_or_capture(tmp_path):
    from app.perception.target_spatial import bind_target_measurements
    f=frame(tmp_path,'a');original=candidate(f,map_xyz=[9.,0.,0.])
    other=dict(original,bbox_2d=[0.,0.,.2,.2],map_xyz=[1.,2.,3.])
    bind_target_measurements([original],[other])
    assert original['map_xyz'] is None and original['camera_xyz'] is None
