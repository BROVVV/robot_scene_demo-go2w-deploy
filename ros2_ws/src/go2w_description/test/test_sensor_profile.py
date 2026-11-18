import copy
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import shutil

import pytest
import yaml

from go2w_description.sensor_profile import load_sensor_profile
from go2w_description.description_config import load_official_reference,render_official_sensor_urdf

ROOT=Path(__file__).resolve().parents[4]


def fixture_profile(tmp_path):
    profile=yaml.safe_load((ROOT/'configs/go2w/navigation_sensor_profile.yaml').read_text())
    profile['execution_scope']='bounded_trial'
    profile.pop('raw_self_return_model',None)
    profile.pop('floor_attitude_compensation',None)
    target=tmp_path/'configs/go2w/profile.yaml';target.parent.mkdir(parents=True)
    evidence=tmp_path/'evidence.json';evidence.write_text('{"scope":"test fixture"}')
    profile['evidence']=[dict(path='evidence.json',sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())]
    target.write_text(yaml.safe_dump(profile))
    return target,profile


def test_calibrated_cloud_is_separate_from_mount_reference_and_has_camera_tf(tmp_path):
    path,_=fixture_profile(tmp_path);profile=load_sensor_profile(path)
    reference=load_official_reference(str(ROOT/'configs/go2w/official_reference.yaml'))
    xml=ET.fromstring(render_official_sensor_urdf(reference,profile))
    frames={j.find('child').attrib['link']:j for j in xml.findall('joint')}
    assert 'utlidar_mount_reference' in frames
    assert frames['utlidar_lidar'].find('parent').attrib['link']=='base_link'
    actual=list(map(float,frames['utlidar_lidar'].find('origin').attrib['xyz'].split()))
    assert actual==pytest.approx(profile['frames']['base_to_cloud']['translation_m'])
    assert 'front_camera_optical_frame' in frames and 'd435_color_optical_frame' in frames
    assert profile['execution_scope']=='bounded_trial'


def test_runtime_profile_can_load_without_the_artifacts_tree(tmp_path):
    target=tmp_path/'configs/go2w';target.mkdir(parents=True)
    shutil.copy2(ROOT/'configs/go2w/navigation_sensor_profile.yaml',target/'navigation_sensor_profile.yaml')
    raw=yaml.safe_load((target/'navigation_sensor_profile.yaml').read_text())
    for entry in raw['evidence']:
        relative=Path(entry['path'])
        assert relative.parts[:2]==('configs','go2w')
        destination=tmp_path/relative;destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/relative,destination)
    profile=load_sensor_profile(target/'navigation_sensor_profile.yaml')
    assert Path(profile['raw_self_return_model']['absolute_path']).is_file()
    assert not (tmp_path/'artifacts').exists()


@pytest.mark.parametrize('change',['nan','wrong_frame','old_mask','scope','evidence','small_body'])
def test_invalid_or_unbounded_geometry_cannot_load(tmp_path,change):
    path,p=fixture_profile(tmp_path)
    if change=='nan':p['frames']['base_to_cloud']['translation_m'][0]=float('nan')
    elif change=='wrong_frame':p['frames']['base_to_cloud']['child']='another_lidar'
    elif change=='old_mask':p['preprocess']['self_regions']=[[-1,1,-1,1,-1,1]]
    elif change=='scope':p['execution_scope']='autonomous_search'
    elif change=='evidence':(tmp_path/'evidence.json').write_text('changed')
    else:p['preprocess']['self_half_width']=.1
    path.write_text(yaml.safe_dump(p))
    with pytest.raises(ValueError):load_sensor_profile(path)


@pytest.mark.parametrize('bad',['none','no_supervision','no_guard','distance','speed','quality'])
def test_field_profile_requires_explicit_engineering_contract(tmp_path,bad):
    path,p=fixture_profile(tmp_path)
    p['execution_scope']='operator_supervised_field'
    p['field_operation']=dict(operator_supervised=True,independent_wheel_guard=True,
        geometry_quality='engineering_estimate',maximum_goal_travel_m=3.,maximum_linear_speed_mps=.30)
    if bad=='no_supervision':p['field_operation']['operator_supervised']=False
    elif bad=='no_guard':p['field_operation']['independent_wheel_guard']=False
    elif bad=='distance':p['field_operation']['maximum_goal_travel_m']=3.1
    elif bad=='speed':p['field_operation']['maximum_linear_speed_mps']=float('nan')
    elif bad=='quality':p['field_operation']['geometry_quality']='manufacturer_calibrated'
    path.write_text(yaml.safe_dump(p))
    if bad=='none':assert load_sensor_profile(path)['manufacturer_calibration'] is False
    else:
        with pytest.raises(ValueError,match='field geometry'):load_sensor_profile(path)


@pytest.mark.parametrize('change', ['valid','method','height','hash','missing_evidence'])
def test_floor_compensation_is_bound_to_official_model(tmp_path,change):
    path,p=fixture_profile(tmp_path)
    relative='configs/go2w/calibration/official_go2w.urdf'
    source=tmp_path/relative;source.parent.mkdir(parents=True)
    shutil.copy2(ROOT/'configs/go2w/calibration_20261019/official_go2w.urdf',source)
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    p['evidence'].append(dict(path=relative,sha256=digest))
    p['floor_attitude_compensation']=dict(method='imu_fk_with_observed_floor',
        urdf_path=relative,urdf_sha256=digest,minimum_obstacle_height_m=.08)
    if change=='method':p['floor_attitude_compensation']['method']='trust_imu_without_floor'
    elif change=='height':p['floor_attitude_compensation']['minimum_obstacle_height_m']=.2
    elif change=='hash':p['floor_attitude_compensation']['urdf_sha256']='0'*64
    elif change=='missing_evidence':p['evidence'].pop()
    path.write_text(yaml.safe_dump(p))
    if change=='valid':assert Path(load_sensor_profile(path)['floor_attitude_compensation']['urdf_absolute_path']).is_file()
    else:
        with pytest.raises(ValueError):load_sensor_profile(path)
