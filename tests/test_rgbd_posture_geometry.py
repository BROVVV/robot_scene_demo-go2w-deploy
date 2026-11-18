"""A filter cutoff must never masquerade as a measured camera-floor height."""
import copy
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('rgbd_posture_geometry_test',
    ROOT/'scripts/go2w/rgbd_posture_geometry.py')
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)


def profile():
    return {'raw_floor_plane':{'normal':[0.,0.,-1.],'distance_m':.5},
        'frames':{'base_to_cloud':{'rotation_rpy_rad':[0.,0.,0.],'translation_m':[0.,0.,0.]},
                  'base_to_camera':{'rotation_rpy_rad':[-math.pi/2,0.,-math.pi/2],'translation_m':[0.,0.,.1]}},
        'posture':{'body_roll_pitch_rad':[0.,0.]},'preprocess':{'ground_height':-.45}}


def test_known_plane_and_cutoff_independence():
    p = profile(); result = geometry.predict_camera_floor(p)
    np.testing.assert_allclose(result['normal_camera'],[0.,1.,0.],atol=1e-12)
    assert result['distance_camera_to_plane_m'] == pytest.approx(.6)
    assert result['vertical_camera_to_plane_m'] == pytest.approx(.6)
    p['preprocess']['ground_height'] += 2.
    assert geometry.predict_camera_floor(p) == result
    del p['preprocess']
    assert geometry.predict_camera_floor(p) == result
    assert not result['calibration_accepted'] and not result['authorizes_motion']


@pytest.mark.parametrize('scale',[-2.,.5,3.])
def test_plane_equation_scale_and_sign_do_not_change_geometry(scale):
    p = profile(); expected = geometry.predict_camera_floor(p)
    p['raw_floor_plane']['normal'] = (np.array(p['raw_floor_plane']['normal'])*scale).tolist()
    p['raw_floor_plane']['distance_m'] *= scale
    actual = geometry.predict_camera_floor(p)
    np.testing.assert_allclose(actual['normal_camera'],expected['normal_camera'],atol=1e-12)
    assert actual['distance_camera_to_plane_m'] == pytest.approx(.6)


def test_prediction_matches_independently_transformed_plane_points():
    p = profile()
    p['frames']['base_to_cloud'].update(rotation_rpy_rad=[.1,-.05,.3],translation_m=[.2,-.1,.03])
    p['frames']['base_to_camera'].update(rotation_rpy_rad=[-1.4,.03,-1.5],translation_m=[.3,.01,.08])
    result = geometry.predict_camera_floor(p)
    x,y = np.meshgrid(np.linspace(-1.,1.,7),np.linspace(-.7,.8,9))
    raw = np.column_stack((x.ravel(),y.ravel(),np.full(x.size,-.5)))
    lidar, camera = p['frames']['base_to_cloud'],p['frames']['base_to_camera']
    body = raw@geometry.rpy_matrix(*lidar['rotation_rpy_rad']).T+lidar['translation_m']
    optical = (body-camera['translation_m'])@geometry.rpy_matrix(*camera['rotation_rpy_rad'])
    np.testing.assert_allclose(optical@np.array(result['normal_camera']),result['distance_camera_to_plane_m'],atol=1e-12)


@pytest.mark.parametrize('kind',['zero_normal','nan_distance','bad_camera_translation','vertical_plane','ceiling'])
def test_malformed_or_vertical_planes_cannot_predict_floor_height(kind):
    p = profile()
    if kind == 'zero_normal':p['raw_floor_plane']['normal']=[0.,0.,0.]
    elif kind == 'nan_distance':p['raw_floor_plane']['distance_m']=float('nan')
    elif kind == 'bad_camera_translation':p['frames']['base_to_camera']['translation_m']=[0.,0.]
    elif kind == 'ceiling':p['raw_floor_plane']['distance_m']=-.5
    else:p['raw_floor_plane']['normal']=[1.,0.,0.]
    with pytest.raises(ValueError):geometry.predict_camera_floor(p)


def test_current_posture_cannot_fall_back_to_reference_when_parts_are_missing():
    with pytest.raises(ValueError,match='together'):
        geometry.predict_camera_floor(profile(),joint_q=[0.]*12)


def test_recorded_profile_uses_actual_floor_plane_not_its_filter_height():
    p = yaml.safe_load((ROOT/'configs/go2w/navigation_sensor_profile.yaml').read_text())
    before = copy.deepcopy(p)
    actual = geometry.predict_camera_floor(p)
    assert actual['distance_camera_to_plane_m'] == pytest.approx(.4886678034897911)
    assert abs(actual['reference_floor_z_at_body_xy0_m']-p['preprocess']['ground_height']) > .04
    assert p == before


def test_reference_posture_fk_and_plane_predictions_are_consistent():
    p = yaml.safe_load((ROOT/'configs/go2w/navigation_sensor_profile.yaml').read_text())
    urdf = (ROOT/p['floor_attitude_compensation']['urdf_path']).read_bytes()
    result = geometry.predict_camera_floor(p,urdf_bytes=urdf,
        joint_q=p['posture']['joint_q'],roll_pitch=p['posture']['body_roll_pitch_rad'])
    assert result['fk_height_delta_m'] == 0.
    assert result['current_fk_base_height_m'] == pytest.approx(.4274780120626697)
    assert abs(result['kinematic_camera_height_m']-result['vertical_camera_to_plane_m']) < .002


def test_posture_height_delta_is_invariant_to_plane_direction():
    p = yaml.safe_load((ROOT/'configs/go2w/navigation_sensor_profile.yaml').read_text())
    urdf = (ROOT/p['floor_attitude_compensation']['urdf_path']).read_bytes()
    joints = list(p['posture']['joint_q'])
    for leg in range(4):
        joints[leg*3+1] += .015
        joints[leg*3+2] -= .01
    options = dict(urdf_bytes=urdf,joint_q=joints,roll_pitch=p['posture']['body_roll_pitch_rad'])
    expected = geometry.predict_camera_floor(p,**options)
    assert abs(expected['fk_height_delta_m']) > .0001
    p['raw_floor_plane']['normal'] = (-2*np.array(p['raw_floor_plane']['normal'])).tolist()
    p['raw_floor_plane']['distance_m'] *= -2
    actual = geometry.predict_camera_floor(p,**options)
    np.testing.assert_allclose(actual['normal_camera'],expected['normal_camera'],atol=1e-12)
    assert actual['distance_camera_to_plane_m'] == pytest.approx(expected['distance_camera_to_plane_m'])
