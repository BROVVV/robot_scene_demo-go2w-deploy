"""Recorded cloud/camera geometry, separate from manufacturer mount metadata."""
import hashlib
import math
from pathlib import Path

import yaml


def load_sensor_profile(path):
    path=Path(path).resolve()
    profile=yaml.safe_load(path.read_text())
    if (not isinstance(profile,dict) or profile.get('schema_version') != 1
            or profile.get('robot_model') != 'Unitree Go2-W'
            or profile.get('kind') != 'recorded_rgbd_floor_estimate'
            or profile.get('execution_scope') not in ('bounded_trial', 'operator_supervised_field')
            or profile.get('manufacturer_calibration') is not False):
        raise ValueError('unsupported sensor geometry profile')
    if profile['execution_scope'] == 'operator_supervised_field':
        field = profile.get('field_operation', {})
        if (field.get('operator_supervised') is not True
                or field.get('independent_wheel_guard') is not True
                or field.get('geometry_quality') != 'engineering_estimate'
                or type(field.get('maximum_goal_travel_m')) not in (int, float)
                or field['maximum_goal_travel_m'] != 3.
                or type(field.get('maximum_linear_speed_mps')) not in (int, float)
                or field['maximum_linear_speed_mps'] != .30):
            raise ValueError('field geometry requires supervision and independent travel limits')
    for key,child in [('base_to_cloud','utlidar_lidar'),('base_to_camera','front_camera_optical_frame')]:
        frame=profile['frames'][key]
        if frame.get('parent') != 'base_link' or frame.get('child') != child:
            raise ValueError('unexpected calibrated sensor frame')
        for field in ['translation_m','rotation_rpy_rad']:
            vector=frame[field]
            if not isinstance(vector,list) or len(vector)!=3 or not all(
                    type(v) in (int,float) and math.isfinite(v) for v in vector):
                raise ValueError('invalid calibration vector')
        if math.sqrt(sum(v*v for v in frame['translation_m']))>.7:
            raise ValueError('sensor translation exceeds installation envelope')
    geometry=profile['preprocess']
    keys=('minimum_range','maximum_range','minimum_height','maximum_height','ground_height',
          'collision_maximum_height','self_half_length','self_half_width','self_filter_margin',
          'front_corridor_half_width','rotation_envelope_radius')
    if any(type(geometry.get(k)) not in (int,float) or not math.isfinite(geometry[k]) for k in keys):
        raise ValueError('invalid estimated preprocessing geometry')
    if (not 0 < geometry['minimum_range'] < geometry['maximum_range'] <= 10
            or not geometry['minimum_height'] < geometry['ground_height'] < geometry['collision_maximum_height'] <= geometry['maximum_height']
            or geometry['self_half_length'] < .35 or geometry['self_half_width'] < .215
            or not 0 <= geometry['self_filter_margin'] <= .06
            or geometry['front_corridor_half_width'] <= geometry['self_half_width']
            or geometry['rotation_envelope_radius'] < math.hypot(geometry['self_half_length'],geometry['self_half_width'])):
        raise ValueError('unsafe estimated geometry envelope')
    if geometry.get('self_regions'):
        raise ValueError('old empirical masks cannot be copied into calibrated geometry')
    q=profile['posture']['joint_q']
    if not isinstance(q,list) or len(q)!=12 or not all(math.isfinite(v) for v in q):
        raise ValueError('missing calibrated leg posture')
    if not 0 < profile['posture']['maximum_joint_deviation_rad'] <= .1:
        raise ValueError('unbounded posture tolerance')
    attitude=profile['posture'].get('body_roll_pitch_rad')
    if (not isinstance(attitude,list) or len(attitude)!=2
            or not all(math.isfinite(v) for v in attitude)
            or not 0 < profile['posture'].get('maximum_attitude_deviation_rad',0) <= .02):
        raise ValueError('floor estimate needs a bounded body attitude')
    root=path.parents[2]
    evidence=profile.get('evidence')
    if not isinstance(evidence,list) or not evidence:
        raise ValueError('calibration evidence required')
    for entry in evidence:
        source=(root/entry['path']).resolve()
        if not source.is_file() or hashlib.sha256(source.read_bytes()).hexdigest()!=entry['sha256']:
            raise ValueError('sensor calibration evidence missing or changed')
    model=profile.get('raw_self_return_model')
    if model:
        source=(root/model['path']).resolve()
        dynamic_radius=model.get('dynamic_radius_m',model['radius_m'])
        dynamic_region=model.get('dynamic_region_base')
        if (model.get('method')!='raw_background_registration_body_vs_world'
                or not 0 < model['radius_m'] <= .03
                or not model['radius_m'] <= dynamic_radius <= .05
                or (dynamic_radius > model['radius_m'] and not isinstance(dynamic_region,dict))
                or not any(e['path']==model['path'] for e in evidence)):
            raise ValueError('unsupported raw self-return surface model')
        if dynamic_region is not None:
            required=('x_abs_max','y_abs_min','y_abs_max','z_min','z_max')
            if (any(type(dynamic_region.get(key)) not in (int,float)
                    or not math.isfinite(dynamic_region[key]) for key in required)
                    or not 0 < dynamic_region['x_abs_max'] <= .45
                    or not 0 <= dynamic_region['y_abs_min'] < dynamic_region['y_abs_max'] <= .45
                    or not -.6 <= dynamic_region['z_min'] < dynamic_region['z_max'] <= .1):
                raise ValueError('unsafe dynamic self-return region')
        profile['raw_self_return_model']['absolute_path']=str(source)
    plane=profile.get('raw_floor_plane')
    if plane:
        n=plane['normal'];d=plane['distance_m'];tolerance=plane['tolerance_m']
        if (len(n)!=3 or not all(math.isfinite(v) for v in (*n,d,tolerance))
                or abs(sum(v*v for v in n)-1)>.001
                or not .2<d<.8 or not 0<tolerance<=.025):
            raise ValueError('invalid observed floor model')
    compensation=profile.get('floor_attitude_compensation')
    if compensation:
        source=(root/compensation['urdf_path']).resolve()
        if (not plane or compensation.get('method')!='imu_fk_with_observed_floor'
                or compensation.get('minimum_obstacle_height_m')!=.08
                or compensation.get('urdf_sha256')!='f5940cbef9be28b76c075a94ed535c9c8c22c89a79b54ec8c854dc9f023c1ab8'
                or not any(e['path']==compensation['urdf_path'] for e in evidence)
                or not source.is_file()
                or hashlib.sha256(source.read_bytes()).hexdigest()!=compensation['urdf_sha256']):
            raise ValueError('floor compensation needs pinned kinematics and measured floor evidence')
        compensation['urdf_absolute_path']=str(source)
    profile['profile_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    return profile
