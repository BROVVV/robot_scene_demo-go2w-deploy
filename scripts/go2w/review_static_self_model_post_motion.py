#!/usr/bin/env python3
"""Read-only replay of the stopped robot's raw clouds for a small posture bound.

This certifies only the observed parked pose. It does not validate the
transient joint excursion during motion or authorize navigation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'ros2_ws/src/go2w_lidar_preprocessor'),
              str(ROOT/'ros2_ws/src/go2w_description')]
from go2w_description.description_config import _rpy_to_quaternion_xyzw
from go2w_lidar_preprocessor.config import load_safety_ready_config
from go2w_lidar_preprocessor.floor_attitude import estimate_floor, rpy_matrix, standing_height
from go2w_lidar_preprocessor.preprocess_core import (
    collision_obstacles, directional_clearance, filter_points_base_link, transform_points,
)


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def review(capture, floor_review, profile_path):
    import yaml
    capture=Path(capture);profile_path=Path(profile_path)
    profile=yaml.safe_load(profile_path.read_text())
    floor=json.loads(Path(floor_review).read_text())
    metadata=json.loads((capture/'metadata.json').read_text())
    if metadata.get('motion_sent') is not False or floor.get('floor_valid_frames')!=93:
        raise ValueError('expected the independent, parked, 93-frame capture')
    current_q=np.asarray(floor['current_joint_q'],dtype=float)
    reference_q=np.asarray(profile['posture']['joint_q'],dtype=float)
    deviations=np.abs(current_q-reference_q)
    if not .08<float(deviations.max())<.09:
        raise ValueError('this revalidation applies only to the measured 0.08..0.09 rad pose')
    model_path=ROOT/profile['raw_self_return_model']['path']
    model=np.load(model_path,allow_pickle=False)['points']
    tree=cKDTree(model)
    _,geometry=load_safety_ready_config(str(ROOT/'configs/go2w/lidar_preprocess.yaml'),
        str(ROOT/'configs/go2w/official_reference.yaml'),str(profile_path))
    urdf=(ROOT/profile['floor_attitude_compensation']['urdf_path']).read_bytes()
    height_delta=(standing_height(urdf,current_q,floor['roll_pitch'])-
                  standing_height(urdf,reference_q,profile['posture']['body_roll_pitch_rad']))
    extr=profile['frames']['base_to_cloud']
    rotation=rpy_matrix(*extr['rotation_rpy_rad'])
    translation=np.asarray(extr['translation_m'],dtype=float)
    quaternion=_rpy_to_quaternion_xyzw(tuple(extr['rotation_rpy_rad']))
    archive=np.load(capture/'cloud.npz',allow_pickle=False)
    fronts={True:[],False:[]};removed=[];far_removed=0;offset=0
    for size in archive['sizes']:
        source=archive['points'][offset:offset+size].astype(float);offset+=size
        observed=estimate_floor(source,profile,floor['roll_pitch'],height_delta_m=height_delta)
        distances=tree.query(source)[0]
        retain=distances>profile['raw_self_return_model']['radius_m']
        removed.append(int((~retain).sum()))
        removed_base=transform_points(source[~retain],translation,quaternion)
        far_removed+=int((np.hypot(removed_base[:,0],removed_base[:,1])>1.).sum())
        normal=rotation@observed.normal
        distance=observed.distance_m+normal@translation
        for use_model in (True,False):
            body=transform_points(source[retain] if use_model else source,translation,quaternion)
            filtered,_=filter_points_base_link(body,geometry)
            obstacles=filtered[(distance-filtered@normal)>
                               profile['floor_attitude_compensation']['minimum_obstacle_height_m']]
            clearance=directional_clearance(collision_obstacles(obstacles,geometry),geometry)
            fronts[use_model].append(float(clearance.front))
    if offset!=len(archive['points']) or len(fronts[True])!=93 or far_removed:
        raise ValueError('capture size/model consistency or far-return check failed')
    return dict(schema='go2w.post_motion_static_self_model_review.v1',
        source='parked_raw_capture_with_independent_floor_review',motion_sent=False,
        frames=93,max_joint_deviation_rad=float(deviations.max()),
        maximum_observed_static_deviation_rad=.09,
        floor_valid_frames=len(fronts[True]),
        model_removed_points_median=float(np.median(removed)),
        model_removed_points_beyond_one_meter=far_removed,
        front_clearance_with_model_min_m=float(min(fronts[True])),
        front_clearance_without_model_min_m=float(min(fronts[False])),
        input_sha256={'cloud.npz':file_hash(capture/'cloud.npz'),
                      'metadata.json':file_hash(capture/'metadata.json'),
                      'floor_review':file_hash(floor_review),
                      'surface_model':file_hash(model_path)},
        motion_posture_validated=False,authorizes_motion=False,
        limitation='Only the stopped 0.0818 rad pose is checked; the 0.292 rad moving pose needs synchronized cloud/joint evidence.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',required=True)
    parser.add_argument('--floor-review',required=True)
    parser.add_argument('--profile',default=str(ROOT/'configs/go2w/navigation_sensor_profile.yaml'))
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    result=review(args.capture,args.floor_review,args.profile)
    target=Path(args.output);target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':
    main()
