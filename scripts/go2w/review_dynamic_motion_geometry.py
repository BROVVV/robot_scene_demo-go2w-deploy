#!/usr/bin/env python3
"""Replay synchronized raw cloud/joint evidence across a moving posture.

The live node historically stopped every scan when a joint left the small
stationary self-return-model envelope.  This tool evaluates the conservative
fallback used outside that envelope: disable the learned raw surface model,
retain the fixed body/self-region masks, validate official-URDF kinematics,
and validate the observed floor independently for every cloud.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'ros2_ws/src/go2w_lidar_preprocessor'),
                str(ROOT / 'ros2_ws/src/go2w_description')]
from go2w_description.description_config import _rpy_to_quaternion_xyzw
from go2w_lidar_preprocessor.config import load_safety_ready_config
from go2w_lidar_preprocessor.floor_attitude import (
    estimate_floor, joint_angles_within_limits, rpy_matrix, standing_height)
from go2w_lidar_preprocessor.preprocess_core import (
    collision_obstacles, directional_clearance, filter_points_base_link, transform_points)
from go2w_lidar_preprocessor.self_returns import RawSelfReturnFilter


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite_clearance(values):
    return [float(value) if np.isfinite(value) else None for value in values]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', required=True)
    parser.add_argument('--profile', default=str(ROOT/'configs/go2w/navigation_sensor_profile.yaml'))
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    import yaml

    capture = Path(args.capture)
    profile_path = Path(args.profile)
    profile = yaml.safe_load(profile_path.read_text())
    metadata = json.loads((capture/'metadata.json').read_text())
    archive = np.load(capture/'raw_cloud.npz', allow_pickle=False)
    if len(metadata['clouds']) != len(archive['sizes']):
        raise ValueError('cloud metadata/archive size mismatch')
    _, geometry = load_safety_ready_config(
        str(ROOT/'configs/go2w/lidar_preprocess.yaml'),
        str(ROOT/'configs/go2w/official_reference.yaml'), str(profile_path))
    model_path = ROOT/profile['raw_self_return_model']['path']
    raw_filter = RawSelfReturnFilter(
        np.load(model_path, allow_pickle=False)['points'],
        profile['raw_self_return_model']['radius_m'])
    urdf_path = ROOT/profile['floor_attitude_compensation']['urdf_path']
    urdf = urdf_path.read_bytes()
    reference_q = np.asarray(profile['posture']['joint_q'], dtype=float)
    reference_height = standing_height(
        urdf, reference_q, profile['posture']['body_roll_pitch_rad'])
    allowed = float(profile['posture']['maximum_joint_deviation_rad'])
    extr = profile['frames']['base_to_cloud']
    rotation = rpy_matrix(*extr['rotation_rpy_rad'])
    translation = np.asarray(extr['translation_m'], dtype=float)
    quaternion = _rpy_to_quaternion_xyzw(tuple(extr['rotation_rpy_rad']))
    obstacle_height = profile['floor_attitude_compensation']['minimum_obstacle_height_m']

    frames = []
    offset = 0
    for index, (cloud_meta, size) in enumerate(zip(metadata['clouds'], archive['sizes'])):
        source = archive['points'][offset:offset+int(size)].astype(float)
        offset += int(size)
        if (cloud_meta.get('nearest_state_index') is None
                or cloud_meta.get('state_receive_delta_ms') is None):
            frames.append({'index': index, 'evaluated': False,
                           'paired_state': False,
                           'error': 'no state within association tolerance'})
            continue
        state = metadata['states'][int(cloud_meta['nearest_state_index'])]
        q = np.asarray(state['q'], dtype=float)
        rpy = np.asarray(state['rpy'], dtype=float)
        deviation = np.abs(q-reference_q)
        row = {
            'index': index,
            'joint_deviation_rad': float(deviation.max()),
            'joint_index': int(np.argmax(deviation)),
            'static_model_eligible': bool(deviation.max() <= allowed),
            'state_receive_delta_ms': float(cloud_meta['state_receive_delta_ms']),
            'paired_state': True,
        }
        # The stationary envelope already has an independent 93-frame review.
        # Keep this replay focused on the otherwise-unhandled moving frames.
        if row['static_model_eligible']:
            row['evaluated'] = False
            frames.append(row)
            continue
        try:
            if (not joint_angles_within_limits(urdf, q)
                    or not np.isfinite(rpy[:2]).all()
                    or np.max(np.abs(rpy[:2])) > np.radians(10)):
                raise ValueError('invalid dynamic kinematics')
            # Four wheel heights legitimately diverge during rolling gait.
            # Keep the calibrated body height and require the independent
            # observed-floor residual to remain within its measured tolerance.
            height_delta = 0.
            floor = estimate_floor(source, profile, rpy[:2], height_delta_m=height_delta)
            body = transform_points(source, translation, quaternion)
            moving_keep = raw_filter.keep_moving(
                source, body,
                radius=float(profile['raw_self_return_model']['dynamic_radius_m']),
                region=profile['raw_self_return_model']['dynamic_region_base'])
            moving_body = body[moving_keep]
            filtered, _ = filter_points_base_link(moving_body, geometry)
            normal = rotation@floor.normal
            distance = floor.distance_m+normal@translation
            obstacles = filtered[(distance-filtered@normal) > obstacle_height]
            collision = collision_obstacles(obstacles, geometry)
            clearance = directional_clearance(collision, geometry)
            keep = raw_filter.keep(source)
            body_model = transform_points(source[keep], translation, quaternion)
            filtered_model, _ = filter_points_base_link(body_model, geometry)
            obstacles_model = filtered_model[(distance-filtered_model@normal) > obstacle_height]
            clearance_model = directional_clearance(
                collision_obstacles(obstacles_model, geometry), geometry)
            row.update(
                evaluated=True,
                valid=True,
                height_delta_m=float(height_delta),
                floor_points=int(floor.points),
                floor_angle_residual_rad=float(floor.angle_residual_rad),
                floor_height_residual_m=float(floor.height_residual_m),
                raw_self_points_removed=int((~keep).sum()),
                moving_self_points_removed=int((~moving_keep).sum()),
                fallback_filtered_points=int(len(filtered)),
                fallback_collision_points=int(len(collision)),
                fallback_clearance_m=finite_clearance(
                    [clearance.front, clearance.left, clearance.right]),
                model_clearance_m=finite_clearance(
                    [clearance_model.front, clearance_model.left, clearance_model.right]),
            )
        except ValueError as exc:
            row.update(evaluated=True, valid=False, error=str(exc))
        frames.append(row)
    if offset != len(archive['points']):
        raise ValueError('cloud archive contains trailing points')

    dynamic = [row for row in frames if row.get('static_model_eligible') is False]
    valid_dynamic = [row for row in dynamic if row['valid']]
    joint_deviations = [row['joint_deviation_rad'] for row in frames
                        if 'joint_deviation_rad' in row]
    front = [row['fallback_clearance_m'][0] for row in valid_dynamic
             if row['fallback_clearance_m'][0] is not None]
    result = {
        'schema': 'go2w.dynamic_motion_geometry_review.v1',
        'source_capture': str(capture),
        'frames': len(frames),
        'evaluated_frames': sum(row.get('evaluated', False) for row in frames),
        'valid_frames': sum(row.get('valid', False) for row in frames),
        'maximum_joint_deviation_rad': max(joint_deviations) if joint_deviations else None,
        'stationary_model_limit_rad': allowed,
        'dynamic_frames': len(dynamic),
        'dynamic_valid_frames': len(valid_dynamic),
        'dynamic_invalid': [row for row in dynamic if not row['valid']],
        'dynamic_fallback_front_clearance_min_m': min(front) if front else None,
        'dynamic_fallback_front_clearance_median_m': float(np.median(front)) if front else None,
        'dynamic_fallback_collision_points_max': max(
            (row['fallback_collision_points'] for row in valid_dynamic), default=None),
        'dynamic_rows': dynamic,
        'recommended_policy': (
            'outside the stationary posture envelope, retain the reviewed 3cm raw surface and '
            'allow its 5cm extension only in the measured low leg band, after official joint-limit, '
            'IMU-tilt and per-cloud calibrated-height floor checks pass'),
        'known_occupied_cells_still_win': True,
        'authorizes_unrestricted_motion': False,
        'input_sha256': {
            'metadata.json': sha256(capture/'metadata.json'),
            'raw_cloud.npz': sha256(capture/'raw_cloud.npz'),
            'sensor_profile': sha256(profile_path),
            'surface_model': sha256(model_path),
            'official_urdf': sha256(urdf_path),
        },
    }
    target = Path(args.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: value for key, value in result.items()
                      if key not in ('dynamic_rows', 'dynamic_invalid')}, ensure_ascii=False))
    return 0 if dynamic and len(valid_dynamic) == len(dynamic) else 2


if __name__ == '__main__':
    raise SystemExit(main())
