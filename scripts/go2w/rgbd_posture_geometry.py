"""Offline floor prediction from the recorded plane, never a ground-filter cutoff.

This reports model consistency only. It neither changes calibration nor grants
motion permission. Receipt-time pairing and the contact/model assumptions must
be reviewed separately before using a capture as calibration evidence.
"""
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/go2w_lidar_preprocessor'))
from go2w_lidar_preprocessor.floor_attitude import rpy_matrix, standing_height


def _vector3(value, name):
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.isfinite(array).all():
        raise ValueError('invalid '+name)
    return array


def predict_camera_floor(profile, *, urdf_bytes=None, joint_q=None, roll_pitch=None):
    """Project n.raw=distance through the recorded rigid sensor transforms.

    Optional posture uses the same pinned FK/attitude adjustment as the runtime
    independent floor checker. `preprocess.ground_height` is not used: it is an
    obstacle separation cutoff above the measured floor, not the floor itself.
    """
    raw = profile['raw_floor_plane']
    normal = _vector3(raw['normal'], 'raw plane normal')
    norm = float(np.linalg.norm(normal))
    distance = float(raw['distance_m'])
    if norm < 1e-9 or not np.isfinite(distance):
        raise ValueError('invalid raw floor plane')
    normal, distance = normal/norm, distance/norm
    lidar, camera = profile['frames']['base_to_cloud'], profile['frames']['base_to_camera']
    rl = rpy_matrix(*_vector3(lidar['rotation_rpy_rad'], 'LiDAR rotation'))
    tl = _vector3(lidar['translation_m'], 'LiDAR translation')
    rc = rpy_matrix(*_vector3(camera['rotation_rpy_rad'], 'camera rotation'))
    tc = _vector3(camera['translation_m'], 'camera translation')
    body_reference_normal = rl@normal
    body_reference_distance = distance + body_reference_normal@tl
    if body_reference_normal[2] > 0:
        body_reference_normal = -body_reference_normal
        body_reference_distance = -body_reference_distance
    if body_reference_distance <= 0:
        raise ValueError('recorded floor is not below the body origin')
    reference_rp = np.asarray(profile['posture']['body_roll_pitch_rad'], dtype=float)
    if reference_rp.shape != (2,) or not np.isfinite(reference_rp).all():
        raise ValueError('invalid reference body roll/pitch')
    rp = reference_rp
    body_normal = body_reference_normal
    height_delta = 0.
    reference_height = current_height = None
    if joint_q is not None or roll_pitch is not None:
        if joint_q is None or roll_pitch is None or urdf_bytes is None:
            raise ValueError('current posture requires joints, roll/pitch and pinned URDF together')
        rp = np.asarray(roll_pitch, dtype=float)
        reference_height = standing_height(urdf_bytes, profile['posture']['joint_q'], reference_rp)
        current_height = standing_height(urdf_bytes, joint_q, rp)
        height_delta = current_height-reference_height
        body_normal = rpy_matrix(*rp).T@rpy_matrix(*reference_rp)@body_reference_normal
    body_distance = body_reference_distance+height_delta
    camera_normal = rc.T@body_normal
    camera_distance = float(body_distance-body_normal@tc)
    world_normal = rpy_matrix(*rp)@body_normal
    if abs(body_normal[2]) < .5 or abs(world_normal[2]) < .5 or camera_distance <= 0:
        raise ValueError('plane does not establish vertical floor separation')
    return dict(
        scope='offline model prediction; not calibration acceptance',
        normal_camera=camera_normal.tolist(), distance_camera_to_plane_m=camera_distance,
        vertical_camera_to_plane_m=float(camera_distance/abs(world_normal[2])),
        reference_floor_z_at_body_xy0_m=float(body_reference_distance/body_reference_normal[2]),
        current_floor_z_at_body_xy0_m=float(body_distance/body_normal[2]),
        reference_fk_base_height_m=reference_height, current_fk_base_height_m=current_height,
        fk_height_delta_m=float(height_delta),
        kinematic_camera_height_m=(float(current_height+(rpy_matrix(*rp)@tc)[2])
                                   if current_height is not None else None),
        ground_filter_cutoff_not_used=True,
        posture_scope='provided posture; pairing not established here' if current_height is not None else 'recorded reference posture only',
        posture_time_not_validated=True,
        assumptions=['recorded sensor transforms', 'pinned URDF and joint zero/order',
                     'level floor and four-wheel contact for FK', 'nominal tire radius; compression uncalibrated'],
        calibration_written=False, calibration_accepted=False, authorizes_motion=False)
