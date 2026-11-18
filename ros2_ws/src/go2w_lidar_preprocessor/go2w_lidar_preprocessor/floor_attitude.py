"""IMU-predicted floor with an independent per-cloud plane check.

This estimates geometry only. Invalid/poorly observed floors fail closed; no
map occupancy, clearance rule or self-return surface is changed here.
"""
from dataclasses import dataclass
from functools import lru_cache
import hashlib
import math
import xml.etree.ElementTree as ET
import numpy as np


def rpy_matrix(roll, pitch, yaw=0.):
    cr,sr=math.cos(roll),math.sin(roll);cp,sp=math.cos(pitch),math.sin(pitch);cy,sy=math.cos(yaw),math.sin(yaw)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


@dataclass(frozen=True)
class FloorEstimate:
    normal: np.ndarray
    distance_m: float
    points: int
    angle_residual_rad: float
    height_residual_m: float


@lru_cache(maxsize=1)
def _leg_chains(urdf_bytes):
    if hashlib.sha256(urdf_bytes).hexdigest() != 'f5940cbef9be28b76c075a94ed535c9c8c22c89a79b54ec8c854dc9f023c1ab8':
        raise ValueError('unexpected official Go2-W geometry')
    joints={j.attrib['name']:j for j in ET.fromstring(urdf_bytes).findall('joint')}
    chains=[]
    for leg in ('FR','FL','RR','RL'):
        chain=[]
        for name in ('hip','thigh','calf','foot'):
            j=joints[f'{leg}_{name}_joint'];o=j.find('origin');T=np.eye(4)
            T[:3,3]=np.fromstring(o.attrib.get('xyz','0 0 0'),sep=' ')
            T[:3,:3]=rpy_matrix(*np.fromstring(o.attrib.get('rpy','0 0 0'),sep=' '))
            axis=None if name=='foot' else np.fromstring(j.find('axis').attrib['xyz'],sep=' ')
            chain.append((T,axis))
        chains.append(chain)
    return chains


@lru_cache(maxsize=1)
def _leg_joint_limits(urdf_bytes):
    # Share the same pinned manufacturer geometry as the FK height estimate.
    _leg_chains(urdf_bytes)
    joints={j.attrib['name']:j for j in ET.fromstring(urdf_bytes).findall('joint')}
    limits=[]
    for leg in ('FR','FL','RR','RL'):
        for name in ('hip','thigh','calf'):
            limit=joints[f'{leg}_{name}_joint'].find('limit')
            if limit is None:
                raise ValueError('official joint limit missing')
            lower,upper=float(limit.attrib['lower']),float(limit.attrib['upper'])
            if not math.isfinite(lower) or not math.isfinite(upper) or lower>=upper:
                raise ValueError('invalid official joint limit')
            limits.append((lower,upper))
    return np.asarray(limits,dtype=float)


def joint_angles_within_limits(urdf_bytes, joint_q):
    q=np.asarray(joint_q,dtype=float)
    if q.shape!=(12,) or not np.isfinite(q).all():
        return False
    limits=_leg_joint_limits(urdf_bytes)
    return bool(np.all((q>=limits[:,0])&(q<=limits[:,1])))


@lru_cache(maxsize=1)
def _standing_chain_terms(urdf_bytes):
    """Cache only immutable manufacturer geometry, never measured posture."""
    chains=_leg_chains(urdf_bytes)
    rotations=np.asarray([[origin[:3,:3] for origin,axis in chain] for chain in chains])
    translations=np.asarray([[origin[:3,3] for origin,axis in chain] for chain in chains])
    skew=np.zeros((3,4,3,3));outer=np.zeros((3,4,3,3))
    for leg,chain in enumerate(chains):
        for offset,(_,axis) in enumerate(chain[:3]):
            axis=axis/np.linalg.norm(axis);x,y,z=axis
            skew[offset,leg]=[[0,-z,y],[z,0,-x],[-y,x,0]]
            outer[offset,leg]=np.outer(axis,axis)
    for value in (rotations,translations,skew,outer):value.flags.writeable=False
    return rotations,translations,skew,outer


def standing_height(urdf_bytes, joint_q, roll_pitch):
    q=np.asarray(joint_q,dtype=float);rp=np.asarray(roll_pitch,dtype=float)
    if (q.shape!=(12,) or rp.shape!=(2,) or not np.isfinite(q).all()
            or not np.isfinite(rp).all() or max(abs(rp))>math.radians(10)
            or not joint_angles_within_limits(urdf_bytes,q)):
        raise ValueError('invalid standing kinematics')
    origins,translations,skew,outer=_standing_chain_terms(urdf_bytes)
    # Evaluate all four unchanged URDF chains together. Separate position and
    # rotation avoid allocating sixteen homogeneous matrices per state frame.
    rotation=np.broadcast_to(np.eye(3),(4,3,3)).copy()
    position=np.zeros((4,3));joints=q.reshape(4,3);identity=np.eye(3)
    for offset in range(4):
        position+=(rotation@translations[:,offset,:,None])[:,:,0]
        rotation=rotation@origins[:,offset]
        if offset<3:
            # Match the scalar reference's libm trig inputs/results.
            cosine=np.array([math.cos(v) for v in joints[:,offset]])[:,None,None]
            sine=np.array([math.sin(v) for v in joints[:,offset]])[:,None,None]
            turn=cosine*identity+(1-cosine)*outer[offset]+sine*skew[offset]
            rotation=rotation@turn
    body=rpy_matrix(*rp)
    axles=position@body.T;axes=rotation[:,:,1]@body.T
    heights=.0889*np.sqrt(np.maximum(0.,1-axes[:,2]**2))-axles[:,2]
    if max(heights)-min(heights)>.04:
        raise ValueError('four-wheel contact assumption inconsistent')
    return float(np.median(heights))


def estimate_floor(points, profile, roll_pitch, *, height_delta_m=0.):
    """Validate a floor near the IMU prediction, retaining original tolerances.

    Rotation is about the body origin. Height remains independently checked
    against the calibrated plane; a changed leg height is NOT silently accepted.
    """
    values=np.asarray(points,dtype=float);rp=np.asarray(roll_pitch,dtype=float)
    if (values.ndim!=2 or values.shape[1]!=3 or rp.shape!=(2,)
            or not np.isfinite(rp).all() or np.max(abs(rp))>math.radians(10)
            or not math.isfinite(height_delta_m)):
        raise ValueError('invalid floor input or excessive body tilt')
    plane=profile['raw_floor_plane'];extr=profile['frames']['base_to_cloud'];posture=profile['posture']
    R=rpy_matrix(*extr['rotation_rpy_rad']);t=np.asarray(extr['translation_m'])
    ref=np.asarray(plane['normal']);base_ref=R@ref
    body_to_level_ref=rpy_matrix(*posture['body_roll_pitch_rad'])
    body_to_level_current=rpy_matrix(*rp)
    normal=R.T@body_to_level_current.T@body_to_level_ref@base_ref
    normal/=np.linalg.norm(normal)
    distance=float(plane['distance_m']+base_ref@t-(R@normal)@t+height_delta_m)
    base=values@R.T+t
    radial=np.linalg.norm(base[:,:2],axis=1)
    tolerance=float(plane['tolerance_m'])
    choose=np.isfinite(values).all(axis=1)&(radial>=.55)&(radial<=3.)&(abs(values@normal-distance)<=2*tolerance)
    candidate=values[choose]
    if len(candidate)<80:raise ValueError('floor insufficiently observed')
    for _ in range(3):
        center=candidate.mean(axis=0);cov=(candidate-center).T@(candidate-center)/len(candidate)
        eigen,vectors=np.linalg.eigh(cov);fitted=vectors[:,0]
        if fitted@normal<0:fitted=-fitted
        fitted_distance=float(fitted@center)
        inlier=abs(candidate@fitted-fitted_distance)<=tolerance
        if inlier.all():break
        candidate=candidate[inlier]
        if len(candidate)<80:raise ValueError('floor lost support')
    # A line or compact patch cannot establish a floor plane.
    if eigen[1]<.02:raise ValueError('floor lacks two-dimensional support')
    angle=math.acos(float(np.clip(fitted@normal,-1,1)))
    height=fitted_distance-distance
    if angle>posture['maximum_attitude_deviation_rad'] or abs(height)>tolerance:
        raise ValueError('observed floor disagrees with IMU or calibrated height')
    return FloorEstimate(fitted,fitted_distance,len(candidate),angle,height)
