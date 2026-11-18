"""Batched FK must match the independent homogeneous-matrix reference."""
from functools import lru_cache
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from go2w_lidar_preprocessor.floor_attitude import standing_height, rpy_matrix

ROOT=Path(__file__).resolve().parents[4]
URDF=(ROOT/'configs/go2w/calibration_20261019/official_go2w.urdf').read_bytes()
PROFILE=yaml.safe_load((ROOT/'configs/go2w/navigation_sensor_profile.yaml').read_text())
Q=np.asarray(PROFILE['posture']['joint_q']);RP=PROFILE['posture']['body_roll_pitch_rad']

@lru_cache(maxsize=1)
def reference_chains():
    joints={j.attrib['name']:j for j in ET.fromstring(URDF).findall('joint')}
    result=[]
    for leg in ('FR','FL','RR','RL'):
        chain=[]
        for name in ('hip','thigh','calf','foot'):
            j=joints[f'{leg}_{name}_joint'];o=j.find('origin');t=np.eye(4)
            t[:3,3]=np.fromstring(o.attrib.get('xyz','0 0 0'),sep=' ')
            t[:3,:3]=rpy_matrix(*np.fromstring(o.attrib.get('rpy','0 0 0'),sep=' '))
            axis=None if name=='foot' else np.fromstring(j.find('axis').attrib['xyz'],sep=' ')
            chain.append((t,axis))
        result.append(chain)
    return result


def reference(q,rp,*,return_spread=False):
    # Preserve the old 4x4 algorithm rather than reconstructing the batched one.
    body=rpy_matrix(*rp);heights=[]
    for leg,chain in enumerate(reference_chains()):
        transform=np.eye(4)
        for offset,(origin,axis) in enumerate(chain):
            transform=transform@origin
            if axis is not None:
                axis=axis/np.linalg.norm(axis);x,y,z=axis;angle=q[leg*3+offset]
                skew=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
                turn=np.eye(4)
                turn[:3,:3]=math.cos(angle)*np.eye(3)+(1-math.cos(angle))*np.outer(axis,axis)+math.sin(angle)*skew
                transform=transform@turn
        axle=body@transform[:3,3];axis=body@transform[:3,:3]@np.array([0.,1.,0.])
        heights.append(.0889*math.sqrt(max(0.,1-axis[2]**2))-axle[2])
    spread=max(heights)-min(heights)
    if return_spread:return spread
    if spread>.04:raise ValueError('four-wheel contact assumption inconsistent')
    return float(np.median(heights))


def test_many_live_postures_match_reference_and_contact_rejections():
    rng=np.random.default_rng(9441);accepted=rejected=0
    for scale,count in ((.004,250),(.025,250),(.10,250)):
        for _ in range(count):
            q=Q+rng.normal(0,scale,12);rp=np.asarray(RP)+rng.uniform(-.04,.04,2)
            try:expected=reference(q,rp)
            except ValueError:
                rejected+=1
                with pytest.raises(ValueError,match='contact assumption'):standing_height(URDF,q,rp)
            else:
                accepted+=1
                assert standing_height(URDF,q,rp)==pytest.approx(expected,abs=2e-15,rel=0)
    assert accepted>250 and rejected>50


@pytest.mark.parametrize('sign',[-1,1])
def test_contact_threshold_still_rejects_outside_and_accepts_inside(sign):
    # Locate the 4cm boundary using the independent reference, then test
    # micrometre-separated sides without changing the production threshold.
    low=0.;high=.7
    bound=Q.copy();bound[1]+=sign*high
    assert reference(bound,RP,return_spread=True)>.04
    for _ in range(45):
        mid=(low+high)/2;q=Q.copy();q[1]+=sign*mid
        if reference(q,RP,return_spread=True)>.04:high=mid
        else:low=mid
    for delta in (-1e-5,1e-5):
        q=Q.copy();q[1]+=sign*((low+high)/2+delta)
        if delta>0:
            with pytest.raises(ValueError,match='contact assumption'):standing_height(URDF,q,RP)
        else:assert standing_height(URDF,q,RP)==pytest.approx(reference(q,RP),abs=2e-15,rel=0)


def test_geometry_cache_does_not_cache_measured_joints():
    first=standing_height(URDF,Q,RP);q=Q.copy();q[1::3]+=.01
    second=standing_height(URDF,q,RP)
    assert abs(first-second)>1e-6
    assert standing_height(URDF,Q,RP)==first
    assert second==pytest.approx(reference(q,RP),abs=2e-15,rel=0)
