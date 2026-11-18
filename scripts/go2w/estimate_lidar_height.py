#!/usr/bin/python3
"""Read-only posture-dependent height estimate from pinned URDF and joint data.

Assumes a level floor and all four wheels contacting it. This is a kinematic
estimate, not a physical calibration or permission to change safety geometry.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def axis_rotation(axis, angle):
    a = np.asarray(axis,dtype=float);a /= np.linalg.norm(a)
    x,y,z=a;c,s=math.cos(angle),math.sin(angle)
    k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return c*np.eye(3)+(1-c)*np.outer(a,a)+s*k


def rpy_rotation(rpy):
    r,p,y=rpy
    return (axis_rotation((0,0,1),y) @ axis_rotation((0,1,0),p)
            @ axis_rotation((1,0,0),r))


def estimate(urdf_bytes, posture):
    expected='f5940cbef9be28b76c075a94ed535c9c8c22c89a79b54ec8c854dc9f023c1ab8'
    digest=hashlib.sha256(urdf_bytes).hexdigest()
    if digest != expected:raise ValueError('URDF differs from the pinned official Go2-W model')
    model=ET.fromstring(urdf_bytes)
    joints={j.attrib['name']:j for j in model.findall('joint')}
    q=posture['joint_q'];body_rpy=posture['rpy']
    if len(q)<12 or len(body_rpy)!=3 or not all(math.isfinite(v) for v in (*q[:12],*body_rpy)):
        raise ValueError('finite joint and body attitude samples required')
    if any(abs(v)>math.radians(10) for v in body_rpy[:2]):
        raise ValueError('large body tilt: level four-wheel-contact assumption unsuitable')
    body=rpy_rotation(body_rpy)
    radius=.1778/2
    def origin(joint):
        e=joint.find('origin')
        xyz=[float(v) for v in e.attrib.get('xyz','0 0 0').split()]
        rpy=[float(v) for v in e.attrib.get('rpy','0 0 0').split()]
        t=np.eye(4);t[:3,:3]=rpy_rotation(rpy);t[:3,3]=xyz
        return t
    legs=[]
    for index,leg in enumerate(('FR','FL','RR','RL')):  # SDK motor ordering.
        t=np.eye(4)
        for offset,name in enumerate(('hip','thigh','calf')):
            joint=joints[f'{leg}_{name}_joint'];t=t@origin(joint)
            a=[float(v) for v in joint.find('axis').attrib['xyz'].split()]
            rot=np.eye(4);rot[:3,:3]=axis_rotation(a,q[index*3+offset]);t=t@rot
        t=t@origin(joints[f'{leg}_foot_joint'])
        axle=body@t[:3,3]
        axle_axis=body@t[:3,:3]@np.array([0,1,0])
        vertical_radius=radius*math.sqrt(max(0,1-float(axle_axis[2])**2))
        legs.append({'leg':leg,'joint_angles_deg':[math.degrees(v) for v in q[index*3:index*3+3]],
                     'axle_xyz_in_level_base_m':axle.tolist(),
                     'inferred_base_height_m':vertical_radius-float(axle[2])})
    heights=[v['inferred_base_height_m'] for v in legs]
    spread=max(heights)-min(heights)
    if spread>.04:raise ValueError('four leg heights inconsistent; wheel contact/model assumption not established')
    radar_origin=origin(joints['radar_joint'])[:3,3]
    radar_vertical_offset=float((body@radar_origin)[2])
    base_height=float(np.median(heights));lidar_height=base_height+radar_vertical_offset
    return {'scope':'kinematic estimate with side-photo posture crosscheck; not measured calibration',
        'urdf_sha256':digest,'posture_captured_at_unix':posture.get('captured_at_unix'),
        'wheel_nominal_radius_m':radius,'legs':legs,'four_leg_height_spread_m':spread,
        'base_height_m':base_height,'radar_vertical_offset_m':radar_vertical_offset,
        'lidar_center_height_m':lidar_height,
        'practical_estimate_interval_m':[lidar_height-.03,lidar_height+.03],
        'interval_kind':'engineering allowance, not statistical confidence interval',
        'assumptions':['level floor','all four wheels in contact','official link geometry and SDK joint zero/order',
                       'nominal 7-inch tire diameter; compression and wheel lateral offsets not calibrated'],
        'authorizes_motion':False,'writes_safety_configuration':False}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--urdf',required=True);p.add_argument('--posture-json',required=True);p.add_argument('--json',required=True)
    args=p.parse_args()
    result=estimate(Path(args.urdf).read_bytes(),json.loads(Path(args.posture_json).read_text()))
    Path(args.json).write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
