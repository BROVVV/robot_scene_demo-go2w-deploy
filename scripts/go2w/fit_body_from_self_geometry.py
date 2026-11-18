#!/usr/bin/python3
"""Refine body planar alignment against pinned wheel meshes and live joint FK.

Only previously cross-checked body-fixed points participate. Ground orientation
and height are preserved. This removes the guessed camera horizontal position.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from estimate_lidar_height import axis_rotation,rpy_rotation


def mesh(path):
    if Path(path).suffix == '.stl':
        data=Path(path).read_bytes()
        count=int.from_bytes(data[80:84], 'little')
        if len(data)!=84+50*count:raise ValueError('unexpected binary STL size')
        triangles=np.frombuffer(data, dtype=np.dtype([('normal','<f4',(3,)),('points','<f4',(3,3)),('attr','<u2')]), offset=84)
        return triangles['points'].reshape(-1,3).astype(float)
    root=ET.parse(path).getroot();ns={'c':root.tag.split('}')[0][1:]}
    unit=root.find('c:asset/c:unit',ns)
    if unit is None or float(unit.attrib['meter'])!=1:raise ValueError('unexpected mesh unit')
    geometries={}
    for g in root.findall('.//c:geometry',ns):
        positions=[s.find('c:float_array',ns) for s in g.findall('.//c:source',ns) if 'position' in s.attrib.get('id','').lower()]
        geometries[g.attrib['id']]=np.vstack([np.fromstring(a.text,sep=' ').reshape(-1,3) for a in positions])
    output=[]
    def visit(node,parent):
        T=parent.copy()
        for e in node:
            kind=e.tag.rsplit('}',1)[-1]
            if kind=='matrix':T=T@np.fromstring(e.text,sep=' ').reshape(4,4)
            elif kind in ('translate','rotate','scale'):raise ValueError('unsupported Collada scene transform')
        for instance in node.findall('c:instance_geometry',ns):
            points=geometries[instance.attrib['url'].lstrip('#')]
            output.append(points@T[:3,:3].T+T[:3,3])
        for child in node.findall('c:node',ns):visit(child,T)
    for scene in root.findall('.//c:visual_scene',ns):
        for node in scene.findall('c:node',ns):visit(node,np.eye(4))
    if not output:raise ValueError('missing mesh scene geometry')
    return np.vstack(output)


def origin(element):
    e=element.find('origin');t=np.eye(4)
    if e is not None:
        t[:3,3]=np.fromstring(e.attrib.get('xyz','0 0 0'),sep=' ')
        t[:3,:3]=rpy_rotation(np.fromstring(e.attrib.get('rpy','0 0 0'),sep=' '))
    return t


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--surface-model',required=True);p.add_argument('--body-transform',required=True)
    p.add_argument('--posture',required=True);p.add_argument('--urdf',required=True)
    p.add_argument('--left-wheel',required=True);p.add_argument('--right-wheel',required=True)
    p.add_argument('--leg-mesh-dir')
    p.add_argument('--json',required=True);args=p.parse_args()
    profile=json.loads(Path(args.body_transform).read_text());posture=json.loads(Path(args.posture).read_text())
    model=ET.parse(args.urdf).getroot();joints={j.attrib['name']:j for j in model.findall('joint')}
    q=posture['joint_q'];targets=[];axles=[]
    for i,leg in enumerate(('FR','FL','RR','RL')):
        T=np.eye(4)
        for k,name in enumerate(('hip','thigh','calf')):
            j=joints[f'{leg}_{name}_joint'];T=T@origin(j)
            rot=np.eye(4);rot[:3,:3]=axis_rotation(np.fromstring(j.find('axis').attrib['xyz'],sep=' '),q[i*3+k]);T=T@rot
            if args.leg_mesh_dir:
                visual=model.find(f"link[@name='{leg}_{name}']/visual")
                filename=visual.find('geometry/mesh').attrib['filename'].rsplit('/',1)[-1]
                v=mesh(Path(args.leg_mesh_dir)/filename);V=T@origin(visual)
                targets.append(v@V[:3,:3].T+V[:3,3])
        T=T@origin(joints[f'{leg}_foot_joint']);axles.append(T[:3,3].tolist())
        v=mesh(args.left_wheel if leg.endswith('L') else args.right_wheel)
        targets.append(v@T[:3,:3].T+T[:3,3])
    target=np.vstack(targets)
    _,index=np.unique(np.floor(target/.004).astype(int),axis=0,return_index=True)
    target=target[index];tree=cKDTree(target)
    raw=np.load(args.surface_model)['points'];Rb=Rotation.from_euler('xyz',profile['rotation_rpy_rad']).as_matrix();tb=np.array(profile['translation_m'])
    source=raw@Rb.T+tb
    # The observed floor establishes gravity. Rotation about that axis and
    # in-plane translations preserve the floor constraint exactly.
    u=Rotation.from_euler('xyz',[*posture['rpy'][:2],0]).as_matrix().T@np.array([0.,0.,1.])
    fx=np.array([1.,0.,0.])-u*u[0];fx/=np.linalg.norm(fx);fy=np.cross(u,fx)
    def project(params):
        R=Rotation.from_rotvec(u*params[0]).as_matrix();t=fx*params[1]+fy*params[2]
        return source@R.T+t,R,t
    def residual(params):
        transformed,_,_=project(params);distance,index=tree.query(transformed)
        # Far multipath/chassis points are not a wheel surface. Capping their
        # residual prevents them dragging an otherwise supported wheel fit.
        return np.minimum(distance,.12)
    fits=[]
    for x in (.0,.10,.20,.30):
        for yaw in (-.08,.08):
            result=least_squares(residual,[yaw,x,0.],bounds=([-.3,-.10,-.12],[.3,.40,.12]),
                loss='soft_l1',f_scale=.015,max_nfev=100,diff_step=1e-4)
            points,R,t=project(result.x);d=tree.query(points)[0]
            fits.append((float(np.median(d)),float((d<.03).mean()),result,R,t,d))
    best=min(fits,key=lambda f:(-f[1],f[0]));_,ratio,opt,R,t,d=best
    newR=R@Rb;newt=R@tb+t
    output=dict(scope='body alignment candidate from body-fixed points and wheel CAD; not motion acceptance',
        rotation_rpy_rad=Rotation.from_matrix(newR).as_euler('xyz').tolist(),translation_m=newt.tolist(),
        planar_adjustment=opt.x.tolist(),wheel_axles_body_m=axles,
        surface_distance_quantiles_m=np.quantile(d,[.25,.5,.75,.9]).tolist(),within_3cm_ratio=ratio,
        fits=[dict(parameters=f[2].x.tolist(),median_distance_m=f[0],within_3cm_ratio=f[1],optimality=f[2].optimality) for f in fits],
        authorizes_motion=False)
    Path(args.json).write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(output))


if __name__=='__main__':main()
