#!/usr/bin/python3
"""Offline camera/cloud refinement with independently observed floor planes.

No ROS publisher, no configuration writes, and no motion authorization.
The floor adds the tilt/height constraints absent from box-only ICP.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from fit_rgbd_lidar_geometry import load_capture, matches, metrics


def plane(points, normal_hint, *, threshold=.02, iterations=500):
    points=np.asarray(points,dtype=float)
    if len(points)<100:
        raise ValueError('insufficient floor candidate points')
    hint=np.asarray(normal_hint,dtype=float);hint/=np.linalg.norm(hint)
    rng=np.random.default_rng(20261015);best=None;best_count=0
    for _ in range(iterations):
        sample=points[rng.choice(len(points),3,replace=False)]
        normal=np.cross(sample[1]-sample[0],sample[2]-sample[0])
        length=np.linalg.norm(normal)
        if length<.01:
            continue
        normal/=length
        if normal@hint<0:normal=-normal
        if normal@hint<.97:continue
        distance=float(normal@sample[0])
        good=abs(points@normal-distance)<threshold
        if good.sum()>best_count:
            best,best_count=good,int(good.sum())
    if best is None or best_count<100:
        raise ValueError('no supported plane near the floor orientation')
    for _ in range(3):
        selected=points[best];center=selected.mean(0)
        _,_,v=np.linalg.svd(selected-center,full_matrices=False)
        normal=v[-1]
        if normal@hint<0:normal=-normal
        distance=float(normal@center)
        best=abs(points@normal-distance)<threshold
    return {'normal':normal.tolist(),'distance_m':distance,'inliers':int(best.sum()),
            'candidates':len(points),'rmse_m':float(np.sqrt(np.mean((points[best]@normal-distance)**2)))}


def floors(capture, proxy):
    from PIL import Image
    data=load_capture(capture)
    source=data[0]
    Rb=Rotation.from_euler('xyz',proxy['rotation_rpy_rad']).as_matrix()
    tb=np.asarray(proxy['translation_m'])
    body=source@Rb.T+tb
    select=(np.linalg.norm(body[:,:2],axis=1)>.7)&(np.linalg.norm(body[:,:2],axis=1)<4.)
    select&=(body[:,2]>-.7)&(body[:,2]<-.25)
    raw_floor=plane(source[select],Rb.T@np.array([0.,0.,-1.]))
    meta=data[4]
    depth=np.asarray(Image.open(meta['depth_ref']),dtype=float)*meta['depth_unit_m']
    v,u=np.indices(depth.shape)
    cloud=np.stack(((u-meta['cx'])*depth/meta['fx'],(v-meta['cy'])*depth/meta['fy'],depth),axis=-1)
    hint=np.array([-.02,.9985,.05])
    # A height strip computed using the OLD optical tilt can exclude the
    # entire real floor after a small camera adjustment, then fit box/noise
    # points instead. Estimate the plane first, validate height/tilt/support
    # afterwards. The lower image ROI still excludes most furniture tops.
    # Match the 4 m support range used by the independent LiDAR floor fit.
    mask=(v>depth.shape[0]*.55)&(u>depth.shape[1]*.30)&(u<depth.shape[1]*.70)&(depth>.3)&(depth<4.)
    points=cloud[mask]
    # Voxel-equal weighting can amplify dispersed stereo outliers while
    # discarding the dense, consistent floor pixels. Keep pixel support for
    # this image-plane estimate, with a bounded deterministic random sample.
    if len(points)>12000:
        points=points[np.random.default_rng(20261016).choice(len(points),12000,replace=False)]
    camera_floor=plane(points,hint,threshold=.012)
    normal=np.asarray(camera_floor['normal'])
    if (normal@hint/np.linalg.norm(hint)<.97
            or not .3<=camera_floor['distance_m']<=.75
            or camera_floor['inliers']/camera_floor['candidates']<.3):
        raise ValueError('camera floor is not supported at a plausible height/tilt')
    return data,raw_floor,camera_floor


def skew(v):
    x,y,z=v
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])


def floor_error(R,t,raw,camera):
    nr=np.asarray(raw['normal']);nc=np.asarray(camera['normal'])
    return dict(angle_deg=float(np.rad2deg(np.arccos(np.clip((R@nr)@nc,-1,1)))),
        distance_m=float(nc@t+raw['distance_m']-camera['distance_m']))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--fit',required=True);p.add_argument('--validate',required=True,nargs='+')
    p.add_argument('--initial',required=True);p.add_argument('--body-proxy',required=True)
    p.add_argument('--json',required=True)
    args=p.parse_args()
    initial=json.loads(Path(args.initial).read_text())['candidate']
    proxy=json.loads(Path(args.body_proxy).read_text())
    R=Rotation.from_euler('xyz',initial['rotation_rpy_rad']).as_matrix();t=np.array(initial['translation_m'])
    fit,raw,camera=floors(Path(args.fit),proxy)
    nr,nc=np.array(raw['normal']),np.array(camera['normal'])
    result=dict(scope='offline refinement with independent floor constraints; not navigation acceptance',
        fit_capture=args.fit,raw_floor=raw,camera_floor=camera,
        initial_floor_error=floor_error(R,t,raw,camera))
    history=[]
    for iteration in range(80):
        x,y,n,_=matches(fit,R,t,.3 if iteration<15 else .2)
        residual=np.einsum('ij,ij->i',x-y,n)
        good=abs(residual)<.12;x,y,n,residual=x[good],y[good],n[good],residual[good]
        if len(x)<300:raise ValueError('insufficient common surfaces')
        A=np.column_stack((np.cross(x,n),n))
        weight=np.minimum(1.,.025/np.maximum(abs(residual),1e-9))
        Aw=A*np.sqrt(weight[:,None]);bw=-residual*np.sqrt(weight)
        # A metre lever arm makes a radian of tilt comparable to metres.
        floor_A=np.vstack((np.column_stack((-skew(R@nr),np.zeros((3,3)))),
                           np.r_[np.cross(t,nc),nc]))
        floor_b=-np.r_[R@nr-nc,nc@t+raw['distance_m']-camera['distance_m']]
        scale=np.sqrt(10000.)
        step,_,_,sv=np.linalg.lstsq(np.vstack((Aw,floor_A*scale)),
                                  np.r_[bw,floor_b*scale],rcond=None)
        step/=max(np.linalg.norm(step[:3])/np.deg2rad(2),np.linalg.norm(step[3:])/.025,1)
        dR=Rotation.from_rotvec(step[:3]).as_matrix();R=dR@R;t=dR@t+step[3:]
        history.append(dict(iteration=iteration,step_norm=float(np.linalg.norm(step)),
            floor_error=floor_error(R,t,raw,camera),singular_values=sv.tolist()))
        if np.linalg.norm(step)<1e-6:break
    result.update(candidate=dict(rotation_rpy_rad=Rotation.from_matrix(R).as_euler('xyz').tolist(),
        translation_m=t.tolist(),fit=metrics(fit,R,t),floor_error=floor_error(R,t,raw,camera)),
        held_out={},history=history,confirmed=False,navigation_geometry_validated=False,authorizes_motion=False)
    for path in args.validate:
        data,rf,cf=floors(Path(path),proxy)
        result['held_out'][path]=dict(metrics=metrics(data,R,t),raw_floor=rf,camera_floor=cf,
                                      floor_error=floor_error(R,t,rf,cf))
    Path(args.json).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='history'}))


if __name__=='__main__':main()
