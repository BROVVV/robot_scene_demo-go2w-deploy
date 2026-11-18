#!/usr/bin/python3
"""Offline diagnostic ICP candidate; never writes calibration or live TF.

Fit a captured LiDAR cloud to aligned RGB-D surfaces, then validate on a
separate capture. Neither statistical fit nor this report authorizes motion.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import yaml


def load_capture(path):
    meta=json.loads((path/'rgbd_materialized.json').read_text())
    depth=np.asarray(Image.open(meta['depth_ref']),dtype=float)*meta['depth_unit_m']
    v,u=np.indices(depth.shape);xyz=np.stack(((u-meta['cx'])*depth/meta['fx'],
        (v-meta['cy'])*depth/meta['fy'],depth),axis=-1)
    horizontal=xyz[2:-2,4:]-xyz[2:-2,:-4]
    vertical=xyz[4:,2:-2]-xyz[:-4,2:-2]
    normal=np.cross(horizontal,vertical);norm=np.linalg.norm(normal,axis=-1)
    normal/=np.maximum(norm[...,None],1e-12)
    z=depth[2:-2,2:-2];uu=u[2:-2,2:-2];vv=v[2:-2,2:-2]
    valid=(z>.3)&(z<5.5)&(uu>350)&(uu<1150)&(vv>40)&(vv<695)&(norm>1e-6)
    # Reject depth discontinuities and grazing/unstable normals.
    valid&=(np.linalg.norm(horizontal,axis=-1)<.10)&(np.linalg.norm(vertical,axis=-1)<.10)
    target=xyz[2:-2,2:-2][valid];normals=normal[valid]
    _,index=np.unique(np.floor(target/.025).astype(int),axis=0,return_index=True)
    target,normals=target[index],normals[index]
    source=np.load(path/'cloud.npz')['points'];source=source[np.isfinite(source).all(axis=1)]
    _,index=np.unique(np.floor(source/.025).astype(int),axis=0,return_index=True)
    return source[index],target,normals,cKDTree(target),meta


def matches(data,R,t,maximum_distance):
    source,target,normals,tree,meta=data
    transformed=source@R.T+t
    valid=(transformed[:,2]>.3)&(transformed[:,2]<5.5)
    transformed=transformed[valid]
    u=meta['fx']*transformed[:,0]/transformed[:,2]+meta['cx']
    v=meta['fy']*transformed[:,1]/transformed[:,2]+meta['cy']
    transformed=transformed[(u>350)&(u<1150)&(v>40)&(v<695)]
    distance,index=tree.query(transformed)
    keep=distance<maximum_distance
    return transformed[keep],target[index[keep]],normals[index[keep]],distance[keep]


def metrics(data,R,t):
    x,y,n,d=matches(data,R,t,.5)
    residual=np.einsum('ij,ij->i',x-y,n)
    return {'points_within_50cm':len(d),
        'nearest_distance_quantiles_m':np.quantile(d,[.25,.5,.75,.9]).tolist(),
        'plane_distance_quantiles_m':np.quantile(abs(residual),[.25,.5,.75,.9]).tolist(),
        'nearest_within_5cm_ratio':float((d<.05).mean())}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--fit',required=True);p.add_argument('--validate',required=True)
    p.add_argument('--initial-extrinsics',required=True);p.add_argument('--json',required=True)
    args=p.parse_args()
    initial=yaml.safe_load(Path(args.initial_extrinsics).read_text())
    R=Rotation.from_euler('xyz',[initial['rotation_rpy_rad'][k] for k in ('roll','pitch','yaw')]).as_matrix()
    t=np.array([initial['translation_m'][k] for k in ('x','y','z')])
    prior_R,prior_t=R.copy(),t.copy()
    fit,validation=load_capture(Path(args.fit)),load_capture(Path(args.validate))
    result={'scope':'offline candidate; aligned depth and cloud captures; not a body-frame calibration or motion acceptance',
            'initial':{'fit':metrics(fit,R,t),'held_out_capture':metrics(validation,R,t)}}
    history=[]
    for iteration in range(60):
        x,y,n,d=matches(fit,R,t,.35 if iteration<15 else .20)
        residual=np.einsum('ij,ij->i',x-y,n)
        if len(x)<300:raise RuntimeError('insufficient common surfaces')
        keep=abs(residual)<.12;x,y,n,d,residual=x[keep],y[keep],n[keep],d[keep],residual[keep]
        A=np.column_stack((np.cross(x,n),n))
        weight=np.minimum(1,.025/np.maximum(abs(residual),1e-9))
        Aw=A*np.sqrt(weight[:,None]);bw=-residual*np.sqrt(weight)
        step,_,_,sv=np.linalg.lstsq(Aw,bw,rcond=None)
        ratio=max(np.linalg.norm(step[:3])/np.deg2rad(2),np.linalg.norm(step[3:])/.025,1)
        step/=ratio;dR=Rotation.from_rotvec(step[:3]).as_matrix();R=dR@R;t=dR@t+step[3:]
        history.append({'iteration':iteration,'matches':len(x),'median_plane_error_m':float(np.median(abs(residual))),
                        'step_norm':float(np.linalg.norm(step)),'normal_equation_singular_values':sv.tolist()})
        if np.linalg.norm(step)<1e-6:break
    result.update(candidate={'rotation_rpy_rad':Rotation.from_matrix(R).as_euler('xyz').tolist(),
        'translation_m':t.tolist(),'rotation_change_deg':float(np.rad2deg(Rotation.from_matrix(R@prior_R.T).magnitude())),
        'translation_change_m':float(np.linalg.norm(t-prior_t)),
        'fit':metrics(fit,R,t),'held_out_capture':metrics(validation,R,t)},
        history=history,confirmed=False,navigation_geometry_validated=False,authorizes_motion=False)
    Path(args.json).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='history'}))


if __name__=='__main__':main()
