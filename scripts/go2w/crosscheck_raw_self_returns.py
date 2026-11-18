#!/usr/bin/python3
"""Register static background, then distinguish body-fixed from world-fixed returns.

Uses unfiltered raw captures. Flat floor must fit BOTH hypotheses, so nearest
body matches alone are never evidence of self returns. No motion or live writes.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def voxel(points,size=.025):
    _,index=np.unique(np.floor(points/size).astype(int),axis=0,return_index=True)
    return points[index]


def register(source,target,R,t):
    tree=cKDTree(target);_,neighbors=tree.query(target,k=20)
    batches=target[neighbors];batches-=batches.mean(axis=1)[:,None,:]
    _,singular,v=np.linalg.svd(batches,full_matrices=False);normals=v[:,-1,:]
    stable=(singular[:,-1]<.5*singular[:,-2])
    history=[]
    for iteration in range(60):
        x=source@R.T+t;distance,index=tree.query(x)
        keep=(distance<(.4 if iteration<15 else .15))&stable[index]
        x,y,n=x[keep],target[index[keep]],normals[index[keep]]
        residual=np.einsum('ij,ij->i',x-y,n)
        keep=abs(residual)<.12;x,y,n,residual=x[keep],y[keep],n[keep],residual[keep]
        A=np.column_stack((np.cross(x,n),n));weight=np.minimum(1.,.02/np.maximum(abs(residual),1e-9))
        step,_,_,sv=np.linalg.lstsq(A*np.sqrt(weight[:,None]),-residual*np.sqrt(weight),rcond=None)
        step/=max(np.linalg.norm(step[:3])/np.deg2rad(2),np.linalg.norm(step[3:])/.05,1)
        dR=Rotation.from_rotvec(step[:3]).as_matrix();R=dR@R;t=dR@t+step[3:]
        history.append(dict(iteration=iteration,matches=len(x),median_plane_error=float(np.median(abs(residual))),singular_values=sv.tolist()))
        if np.linalg.norm(step)<1e-6:break
    return R,t,history


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--before',required=True);p.add_argument('--after',required=True)
    p.add_argument('--body-transform',required=True);p.add_argument('--output-dir',required=True)
    args=p.parse_args();out=Path(args.output_dir);out.mkdir(parents=True,exist_ok=True)
    profile=json.loads(Path(args.body_transform).read_text())
    Rb=Rotation.from_euler('xyz',profile['rotation_rpy_rad']).as_matrix();tb=np.array(profile['translation_m'])
    a=voxel(np.load(Path(args.before)/'cloud.npz')['points'],.015)
    b=voxel(np.load(Path(args.after)/'cloud.npz')['points'],.015)
    ab=a@Rb.T+tb;bb=b@Rb.T+tb
    background_a=a[(np.linalg.norm(ab[:,:2],axis=1)>.8)&(np.linalg.norm(ab,axis=1)<5.)]
    background_b=b[(np.linalg.norm(bb[:,:2],axis=1)>.8)&(np.linalg.norm(bb,axis=1)<5.)]
    fits=[]
    for yaw in (0.,.15,-.15):
        Rinitial=Rb.T@Rotation.from_euler('z',yaw).as_matrix()@Rb
        R,t,h=register(voxel(background_a),voxel(background_b),Rinitial,Rb.T@np.array([.54,0,0]))
        fits.append((R,t,h))
    R,t,h=min(fits,key=lambda f:f[2][-1]['median_plane_error'])
    tree=cKDTree(b)
    if h[-1]['median_plane_error']>.02 or min(h[-1]['singular_values'])<10:
        raise ValueError('background registration is too weak for a self-return model')
    body=tree.query(a)[0]
    # If ANY fitted world-motion hypothesis explains a return, keep it.
    # Selecting only the best fit would hide registration uncertainty.
    world=np.minimum.reduce([tree.query(a@fit[0].T+fit[1])[0] for fit in fits])
    near=(np.linalg.norm(ab[:,:2],axis=1)<.75)&(ab[:,2]>-.4)&(ab[:,2]<.5)
    selected=near&(body<.020)&(world>.060)
    inverse_world=np.minimum.reduce([cKDTree(a).query((b-fit[1])@fit[0])[0] for fit in fits])
    near_b=(np.linalg.norm(bb[:,:2],axis=1)<.75)&(bb[:,2]>-.4)&(bb[:,2]<.5)
    selected_b=near_b&(cKDTree(a).query(b)[0]<.020)&(inverse_world>.060)
    candidates=voxel(np.vstack((a[selected],b[selected_b])),.01)
    np.savez_compressed(out/'raw_self_candidates.npz',points=candidates)
    body_R=Rb@R@Rb.T;body_t=Rb@t+tb-body_R@tb
    report=dict(scope='raw-background registration and self-return hypothesis test; not motion authorization',
        before=args.before,after=args.after,body_frame_environment_transform=dict(rotation_rpy=Rotation.from_matrix(body_R).as_euler('xyz').tolist(),translation_m=body_t.tolist()),
        fits=[dict(rotation_rpy=Rotation.from_matrix(f[0]).as_euler('xyz').tolist(),translation_m=f[1].tolist(),last=f[2][-1]) for f in fits],
        source_points=len(a),target_points=len(b),near_points=int(near.sum()),candidate_points=int(selected.sum()),
        symmetric_candidate_points=int(selected_b.sum()),surface_model_points=len(candidates),
        candidate_body_bounds=np.quantile(ab[selected],[0,.5,1],axis=0).tolist() if selected.any() else [],
        thresholds=dict(body_match_m=.020,world_mismatch_m=.060),authorizes_motion=False)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))


if __name__=='__main__':main()
