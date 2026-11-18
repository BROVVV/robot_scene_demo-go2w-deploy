"""A slightly upward camera still sees a real floor beyond the old height strip."""
import importlib.util
from pathlib import Path

import numpy as np
from PIL import Image
import pytest


@pytest.mark.parametrize('bad_depth_fraction',[0.,.35])
def test_camera_floor_survives_a_small_upward_mount_adjustment(tmp_path,monkeypatch,bad_depth_fraction):
    monkeypatch.syspath_prepend(str(Path('scripts/go2w').resolve()))
    source=Path('scripts/go2w/refine_lidar_geometry.py')
    spec=importlib.util.spec_from_file_location('floor_camera_regression',source)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    v,u=np.indices((720,1280));normal=np.array([0.,np.cos(.07),-np.sin(.07)])
    denominator=normal[1]*(v-357.)/912.+normal[2]
    depth=np.divide(.49,denominator,out=np.zeros_like(denominator),where=denominator>.01)
    depth[(depth<.3)|(depth>5.)]=0.
    rng=np.random.default_rng(17)
    bad=(rng.random(depth.shape)<bad_depth_fraction)&(depth>0.)
    depth[bad]=rng.uniform(.3,5.,int(bad.sum()))
    path=tmp_path/'depth.png';Image.fromarray(np.round(depth*1000).astype(np.uint16)).save(path)
    x,y=np.meshgrid(np.linspace(.8,3.5,40),np.linspace(-1.,1.,30))
    raw=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,-.43)))
    meta=dict(depth_ref=str(path),depth_unit_m=.001,fx=912.,fy=912.,cx=637.,cy=357.)
    monkeypatch.setattr(module,'load_capture',lambda _: (raw,None,None,None,meta))
    _,rf,cf=module.floors(tmp_path,dict(rotation_rpy_rad=[0,0,0],translation_m=[0,0,0]))
    assert abs(cf['distance_m']-.49)<.003
    assert np.dot(cf['normal'],normal)>.999
    assert cf['inliers']/cf['candidates']>.95-bad_depth_fraction
    assert abs(rf['distance_m']-.43)<.001


def test_missing_depth_never_produces_a_floor_calibration(tmp_path,monkeypatch):
    monkeypatch.syspath_prepend(str(Path('scripts/go2w').resolve()))
    source=Path('scripts/go2w/refine_lidar_geometry.py')
    spec=importlib.util.spec_from_file_location('floor_empty_regression',source)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    path=tmp_path/'depth.png';Image.fromarray(np.zeros((720,1280),dtype=np.uint16)).save(path)
    x,y=np.meshgrid(np.linspace(.8,3.5,40),np.linspace(-1.,1.,30))
    raw=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,-.43)))
    meta=dict(depth_ref=str(path),depth_unit_m=.001,fx=912.,fy=912.,cx=637.,cy=357.)
    monkeypatch.setattr(module,'load_capture',lambda _: (raw,None,None,None,meta))
    with pytest.raises(ValueError,match='insufficient floor'):
        module.floors(tmp_path,dict(rotation_rpy_rad=[0,0,0],translation_m=[0,0,0]))
