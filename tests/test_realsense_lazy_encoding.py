"""Lazy encoding preserves the full RGB-D pair and existing HTTP contract."""
from concurrent.futures import ThreadPoolExecutor
from collections import deque
import importlib.util
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import urllib.request
import urllib.error

import cv2
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
@pytest.fixture
def module(monkeypatch):
    # Importing the server must never open a physical device in these tests.
    monkeypatch.setitem(sys.modules,'pyrealsense2',SimpleNamespace())
    spec=importlib.util.spec_from_file_location('lazy_camera_unit',ROOT/'scripts/go2w/realsense_stream.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    return mod

def images(seed=1):
    rng=np.random.default_rng(seed)
    return rng.integers(0,256,(48,64,3),dtype=np.uint8),rng.integers(0,65000,(48,64),dtype=np.uint16)

def old_preview(depth):
    z=np.clip(depth.astype(np.float32)/1000.,0.,6.)
    image=cv2.applyColorMap((z/6.*255.).astype(np.uint8),cv2.COLORMAP_JET)
    cy,cx=depth.shape[0]//2,depth.shape[1]//2
    cv2.circle(image,(cx,cy),4,(255,255,255),-1)
    cv2.putText(image,'%.2f m'%(int(depth[cy,cx])/1000.),(cx+10,cy-8),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),2)
    return cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,82])[1].tobytes()

@pytest.mark.parametrize('seed',[1,5,18])
def test_encodings_match_old_bytes_and_depth_pixels(module,seed):
    color,depth=images(seed);encoder=module.FrameEncoder(color,depth,6.)
    assert encoder._bytes=={}
    assert encoder.get('color_jpg')==cv2.imencode('.jpg',color,[cv2.IMWRITE_JPEG_QUALITY,82])[1].tobytes()
    assert encoder.get('depth_jpg')==old_preview(depth)
    png=encoder.get('depth_raw_png')
    assert png==cv2.imencode('.png',depth)[1].tobytes()
    np.testing.assert_array_equal(cv2.imdecode(np.frombuffer(png,np.uint8),cv2.IMREAD_UNCHANGED),depth)

def test_encoder_owns_raw_pair_and_does_not_encode_during_capture(module,monkeypatch):
    color,depth=images();oldcolor=color.copy();olddepth=depth.copy()
    original=cv2.imencode;calls=[]
    monkeypatch.setattr(module.cv2,'imencode',lambda *a,**kw:(calls.append(a[0]),original(*a,**kw))[1])
    encoder=module.FrameEncoder(color,depth,6.);assert calls==[]
    color.fill(0);depth.fill(0)
    assert encoder.get('color_jpg')==original('.jpg',oldcolor,[cv2.IMWRITE_JPEG_QUALITY,82])[1].tobytes()
    np.testing.assert_array_equal(cv2.imdecode(np.frombuffer(encoder.get('depth_raw_png'),np.uint8),-1),olddepth)

def test_concurrent_consumers_share_one_encoding_per_format(module,monkeypatch):
    color,depth=images();encoder=module.FrameEncoder(color,depth,6.);original=cv2.imencode;calls=[]
    monkeypatch.setattr(module.cv2,'imencode',lambda *a,**kw:(calls.append(a[0]),original(*a,**kw))[1])
    keys=['color_jpg','depth_raw_png','depth_jpg']*10
    with ThreadPoolExecutor(max_workers=10) as executor:results=list(executor.map(encoder.get,keys))
    assert len(calls)==3
    for key in set(keys):assert len({value for k,value in zip(keys,results) if k==key})==1

def test_failed_encoding_is_not_cached_as_a_success(module,monkeypatch):
    encoder=module.FrameEncoder(*images(),6.);original=cv2.imencode
    monkeypatch.setattr(module.cv2,'imencode',lambda *a,**kw:(False,None))
    with pytest.raises(RuntimeError):encoder.get('depth_raw_png')
    assert encoder._bytes=={}
    monkeypatch.setattr(module.cv2,'imencode',original)
    assert encoder.get('depth_raw_png').startswith(b'\x89PNG')
    with pytest.raises(ValueError):encoder.get('unknown')

@pytest.mark.parametrize('query,interval',[('/color',0.),('/color?fps=10',.1),('/depth?fps=30',1/30),('/color?fps=.5',2.)])
def test_optional_mjpeg_cadence(module,query,interval):
    assert module._mjpeg_interval(query)==pytest.approx(interval)

@pytest.mark.parametrize('query',['fps=0','fps=-1','fps=31','fps=nan','fps=inf','fps=a','fps=10&fps=20','fps='])
def test_bad_mjpeg_cadence_is_rejected(module,query):
    with pytest.raises(ValueError):module._mjpeg_interval('/color?'+query)


def make_frame(module,number):
    color,depth=images(number)
    return {'frame_id':str(number),'host_timestamp':module.time.time(),'device_timestamp_ms':number*33.,
            'width':64,'height':48,'intrinsics':{'fx':1.,'fy':1.,'cx':32.,'cy':24.},
            'depth_aligned_to_color':True,'depth_unit_m':.001,
            'color_url':f'/rgbd/frame/{number}/color.jpg','depth_url':f'/rgbd/frame/{number}/depth.png',
            'info':{'frame_id':str(number),'center_depth_mm':int(depth[24,32]),'depth_stats_m':{}},
            'color_jpg':None,'depth_jpg':None,'depth_raw_png':None,
            '_encoder':module.FrameEncoder(color,depth,6.)}

def test_http_atomic_pairs_latest_depth_snapshot_and_eviction(module,tmp_path):
    a=make_frame(module,1);b=make_frame(module,2)
    module.FRAME_CACHE=deque([a,b],maxlen=32);module.STREAM_ACTIVE.set()
    module.ARGS=SimpleNamespace(snap_dir=str(tmp_path),verbose=False)
    module.BUFFER={'frame':b,'color_jpg':None,'depth_jpg':None,'depth_raw_png':None,
                   'info':b['info'],'stamp':b['host_timestamp']}
    server=module.ThreadingHTTPServer(('127.0.0.1',0),module.Handler)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}));url='http://127.0.0.1:'+str(server.server_port)
    def read(path):
        with opener.open(url+path,timeout=2) as response:return response.read()
    try:
        meta=json.loads(read('/rgbd/latest.json'));assert meta['frame_id']=='2'
        assert '_encoder' not in meta and meta['depth_unit_m']==.001
        for frame in (a,b):
            assert read(frame['color_url'])==frame['_encoder'].get('color_jpg')
            raw=read(frame['depth_url']);expected=images(int(frame['frame_id']))[1]
            np.testing.assert_array_equal(cv2.imdecode(np.frombuffer(raw,np.uint8),-1),expected)
        assert read('/depth_raw')==b['_encoder'].get('depth_raw_png')
        snap=json.loads(read('/snapshot'));directory=Path(snap['dir'])
        assert (directory/'color.jpg').read_bytes()==b['_encoder'].get('color_jpg')
        assert (directory/'depth.jpg').read_bytes()==b['_encoder'].get('depth_jpg')
        assert (directory/'depth_raw.png').read_bytes()==b['_encoder'].get('depth_raw_png')
        assert json.loads((directory/'info.json').read_text())['frame_id']=='2'
        for path,key in [('/color?fps=10','color_jpg'),('/depth?fps=10','depth_jpg')]:
            response=opener.open(url+path,timeout=2)
            assert response.headers['Content-Type'].startswith('multipart/')
            assert response.readline().strip()==b'--frame'
            length=None
            while True:
                line=response.readline().strip()
                if line.lower().startswith(b'content-length:'):length=int(line.split(b':')[1])
                if not line:break
            assert length is not None and response.read(length)==b['_encoder'].get(key)
            response.close()
            # Wake the old polling stream so a closed peer is detected.
            import time
            for _ in range(3):module.BUFFER['stamp']+=1;time.sleep(.015)
        with pytest.raises(urllib.error.HTTPError) as error:read('/color?fps=nan')
        assert error.value.code==400
        module.FRAME_CACHE.popleft()
        with pytest.raises(urllib.error.HTTPError) as error:read(a['color_url'])
        assert error.value.code==404
    finally:server.shutdown();server.server_close();thread.join(timeout=2)

@pytest.mark.parametrize('mode,expected_encodings',[('eager',3),('on-demand',0)])
def test_capture_loop_populates_eager_and_lazy_frames(module,monkeypatch,mode,expected_encodings):
    color,depth=images();calls=[];original=cv2.imencode
    monkeypatch.setattr(module.cv2,'imencode',lambda *a,**kw:(calls.append(a[0]),original(*a,**kw))[1])
    def image(array):return SimpleNamespace(get_data=lambda:array,get_width=lambda:64,get_height=lambda:48,get_timestamp=lambda:33.)
    frames=SimpleNamespace(frame_number=99,get_color_frame=lambda:image(color),get_depth_frame=lambda:image(depth))
    device=SimpleNamespace(first_depth_sensor=lambda:None,query_sensors=lambda:[None,None])
    profile=SimpleNamespace(get_device=lambda:device,get_streams=lambda:[])
    class Pipeline:
        count=0
        def start(self,config):return profile
        def wait_for_frames(self):
            self.count+=1
            if self.count>1:raise RuntimeError('end fixture')
            return frames
    class EndCapture(BaseException):pass
    module.rs=SimpleNamespace(context=lambda:SimpleNamespace(devices=[device]),pipeline=Pipeline,
       config=lambda:SimpleNamespace(enable_stream=lambda *a:None),
       align=lambda stream:SimpleNamespace(process=lambda x:x),stream=SimpleNamespace(color=1,depth=2),
       format=SimpleNamespace(bgr8=1,z16=2),option=SimpleNamespace())
    module.ARGS=SimpleNamespace(width=64,height=48,fps=30,max_depth=6.,encoding_mode=mode)
    monkeypatch.setattr(module.time,'sleep',lambda seconds:(_ for _ in ()).throw(EndCapture()))
    with pytest.raises(EndCapture):module.rs_capture_loop()
    frame=module.FRAME_CACHE[-1]
    assert len(calls)==expected_encodings
    assert frame['frame_id']=='99' and frame['info']['center_depth_mm']==int(depth[24,32])
    assert module.BUFFER['frame'] is frame
    if mode=='on-demand':assert frame['_encoder']._bytes=={}
    else:assert frame['_encoder'] is None
    raw=module._frame_bytes(frame,'depth_raw_png')
    np.testing.assert_array_equal(cv2.imdecode(np.frombuffer(raw,np.uint8),-1),depth)
