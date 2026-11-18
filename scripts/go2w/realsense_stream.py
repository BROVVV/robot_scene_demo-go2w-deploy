#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
realsense_stream.py — Intel RealSense D435 RGB-D 网络流服务（运行在机器狗内部 Ubuntu）

功能（HTTP 端口默认 8080，机器狗 IP 192.168.123.18）：
  /             HTML 查看页：彩色画面 + 深度伪彩画面 + 设备/内参/外参/传感器信息
  /color        MJPEG 彩色流
  /depth        MJPEG 深度流（jet 伪彩 + 中心点距离叠加，深度对齐到彩色）
  /depth_raw    原始 16-bit 深度 PNG（z16，单位毫米）
  /info.json    设备信息、内参、外参、传感器选项、帧统计
  /health       服务健康检查（fps、帧龄）
  /snapshot     在机器狗上保存一帧快照（color.jpg + depth.jpg + depth_raw.png + info.json）
  /snap/        快照文件目录浏览与下载

用法：
  python3 realsense_stream.py [--width 848] [--height 480] [--fps 30]
                              [--port 8080] [--max-depth 6.0] [--snap-dir ~/realsense_snapshots]
依赖：pyrealsense2（aarch64 wheel，2.55.1）、numpy、opencv-python（机器狗已有 cv2 4.2.0）
"""
import argparse
import json
import math
import os
import threading
import time
import traceback
import uuid
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, parse_qs, urlsplit

import cv2
import numpy as np
import pyrealsense2 as rs

# ---------------------------------------------------------------------------
# 全局配置
# ---------------------------------------------------------------------------
ARGS = None
BUFFER = {}          # 最新帧缓存: color_jpg, depth_jpg, depth_raw_png, depth_raw, shape, stamp, info
BUFFER_LOCK = threading.Lock()
FRAME_CACHE = deque(maxlen=32)  # 原子 RGB-D frame 缓存（color/depth 来自同一 frameset）
STREAM_ACTIVE = threading.Event()
DEVICE_INFO = {}     # 设备/内参/外参缓存（由采集线程启动后填充一次）


class CaptureCadence:
    """Recent completed RGB-D updates, independent of HTTP polling phase.

    Call under BUFFER_LOCK. This counts distinct delivered framesets, so an
    eager encoder that cannot keep up reports its actual prepared-frame rate,
    rather than the configured sensor rate or skipped hardware frame numbers.
    """
    def __init__(self):
        self._samples = deque(maxlen=128)
        self._last_frame_id = None

    def reset(self):
        self._samples.clear()
        self._last_frame_id = None

    def _prune(self, now):
        while self._samples and self._samples[0] < now - 2.:
            self._samples.popleft()

    def record(self, frame_id):
        now = time.monotonic()
        if not math.isfinite(now):
            self.reset()
            return
        if self._samples and now < self._samples[-1]:
            self.reset()
        self._prune(now)
        if frame_id != self._last_frame_id:
            self._samples.append(now)
            self._last_frame_id = frame_id

    def value(self):
        now = time.monotonic()
        self._prune(now)
        if (len(self._samples) < 2 or not math.isfinite(now)
                or not 0 <= now - self._samples[-1] <= 1.):
            return 0.
        elapsed = self._samples[-1] - self._samples[0]
        return (len(self._samples) - 1) / elapsed if elapsed > 0 else 0.


CAPTURE_CADENCE = CaptureCadence()


# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# 视场（FOV）计算：D435 镜头的最大硬件视场是彩色约 69.4°x42.5°、深度约 87°x58°。
# 但低分辨率模式（640x480 默认）是对更宽传感器中心区域的“裁剪”，实际 HFOV 只有
# 彩色约 55°、深度约 70°——这正是“只能看到正前方一小块”的常见原因。
# 切到 848x480 / 1280x720 用满传感器宽度就能把取景范围真正“拉宽”到约 69°/87°。
# 这里从内参实时算出 HFOV/VFOV 用于校验模式；真正超过硬件上限需要改镜头或让机器狗
# 转头/换位扫视。见 docs/REALSENSE_D435_DEPLOYMENT.md。
# ---------------------------------------------------------------------------
def _depth_preview_jpeg(depth, max_depth, clipped=None):
    z_m = (np.clip(depth.astype(np.float32) / 1000., 0., max_depth)
           if clipped is None else clipped)
    d8 = (z_m / max_depth * 255.).astype(np.uint8)
    image = cv2.applyColorMap(d8, cv2.COLORMAP_JET)
    cx, cy = depth.shape[1] // 2, depth.shape[0] // 2
    cv2.circle(image, (cx, cy), 4, (255, 255, 255), -1)
    cv2.putText(image, "%.2f m" % (int(depth[cy, cx]) / 1000.), (cx + 10, cy - 8),
                cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 2)
    ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 82])
    if not ok:
        raise RuntimeError('depth preview encoding failed')
    return encoded.tobytes()


class FrameEncoder:
    """Own an immutable RGB-D pair; encode each representation at most once."""
    def __init__(self, color, depth, max_depth):
        self.color = np.array(color, copy=True)
        self.depth = np.array(depth, copy=True)
        self.color.flags.writeable = False
        self.depth.flags.writeable = False
        self.max_depth = max_depth
        self._bytes = {}
        self._lock = threading.Lock()

    def get(self, key):
        with self._lock:
            if key not in self._bytes:
                if key == 'depth_jpg':
                    value = _depth_preview_jpeg(self.depth, self.max_depth)
                else:
                    if key == 'color_jpg':
                        ok, encoded = cv2.imencode('.jpg', self.color,
                                                  [cv2.IMWRITE_JPEG_QUALITY, 82])
                    elif key == 'depth_raw_png':
                        ok, encoded = cv2.imencode('.png', self.depth)
                    else:
                        raise ValueError('unknown frame encoding: ' + key)
                    if not ok:
                        raise RuntimeError('frame encoding failed: ' + key)
                    value = encoded.tobytes()
                self._bytes[key] = value
            return self._bytes[key]


def _frame_bytes(frame, key):
    encoder = frame.get('_encoder')
    return encoder.get(key) if encoder is not None else frame.get(key)


def _mjpeg_interval(url):
    values = parse_qs(urlsplit(url).query, keep_blank_values=True).get('fps')
    if values is None:
        return 0.
    if len(values) != 1:
        raise ValueError('fps must be one finite number in (0,30]')
    value = float(values[0])
    if not math.isfinite(value) or not 0 < value <= 30:
        raise ValueError('fps must be one finite number in (0,30]')
    return 1. / value


def fov_deg_from_intrinsics(intr):
    """从内参（fx/fy/width/height）计算水平/垂直视场（度）。"""
    if not intr:
        return None
    fx = float(intr.get("fx") or 0.0)
    fy = float(intr.get("fy") or 0.0)
    w = float(intr.get("width") or intr.get("width", 0) or 0)
    h = float(intr.get("height") or intr.get("height", 0) or 0)
    depth_intr = None
    # DEVICE_INFO 的内参结构里 width/height 就是颜色流尺寸
    if fx > 0 and w > 0:
        hfov = 2.0 * math.degrees(math.atan((w / 2.0) / fx))
    else:
        hfov = None
    if fy > 0 and h > 0:
        vfov = 2.0 * math.degrees(math.atan((h / 2.0) / fy))
    else:
        vfov = None
    if hfov is None and vfov is None:
        return None
    return {"h_deg": (round(hfov, 2) if hfov is not None else None),
            "v_deg": (round(vfov, 2) if vfov is not None else None)}


# RealSense 采集线程
# ---------------------------------------------------------------------------
def rs_capture_loop():
    global BUFFER
    while True:
        try:
            ctx = rs.context()
            if len(ctx.devices) == 0:
                STREAM_ACTIVE.clear()
                print("[rs] no device, retry in 5s", flush=True)
                time.sleep(5)
                continue
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, ARGS.width, ARGS.height, rs.format.bgr8, ARGS.fps)
            cfg.enable_stream(rs.stream.depth, ARGS.width, ARGS.height, rs.format.z16, ARGS.fps)
            profile = pipe.start(cfg)
            capture_epoch = uuid.uuid4().hex
            capture_sequence = 0
            with BUFFER_LOCK:
                CAPTURE_CADENCE.reset()
            depth_sensor = profile.get_device().first_depth_sensor()
            color_sensor = profile.get_device().query_sensors()[1]
            align = rs.align(rs.stream.color)
            print("[rs] stream started %dx%d@%d" % (ARGS.width, ARGS.height, ARGS.fps), flush=True)

            # ---- 设备 / 内参 / 外参（从当前运行管线读取一次，避免二次打开设备冲突）----
            try:
                dev = profile.get_device()
                def gi(info):
                    try:
                        return dev.get_info(info)
                    except Exception:
                        return None
                streams = profile.get_streams()
                def intrinsics(s):
                    if s is None:
                        return None
                    i = s.as_video_stream_profile().get_intrinsics()
                    return {"width": i.width, "height": i.height, "fx": i.fx, "fy": i.fy,
                            "ppx": i.ppx, "ppy": i.ppy, "model": str(i.model),
                            "coeffs": list(i.coeffs)}
                color_stream = next((s for s in streams if s.stream_type() == rs.stream.color), None)
                depth_stream = next((s for s in streams if s.stream_type() == rs.stream.depth), None)
                def extrinsics(a, b):
                    try:
                        e = a.get_extrinsics_to(b)
                        return {"rotation": [round(x, 6) for x in e.rotation],
                                "translation": [round(x, 6) for x in e.translation]}
                    except Exception:
                        return None
                DEVICE_INFO.clear()
                DEVICE_INFO.update({
                    "device": {
                        "name": gi(rs.camera_info.name),
                        "serial_number": gi(rs.camera_info.serial_number),
                        "firmware_version": gi(rs.camera_info.firmware_version),
                        "usb_type": gi(rs.camera_info.usb_type_descriptor),
                        "product_line": gi(rs.camera_info.product_line),
                    },
                    "depth_intrinsics": intrinsics(depth_stream),
                    "color_intrinsics": intrinsics(color_stream),
                    "depth_fov_deg": fov_deg_from_intrinsics(intrinsics(depth_stream)),
                    "color_fov_deg": fov_deg_from_intrinsics(intrinsics(color_stream)),
                    "depth_to_color_extrinsics": extrinsics(depth_stream, color_stream),
                    "color_to_depth_extrinsics": extrinsics(color_stream, depth_stream),
                    "notes": "extrinsics 为相机内部标定；相对机器狗 base_link 的外参尚未标定。"
                             "FOV 由 D435 镜头硬件决定，软件无法拉宽；当前 FOV 仅用于校验模式未裁剪。",
                })
            except Exception as e:
                print("[rs] device info warn:", e, flush=True)

            # 传感器信息（启动后读取一次；精选常用选项，避免枚举不支持项）
            sensor_info = {}
            CURATED_OPTIONS = [
                "enable_auto_exposure", "exposure", "gain", "laser_power",
                "white_balance", "enable_auto_white_balance", "frames_queue_size",
                "depth_units", "visual_preset", "emitter_enabled",
            ]
            try:
                for sname, sens in (("depth", depth_sensor), ("color", color_sensor)):
                    opts = {}
                    for oname in CURATED_OPTIONS:
                        o = getattr(rs.option, oname, None)
                        if o is None:
                            continue
                        try:
                            opts[oname] = {
                                "value": sens.get_option(o),
                                "min": sens.get_option_range(o).min,
                                "max": sens.get_option_range(o).max,
                                "step": sens.get_option_range(o).step,
                            }
                        except Exception:
                            pass  # 该传感器不支持此选项
                    sensor_info[sname] = opts
            except Exception as e:
                print("[rs] sensor info warn:", e, flush=True)

            STREAM_ACTIVE.set()
            while True:
                frames = pipe.wait_for_frames()
                aligned = align.process(frames)
                color = aligned.get_color_frame()
                depth = aligned.get_depth_frame()
                if not color or not depth:
                    continue

                cimg = np.asanyarray(color.get_data())
                dimg = np.asanyarray(depth.get_data())  # uint16, mm

                z_m=np.clip(dimg.astype(np.float32)/1000.,0.,ARGS.max_depth)
                cy,cx=dimg.shape[0]//2,dimg.shape[1]//2
                center_mm=int(dimg[cy,cx])
                encoder=None
                if ARGS.encoding_mode=='on-demand':
                    encoder=FrameEncoder(cimg,dimg,ARGS.max_depth)
                    color_jpeg=depth_jpeg=depth_png=None
                else:
                    ok,cjpg=cv2.imencode('.jpg',cimg,[cv2.IMWRITE_JPEG_QUALITY,82])
                    if not ok:continue
                    depth_jpeg=_depth_preview_jpeg(dimg,ARGS.max_depth,z_m)
                    ok,d16png=cv2.imencode('.png',dimg)
                    if not ok:continue
                    color_jpeg=cjpg.tobytes();depth_png=d16png.tobytes()

                stamp = time.time()
                frame_id = str(frames.frame_number)
                capture_sequence += 1
                capture_id = f"{capture_epoch}-{capture_sequence}"
                device_timestamp_ms = _safe_frame_timestamp_ms(color)
                color_intr = ((DEVICE_INFO.get("color_intrinsics") or {}) if DEVICE_INFO else {})
                color_fov = fov_deg_from_intrinsics(color_intr)
                depth_intr = ((DEVICE_INFO.get("depth_intrinsics") or {}) if DEVICE_INFO else {})
                depth_fov = fov_deg_from_intrinsics(depth_intr)
                info = {
                    "stamp": stamp,
                    "frame_number": frames.frame_number,
                    "frame_id": frame_id,
                    "capture_id": capture_id,
                    "device_timestamp_ms": device_timestamp_ms,
                    "color": {"width": color.get_width(), "height": color.get_height(),
                              "fps": ARGS.fps, "format": "bgr8"},
                    "depth": {"width": depth.get_width(), "height": depth.get_height(),
                              "fps": ARGS.fps, "format": "z16(mm)"},
                    "center_depth_mm": center_mm,
                    "depth_stats_m": {
                        "min": float(np.min(z_m)), "max": float(np.max(z_m)),
                        "mean": float(np.mean(z_m)),
                    },
                    "intrinsics": {
                        "fx": float(color_intr.get("fx", 0.0)),
                        "fy": float(color_intr.get("fy", 0.0)),
                        "cx": float(color_intr.get("ppx", 0.0)),
                        "cy": float(color_intr.get("ppy", 0.0)),
                        "width": int(color.get_width()),
                        "height": int(color.get_height()),
                    },
                    "fov_deg": {
                        "color": color_fov,
                        "depth": depth_fov,
                    },
                    "fov_widen_note": "D435 满宽度视场彩色约69°x42°、深度约87°x58°；"
                                       "若当前模式 HFOV 明显小于该值（如 640x480 约为55°），请切到 848x480/1280x720 即可明显拉宽取景范围。",
                    "depth_aligned_to_color": True,
                    "depth_unit_m": 0.001,
                    "sensor": sensor_info,
                }
                rgbd_frame = {
                    "frame_id": frame_id,
                    "capture_id": capture_id,
                    "device_timestamp_ms": device_timestamp_ms,
                    "host_timestamp": stamp,
                    "color_url": f"/rgbd/frame/{frame_id}/color.jpg?capture_id={capture_id}",
                    "depth_url": f"/rgbd/frame/{frame_id}/depth.png?capture_id={capture_id}",
                    "depth_aligned_to_color": True,
                    "depth_unit_m": 0.001,
                    "width": int(color.get_width()),
                    "height": int(color.get_height()),
                    "intrinsics": {
                        "fx": float(color_intr.get("fx", 0.0)),
                        "fy": float(color_intr.get("fy", 0.0)),
                        "cx": float(color_intr.get("ppx", 0.0)),
                        "cy": float(color_intr.get("ppy", 0.0)),
                    },
                    "info": info,
                    "color_jpg": color_jpeg,
                    "depth_raw_png": depth_png,
                    "depth_jpg": depth_jpeg,
                    "_encoder": encoder,
                }
                with BUFFER_LOCK:
                    BUFFER.update({
                        "frame": rgbd_frame,
                        "color_jpg": color_jpeg,
                        "depth_jpg": depth_jpeg,
                        "depth_raw_png": depth_png,
                        "color_shape": cimg.shape,
                        "depth_shape": dimg.shape,
                        "stamp": stamp,
                        "info": info,
                    })
                    FRAME_CACHE.append(rgbd_frame)
                    CAPTURE_CADENCE.record(frame_id)
        except RuntimeError as e:
            print("[rs] realsense error: %s, retry in 5s" % e, flush=True)
            STREAM_ACTIVE.clear()
            time.sleep(5)
        except Exception:
            print("[rs] unexpected: %s" % traceback.format_exc(), flush=True)
            STREAM_ACTIVE.clear()
            time.sleep(5)


# ---------------------------------------------------------------------------
# 设备/内参/外参信息
# ---------------------------------------------------------------------------
def _safe_frame_timestamp_ms(frame):
    """Best-effort device timestamp in milliseconds; never blocks the loop."""
    try:
        value = float(frame.get_timestamp())
        return value if value == value else 0.0  # NaN guard
    except Exception:
        return 0.0


def build_device_info():
    """返回采集线程缓存的设备级信息（不二次打开设备，避免 Device busy）"""
    if DEVICE_INFO:
        return DEVICE_INFO
    return {"loading": True, "note": "capture thread has not initialized yet"}


# ---------------------------------------------------------------------------
# HTTP 处理
# ---------------------------------------------------------------------------
HTML_PAGE = None

def get_html_page():
    global HTML_PAGE
    if HTML_PAGE is None:
        HTML_PAGE = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>Go2-W RealSense D435 RGB-D</title>
<style>
body{background:#101418;color:#e8e8e8;font-family:monospace;margin:0;padding:16px}
h1{font-size:18px;color:#4fc3f7}
.grid{display:flex;gap:16px;flex-wrap:wrap}
.panel{background:#1a2026;border:1px solid #2a323a;border-radius:8px;padding:10px;flex:1;min-width:340px}
.panel h2{font-size:13px;color:#81c784;margin:4px 0 8px}
img{width:100%;max-width:640px;border-radius:4px;background:#000}
pre{font-size:11px;line-height:1.5;max-height:420px;overflow:auto;white-space:pre-wrap}
button{background:#1565c0;color:#fff;border:none;padding:8px 16px;border-radius:6px;cursor:pointer;font-size:13px}
button:hover{background:#1976d2}
.status{font-size:12px;margin:8px 0}
#stamp{color:#ffb74d}
a{color:#4fc3f7}
</style></head><body>
<h1>Go2-W &rsaquo; Intel RealSense D435 (192.168.123.18:8080)</h1>
<div class="status">彩色 <span id="s_color">-</span> &nbsp; 深度 <span id="s_depth">-</span> &nbsp;
中心距离 <span id="s_center">-</span> &nbsp; 帧龄 <span id="s_age">-</span> &nbsp; FOV <span id="s_fov">-</span></div>
<div class="status" style="color:#ffb74d;font-size:11px">提示：D435 满宽度视场约彩色 69°×42°、深度 87°×58°；
低分辨率 640x480 是中心裁剪，只有约 55°，会明显显得“取景小”。切 848x480 / 1280x720 即可真正拉宽取景范围。</div>
<div class="grid">
  <div class="panel"><h2>&#9654; COLOR (MJPEG)</h2><img id="img_color" src="/color"></div>
  <div class="panel"><h2>&#9654; DEPTH (jet, aligned to color)</h2><img id="img_depth" src="/depth"></div>
</div>
<div style="margin-top:12px">
  <button onclick="snap()">&#128247; 保存快照到机器狗</button>
  <span id="snap_res" style="font-size:12px;margin-left:10px"></span>
</div>
<div class="grid" style="margin-top:12px">
  <div class="panel"><h2>&#8505; 设备 / 内参 / 外参 / 传感器</h2><pre id="info">loading...</pre></div>
  <div class="panel"><h2>&#128193; 快照文件</h2><pre id="snaps">(点上方按钮保存)</pre></div>
</div>
<script>
function snap(){
  fetch('/snapshot').then(r=>r.json()).then(j=>{
    document.getElementById('snap_res').textContent = j.message || JSON.stringify(j);
    refreshSnaps();
  });
}
function refreshSnaps(){
  fetch('/snap/?list=1').then(r=>r.json()).then(j=>{
    let t = j.files ? j.files.map(f=>'<a href="/snap/'+f+'">'+f+'</a>').join('\\n') : '(空)';
    document.getElementById('snaps').innerHTML = t;
  });
}
setInterval(()=>{
  fetch('/info.json?_='+Date.now()).then(r=>r.json()).then(j=>{
    if(j.error){document.getElementById('info').textContent=JSON.stringify(j,null,1);return;}
    document.getElementById('info').textContent = JSON.stringify(j,null,1);
    if(j.color) document.getElementById('s_color').textContent = j.color.width+'x'+j.color.height+'@'+j.color.fps+'fps';
    if(j.depth) document.getElementById('s_depth').textContent = j.depth.width+'x'+j.depth.height+'@'+j.depth.fps+'fps';
    document.getElementById('s_center').textContent = (j.center_depth_mm/1000).toFixed(3)+' m';
    var fovh = j.fov_deg && j.fov_deg.color;
    if (fovh && fovh.h_deg != null) {
      document.getElementById('s_fov').textContent =
        'H '+fovh.h_deg+'° V '+(fovh.v_deg != null ? fovh.v_deg+'°' : '-');
    }
  });
  fetch('/health?_='+Date.now()).then(r=>r.json()).then(j=>{
    document.getElementById('s_age').textContent = 'age '+j.age_s.toFixed(2)+'s / fps '+j.fps.toFixed(1);
  });
}, 1000);
refreshSnaps();
</script></body></html>"""
    return HTML_PAGE


def mjpeg_headers():
    return (b"HTTP/1.1 200 OK\r\n"
            b"Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
            b"Cache-Control: no-cache\r\n"
            b"Connection: close\r\n\r\n")


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        if ARGS and ARGS.verbose:
            print("[http] " + fmt % args, flush=True)

    # ---- helpers ----
    def _send_json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False, indent=1).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    def _send_bytes(self, data, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(data)

    def _stream_mjpeg(self, key, snap_key):
        """从共享缓存持续发送最新一帧 MJPEG"""
        try:
            interval = _mjpeg_interval(self.path)
        except ValueError:
            self._send_json({'error': 'fps must be one finite number in (0,30]'}, 400)
            return
        self.wfile.write(mjpeg_headers())
        self.wfile.flush()
        last = -1
        next_at=0.
        while True:
            delay = next_at - time.monotonic()
            if delay > 0:
                time.sleep(min(.01, delay));continue
            with BUFFER_LOCK:
                frame=BUFFER.get('frame')
                stamp=BUFFER.get('stamp',0)
                data=BUFFER.get(key) if frame is None else None
            if stamp!=last:
                if frame is not None:data=_frame_bytes(frame,key)
                if data is None:time.sleep(.01);continue
                last=stamp
                chunk=(b"--frame\r\nContent-Type: image/jpeg\r\n"
                       b"Content-Length: "+str(len(data)).encode()+b"\r\n\r\n")
                self.wfile.write(chunk);self.wfile.write(data);self.wfile.write(b"\r\n");self.wfile.flush()
                next_at=time.monotonic()+interval
            else:time.sleep(.01)

    # ---- routes ----
    def do_POST(self):
        # 与 GET 相同处理（快照按钮兼容 POST 调用）
        self.do_GET()

    def do_GET(self):
        path = unquote(self.path.split("?")[0])
        try:
            if path in ("/", "/index.html"):
                page = get_html_page().encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(page)))
                self.end_headers()
                self.wfile.write(page)
            elif path == "/rgbd/latest.json":
                self._send_rgbd_latest()
            elif path.startswith("/rgbd/frame/"):
                self._send_rgbd_frame(path)
            elif path == "/color":
                self._stream_mjpeg("color_jpg", "stamp")
            elif path == "/depth":
                self._stream_mjpeg("depth_jpg", "stamp")
            elif path == "/depth_raw":
                with BUFFER_LOCK:
                    frame=BUFFER.get("frame")
                    data=BUFFER.get("depth_raw_png") if frame is None else None
                if frame is not None:data=_frame_bytes(frame,"depth_raw_png")
                if data:
                    self._send_bytes(data, "image/png")
                else:
                    self._send_json({"error": "no frame yet"}, 503)
            elif path == "/fov":
                with BUFFER_LOCK:
                    info = BUFFER.get("info")
                dev = build_device_info()
                out = {
                    "ok": True,
                    "fov_deg": (info or {}).get("fov_deg"),
                    "device_fov_deg": {
                        "color": dev.get("color_fov_deg"),
                        "depth": dev.get("depth_fov_deg"),
                    },
                    "note": "D435 FOV 由镜头硬件固定；软件只负责保持满分辨率非裁剪模式。",
                }
                self._send_json(out)
            elif path == "/info.json":
                with BUFFER_LOCK:
                    info = BUFFER.get("info")
                dev = build_device_info()
                out = dict(info or {})
                out.update({"device_info": dev})
                self._send_json(out)
            elif path == "/health":
                with BUFFER_LOCK:
                    stamp = BUFFER.get("stamp", 0)
                    active = STREAM_ACTIVE.is_set()
                    fps = CAPTURE_CADENCE.value() if active else 0.
                age = time.time() - stamp if stamp else 999
                self._send_json({
                    "ok": active and age < 5,
                    "streaming": active,
                    "age_s": round(age, 3),
                    "fps": round(fps, 2),
                })
            elif path == "/snapshot":
                self._do_snapshot()
            elif path == "/snap/":
                self._send_snap_list()
            elif path.startswith("/snap/"):
                self._send_snap_file(path[len("/snap/"):])
            else:
                self._send_json({"error": "not found: " + path}, 404)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception:
            try:
                self._send_json({"error": traceback.format_exc()}, 500)
            except Exception:
                pass

    def _rgbd_meta(self, frame: dict) -> dict:
        """Return the JSON-safe metadata view of one atomic RGB-D frame."""
        info = frame.get("info") or {}
        return {
            "frame_id": frame["frame_id"],
            "capture_id": frame.get("capture_id"),
            "device_timestamp_ms": frame.get("device_timestamp_ms"),
            "host_timestamp": frame.get("host_timestamp"),
            "color_url": frame.get("color_url"),
            "depth_url": frame.get("depth_url"),
            "depth_aligned_to_color": bool(frame.get("depth_aligned_to_color", True)),
            "depth_unit_m": frame.get("depth_unit_m", 0.001),
            "width": frame.get("width"),
            "height": frame.get("height"),
            "intrinsics": frame.get("intrinsics") or {},
            "health": {
                "age_s": round(max(0.0, time.time() - float(frame.get("host_timestamp", 0.0))), 3),
                "streaming": STREAM_ACTIVE.is_set(),
                "center_depth_mm": info.get("center_depth_mm"),
                "depth_stats_m": info.get("depth_stats_m"),
            },
        }

    def _send_rgbd_latest(self):
        with BUFFER_LOCK:
            if not FRAME_CACHE:
                self._send_json({"error": "no frame yet"}, 503)
                return
            frame = FRAME_CACHE[-1]
        self._send_json(self._rgbd_meta(frame))

    def _send_rgbd_frame(self, path: str):
        parts = path[len("/rgbd/frame/"):].split("/", 1)
        if len(parts) != 2:
            self._send_json({"error": "expected /rgbd/frame/<id>/color.jpg|depth.png"}, 400)
            return
        frame_id, name = parts
        query = parse_qs(urlsplit(self.path).query, keep_blank_values=True)
        tokens = query.get("capture_id")
        if tokens is not None and (len(tokens) != 1 or not tokens[0]):
            self._send_json({"error": "expected one nonempty capture_id"}, 400)
            return
        if name not in {"color.jpg", "depth.png"}:
            self._send_json({"error": "only color.jpg and depth.png are served"}, 404)
            return
        with BUFFER_LOCK:
            matches = [item for item in FRAME_CACHE if item["frame_id"] == frame_id
                       and (tokens is None or item.get("capture_id") == tokens[0])]
        if len(matches) > 1:
            self._send_json({"error": "ambiguous frame counter; use capture_id URL"}, 409)
            return
        frame = matches[0] if matches else None
        if frame is None:
            self._send_json({"error": f"frame {frame_id} not in cache"}, 404)
            return
        data = _frame_bytes(frame,"color_jpg" if name == "color.jpg" else "depth_raw_png")
        self._send_bytes(data, "image/jpeg" if name == "color.jpg" else "image/png")

    def _do_snapshot(self):
        with BUFFER_LOCK:
            if "color_jpg" not in BUFFER:
                self._send_json({"error": "no frame yet"}, 503)
                return
            frame=BUFFER.get('frame')
            color=BUFFER['color_jpg'];depth=BUFFER['depth_jpg'];raw=BUFFER['depth_raw_png']
            info=BUFFER['info']
        if frame is not None:
            color=_frame_bytes(frame,'color_jpg');depth=_frame_bytes(frame,'depth_jpg')
            raw=_frame_bytes(frame,'depth_raw_png')
        ts = time.strftime("%Y%m%d_%H%M%S")
        d = os.path.join(ARGS.snap_dir, ts)
        os.makedirs(d, exist_ok=True)
        paths = {
            "color.jpg": color,
            "depth.jpg": depth,
            "depth_raw.png": raw,
            "info.json": json.dumps(info, ensure_ascii=False, indent=1).encode("utf-8"),
        }
        for name, data in paths.items():
            with open(os.path.join(d, name), "wb") as f:
                f.write(data)
        self._send_json({"ok": True, "dir": d, "message": "saved: " + d,
                         "files": sorted(paths.keys())})

    def _send_snap_list(self):
        try:
            names = sorted(os.listdir(ARGS.snap_dir))
        except OSError:
            names = []
        files = []
        for n in names:
            p = os.path.join(ARGS.snap_dir, n)
            if os.path.isfile(p):
                files.append(n)
            else:
                files.extend(sorted(os.listdir(p)))
        self._send_json({"snapshots": names, "files": files})

    def _send_snap_file(self, rel):
        rel = os.path.normpath(rel).lstrip("/")
        fp = os.path.join(ARGS.snap_dir, rel)
        if not fp.startswith(os.path.realpath(ARGS.snap_dir)) or not os.path.isfile(fp):
            self._send_json({"error": "no such file"}, 404)
            return
        ext = rel.rsplit(".", 1)[-1].lower()
        ctype = {"jpg": "image/jpeg", "jpeg": "image/jpeg", "png": "image/png",
                 "json": "application/json"}.get(ext, "application/octet-stream")
        with open(fp, "rb") as f:
            self._send_bytes(f.read(), ctype)


def main():
    global ARGS
    ap = argparse.ArgumentParser(description="RealSense D435 RGB-D stream server")
    # 默认 1280x720：D435 满宽度分辨率，彩色约 69°x42°、深度约 87°x58° 的完整
    # 硬件视场都拿到，且比 640x480（中心裁剪只有约 55°）像素多、更清晰。需 USB3。
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--wide", action="store_true",
                    help="使用最大分辨率 1280x720 满视场模式（与 848x480 同样用满宽度视场约 69°；像素更多更清晰；需 USB3）")
    ap.add_argument("--fov-warn", action="store_true",
                    help="启动时若检测到分辨率导致可用视场明显偏小（<约65°）则告警打印")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--max-depth", type=float, default=6.0, help="depth colormap range (m)")
    ap.add_argument("--snap-dir", type=str,
                    default=os.path.expanduser("~/realsense_snapshots"))
    ap.add_argument("--encoding-mode", choices=("eager","on-demand"), default="eager",
                    help="on-demand retains full raw RGB-D frames and memoizes requested encodings")
    ap.add_argument("--verbose", action="store_true")
    ARGS = ap.parse_args()
    if ARGS.wide:
        ARGS.width, ARGS.height, ARGS.fps = 1280, 720, max(ARGS.fps, 15)
        print("[rs] --wide: 使用 1280x720 满分辨率/满视场模式（FOV 由镜头固定，不因分辨率变化）",
              flush=True)
    os.makedirs(ARGS.snap_dir, exist_ok=True)

    t = threading.Thread(target=rs_capture_loop, daemon=True)
    t.start()

    srv = ThreadingHTTPServer(("0.0.0.0", ARGS.port), Handler)
    print("[http] listening on 0.0.0.0:%d" % ARGS.port, flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
