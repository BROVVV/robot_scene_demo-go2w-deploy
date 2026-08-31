#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
view_realsense.py — 本机（此电脑）查看机器狗上 RealSense D435 RGB-D 流的工具

用法（建议用项目 conda 环境，含 OpenCV）：
  /home/brov/miniconda3/envs/go2_robot_scene_demo/bin/python view_realsense.py \
      [--host 192.168.123.18] [--port 8080] [--out-dir outputs/realsense_d435/frames]

键盘操作：
  s  保存一帧快照（color.jpg + depth.jpg + depth_raw.png + info.json 到 out-dir）
  r  开始/停止录制（color.avi + depth.avi 同步记录）
  d  抓取一帧原始 16-bit 深度并打印统计
  q  退出

说明：两个窗口实时显示彩色画面与深度伪彩（jet），顶部叠加中心距离。
"""
import argparse
import json
import os
import sys
import time
import urllib.request

import cv2
import numpy as np


def fetch(url, timeout=5):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.123.18")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--out-dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "frames"))
    args = ap.parse_args()
    base = "http://%s:%d" % (args.host, args.port)
    os.makedirs(args.out_dir, exist_ok=True)

    # 打印设备/内参信息
    try:
        info = json.loads(fetch(base + "/info.json").decode("utf-8"))
        dev = info.get("device_info", {}).get("device", {})
        print("== RealSense D435 ==")
        print("  name : %s  serial: %s  fw: %s  usb: %s" % (
            dev.get("name"), dev.get("serial_number"),
            dev.get("firmware_version"), dev.get("usb_type")))
        ci = info.get("device_info", {}).get("color_intrinsics", {})
        di = info.get("device_info", {}).get("depth_intrinsics", {})
        print("  color: %dx%d fx=%.3f fy=%.3f ppx=%.3f ppy=%.3f" % (
            ci.get("width", 0), ci.get("height", 0), ci.get("fx", 0),
            ci.get("fy", 0), ci.get("ppx", 0), ci.get("ppy", 0)))
        print("  depth: %dx%d fx=%.3f fy=%.3f ppx=%.3f ppy=%.3f" % (
            di.get("width", 0), di.get("height", 0), di.get("fx", 0),
            di.get("fy", 0), di.get("ppx", 0), di.get("ppy", 0)))
    except Exception as e:
        print("warn: 无法读取 info.json: %s" % e)

    cap_c = cv2.VideoCapture(base + "/color")
    cap_d = cv2.VideoCapture(base + "/depth")
    if not cap_c.isOpened() or not cap_d.isOpened():
        print("ERROR: 无法打开 %s 的 MJPEG 流（服务是否运行？）" % base)
        sys.exit(1)
    print("实时画面已打开（彩色 + 深度），按 q 退出，s 存快照，r 录制，d 抓原始深度\n")

    rec = None
    rec_count = 0
    while True:
        ok_c, c = cap_c.read()
        ok_d, d = cap_d.read()
        if not ok_c or not ok_d:
            print("warn: 丢帧，继续...")
            continue

        # 叠加中心距离（从深度伪彩图顶部提取由服务端画好的文字，此处直接显示）
        h, w = d.shape[:2]
        cv2.putText(c, "COLOR %dx%d" % (c.shape[1], c.shape[0]), (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(d, "DEPTH (jet)  center: see overlay", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.imshow("RealSense D435 - COLOR", c)
        cv2.imshow("RealSense D435 - DEPTH", d)

        if rec is not None:
            rec[0].write(c)
            rec[1].write(d)
            rec_count += 1
            cv2.putText(c, "REC %d" % rec_count, (w - 90, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.imshow("RealSense D435 - COLOR", c)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            ts = time.strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(os.path.join(args.out_dir, "view_color_%s.jpg" % ts), c)
            cv2.imwrite(os.path.join(args.out_dir, "view_depth_%s.jpg" % ts), d)
            try:
                raw = fetch(base + "/depth_raw")
                fn = os.path.join(args.out_dir, "view_depth_raw_%s.png" % ts)
                with open(fn, "wb") as f:
                    f.write(raw)
                dz = cv2.imread(fn, cv2.IMREAD_UNCHANGED)
                if dz is not None:
                    valid = dz[dz > 0]
                    print("[snap] %s  depth p50=%.0fmm p95=%.0fmm valid=%d%%" % (
                        ts, np.percentile(valid, 50), np.percentile(valid, 95),
                        int(100 * (dz > 0).mean())))
            except Exception as e:
                print("[snap] %s (depth_raw 失败: %s)" % (ts, e))
            print("[snap] 已保存到 %s" % args.out_dir)
        elif key == ord("r"):
            if rec is None:
                ts = time.strftime("%Y%m%d_%H%M%S")
                fourcc = cv2.VideoWriter_fourcc(*"MJPG")
                rec = (cv2.VideoWriter(os.path.join(args.out_dir, "view_color_%s.avi" % ts), fourcc, 25, (c.shape[1], c.shape[0])),
                       cv2.VideoWriter(os.path.join(args.out_dir, "view_depth_%s.avi" % ts), fourcc, 25, (d.shape[1], d.shape[0])))
                rec_count = 0
                print("[rec] 开始录制")
            else:
                rec[0].release()
                rec[1].release()
                rec = None
                print("[rec] 停止录制，共 %d 帧" % rec_count)
        elif key == ord("d"):
            try:
                raw = fetch(base + "/depth_raw")
                fn = os.path.join(args.out_dir, "depth_raw_manual.png")
                with open(fn, "wb") as f:
                    f.write(raw)
                dz = cv2.imread(fn, cv2.IMREAD_UNCHANGED)
                valid = dz[dz > 0]
                print("[depth] %s: min=%.0f p50=%.0f p95=%.0f max=%.0f mm (有效 %d%%)" % (
                    fn, valid.min(), np.percentile(valid, 50),
                    np.percentile(valid, 95), valid.max(), int(100 * (dz > 0).mean())))
            except Exception as e:
                print("[depth] 失败: %s" % e)

    cap_c.release()
    cap_d.release()
    if rec:
        rec[0].release()
        rec[1].release()
    cv2.destroyAllWindows()
    print("bye")


if __name__ == "__main__":
    main()
