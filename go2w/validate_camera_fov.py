#!/usr/bin/env python3
"""validate_camera_fov.py — 校验机器狗 D435 当前流模式是否用满硬件视场。

通过 HTTP 拉取 D435 服务的 /info.json / /fov，计算并打印当前彩色/深度视场
（HFOV/VFOV），与 D435 硬件标称值对比，给出“取景范围”建议。

关键结论（务必理解）：
  * D435 的镜头视场是硬件固定值（彩色约 69.4°x42.5°，深度约 87°x58°）。
  * SSH/改设置不能把镜头视场“拉宽”；只能保证非裁剪 + 满分辨率。
  * 想让机器人看到更大实际范围：调高相机/倾角、让机器狗转头扫视、换广角镜头。

用法：
  python3 scripts/go2w/validate_camera_fov.py [--base http://192.168.123.18:8080]
"""
from __future__ import annotations

import argparse
import json
import urllib.request

# D435 硬件满宽度视场（度）reference values
SPEC = {
    "color": {"h_deg": 69.4, "v_deg": 42.5},
    "depth": {"h_deg": 87.0, "v_deg": 58.0},
}
# 低于该 HFOV 说明明显处于低分辨率中心裁剪模式（如 640x480 只有约 55°）
COLOR_CROP_WARN_H_DEG = 65.0
DEPTH_CROP_WARN_H_DEG = 80.0


def fetch(base: str, path: str, timeout: float = 5.0) -> dict:
    url = base.rstrip("/") + path
    with urllib.request.urlopen(urllib.request.Request(url, headers={"Cache-Control": "no-cache"}),
                                timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def main() -> int:
    ap = argparse.ArgumentParser(description="Validate D435 stream FOV usage")
    ap.add_argument("--base", default="http://192.168.123.18:8080")
    args = ap.parse_args()

    try:
        info = fetch(args.base, "/info.json")
        fov = info.get("fov_deg") or {}
        color = fov.get("color") or {}
        depth = fov.get("depth") or {}
    except Exception as exc:  # noqa: BLE001
        print(f"FAIL: cannot reach D435 stream at {args.base}: {exc}")
        return 1

    print(f"== D435 当前流模式（{args.base}） ==")
    c = info.get("color") or {}
    d = info.get("depth") or {}
    print(f"  彩色 {c.get('width')}x{c.get('height')}@{c.get('fps')}fps  "
          f"HFOV={color.get('h_deg')}° VFOV={color.get('v_deg')}°")
    print(f"  深度 {d.get('width')}x{d.get('height')}@{d.get('fps')}fps  "
          f"HFOV={depth.get('h_deg')}° VFOV={depth.get('v_deg')}°")

    issues = []
    warns = []
    for key, label in (("color", "彩色"), ("depth", "深度")):
        actual = color if key == "color" else depth
        spec = SPEC[key]
        h = actual.get("h_deg")
        if h is None:
            issues.append(f"{label}: 无法取得 HFOV")
            continue
        warn_h = COLOR_CROP_WARN_H_DEG if key == "color" else DEPTH_CROP_WARN_H_DEG
        if h < warn_h:
            warns.append(
                f"{label} HFOV {h}° < 满宽度标称 {spec['h_deg']}°：当前是低分辨率中心裁剪模式"
                f"（如 640x480 只有约 55°）。切换到 848x480 / 1280x720 可把取景范围真正拉宽到约 69°。"
            )
    if warns:
        print("\n== 诊断：取景范围偏窄（可拉宽） ==")
        for msg in warns:
            print("  - " + msg)
        print("  - 修复：bash scripts/go2w/tune_d435_fov.sh wide   (或 mode 848 480 30)")
    else:
        print("\n当前模式已用满 D435 满宽度视场（彩色约69°/深度约87°）。")

    print("\n== 想“看得更多 / 更广”的可行办法 ==")
    print("  1) 保持满分辨率非裁剪模式（当前状态即如此；--wide 可用 1280x720 更清晰）。")
    print("  2) 让机器狗转头/换位扫视——不要只盯正前方（本仓库的语义搜索本身会多方向旋转扫视）。")
    print("  3) 调高相机或压低倾角，扩大近处地面可视范围。")
    print("  4) 硬件层面更换广角镜头/鱼眼相机（超出本次代码范围）。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
