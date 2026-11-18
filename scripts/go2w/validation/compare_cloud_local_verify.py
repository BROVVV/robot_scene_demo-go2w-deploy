#!/usr/bin/env python3
"""同输入对照：把**完全相同的**图像与 bbox 分别喂给云端与本地 provider。

交接书要求「与旧云端流程同输入对比」。回放脚本 `replay_local_vlm_verify.py`
只跑本地；daemon 探测拿到的是云端。两者用的图不同（预制裁剪图 vs 整帧+bbox），
所以那种比法不算严格对照。

这里两条腿都走**同一个** `_verify_detect`：提示词、图像集、`max_tokens` 全由它
自己决定，唯一的自变量是 provider 开关。于是差值可以归因到"模型"本身。

用法::

    .venv/bin/python scripts/go2w/validation/compare_cloud_local_verify.py
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from app.config import Settings  # noqa: E402
from app.detectors.siliconflow_vision_worker import _verify_detect  # noqa: E402

REPLAY_DIR = PROJECT_ROOT / "runtime/go2w/local_vlm_replay"
TARGET = "绿色垃圾桶"

# bbox 取满幅 —— 图像本身就是裁剪图，与云端基线探测逐字同一输入
CASES = [
    ("green_bin_crop.jpg", True, 147.6, 23.3),
    ("blue_bin_crop.jpg", False, 174.5, 35.5),
]


def _load_dotenv() -> dict[str, str]:
    """手读 .env（不依赖 python-dotenv，系统解释器里没有）。"""
    values: dict[str, str] = {}
    env_path = PROJECT_ROOT / ".env"
    if not env_path.is_file():
        return values
    for line in env_path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        values[key.strip()] = value.strip().strip('"').strip("'")
    return values


def _run(settings: Settings, image: Path) -> dict:
    started = time.perf_counter()
    try:
        result = _verify_detect(settings, str(image), TARGET, "0.0000,0.0000,1.0000,1.0000")
    except Exception as exc:  # noqa: BLE001
        return {"elapsed_seconds": round(time.perf_counter() - started, 3),
                "error": f"{type(exc).__name__}: {exc}"}
    return {"elapsed_seconds": round(time.perf_counter() - started, 3), "result": result}


def main() -> int:
    env = _load_dotenv()
    api_key = env.get("SILICONFLOW_API_KEY", "")
    base_url = env.get("SILICONFLOW_BASE_URL", "https://api.siliconflow.cn/v1")
    vision_model = env.get("SILICONFLOW_VISION_MODEL", "")
    if not api_key:
        print("缺少 SILICONFLOW_API_KEY（在 .env 里）", file=sys.stderr)
        return 2
    # **必须显式给超时**：本脚本不把 .env 灌进 os.environ，`Settings` 于是回落到
    # 代码默认的 25 s，而云端延迟实测会漂到 58 s（早先只要 32 s）—— 那样每次
    # 对照都会以 APITimeoutError 收场，看起来像"云端挂了"，其实是本地把闸门
    # 设得比现实还紧。取 .env 的值与 180 s 的较大者，宁可等也别误判。
    try:
        env_timeout = float(env.get("SILICONFLOW_TIMEOUT_SECONDS") or 0)
    except ValueError:
        env_timeout = 0.0
    timeout = max(env_timeout, 180.0)
    print(f"云端 base_url={base_url} model={vision_model or '(settings 默认)'} timeout={timeout}s")

    modes = sys.argv[1:] or ["full+crop"]
    report: dict = {"target": TARGET, "cloud": {}, "local": {}, "modes": modes}

    for image_name, expected, hue, sat in CASES:
        image = REPLAY_DIR / image_name
        if not image.is_file():
            print(f"缺图 {image}", file=sys.stderr)
            return 2

        cloud_kwargs: dict = {"siliconflow_api_key": api_key,
                              "siliconflow_base_url": base_url,
                              "siliconflow_timeout_seconds": timeout}
        if vision_model:
            cloud_kwargs["siliconflow_vision_model"] = vision_model
        cloud_settings = Settings(**cloud_kwargs)
        cloud = _run(cloud_settings, image)
        report["cloud"][image_name] = cloud
        print(f"\n=== {image_name} (期望 is_target={expected}, 色相 {hue}°, S {sat}%) ===")
        print(f"  [cloud] {cloud['elapsed_seconds']:7.2f}s  {json.dumps(cloud.get('result', cloud.get('error')), ensure_ascii=False)}")

        for mode in modes:
            local_settings = Settings(
                siliconflow_api_key="unused-cloud-key",
                local_vlm_verify_enabled=True,
                local_vlm_base_url="http://127.0.0.1:18081/v1",
                local_vlm_model="lfm-local",
                local_vlm_timeout_seconds=180.0,
                local_vlm_verify_images=mode,
            )
            local = _run(local_settings, image)
            report["local"].setdefault(mode, {})[image_name] = local
            got = local.get("result", {}).get("is_target")
            verdict = "命中" if got is expected else "不符"
            print(f"  [local/{mode}] {local['elapsed_seconds']:7.2f}s  "
                  f"{json.dumps(local.get('result', local.get('error')), ensure_ascii=False)}  [{verdict}]")

    dest = REPLAY_DIR / "cloud_vs_local_same_input.json"
    dest.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n写入 {dest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
