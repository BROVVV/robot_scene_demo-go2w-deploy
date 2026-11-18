#!/usr/bin/env python3
"""离线回放：把记录下来的真实帧喂进**真实的** `_verify_detect`，走本地机器狗 VLM。

这是接入前的验收工具（交接书「接入现有搜索流程」第 1 条：先离线回放验证）。
它不启 ROS、不调运动接口，只把「整帧 + 目标 bbox」交给
`app/detectors/siliconflow_vision_worker._verify_detect`，并打开
`GO2W_LOCAL_VLM_VERIFY` 那条旁路 —— 也就是说，**验证的是生产代码路径本身**，
不是一份重写的实现。

拿到的三样东西：

1. **接线正确性**：返回结构仍是 `object_name_zh / is_target / confidence /
   reason_zh`，且带上 `GO2W_LOCAL_VLM_VERIFY_IMAGES` 两种模式各跑一遍；
2. **与云端同输入的可比延迟**：提示词、图像集、`max_tokens` 都由
   `_verify_detect` 自己决定，所以与旧云端流程是同一条路径，只有 provider 不同；
3. **可恢复错误语义**：连不上机器狗时必须是异常上抛，而不是
   `is_target=False` —— 后者会被搜索读成「这一帧没有目标」。

用法（工作站，仓库 venv）::

    ssh -F /dev/null -N -L 18081:127.0.0.1:18081 \
        -i ~/.ssh/id_ed25519_unitree_robotscene unitree@192.168.123.18 &
    .venv/bin/python scripts/go2w/validation/replay_local_vlm_verify.py

**bbox 是归一化 (x1,y1,x2,y2)**，与 `_verify_detect` 的约定一致；本文件里的框
已逐个人工核对过确实框住了目标（见 reports/ 里的回放记录）。
"""
from __future__ import annotations

import argparse
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
DEFAULT_RESULTS = REPLAY_DIR / "replay_results.json"

# 每个用例：真实帧 + 人工核对过的 bbox + 期望判定 + 该样本的客观颜色（H/S/V，色相角）
# 颜色是**实测**的（色相角），不是感觉 —— 它决定了「蓝色桶」这类标签到底有多硬。
CASES = [
    {
        "name": "green_basket_full",
        "image": "color.jpg",
        "bbox": (0.820, 0.576, 0.906, 0.826),
        "expected": True,
        "hue_deg": 152.6,
        "saturation_pct": 22.1,
        "note": "浅绿藤编篮：真·目标，但饱和度仅 22%，偏灰",
    },
    {
        "name": "teal_bin_full",
        "image": "color.jpg",
        "bbox": (0.330, 0.520, 0.420, 0.730),
        "expected": False,
        "hue_deg": 186.7,
        "saturation_pct": 33.2,
        "note": "青绿实心垃圾桶：色相 187° 属 cyan，正是交接书所称的「蓝色桶」",
    },
    {
        "name": "blue_basket_full",
        "image": "color.jpg",
        "bbox": (0.120, 0.640, 0.265, 0.925),
        "expected": False,
        "hue_deg": 218.8,
        "saturation_pct": 56.0,
        "note": "藤编篮：画面里**真正**蓝的那个（219°），用作强反例对照",
    },
    {
        "name": "teal_bin_current",
        "image": "current.jpg",
        "bbox": (0.469, 0.562, 0.582, 0.806),
        "expected": False,
        "hue_deg": 190.1,
        "saturation_pct": 36.5,
        "note": "当前帧的同一个青绿桶，换机位复测",
    },
]


def _filter_import_error(exc: ModuleNotFoundError) -> str:
    return (
        f"缺少运行依赖（{exc}）。本脚本要用仓库 venv 跑："
        ".venv/bin/python scripts/go2w/validation/replay_local_vlm_verify.py"
    )


def run_case(settings: Settings, case: dict, index: int, repeat: int) -> dict:
    image_path = REPLAY_DIR / case["image"]
    bbox_text = ",".join(f"{value:.4f}" for value in case["bbox"])
    record = {
        "index": index,
        "case": case["name"],
        "mode": settings.local_vlm_verify_images,
        "repeat": repeat,
        "image": case["image"],
        "expected_target": case["expected"],
        "hue_deg": case["hue_deg"],
        "saturation_pct": case["saturation_pct"],
        "note": case["note"],
    }
    started = time.perf_counter()
    try:
        result = _verify_detect(settings, str(image_path), "绿色垃圾桶", bbox_text)
    except Exception as exc:  # noqa: BLE001
        record["elapsed_seconds"] = round(time.perf_counter() - started, 3)
        record["error"] = f"{type(exc).__name__}: {exc}"
        return record

    record["elapsed_seconds"] = round(time.perf_counter() - started, 3)
    record["result"] = result
    record["shape_ok"] = set(result) == {
        "object_name_zh", "is_target", "confidence", "reason_zh"
    }
    record["matches_expected"] = bool(result.get("is_target")) is case["expected"]
    return record


def _percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return round(ordered[0], 3)
    pos = (len(ordered) - 1) * q
    low = int(pos)
    high = min(low + 1, len(ordered) - 1)
    return round(ordered[low] * (1 - (pos - low)) + ordered[high] * (pos - low), 3)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:18081/v1",
                        help="本地 llama-server 的 OpenAI 兼容端点（经 SSH 本地转发）")
    parser.add_argument("--model", default="lfm-local")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--repeat", type=int, default=1,
                        help="每个 (用例, 模式) 重复次数，用于稳定态延迟分布")
    parser.add_argument("--modes", default="full+crop,crop",
                        help="逗号分隔：full+crop（与云端两图对齐）或 crop")
    parser.add_argument("--results", default=str(DEFAULT_RESULTS))
    args = parser.parse_args()

    missing = [case["image"] for case in CASES
               if not (REPLAY_DIR / case["image"]).is_file()]
    if missing:
        print(f"回放图像缺失：{sorted(set(missing))}（应在 {REPLAY_DIR}）", file=sys.stderr)
        return 2

    modes = [mode.strip() for mode in args.modes.split(",") if mode.strip()]
    report: dict = {
        "base_url": args.base_url,
        "model": args.model,
        "target": "绿色垃圾桶",
        "modes": modes,
        "repeat": args.repeat,
        "runs": [],
    }

    index = 0
    for mode in modes:
        settings = Settings(
            siliconflow_api_key="unused-cloud-key",
            local_vlm_verify_enabled=True,
            local_vlm_base_url=args.base_url,
            local_vlm_model=args.model,
            local_vlm_timeout_seconds=args.timeout,
            local_vlm_verify_images=mode,
        )
        for case in CASES:
            for repeat in range(args.repeat):
                # 第 0 次之后都是稳定态；印出来便于分辨预热
                tag = f"[{mode}] {case['name']}" + (f" #{repeat}" if repeat else "")
                try:
                    record = run_case(settings, case, index, repeat)
                except ModuleNotFoundError as exc:
                    print(_filter_import_error(exc), file=sys.stderr)
                    return 3
                report["runs"].append(record)
                if "error" in record:
                    print(f"{tag:44s} ERROR  {record['error'][:90]}", flush=True)
                else:
                    verdict = "命中" if record["matches_expected"] else "不符"
                    print(
                        f"{tag:44s} {record['elapsed_seconds']:7.2f}s  "
                        f"is_target={str(record['result']['is_target']):5s} "
                        f"conf={record['result']['confidence']:.2f}  {verdict}  "
                        f"| {record['result']['reason_zh'][:40]}",
                        flush=True,
                    )
                index += 1

    latencies = [r["elapsed_seconds"] for r in report["runs"] if "elapsed_seconds" in r]
    scored = [r for r in report["runs"] if "matches_expected" in r]
    report["summary"] = {
        "runs": len(report["runs"]),
        "errors": sum(1 for r in report["runs"] if "error" in r),
        "shape_violations": sum(1 for r in report["runs"] if r.get("shape_ok") is False),
        "matched": sum(1 for r in scored if r["matches_expected"]),
        "scored": len(scored),
        "latency_p50_seconds": _percentile(latencies, 0.50),
        "latency_p95_seconds": _percentile(latencies, 0.95),
        "latency_min_seconds": round(min(latencies), 3) if latencies else None,
        "latency_max_seconds": round(max(latencies), 3) if latencies else None,
        "by_mode": {
            mode: {
                "matched": sum(1 for r in scored
                               if r["mode"] == mode and r["matches_expected"]),
                "scored": sum(1 for r in scored if r["mode"] == mode),
                "latency_p50_seconds": _percentile(
                    [r["elapsed_seconds"] for r in report["runs"]
                     if r.get("mode") == mode and "elapsed_seconds" in r], 0.50),
            }
            for mode in modes
        },
    }
    Path(args.results).write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print("\n" + json.dumps(report["summary"], ensure_ascii=False, indent=2))
    print(f"\n写入 {args.results}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
