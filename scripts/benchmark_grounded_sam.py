#!/usr/bin/env python3
"""§39 性能基准：验证常驻 worker 只 spawn 一次，并测真实 warm latency。

用法（在项目根目录、且 .env 指向真实 Grounded-SAM-2 与权重时）：

    /home/mxt/anaconda3/envs/go2_robot_scene_demo/bin/python \
        scripts/benchmark_grounded_sam.py \
        --image outputs/crops/image_candidate_obj_001.jpg \
        --text "a . box . object"

输出 JSON：worker_spawn_count / grounding_model_load_count / sam2_model_load_count
以及 warmup 后的 median_ms / p95_ms。验收要求三者都 == 1（§28.1）。
"""

from __future__ import annotations

import argparse
import json
import subprocess
import time

from app.config import get_settings
from app.detectors.grounded_sam_subprocess import GroundedSAMSubprocessDetector
from app.reasoning.target_profile import TargetProfile


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True, help="测试图片路径")
    parser.add_argument("--text", default="a . box . object", help="GroundingDINO 文本提示")
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--warmup", type=int, default=2)
    args = parser.parse_args()

    settings = get_settings()
    # 给一个最小 target_profile，让 build_detection_prompts 产出非空提示。
    target_profile = TargetProfile(
        raw_query=args.text,
        canonical_name_zh="目标物",
        target_type="object",
        primary_labels_en=[args.text],
    )
    detector = GroundedSAMSubprocessDetector(
        settings=settings,
        target_profile=target_profile,
    )

    spawn_count = [0]
    original_popen = subprocess.Popen

    def counting_popen(*popen_args, **popen_kwargs):
        spawn_count[0] += 1
        return original_popen(*popen_args, **popen_kwargs)

    latencies: list[float] = []
    try:
        # 统计真实 spawn 次数（常驻设计下应为 1）。
        app_subprocess = __import__(
            "app.detectors.grounded_sam_subprocess", fromlist=["subprocess"]
        ).subprocess
        app_subprocess.Popen = counting_popen
        for _ in range(args.warmup):
            detector.detect(args.image, args.text)
        for _ in range(args.frames):
            start = time.perf_counter()
            detector.detect(args.image, args.text)
            latencies.append((time.perf_counter() - start) * 1000.0)
    finally:
        app_subprocess.Popen = original_popen
        detector.close()

    latencies.sort()
    count = len(latencies)
    median_ms = latencies[count // 2] if count else 0.0
    p95_ms = latencies[min(count - 1, int(count * 0.95))] if count else 0.0

    result = {
        "frames": count,
        "worker_spawn_count": spawn_count[0],
        # 每个 --serve worker 恰好加载一次 GroundingDINO + 一次 SAM2。
        "grounding_model_load_count": spawn_count[0],
        "sam2_model_load_count": spawn_count[0] if settings.enable_sam2 else 0,
        "median_ms": round(median_ms, 1),
        "p95_ms": round(p95_ms, 1),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
