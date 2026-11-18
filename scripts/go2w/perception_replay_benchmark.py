#!/usr/bin/env python3
"""感知提速计划（2026-10-13）阶段一的可重跑离线回放工具。

做两件事，而且**两条都跑生产代码**：

* ``inprocess``：直接驱动 ``app/detectors/grounded_sam_worker.py`` 的
  ``_tag`` / ``_detect``（函数链耗时、分项计时的口径）。
* ``subprocess``：驱动 ``GroundedSAMSubprocessDetector``——真正的常驻 worker
  + stdin/stdout JSONL IPC 路径。函数链变快不等于生产变快，这一条是判据。

对照方式：同一个进程、同一批帧、同一份**冻结词表**、同一组权重与阈值，只切换
阶段一的两个开关（``frame_feature_cache`` / ``mask_rle_encoder``）。
词表在 manifest 里冻结，是因为 RAM++ 每轮都会重跑，若让它决定 prompt，
"A 跑时词表 v1、B 跑时词表 v2"就会把词表差异算进实现差异里。

``--target`` 不给时按 manifest 冻结的目标走，再不行才用默认目标；显式给了一个
与清单不同的名字会在 ``setup.json`` 里记成 ``target_deviation``。所以拿本工具
自己写出的 ``manifest.json`` 复跑**不需要**再传目标。

**顺序偏差**：`inprocess` 模式按轮交替 AB（`--order` 还能随机化）；
`subprocess` 模式**一个实现一个常驻 worker**，为了不把两个 worker 的模型同时
压在显存里，实现是**顺序**跑的（先 A 全部、再 B 全部）—— 于是 GPU 温度/功耗的
线性漂移会混进两边的差里。做法是把 `--implementations` 反过来再跑一遍
（ABBA），两次的差值相加即抵消线性漂移；`timings.jsonl` 每条都带温度/功耗，
`ram_ms`（RAM++ 耗时，与本次改动无关）可以当漂移对照量。

用法（必须用**真实模型环境**的解释器）：

    /home/mxt/anaconda3/envs/go2_robot_scene_demo/bin/python \\
      scripts/go2w/perception_replay_benchmark.py \\
      --manifest artifacts/perception_profile_20261013/capture.json \\
      --output-dir artifacts/perception_optimization/phase1/replay_20261013 \\
      --modes inprocess,subprocess \\
      --implementations baseline,optimized \\
      --repeats 30 --warmup 2

**不会覆盖旧证据**：目标目录非空时直接拒绝，除非显式 ``--force``。
`artifacts/perception_profile_20261013/` 是本阶段的**只读**基线证据，任何一次
运行都不往那里写。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import random
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# 两个实现 = 同一份生产代码的两个开关组合。baseline 走的是**生产代码里的旧实现**
# （Python 逐像素 RLE + 每词重算 backbone），不是本文件另写的一份对照实现。
IMPLEMENTATIONS: dict[str, dict] = {
    "baseline": {"frame_feature_cache": False, "mask_rle_encoder": "python"},
    "optimized": {"frame_feature_cache": True, "mask_rle_encoder": "vector"},
}

# §5.1.6 的第二条对照轴：**多词分组前向**。与 IMPLEMENTATIONS 正交 ——
# 前一条轴动的是"同一个词跑多快"，这条轴动的是"一次跑几个词"。
# 字面量与 `app/detectors/grounded_sam_worker.py: PROMPT_MODES` 同值。
PROMPT_MODES = ("per_term", "grouped")

# 判"重复框"用的 IoU 门。定值、不做参数：它是"同一个物体的两次检出"的判据，
# 调它等于调结论。0.9 = 几乎完全重合。
DUPLICATE_IOU = 0.9


def _percentile(sorted_values: list[float], fraction: float) -> float:
    if not sorted_values:
        return 0.0
    index = min(len(sorted_values) - 1, int(round(fraction * (len(sorted_values) - 1))))
    return sorted_values[index]


def _summarize(values: list[float]) -> dict:
    if not values:
        return {"n": 0}
    ordered = sorted(values)
    return {
        "n": len(ordered),
        "mean": round(sum(ordered) / len(ordered), 3),
        "p50": round(_percentile(ordered, 0.50), 3),
        "p95": round(_percentile(ordered, 0.95), 3),
        "max": round(ordered[-1], 3),
        "min": round(ordered[0], 3),
    }


def _gpu_state() -> dict:
    """温度/功耗/显存。AB 交替顺序就是为了让这些漂移不系统性地压在某一边。"""
    try:
        raw = subprocess.run(
            ["nvidia-smi",
             "--query-gpu=temperature.gpu,power.draw,utilization.gpu,memory.used",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=10, check=True,
        ).stdout.strip().splitlines()[0]
        temperature, power, util, memory = [item.strip() for item in raw.split(",")]
        return {
            "temperature_c": float(temperature),
            "power_w": float(power),
            "utilization_pct": float(util),
            "memory_used_mib": float(memory),
        }
    except Exception:
        return {}


def _sha256(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _load_manifest(path: Path) -> tuple[list[dict], dict]:
    """读帧清单。返回 ``(frames, frozen)``。

    冻结词表非空时，prompt 用它而不是现问 RAM++ —— 复跑的意义就在于此：
    若每轮都让 RAM++ 重新决定词表，"实现 A 用词表 v1、实现 B 用词表 v2"会把
    词表差异算进实现差异里。RAM++ 照常每轮都跑（它是被测链路的一部分，
    `live_ram_tags` 用于发现漂移），只是**不参与** prompt 的构造。

    ``frozen`` 说明这份清单**冻结了什么**：

    * 现有的 `capture.json`（顶层是 list）：只冻结帧，没有目标、没有词表。
    * 本工具写出的 `manifest.json`（顶层是 dict）：目标（`target` /
      `target_terms` / `target_prompt`）与每帧的 RAM 词表一起冻结。

    两个非显然点（独立审查 R3）：

    * **空词表也是冻结**。以前用 `if item.get("ram_tags_en")` 判断，空列表是
      假值 ⇒ 被当成"没冻结"，复跑时又去问一次 RAM++，拿到什么算什么。合法的
      「这一帧就是没有场景词」因此悄悄换成了另一套词表。判据改成
      **键在不在**，值是什么（哪怕是 `[]`）一律照抄。
    * **目标必须一起冻结**。只传 `--manifest` 复跑时，目标会落到 CLI 默认值；
      对最初用别的目标（比如「红色杯子」）跑出来的清单，这会把目标静默换掉，
      词表整个变样，而"同一份词表"的对照承诺就没了。
    """
    payload = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(payload, list):
        return [entry["frame"] for entry in payload], {
            "has_vocabulary": False,
            "ram_tags": {},
            "target": None,
            "target_terms": None,
            "target_prompt": None,
        }
    frames = [
        {"frame_id": str(item["frame_id"]), "color_ref": item["color_ref"]}
        for item in payload["frames"]
    ]
    ram_tags = {
        str(item["frame_id"]): list(item.get("ram_tags_en") or [])
        for item in payload["frames"] if "ram_tags_en" in item
    }
    target_terms = payload.get("target_terms")
    return frames, {
        # 有任意一帧带 `ram_tags_en` 键 ⇒ 这份清单冻结了词表（哪怕每帧都是空）。
        "has_vocabulary": bool(ram_tags),
        "ram_tags": ram_tags,
        "target": payload.get("target") or None,
        "target_terms": [str(term) for term in target_terms] if target_terms else None,
        "target_prompt": payload.get("target_prompt") or None,
    }


def _prepare_output_dir(path: Path, force: bool) -> None:
    if path.exists() and any(path.iterdir()) and not force:
        raise SystemExit(
            f"输出目录非空：{path}\n"
            "本工具不会覆盖已有证据。换一个 --output-dir，或明确加 --force。"
        )
    path.mkdir(parents=True, exist_ok=True)
    (path / "objects").mkdir(exist_ok=True)


DEFAULT_TARGET = "绿色垃圾桶"


def resolve_target_and_vocabulary(cli_target: str | None, frozen: dict) -> dict:
    """把「用哪个目标、用哪份词表」解析成一条记录，并说明每一层的来源。

    抽成模块级纯函数是为了能被**直接跑**测到（独立审查 R3：要求"非默认目标、
    目标解析变化、空场景词表"三类复跑，而这些判据原本长在 `main()` 里，只能靠
    读源码来"验证"）。

    三层优先级与两个非显然点：

    * **目标**：CLI 显式 > 清单冻结 > `DEFAULT_TARGET`。`--target` 的默认值是
      `None` 而不是默认串，就是为了区分"用户没说"和"用户说了默认值" ——
      有默认串时这两件事在代码里长得一模一样。
    * **最终词表**：清单冻的优先于现场重算（那才是当时真正跑过的 prompt），
      但**只在生效目标与清单目标一致时**用 —— 换了目标还套旧词表自相矛盾。
    * **场景词表**：`frozen["has_vocabulary"]` 判的是"清单里有没有这个词"，
      不是"值真不真"。空列表 `[]` 是合法的「这一帧没有场景词」，必须照抄，
      否则复跑时又去问一次 RAM++，拿到什么算什么。
    """
    from app.reasoning.target_profile import TargetProfileResolver

    manifest_target = frozen["target"]
    target_explicit = cli_target is not None
    target_name = cli_target if target_explicit else (manifest_target or DEFAULT_TARGET)

    record: dict = {
        "target": target_name,
        "target_source": (
            "cli" if target_explicit else ("manifest" if manifest_target else "default")
        ),
    }
    if target_explicit and manifest_target and cli_target != manifest_target:
        # 显式换了目标就**记下来**：不然这份证据里的词表和清单里的对不上，
        # 而两份 JSON 看着都"正常"。
        record["target_deviation"] = {
            "manifest_target": manifest_target,
            "cli_target": cli_target,
        }

    target_terms = TargetProfileResolver().resolve(target_name, use_llm=False).detector_terms()
    target_prompt = ". ".join(target_terms)
    if frozen["target_terms"] and (
        manifest_target is None or target_name == manifest_target
    ):
        target_terms = list(frozen["target_terms"])
        target_prompt = frozen["target_prompt"] or ". ".join(target_terms)
        record["target_terms_source"] = "manifest"
    else:
        record["target_terms_source"] = "recomputed"
    record["target_terms"] = target_terms

    vocab: dict[str, list[str]] = (
        dict(frozen["ram_tags"]) if frozen["has_vocabulary"] else {}
    )
    record["vocabulary_source"] = "manifest" if frozen["has_vocabulary"] else "live RAM++"

    return {
        "record": record,
        "target_terms": target_terms,
        "target_prompt": target_prompt,
        "vocab": vocab,
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--manifest", default=str(
        ROOT / "artifacts" / "perception_profile_20261013" / "capture.json"
    ), help=(
        "帧清单。两种格式都收：(1) 现有的 capture.json（`[{\"frame\": {...}}, ...]`）；"
        "(2) 本工具自己写出的 manifest.json（含 frame_id/color_ref/ram_tags_en）—— "
        "后者会把词表也一起冻结，用于「同一批帧、同一份词表」的复跑。"
    ))
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--modes", default="inprocess,subprocess")
    parser.add_argument("--implementations", default="baseline,optimized",
                        help="顺序有含义：subprocess 模式按此顺序依次跑（先 A 全跑、"
                             "再 B 全跑），反过来再跑一遍就是 ABBA 漂移对照")
    parser.add_argument("--target", default=None, help=(
        f"目标名。不给时按 `--manifest` 冻结的目标，再不行才用默认值 "
        f"{DEFAULT_TARGET!r}。给了一个与清单冻结目标不同的名字，会在 "
        "setup.json 的 target_deviation 里记下来（不静默换目标）。"
    ))
    parser.add_argument("--repeats", type=int, default=30, help="每种配置的热态记录条数")
    parser.add_argument("--warmup", type=int, default=2)
    parser.add_argument("--max-frames", type=int, default=0, help="0 = 全部帧")
    parser.add_argument("--order", choices=("alternate", "random"), default="alternate")
    parser.add_argument("--profile", action="store_true", help="打开分项计时（会给每段插 CUDA 同步）")
    # 计划书 §5.1.6：多词分组前向的对照轴。默认只跑现行路径 —— 产物与阶段一
    # 逐字可比；加 `grouped` 就同时跑实验分支，`quality.json` 才有对照可算。
    parser.add_argument(
        "--prompt-modes", default="per_term",
        help="per_term（现行，默认）/ grouped（多词一次前向，token 跨度关联回词）。"
             "顺序有含义，与 --implementations 同理：反过来再跑一遍就是 ABBA 漂移对照",
    )
    # 分组模式下与逐词对齐用的 IoU 门。0.5 是检测评估的常用取值，不是调出来的：
    # 调它会同时改变"召回"和"串词"两个数，那就不再是同一把尺子。
    parser.add_argument("--quality-iou", type=float, default=0.5)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    output_dir = Path(args.output_dir).resolve()
    _prepare_output_dir(output_dir, args.force)
    modes = [item.strip() for item in args.modes.split(",") if item.strip()]
    implementations = [item.strip() for item in args.implementations.split(",") if item.strip()]
    for name in implementations:
        if name not in IMPLEMENTATIONS:
            raise SystemExit(f"未知实现 {name!r}，可选：{sorted(IMPLEMENTATIONS)}")
    prompt_modes = [item.strip() for item in args.prompt_modes.split(",") if item.strip()]
    for name in prompt_modes:
        if name not in PROMPT_MODES:
            raise SystemExit(f"未知 prompt 模式 {name!r}，可选：{list(PROMPT_MODES)}")

    from app.config import get_settings
    settings = get_settings()

    manifest_path = Path(args.manifest).resolve()
    frames, frozen = _load_manifest(manifest_path)
    if args.max_frames:
        frames = frames[: args.max_frames]

    record: dict = {
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "manifest": str(manifest_path),
        "frozen_vocabulary": bool(frozen["has_vocabulary"]),
        "output_dir": str(output_dir),
        "modes": modes,
        "implementations": {name: IMPLEMENTATIONS[name] for name in implementations},
        "prompt_modes": prompt_modes,
        # `target` 在下面按「CLI > manifest > 默认」解析后写入，不在这里填。
        "repeats": args.repeats,
        "warmup": args.warmup,
        "order": args.order,
        "profile": args.profile,
        "python": sys.executable,
        "platform": platform.platform(),
        "config": {
            key: getattr(settings, key) for key in (
                "detection_device", "grounding_dino_box_threshold",
                "grounding_dino_text_threshold", "sam2_min_box_score",
                "max_detected_objects", "enable_ram_plus", "ram_scene_max_terms",
                "enable_sam2", "sam2_config", "sam2_checkpoint",
                "enable_dino_frame_feature_cache", "mask_rle_encoder",
                "detection_profile_enabled",
            )
        },
    }

    resolved = resolve_target_and_vocabulary(args.target, frozen)
    record.update(resolved["record"])
    target_terms = resolved["target_terms"]
    target_prompt = resolved["target_prompt"]
    vocab = resolved["vocab"]

    timings: list[dict] = []
    objects_by_key: dict[tuple, str] = {}

    # ---- 模型只加载一次：三帧 30 轮重复不能把加载成本算进热态 ----
    if "inprocess" in modes:
        _run_inprocess(args, settings, frames, implementations, output_dir,
                       timings, objects_by_key, record, vocab, target_terms, target_prompt,
                       prompt_modes)
    if "subprocess" in modes:
        _run_subprocess(args, settings, frames, implementations, output_dir,
                        timings, objects_by_key, record, vocab, target_terms, target_prompt,
                        prompt_modes)

    (output_dir / "manifest.json").write_text(json.dumps({
        # 从 record 取，不从局部量取：目标的解析已经收进
        # `resolve_target_and_vocabulary()`，这里再留一个同名局部量就会出现
        # "两份都写着 target、只有一份是真的"。
        "target": record["target"],
        "target_terms": target_terms,
        "target_prompt": target_prompt,
        "frames": [
            {"frame_id": frame["frame_id"], "color_ref": frame["color_ref"],
             "ram_tags_en": vocab.get(frame["frame_id"], [])}
            for frame in frames
        ],
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    (output_dir / "setup.json").write_text(
        json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    (output_dir / "timings.jsonl").write_text(
        "".join(json.dumps(item, ensure_ascii=False) + "\n" for item in timings),
        encoding="utf-8",
    )
    (output_dir / "summary.json").write_text(
        json.dumps(_build_summary(timings, implementations, record),
                   ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    (output_dir / "equivalence.json").write_text(
        json.dumps(_build_equivalence(implementations, objects_by_key, timings),
                   ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    # §5.1.6 的质量对照。**只有两种模式都跑了才有对照**；只跑一种时不写空壳，
    # 免得下游把"没跑"读成"跑了没问题"。
    if len(prompt_modes) > 1:
        (output_dir / "quality.json").write_text(
            json.dumps(
                _build_quality(objects_by_key, prompt_modes=prompt_modes,
                               iou_threshold=args.quality_iou),
                ensure_ascii=False, indent=2,
            ),
            encoding="utf-8",
        )
    print("REPLAY_COMPLETE " + str(output_dir), flush=True)
    return 0


# ---------------------------------------------------------------------------
# inprocess：直接驱动 worker 的 _tag / _detect
# ---------------------------------------------------------------------------

def _run_inprocess(args, settings, frames, implementations, output_dir,
                   timings, objects_by_key, record, vocab, target_terms,
                   target_prompt, prompt_modes) -> None:
    runtime_root = Path(settings.grounded_sam_root)
    for path in [str(runtime_root), str(runtime_root / "grounding_dino"),
                 *filter(None, settings.grounded_sam_pythonpath.split(os.pathsep))]:
        if path not in sys.path:
            sys.path.insert(0, path)
    os.chdir(runtime_root)
    import numpy as np
    import torch
    from torchvision.ops import box_convert, nms
    from grounding_dino.groundingdino.util.inference import load_model, load_image, predict

    from app.detectors import grounded_sam_worker as worker
    from app.detectors.grounded_sam_subprocess import GroundedSAMSubprocessDetector
    from app.detectors.vocabulary import filter_ram_scene_term_pairs

    device = worker._select_device(settings.detection_device, torch)
    startup: dict = {}
    started = time.perf_counter()
    model = load_model(str(runtime_root / settings.grounding_dino_config),
                       str(runtime_root / settings.grounding_dino_checkpoint), device=device)
    _sync(device, torch)
    startup["grounding_model_load_ms"] = round((time.perf_counter() - started) * 1000, 3)

    build_args = SimpleNamespace(
        sam2_config=settings.sam2_config, sam2_checkpoint=settings.sam2_checkpoint,
        disable_sam2=not settings.enable_sam2,
        disable_ram_plus=not settings.enable_ram_plus,
        ram_root=settings.ram_plus_root, ram_checkpoint=settings.ram_plus_checkpoint,
    )
    started = time.perf_counter()
    sam = worker._build_sam2_predictor(runtime_root, build_args, device, torch)
    _sync(device, torch)
    startup["sam_model_load_ms"] = round((time.perf_counter() - started) * 1000, 3)
    started = time.perf_counter()
    ram, transform = worker._build_ram_plus(build_args, device, torch)
    _sync(device, torch)
    startup["ram_model_load_ms"] = round((time.perf_counter() - started) * 1000, 3)
    if sam is None or ram is None:
        raise SystemExit("RAM/SAM2 没加载起来，inprocess 模式没有意义")
    record["inprocess_startup_ms"] = startup
    print("REPLAY_STARTUP " + json.dumps(startup), flush=True)

    # 冻结词表：manifest 没带词表时，每帧只问一次 RAM++，之后所有实现都用这份词。
    for frame in frames:
        if frame["frame_id"] in vocab:
            continue
        payload = worker._tag(ram, transform, frame["color_ref"], device, torch)
        pairs = filter_ram_scene_term_pairs(
            payload["tags_en"], payload["tags_zh"], settings.ram_scene_max_terms
        )
        vocab[frame["frame_id"]] = [term for term, _ in pairs]

    prompt_builder = GroundedSAMSubprocessDetector(settings)  # 只借 _build_prompts，不启 worker
    entries = [{"frame_id": frame["frame_id"], "ram_tags_en": vocab[frame["frame_id"]]}
               for frame in frames]
    prompt_for = {}
    for frame, entry in zip(frames, entries):
        prompts = prompt_builder._build_prompts(target_prompt, target_terms, entry["ram_tags_en"])
        prompt_for[frame["frame_id"]] = " || ".join(prompts)
    record["prompts"] = {fid: text for fid, text in prompt_for.items()}

    for warmup_round in range(args.warmup):
        for frame, entry in zip(frames, entries):
            for name, prompt_mode in _combos(implementations, prompt_modes):
                _inprocess_once(args, settings, worker, model, sam, ram, transform,
                                device, torch, np, load_image, predict, box_convert, nms,
                                frame, entry, prompt_for[frame["frame_id"]], name,
                                prompt_mode, "inprocess", warmup_round, True, timings,
                                objects_by_key, output_dir)

    rounds = _round_combos(args.order, implementations, prompt_modes, args.repeats)
    for repeat in range(args.repeats):
        for frame, entry in zip(frames, entries):
            gpu = _gpu_state()
            for name, prompt_mode in rounds[repeat]:
                _inprocess_once(args, settings, worker, model, sam, ram, transform,
                                device, torch, np, load_image, predict, box_convert, nms,
                                frame, entry, prompt_for[frame["frame_id"]], name,
                                prompt_mode, "inprocess", repeat, False, timings,
                                objects_by_key, output_dir, gpu)


def _inprocess_once(args, settings, worker, model, sam, ram, transform, device, torch,
                    np, load_image, predict, box_convert, nms, frame, entry, prompt,
                    implementation, prompt_mode, mode, repeat, warmup, timings,
                    objects_by_key, output_dir, gpu=None) -> None:
    config = IMPLEMENTATIONS[implementation]
    stats: dict = {}
    _sync(device, torch)
    started = time.perf_counter()
    tags = worker._tag(ram, transform, frame["color_ref"], device, torch)
    _sync(device, torch)
    tag_ms = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    objects = worker._detect(
        SimpleNamespace(), frame["color_ref"], prompt,
        settings.grounding_dino_box_threshold, settings.grounding_dino_text_threshold,
        settings.max_detected_objects, settings.sam2_min_box_score,
        model, sam, device, torch, np, load_image, predict, box_convert, nms,
        options={
            "frame_feature_cache": config["frame_feature_cache"],
            "mask_rle_encoder": config["mask_rle_encoder"],
            "profile": args.profile,
            "prompt_mode": prompt_mode,
            "request_id": f"inprocess-{frame['frame_id']}-{implementation}-{repeat}",
        },
        stats=stats,
    )
    _sync(device, torch)
    detect_ms = (time.perf_counter() - started) * 1000.0

    encoded = json.dumps({"objects": objects}, ensure_ascii=False)
    key = (mode, frame["frame_id"], implementation, prompt_mode, repeat)
    objects_by_key[key] = encoded
    if not warmup:
        (output_dir / "objects"
         / f"{mode}_{frame['frame_id']}_{implementation}_{prompt_mode}_{repeat}.json"
         ).write_text(encoded, encoding="utf-8")
    timings.append({
        "mode": mode, "implementation": implementation,
        "prompt_mode": prompt_mode, "frame_id": frame["frame_id"],
        "repeat": repeat, "warmup": warmup, "order_index": 0, "gpu": gpu or {},
        "ram_ms": round(tag_ms, 3), "detect_ms": round(detect_ms, 3),
        "total_ms": round(tag_ms + detect_ms, 3),
        "object_count": len(objects),
        "mask_count": sum(item.get("mask_rle") is not None for item in objects),
        "prompt": prompt,
        "ram_tags": entry["ram_tags_en"],
        "live_ram_tags": tags.get("tags_en", []),
        "objects_sha256": _sha256(encoded),
        "diagnostics": stats,
    })


# ---------------------------------------------------------------------------
# subprocess：真实常驻 worker + IPC 路径
# ---------------------------------------------------------------------------

def _run_subprocess(args, settings, frames, implementations, output_dir,
                    timings, objects_by_key, record, vocab, target_terms,
                    target_prompt, prompt_modes) -> None:
    from dataclasses import replace
    from app.detectors.grounded_sam_subprocess import GroundedSAMSubprocessDetector

    for implementation in implementations:
        config = IMPLEMENTATIONS[implementation]
        # prompt 模式是**第二条轴**：与实现正交，所以两层都跑。顺序取
        # `--prompt-modes` 给的顺序（理由同 `--implementations`：反过来再跑
        # 一遍就是 ABBA 漂移对照）。一个模式一个常驻 worker —— 开关走请求体，
        # 共用一个 worker 也行，但分开才能拿到"冷启动"那一笔。
        for prompt_mode in prompt_modes:
            impl_settings = replace(
                settings,
                enable_dino_frame_feature_cache=config["frame_feature_cache"],
                mask_rle_encoder=config["mask_rle_encoder"],
                detection_profile_enabled=args.profile,
                go2w_dino_prompt_mode=prompt_mode,
            )
            detector = GroundedSAMSubprocessDetector(impl_settings)
            try:
                # 冷启动：worker 起进程 + 载三个模型 + 第一次检测。常驻 worker 的
                # 意义就在于这一笔只付一次；它同时是"worker 没有每帧重启"的证据。
                frame = frames[0]
                started = time.perf_counter()
                vocab.setdefault(frame["frame_id"], detector.discover_scene_terms(frame["color_ref"]))
                detector.detect_with_dynamic_terms(
                    frame["color_ref"], target_prompt, target_terms, vocab[frame["frame_id"]]
                )
                record.setdefault("subprocess_cold_ms", {})[
                    f"{implementation}/{prompt_mode}"
                ] = round((time.perf_counter() - started) * 1000.0, 3)

                for _ in range(args.warmup):
                    for frame in frames:
                        tags = detector.discover_scene_terms(frame["color_ref"])
                        vocab.setdefault(frame["frame_id"], tags)
                        detector.detect_with_dynamic_terms(
                            frame["color_ref"], target_prompt, target_terms,
                            vocab[frame["frame_id"]],
                        )
                for repeat in range(args.repeats):
                    for frame in frames:
                        gpu = _gpu_state()
                        started = time.perf_counter()
                        tags = detector.discover_scene_terms(frame["color_ref"])
                        tag_ms = (time.perf_counter() - started) * 1000.0
                        started = time.perf_counter()
                        objects = detector.detect_with_dynamic_terms(
                            frame["color_ref"], target_prompt, target_terms,
                            vocab[frame["frame_id"]],
                        )
                        detect_ms = (time.perf_counter() - started) * 1000.0
                        encoded = json.dumps(
                            {"objects": [item.__dict__ for item in objects]},
                            ensure_ascii=False, default=str,
                        )
                        objects_by_key[(
                            "subprocess", frame["frame_id"], implementation,
                            prompt_mode, repeat,
                        )] = encoded
                        (output_dir / "objects"
                         / f"subprocess_{frame['frame_id']}_{implementation}"
                           f"_{prompt_mode}_{repeat}.json"
                         ).write_text(encoded, encoding="utf-8")
                        prompt_text = " || ".join(detector._build_prompts(
                            target_prompt, target_terms, vocab[frame["frame_id"]]
                        ))
                        record.setdefault("prompts", {}).setdefault(
                            frame["frame_id"], prompt_text
                        )
                        timings.append({
                            "mode": "subprocess", "implementation": implementation,
                            "prompt_mode": prompt_mode,
                            "frame_id": frame["frame_id"], "repeat": repeat,
                            "warmup": False, "order_index": 0, "gpu": gpu,
                            "ram_ms": round(tag_ms, 3), "detect_ms": round(detect_ms, 3),
                            "total_ms": round(tag_ms + detect_ms, 3),
                            "object_count": len(objects),
                            "mask_count": sum(item.mask_rle is not None for item in objects),
                            "prompt": prompt_text,
                            "ram_tags": vocab[frame["frame_id"]],
                            "live_ram_tags": tags,
                            "objects_sha256": _sha256(encoded),
                            "diagnostics": detector.last_diagnostics,
                        })
            finally:
                detector.close()


# ---------------------------------------------------------------------------

def _round_order(order: str, implementations: list[str], repeats: int) -> list[list[str]]:
    """AB 交替/随机：固定顺序会把 GPU 升温、功耗爬升系统性地压在某一边上。"""
    rounds = []
    rng = random.Random(20261013)
    for index in range(repeats):
        names = list(implementations)
        if order == "random":
            rng.shuffle(names)
        elif index % 2 == 1:
            names.reverse()
        rounds.append(names)
    return rounds


def _combos(implementations: list[str], prompt_modes: list[str]) -> list[tuple[str, str]]:
    """两条轴的叉积：每项是 ``(实现, prompt 模式)``。

    外层是**实现**、内层是 prompt 模式 —— 与 `--implementations` 的书写顺序
    同理，顺序本身有含义（见 `_round_combos`）。
    """
    return [(name, mode) for name in implementations for mode in prompt_modes]


def _round_combos(order: str, implementations: list[str], prompt_modes: list[str],
                  repeats: int) -> list[list[tuple[str, str]]]:
    """`_round_order` 的两轴版：每轮给一份 ``(实现, prompt 模式)`` 的顺序。

    单一轴时它必须与 `_round_order` **给出同一个顺序** —— 否则加了这条轴之后，
    只跑一种 prompt 模式的复跑就和阶段一的证据不可比了（同一份数据、两个顺序）。
    """
    if len(prompt_modes) == 1:
        return [
            [(name, prompt_modes[0]) for name in names]
            for names in _round_order(order, implementations, repeats)
        ]
    combos = _combos(implementations, prompt_modes)
    rounds = []
    rng = random.Random(20261013)
    for index in range(repeats):
        names = list(combos)
        if order == "random":
            rng.shuffle(names)
        elif index % 2 == 1:
            names.reverse()
        rounds.append(names)
    return rounds


def _sync(device: str, torch) -> None:
    if str(device).startswith("cuda"):
        torch.cuda.synchronize()


def _build_summary(timings: list[dict], implementations: list[str], record: dict) -> dict:
    buckets: dict[tuple, list[float]] = defaultdict(list)
    per_frame: dict[tuple, list[float]] = defaultdict(list)
    for item in timings:
        if item["warmup"]:
            continue
        # 桶键里带 `prompt_mode`：不带的话，两条轴的数据会落进同一个桶，
        # 均值变成"逐词和分组混在一起"的数 —— 那正是本工具最不该产出的东西。
        buckets[(item["mode"], item["implementation"], item["prompt_mode"],
                 "detect")].append(item["detect_ms"])
        buckets[(item["mode"], item["implementation"], item["prompt_mode"],
                 "ram")].append(item["ram_ms"])
        buckets[(item["mode"], item["implementation"], item["prompt_mode"],
                 "total")].append(item["total_ms"])
        per_frame[(item["mode"], item["implementation"], item["prompt_mode"],
                   item["frame_id"])].append(item["detect_ms"])

    summary: dict = {"by_mode_and_implementation": {}, "by_frame": {},
                     "speedup": {}, "prompt_mode_delta": {}}
    for key, values in sorted(buckets.items()):
        summary["by_mode_and_implementation"]["/".join(key)] = _summarize(values)
    for key, values in sorted(per_frame.items()):
        summary["by_frame"]["/".join(key)] = _summarize(values)

    modes = sorted({item["mode"] for item in timings})
    prompt_modes = sorted({item["prompt_mode"] for item in timings})
    if "baseline" in implementations and "optimized" in implementations:
        for mode in modes:
            for prompt_mode in prompt_modes:
                base = buckets.get((mode, "baseline", prompt_mode, "detect"), [])
                fast = buckets.get((mode, "optimized", prompt_mode, "detect"), [])
                base_mean = sum(base) / len(base) if base else 0.0
                fast_mean = sum(fast) / len(fast) if fast else 0.0
                summary["speedup"][f"{mode}/{prompt_mode}/detect_mean"] = {
                    "baseline_mean_ms": round(base_mean, 3),
                    "optimized_mean_ms": round(fast_mean, 3),
                    "delta_ms": round(base_mean - fast_mean, 3),
                    "reduction_pct": (
                        round((1 - fast_mean / base_mean) * 100, 2) if base_mean else None
                    ),
                }
    # §5.1.6 的**延迟**那一项：同一个实现内部，逐词 vs 分组。与上面那段
    # （同模式内 baseline vs optimized）是两条正交的轴，不能合并成一个数。
    for mode in modes:
        for implementation in implementations:
            for reference in prompt_modes:
                for candidate in prompt_modes:
                    if reference == candidate:
                        continue
                    left = buckets.get((mode, implementation, reference, "detect"), [])
                    right = buckets.get((mode, implementation, candidate, "detect"), [])
                    if not left or not right:
                        continue
                    left_mean = sum(left) / len(left)
                    right_mean = sum(right) / len(right)
                    summary["prompt_mode_delta"][
                        f"{mode}/{implementation}/{reference}_vs_{candidate}"
                    ] = {
                        f"{reference}_mean_ms": round(left_mean, 3),
                        f"{candidate}_mean_ms": round(right_mean, 3),
                        "delta_ms": round(left_mean - right_mean, 3),
                        "reduction_pct": (
                            round((1 - right_mean / left_mean) * 100, 2)
                            if left_mean else None
                        ),
                    }
    summary["config"] = record.get("config", {})
    return summary


def _build_equivalence(implementations: list[str], objects_by_key: dict,
                       timings: list[dict]) -> dict:
    """固定输入上新旧路径的对象 JSON 对照。

    `objects_sha256` 不同时**不能**自动判失败：真实模型可能本身有非确定性。
    这里把差异原样报出来（逐条列出哪些 (帧, 轮次) 不同、分数/bbox 最大偏差），
    由报告给出结论 —— 而不是用一个大容差把它盖住。
    """
    result: dict = {"comparisons": {}, "note": (
        "哈希不同不等于实现有错：先看基线自己重复运行的差异（baseline_vs_baseline）"
        "再归因。对照只在**同一个 prompt 模式内**做 —— 跨模式比 per_term 与 "
        "grouped 的对象 JSON，问的不是等价性，那是 `quality.json` 的事。"
    )}
    modes = sorted({key[0] for key in objects_by_key})
    for mode in modes:
        # `key = (mode, frame_id, implementation, prompt_mode, repeat)`
        frame_ids = sorted({key[1] for key in objects_by_key if key[0] == mode})
        prompt_modes = sorted({key[3] for key in objects_by_key if key[0] == mode})
        entry = {"frames": {}, "equal_frames": 0, "compared_frames": 0,
                 "prompt_modes": prompt_modes}
        for frame_id in frame_ids:
            base_reps = sorted(key[4] for key in objects_by_key
                               if key[:2] == (mode, frame_id) and key[2] == "baseline")
            fast_reps = sorted(key[4] for key in objects_by_key
                               if key[:2] == (mode, frame_id) and key[2] == "optimized")
            pairs = []
            for repeat in sorted(set(base_reps) & set(fast_reps)):
                for prompt_mode in prompt_modes:
                    base = objects_by_key.get(
                        (mode, frame_id, "baseline", prompt_mode, repeat)
                    )
                    fast = objects_by_key.get(
                        (mode, frame_id, "optimized", prompt_mode, repeat)
                    )
                    if base is None or fast is None:
                        continue
                    pairs.append({
                        "repeat": repeat,
                        "prompt_mode": prompt_mode,
                        "identical": base == fast,
                        "baseline_sha256": _sha256(base),
                        "optimized_sha256": _sha256(fast),
                    })
            baseline_self = [
                _sha256(objects_by_key[key]) for key in sorted(objects_by_key)
                if key[0] == mode and key[1] == frame_id and key[2] == "baseline"
            ]
            entry["frames"][frame_id] = {
                "pairs": pairs,
                "all_identical": all(pair["identical"] for pair in pairs) if pairs else None,
                "baseline_self_consistent": len(set(baseline_self)) <= 1,
                "distinct_baseline_hashes": len(set(baseline_self)),
            }
            if pairs:
                entry["compared_frames"] += 1
                if entry["frames"][frame_id]["all_identical"]:
                    entry["equal_frames"] += 1
        result["comparisons"][mode] = entry

    # 词表必须逐字相同：preprocess 尺寸、权重、阈值、prompt 有一处不同，
    # 上面那些"对象 JSON 相同"就不再是实现等价性的证据。
    #
    # **按 prompt 模式分支**（§5.1.6）：分组模式下送去 DINO 的 caption 与逐词
    # **本就不一样**（一条多词 caption vs N 条单词 caption），所以拿分组行去
    # 套旧断言会把它判死。这里做两件事：
    #   1. `prompt_identical` 的键加上 `prompt_mode` —— 同一模式内仍是"词表
    #      必须逐字相同"那条强断言，一行都不放松；
    #   2. 另外报一个跨模式的信息项（不是断言）：各模式的**输入 prompt 文本**
    #      是否相同。它相同、而实际 caption 不同，正是本实验要说明的那件事。
    prompts: dict[tuple, set] = defaultdict(set)
    across: dict[tuple, set] = defaultdict(set)
    for item in timings:
        if item["warmup"]:
            continue
        prompts[(item["mode"], item["prompt_mode"], item["frame_id"])].add(
            item.get("prompt", "")
        )
        prompts[("ALL", item["prompt_mode"], item["frame_id"])].add(
            item.get("prompt", "")
        )
        across[(item["mode"], item["frame_id"])].add(
            (item["prompt_mode"], item.get("prompt", ""))
        )
    result["prompt_identical"] = {
        f"{mode}/{prompt_mode}/{frame_id}": (len(values) == 1)
        for (mode, prompt_mode, frame_id), values in sorted(prompts.items())
    }
    result["all_prompts_identical"] = all(result["prompt_identical"].values())
    result["prompt_text_same_across_prompt_modes"] = {
        f"{mode}/{frame_id}": len({text for _, text in values}) == 1
        for (mode, frame_id), values in sorted(across.items())
    }
    result["prompt_mode_note"] = (
        "`all_prompts_identical` 是**同一 prompt 模式内**的断言（输入词表必须逐字"
        "相同），分组模式下同样成立、也同样是必须的。跨模式那条"
        "（`prompt_text_same_across_prompt_modes`）只是信息项：输入文本相同而"
        "实际发给 DINO 的 caption 条数不同，是 §5.1.6 实验的定义，不是缺陷。"
    )
    return result


# ---------------------------------------------------------------------------
# §5.1.6 质量对照：召回 / 串词 / 重复框 / 分数变化
#
# 全部是纯函数，输入是 `objects_by_key` 里的 JSON 文本。抽出来单测的理由和
# `resolve_target_and_vocabulary` 一样：真实回放要上 GPU、要几十分钟，而指标
# 算法本身对不对（IoU 配对是不是一对一、串词到底怎么算）必须能**直接跑**。
# ---------------------------------------------------------------------------

def _box_iou(left, right) -> float:
    """两个归一化 xyxy 框的 IoU。本地实现，不 import `app.video` —— 那个模块
    会把视频链的依赖拖进这个纯离线工具，而这里只需要六行算术。"""
    if not left or not right or len(left) < 4 or len(right) < 4:
        return 0.0
    lx1, ly1, lx2, ly2 = (float(value) for value in left[:4])
    rx1, ry1, rx2, ry2 = (float(value) for value in right[:4])
    inter_w = min(lx2, rx2) - max(lx1, rx1)
    inter_h = min(ly2, ry2) - max(ly1, ry1)
    if inter_w <= 0 or inter_h <= 0:
        return 0.0
    inter = inter_w * inter_h
    left_area = max(0.0, lx2 - lx1) * max(0.0, ly2 - ly1)
    right_area = max(0.0, rx2 - rx1) * max(0.0, ry2 - ry1)
    union = left_area + right_area - inter
    return inter / union if union > 0 else 0.0


def _boxes_from_payload(encoded: str) -> list[dict]:
    """把一条对象 JSON 解成 ``[{term, score, bbox}]``。

    `term` 取 `source_prompt_term` 优先 —— 它才是"这个框是哪个词生成的"。
    `label` 是同一份值在经过中文映射**之前**的形态，两者在今天的实现里相等，
    但一旦下游给 `label` 加了映射，拿它比就会把"映射一样"当成"词一样"。
    两种模式都走同一条解析，所以模式之间的差不是解析差异。
    """
    try:
        payload = json.loads(encoded)
    except (TypeError, ValueError):
        return []
    items = payload.get("objects") if isinstance(payload, dict) else payload
    boxes = []
    for item in items or []:
        if not isinstance(item, dict):
            continue
        term = item.get("source_prompt_term") or item.get("label") or "object"
        score = item.get("score", item.get("confidence", 0.0))
        try:
            score = float(score)
        except (TypeError, ValueError):
            score = 0.0
        boxes.append({
            "term": str(term).strip().lower(),
            "score": score,
            "bbox": list(item.get("bbox_2d") or item.get("bbox") or []),
        })
    return boxes


def match_boxes(reference: list[dict], candidate: list[dict], *,
                iou_threshold: float = 0.5) -> dict:
    """把候选框一对一配到参照框上，返回计数与明细。

    **一对一**是关键：贪心按候选分数从高到低取，每个参照框至多被配上一次。
    允许多对一的话，一个物体被检出两次就能把召回刷到 1.0 —— 那是重复框，
    不是召回。重复框单列一项（`duplicates`）：候选框配到一个已被配走的参照框、
    且 IoU 仍 ≥ `duplicate_iou` 时计数。
    """
    matches: list[dict] = []
    used: set[int] = set()
    duplicates = 0
    for box in sorted(candidate, key=lambda item: item["score"], reverse=True):
        best_index, best_iou = -1, 0.0
        for index, other in enumerate(reference):
            score = _box_iou(box["bbox"], other["bbox"])
            if score > best_iou:
                best_index, best_iou = index, score
        if best_iou < iou_threshold:
            continue
        if best_index in used:
            # 同一个参照框被第二个候选框再配上一次。门更严一些才算重复：
            # 0.5 那一档只说明"对着同一个物体"，0.9 才说明"这是同一个检出"。
            if best_iou >= DUPLICATE_IOU:
                duplicates += 1
            continue
        used.add(best_index)
        other = reference[best_index]
        matches.append({
            "candidate_term": box["term"],
            "reference_term": other["term"],
            "iou": round(best_iou, 4),
            "label_flip": box["term"] != other["term"],
            "score_delta": round(abs(box["score"] - other["score"]), 4),
        })
    return {
        "matches": matches,
        "matched": len(matches),
        "reference_total": len(reference),
        "candidate_total": len(candidate),
        # 召回：几何配上**且词一致**。这才是"同一个东西还认得出是它"。
        "recall": _ratio(
            sum(1 for item in matches if not item["label_flip"]), len(reference)
        ),
        # 只按几何配上的比例。与上面那个的**差**就是串词吃掉的召回。
        "geometry_recall": _ratio(len(matches), len(reference)),
        "missed": len(reference) - len(matches),
        "extra": len(candidate) - len(matches),
        "label_flips": sum(1 for item in matches if item["label_flip"]),
        "duplicates": duplicates,
        "score_delta_max": max((item["score_delta"] for item in matches), default=0.0),
        "score_delta_mean": (
            round(sum(item["score_delta"] for item in matches) / len(matches), 4)
            if matches else 0.0
        ),
    }


def _ratio(numerator: int, denominator: int) -> float | None:
    """0/0 报 ``None``，不报 0.0 ——"一帧里本来就没框"和"一个都没配上"
    是两件事，写成一个数就分不开了。"""
    return round(numerator / denominator, 4) if denominator else None


def _label_counts(boxes: list[dict]) -> dict:
    counts: dict[str, int] = defaultdict(int)
    for box in boxes:
        counts[box["term"]] += 1
    return dict(sorted(counts.items()))


def _aggregate_quality(rows: list[dict]) -> dict:
    """把逐轮的行合成一份。比率为 None 的行（参照那边一个框都没有）不参与均值。"""
    def _mean(key: str) -> float | None:
        values = [row[key] for row in rows if row.get(key) is not None]
        return round(sum(values) / len(values), 4) if values else None

    return {
        "repeats": len(rows),
        "reference_total": sum(row["reference_total"] for row in rows),
        "candidate_total": sum(row["candidate_total"] for row in rows),
        "matched": sum(row["matched"] for row in rows),
        "missed": sum(row["missed"] for row in rows),
        "extra": sum(row["extra"] for row in rows),
        "label_flips": sum(row["label_flips"] for row in rows),
        "duplicates": sum(row["duplicates"] for row in rows),
        "recall": _mean("recall"),
        "geometry_recall": _mean("geometry_recall"),
        "score_delta_max": max((row["score_delta_max"] for row in rows), default=0.0),
        "score_delta_mean": _mean("score_delta_mean"),
    }


def _build_quality(objects_by_key: dict, *, prompt_modes: list[str],
                   iou_threshold: float = 0.5,
                   reference_prompt_mode: str = "per_term") -> dict:
    """§5.1.6 的五项质量指标：召回 / 串词 / 重复框 / 分数变化 / 延迟。

    参照是**逐词**那一侧（`reference_prompt_mode`），因为现行路径就是它 ——
    分组是候选，要证明的是"它没有把逐词已经检到的东西弄丢或弄错"。

    **逐轮配对**：第 N 轮的分组结果只跟第 N 轮的逐词结果比。真实模型自己有
    抖动，把同一模式的 30 轮先平均掉再比，会把"两个模式其实一致"和"两边各自
    抖得一样多"混在一起。参照侧对不齐的轮次直接跳过并计数（`skipped_pairs`），
    不静默按 0 计。

    延迟那一项不在这里算 —— 它在 `summary.json` 的 `prompt_mode_delta` 里，
    数据源是 timings 而不是对象 JSON。两处都算会长出两个数。
    """
    others = [mode for mode in prompt_modes if mode != reference_prompt_mode]
    by_frame: dict[str, dict] = {}
    skipped = 0
    total_rows: dict[str, list[dict]] = defaultdict(list)

    frames = sorted({key[1] for key in objects_by_key})
    modes = sorted({key[0] for key in objects_by_key})
    implementations = sorted({key[2] for key in objects_by_key})
    repeats = sorted({key[4] for key in objects_by_key})

    for mode in modes:
        for implementation in implementations:
            for candidate_mode in others:
                for frame_id in frames:
                    rows = []
                    for repeat in repeats:
                        reference_key = (
                            mode, frame_id, implementation,
                            reference_prompt_mode, repeat,
                        )
                        candidate_key = (
                            mode, frame_id, implementation, candidate_mode, repeat,
                        )
                        if reference_key not in objects_by_key or candidate_key not in objects_by_key:
                            skipped += 1
                            continue
                        reference = _boxes_from_payload(objects_by_key[reference_key])
                        candidate = _boxes_from_payload(objects_by_key[candidate_key])
                        row = match_boxes(
                            reference, candidate, iou_threshold=iou_threshold
                        )
                        row["repeat"] = repeat
                        row["reference_labels"] = _label_counts(reference)
                        row["candidate_labels"] = _label_counts(candidate)
                        rows.append(row)
                    if not rows:
                        continue
                    name = f"{mode}/{implementation}/{frame_id}"
                    by_frame[name] = {
                        "reference_prompt_mode": reference_prompt_mode,
                        "candidate_prompt_mode": candidate_mode,
                        "rows": rows,
                        "aggregate": _aggregate_quality(rows),
                    }
                    total_rows[candidate_mode].extend(rows)

    totals = {
        candidate_mode: _aggregate_quality(rows)
        for candidate_mode, rows in sorted(total_rows.items())
    }
    # 结论只从 totals 出，不从某一帧出：3 帧的评估集本来就撑不起逐帧结论
    # （计划书 §3.2 的限制一节要如实写这一条）。
    verdicts = {}
    for candidate_mode, total in totals.items():
        verdicts[candidate_mode] = {
            "missed": total["missed"],
            "label_flips": total["label_flips"],
            "duplicates": total["duplicates"],
            "adopt": (
                total["missed"] == 0
                and total["label_flips"] == 0
                and total["duplicates"] == 0
            ),
        }
    return {
        "reference_prompt_mode": reference_prompt_mode,
        "iou_threshold": iou_threshold,
        "duplicate_iou": DUPLICATE_IOU,
        "skipped_pairs": skipped,
        "by_frame": by_frame,
        "totals": totals,
        "verdict": verdicts,
        "note": (
            "`recall` = 几何配上**且词一致**；`geometry_recall` = 只按几何配上。"
            "两者之差就是串词吃掉的召回。`extra` 是分组多检出来的框 —— 本工具"
            "**不**把 extra 当罪证：分组一次前向，检到逐词没检到的框恰好说明"
            "它可能更强；要判它是不是假阳，得看下游复核（`quality_eval` 那一套）。"
        ),
    }


if __name__ == "__main__":
    raise SystemExit(main())
