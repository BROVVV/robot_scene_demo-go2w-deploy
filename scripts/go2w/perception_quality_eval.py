#!/usr/bin/env python3
"""阶段二质量评估集：**3 帧冻结素材 × 多个目标短语**（计划书 §5.1 / §5.2 / §3.2）。

## 它回答什么

计划书 §3.2 的门槛是「固定评估集上不得新增漏检、不得新增假'找到'、不得新增错误
关系」。要判这一条，就必须有一个**能复现**的评估集，而不是跑一次真机看感觉。

素材**只有** `artifacts/perception_profile_20261013/frames/` 那 3 帧（同一静止场景、
10:06 冻结）。目标短语逐条换 —— 这是用户 2026-10-13 拍板的口径。**不新增采集、
不连机器狗**：真机连续行走那一类如实标"未验收"，不伪造（计划书 §3.1.6）。

## 怎么跑

检测那一段**跑真代码**（`GroundedSAMSubprocessDetector` + 常驻 worker，与生产同一
条 IPC 路径），所以必须在**能上 GPU 的那个解释器**里跑：

    /home/mxt/anaconda3/envs/go2_robot_scene_demo/bin/python \\
      scripts/go2w/perception_quality_eval.py \\
      --output-dir artifacts/perception_optimization/20261013_phase2/quality_eval

判决层（角色分类 / 三态 / 复核候选）同样跑真代码 —— `app/perception/` 下的纯函数，
阶段二任务 2、3 落地的就是它们。**不重写一份对照实现**：变体之间差别只在开关取值。

## 两个变体

同一批检测产物喂给两个变体，**检测只跑一遍** —— 变体之间的差别是**下游判决**，
不是前向本身；分两次跑检测就又把 GPU 漂移混进了差异里。

* ``baseline``：阶段二之前的语义。`target_present = bool(detections)`，
  复核在**全量** detections 上取最高分。
* ``phase2``：任务 2 + 3 的语义。三态 + 只在 `target_candidates` 里选复核对象。

`GO2W_WEB_TARGET_PROFILE_USE_LLM` 一律取 false：画像走确定性 fallback，
不把一次网络调用混进可复现性里。哪几条短语的锚点词是**冻结画像**给的，
行里带 `resolver_source="frozen_eval_set"`，一眼看得出来。

## 产出

* ``quality.jsonl``：一行 = (变体 × 帧 × 短语)，带判决结果与判据字段。
* ``summary.json``：聚合指标 + 每个变体的验收结论。
* ``setup.json``：生效配置、git 版本、GPU 状态（§3.1.2 要求记**生效值**）。
* ``detections.jsonl``：原始检测产物，供复核与复算。

**不会覆盖旧证据**：目标目录非空时拒绝，除非显式 ``--force``。
`artifacts/perception_profile_20261013/` 是只读基线，本工具绝不往里写。
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from dataclasses import replace  # noqa: E402

from app.config import get_settings  # noqa: E402
from app.detectors.grounded_sam_subprocess import (  # noqa: E402
    GroundedSAMSubprocessDetector,
)
from app.perception.entity_roles import (  # noqa: E402
    VERIFY_SCOPE_ALL_DETECTIONS,
    VERIFY_SCOPE_TARGET_CANDIDATES,
    classify_roles,
    derive_target_state,
    select_candidate,
)
from app.perception.target_state import (  # noqa: E402
    TARGET_ABSENT,
    TARGET_PRESENT,
)
from app.reasoning.target_profile import TargetProfileResolver  # noqa: E402

FROZEN_DIR = ROOT / "artifacts" / "perception_profile_20261013"


def _load_benchmark():
    """复用阶段一的 `_round_order` / `_summarize` / `_gpu_state`。

    复制一份过来的话，"ABBA 顺序"和"分位数口径"就会有两套，报告里的数字
    与阶段一不可比 —— 而这正是**同一份**感知提速计划的两段。
    """
    path = ROOT / "scripts" / "go2w" / "perception_replay_benchmark.py"
    spec = importlib.util.spec_from_file_location("_phase1_benchmark", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


BENCH = _load_benchmark()


# --------------------------------------------------------------------------- #
# 评估集定义（冻结）
# --------------------------------------------------------------------------- #
# `expect` 是**预期**，不是答案：判据是"实际结果与预期是否一致"，不是"有没有
# 检出东西"。`absent` 那两条是 §3.2"不得新增假找到"的直接判据。
#
# `profile_override` 只给关系那条用：`use_llm=False` 的 fallback 解析器**不会**
# 产出 `relation_anchor_labels_en`（见 `_fallback_profile`），既然评估集要固定
# 画像、又要覆盖锚点词这条路，就把那条画像显式冻在这儿，而不是偷偷改成 use_llm。
PHRASES: list[dict] = [
    {
        "phrase": "绿色垃圾桶",
        "expect": "present",
        "note": "目标存在（基线）：冻结素材里真的有一个绿色垃圾桶。",
    },
    {
        "phrase": "红色灭火器",
        "expect": "absent",
        "note": "目标不存在：**不得**触发复核（§3.2 不得新增假'找到'）。",
    },
    {
        "phrase": "椅子",
        "expect": "present",
        "note": "同类多实例：高分椅子不得抢占复核（验任务 3）。",
    },
    {
        "phrase": "白色的椅子",
        "expect": "present",
        "note": "颜色属性：检测层只看类别，颜色的验收在复核层（见 derive_target_state 文档）。",
    },
    {
        "phrase": "垃圾桶旁边的椅子",
        "expect": "relation",
        "note": "关系锚点：relation_anchors 应含垃圾桶，target_candidates 应含椅子。",
        "profile_override": {
            "canonical_name_zh": "椅子",
            "primary_labels_en": ["chair"],
            "aliases_en": ["seat"],
            "relation_anchor_labels_en": ["trash bin", "garbage can"],
        },
    },
    {
        "phrase": "打印机",
        "expect": "absent",
        "note": "开放词表：词表外的物体，验画像解析与词表构造仍然闭合。",
    },
]

VARIANTS: dict[str, dict] = {
    "baseline": {
        "go2w_result_role_separation": False,
        "go2w_verify_candidate_scope": VERIFY_SCOPE_ALL_DETECTIONS,
    },
    "phase2": {
        "go2w_result_role_separation": True,
        "go2w_verify_candidate_scope": VERIFY_SCOPE_TARGET_CANDIDATES,
    },
}

# 冻结评估集一律关掉 LLM 解析：把一次网络调用混进可复现性里，就没有"复现"了。
EVAL_USE_LLM = False
TARGET_SCORE_MIN = 0.15


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--manifest", default=str(FROZEN_DIR / "capture.json"),
        help="帧清单（默认阶段一冻结的那份）",
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--frames-dir", default=str(FROZEN_DIR / "frames"))
    parser.add_argument("--variants", default="baseline,phase2")
    parser.add_argument("--phrases", default="", help="逗号分隔；不给 = 冻结全表")
    parser.add_argument(
        "--repeats", type=int, default=3,
        help="检测重复轮数（顺序 ABBA，用来给延迟一个可比口径）",
    )
    parser.add_argument("--order", choices=("alternate", "random"), default="alternate")
    parser.add_argument("--force", action="store_true")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="只解析画像与词表、不加载模型 —— 用来在没有 GPU 时验证评估集本身",
    )
    return parser.parse_args()


def _prepare_output_dir(path: Path, force: bool) -> None:
    if path.exists() and any(path.iterdir()) and not force:
        raise SystemExit(
            f"{path} 非空 —— 不覆盖旧证据（要覆盖请显式 --force）"
        )
    path.mkdir(parents=True, exist_ok=True)


def _sha256_file(path: Path) -> str:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return ""


def _detected_object_to_dict(obj) -> dict:
    """`DetectedObject` → 判决层消费的 dict。

    **为什么不 import 生产的 `_gsam_to_dict`**：它在 `run_autonomous_loop.py`
    里，那个模块顶层就 import rclpy，宿主 .venv 与 GPU 环境**都没有 rclpy**
    （实测），评估集根本跑不起来。字段集会不会漂移由
    `tests/test_perception_quality_eval.py` 里的 AST 守卫盯着：它直接比对
    本函数与生产那份的字段名集合，多一个少一个都变红。
    """
    return {
        "label": obj.label,
        "label_zh": obj.label_zh,
        "category": obj.category,
        "source_prompt_term": obj.source_prompt_term,
        "detection_slots": list(getattr(obj, "detection_slots", None) or ()),
        "score": obj.score,
        "text_score": getattr(obj, "text_score", None),
        "color": getattr(obj, "color", None),
        "attributes": list(getattr(obj, "attributes", None) or []),
        "bbox_2d": list(obj.bbox_2d),
        "mask_rle": obj.mask_rle,
        "mask_size": list(obj.mask_size) if obj.mask_size else None,
        "mask_area_ratio": obj.mask_area_ratio,
        "mask_source": getattr(obj, "mask_source", None),
        "source": getattr(obj, "source", "grounded_sam"),
    }


def _objects_sha256(objects: list[dict]) -> str:
    return hashlib.sha256(
        json.dumps(objects, ensure_ascii=False, sort_keys=True).encode("utf-8")
    ).hexdigest()


def _resolve_profiles(settings: Settings, phrases: list[dict]) -> dict[str, dict]:
    """解析每条短语的画像；冻结画像优先，并如实标注来源。

    返回值里 `profile` 是**真画像对象**，其余字段是它的可序列化摘要 ——
    检测那一段要把真对象交给检测器（任务 1 的接线：`_build_prompts` 从
    `detector.target_profile` 取锚点/上下文词），判决那一段也要真对象。
    """
    from app.reasoning.target_profile import TargetProfile

    resolver = TargetProfileResolver(settings=settings)
    out: dict[str, dict] = {}
    for entry in phrases:
        override = entry.get("profile_override")
        if override:
            profile = TargetProfile(
                raw_query=entry["phrase"], resolver_source="frozen_eval_set", **override
            )
        else:
            profile = resolver.resolve(entry["phrase"], use_llm=EVAL_USE_LLM)
        out[entry["phrase"]] = {
            "profile": profile,
            "canonical_name_zh": profile.canonical_name_zh,
            "resolver_source": profile.resolver_source,
            "resolver_error": profile.resolver_error,
            "detector_terms": profile.detector_terms(),
            "anchor_terms": list(profile.relation_anchor_labels_en),
            "context_terms": profile.context_terms(),
            "direct_terms": profile.direct_terms(),
        }
    return out


# --------------------------------------------------------------------------- #
# 判决层：同一批检测产物喂两个变体
# --------------------------------------------------------------------------- #
def _score(item: dict) -> float:
    try:
        value = float(item.get("score", 0.0) or 0.0)
    except (TypeError, ValueError):
        return 0.0
    return value if value == value else 0.0


def _decide_baseline(objects: list[dict], profile) -> dict:
    """阶段二**之前**的语义：二分 + 全量 argmax。

    刻意照抄那两处真实调用点的行为（`target_present = bool(detections)`；
    `best = max(detections, key=score)`），不是"我觉得旧版该是什么样"。
    """
    best = max(objects, key=_score) if objects else None
    return {
        "target_state": TARGET_PRESENT if objects else TARGET_ABSENT,
        "candidate_count": None,  # 旧路径没有"候选"这个概念
        "anchor_count": None,
        "scene_count": None,
        "verify_selected": best is not None,
        "verify_label": str(best.get("label", "")) if best else "",
        "verify_term": str(best.get("source_prompt_term", "")) if best else "",
        "verify_score": round(_score(best), 4) if best else 0.0,
    }


def _decide_phase2(objects: list[dict], profile, *, role_separation: bool,
                   verify_scope: str) -> dict:
    partition = classify_roles(objects, profile)
    state = derive_target_state(objects, profile, TARGET_SCORE_MIN)
    best, authoritative = select_candidate(
        partition.target_candidates,
        objects,
        scope=verify_scope,
        candidates_authoritative=role_separation,
    )
    return {
        "target_state": state,
        "candidate_count": len(partition.target_candidates),
        "anchor_count": len(partition.relation_anchors),
        "scene_count": len(partition.scene_objects),
        "verify_selected": best is not None,
        "verify_label": str(best.get("label", "")) if best else "",
        "verify_term": str(best.get("source_prompt_term", "")) if best else "",
        "verify_score": round(_score(best), 4) if best else 0.0,
        "verify_authoritative": bool(authoritative),
    }


def _label_is_target_term(row: dict, profile) -> bool:
    """选中的那个框，是不是由**目标词**（而不是场景词/锚点词）生成的。

    这就是任务 3 要修的东西：高分椅子抢占复核。
    """
    from app.perception.entity_roles import _matches_any_term, target_terms

    term = row.get("verify_term") or row.get("verify_label") or ""
    return bool(term) and _matches_any_term(str(term), target_terms(profile))


def _judge_row(row: dict, expect: str) -> dict:
    """把一行判决折算成 §3.2 的判据字段。"""
    out = {
        "false_trigger": bool(
            expect == "absent" and row["verify_selected"]
        ),
        "false_find": bool(
            expect == "absent" and row["target_state"] in {TARGET_PRESENT, "POSSIBLE"}
        ),
        "missed": bool(
            expect in {"present", "relation"} and not row["verify_selected"]
        ),
        "stolen_slot": bool(row["verify_selected"] and not row["verify_is_target_term"]),
    }
    return out


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #
def _run_detection(args, settings, frames, phrases, profiles, output_dir):
    """跑真检测。返回 `{(frame_id, phrase): (objects, timings)}`。"""
    detector = GroundedSAMSubprocessDetector(settings=settings)
    results: dict[tuple[str, str], dict] = {}
    try:
        # 顺序 ABBA：固定顺序会把 GPU 升温系统性地压在某一边（阶段一同款做法）。
        rounds = BENCH._round_order(args.order, ["forward", "reverse"], args.repeats)
        for names in rounds:
            ordered = frames if names[0] == "forward" else list(reversed(frames))
            for frame in ordered:
                fid = str(frame["frame_id"])
                # RAM++ **每一帧、每一轮**都要重问，不能跨帧缓存词表。
                #
                # 踩过的坑：`_ram_term_zh`（英文类别 → 中文配对的查找表）是**检测器
                # 实例上的状态**，`discover_scene_terms` 一调就整表覆盖。于是"这一帧
                # 的 `label_zh`"其实取决于**上一次 tag 的是哪一帧**。第一版评估集按帧
                # 缓存词表，正序跑与倒序跑就给出了不同的 `label_zh`，被可复现性断言
                # 当场抓住。生产里 tag 与 detect 是同帧紧挨着的（`_ram_scene_terms`
                # 就在 `_detect_grounded_sam` 里调），照它来才复现得出。
                ram_terms = detector.discover_scene_terms(frame["color_ref"])
                results.setdefault((fid, None), {}).setdefault(
                    "ram_terms", ram_terms
                )
                for entry in phrases:
                    phrase = entry["phrase"]
                    profile_row = profiles[phrase]
                    # 画像要**挂到检测器上**，不能只当 prompt 拼进去：任务 1 接线
                    # 的正是这一跳 —— `_build_prompts` 从 `detector.target_profile`
                    # 取锚点词与上下文词放进 prompt。第一版评估集只把
                    # `detector_terms` 拼成 prompt，于是锚点/上下文词**一个都没
                    # 进过词表**，"验证了关系锚点"就是空话（自己抓到的）。
                    # 逐条短语换画像等价于生产里逐会话换画像：一个实例、一个画像。
                    detector.target_profile = profile_row["profile"]
                    prompt = ". ".join(profile_row["detector_terms"])
                    start = time.perf_counter()
                    objects = detector.detect_with_dynamic_terms(
                        frame["color_ref"],
                        prompt,
                        profile_row["detector_terms"],
                        ram_terms,
                        frame_id=fid,
                    )
                    elapsed_ms = (time.perf_counter() - start) * 1000.0
                    slot = results.setdefault(
                        (fid, phrase), {"timings_ms": [], "object_hashes": []}
                    )
                    slot["timings_ms"].append(round(elapsed_ms, 3))
                    as_dicts = [_detected_object_to_dict(obj) for obj in objects]
                    slot["object_hashes"].append(_objects_sha256(as_dicts))
                    if "objects" not in slot:
                        slot["objects"] = as_dicts
    finally:
        detector.close()
    return results


def _cadence_timeline(*, cycles: int, period_s: float, **ttls) -> dict:
    from app.perception.detection_scheduler import (
        SLOT_ANCHOR,
        SLOT_SCENE,
        SLOT_TARGET,
        decide_detection_refresh,
    )

    counters = {SLOT_TARGET: 0, SLOT_ANCHOR: 0, SLOT_SCENE: 0}
    forward_cycles = 0
    idle_cycles = 0
    last_target = last_anchor = last_scene = None
    for step in range(cycles):
        now = step * period_s
        decision = decide_detection_refresh(
            now=now,
            last_target_s=last_target,
            last_anchor_s=last_anchor,
            last_scene_s=last_scene,
            place_id="p1", last_place_id="p1",
            sector=1, last_sector=1,
            translation_m=0.0,
            **ttls,
        )
        if decision.slots:
            forward_cycles += 1
        else:
            idle_cycles += 1
        for name in decision.slots:
            counters[name] += 1
        # 只有真跑了的槽才推进时间戳（与生产 `observe()` 的记账一致）。
        if SLOT_TARGET in decision.slots:
            last_target = now
        if SLOT_ANCHOR in decision.slots:
            last_anchor = now
        if SLOT_SCENE in decision.slots:
            last_scene = now
    return {
        "cycles": cycles,
        "forward_cycles": forward_cycles,
        "idle_cycles": idle_cycles,
        "slot_refreshes": dict(counters),
        "term_refreshes": sum(counters.values()),
    }


def _cadence_simulation(settings) -> dict:
    """分频在一个 1 Hz、静止 60 轮的时间轴上少跑了多少 —— **模拟**，不是墙钟实测。

    为什么要分开记：分频省的是 **prompt 里的词数**（一次前向的成本≈ backbone
    编码 + 按词数线性增长的对齐/解码），所以

    * `slot_refreshes` 才是省在哪儿的直接账（场景词 8 次 vs 60 次）；
    * `forward_cycles` 在默认 TTL 下**不会减少**（目标槽 TTL=0，每轮都刷）——
      把这两个数混成一个"前向次数"，就会得出"分频没用"的错误结论。

    素材只有 3 帧静止画面，真实收益来自"站着没动、朝向没变"的那些轮次，静止
    素材测不出来。**这一整块必须在报告里写成模拟**。
    """
    cycles = 60
    period_s = 1.0  # 1 Hz：生产 observe 的量级
    default_ttls = {
        "ttl_target_s": float(settings.go2w_target_refresh_interval_s),
        "ttl_anchor_s": float(settings.go2w_anchor_refresh_interval_s),
        "ttl_scene_s": float(settings.go2w_scene_refresh_interval_s),
        "scene_max_stale_s": float(settings.go2w_scene_max_stale_s),
    }
    off = _cadence_timeline(
        cycles=cycles, period_s=period_s,
        ttl_target_s=0.0, ttl_anchor_s=0.0,
        ttl_scene_s=0.0, scene_max_stale_s=default_ttls["scene_max_stale_s"],
    )
    on = _cadence_timeline(cycles=cycles, period_s=period_s, **default_ttls)
    return {
        "cadence_off": off,
        "cadence_on_default_ttls": on,
        "note": "模拟值：1 Hz 静止时间轴上的放行次数，非墙钟实测。",
    }


def _summarize_variant(rows: list[dict], variant: str) -> dict:
    subset = [row for row in rows if row["variant"] == variant]
    absent = [row for row in subset if row["expect"] == "absent"]
    positive = [row for row in subset if row["expect"] in {"present", "relation"}]
    selected = [row for row in subset if row["verify_selected"]]

    def _rate(hits: int, total: int) -> float:
        return round(hits / total, 4) if total else 0.0

    return {
        "variant": variant,
        "rows": len(subset),
        "positive_rows": len(positive),
        "absent_rows": len(absent),
        # §3.2 三条门槛
        "missed_positive_count": sum(1 for row in positive if row["missed"]),
        "false_find_count": sum(1 for row in absent if row["false_find"]),
        "false_trigger_count": sum(1 for row in absent if row["false_trigger"]),
        # 任务 3 的直接判据
        "verify_selected_count": len(selected),
        "stolen_slot_count": sum(1 for row in selected if row["stolen_slot"]),
        "verify_hits_target_term_rate": _rate(
            sum(1 for row in selected if row["verify_is_target_term"]), len(selected)
        ),
    }


def _verdict(summary: dict) -> dict:
    """按 §3.2 给"阶段二判决层是否可采纳"一个可追的结论。"""
    base = summary["baseline"]
    new = summary["phase2"]
    checks = {
        "no_new_false_find": new["false_find_count"] <= base["false_find_count"],
        "no_new_missed_positive": new["missed_positive_count"]
        <= base["missed_positive_count"],
        "no_new_false_trigger": new["false_trigger_count"]
        <= base["false_trigger_count"],
        "no_new_stolen_slot": new["stolen_slot_count"] <= base["stolen_slot_count"],
    }
    return {
        "checks": checks,
        "verdict": "adopt" if all(checks.values()) else "reject",
        "note": (
            "本判决只覆盖**判决层**（任务 2/3）。检测分频（任务 4）与多词分组"
            "（任务 6）另有各自的评估与开关，不在此列。"
        ),
    }


def main() -> int:
    args = _parse_args()
    output_dir = Path(args.output_dir)
    _prepare_output_dir(output_dir, args.force)

    frames = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
    frames = [entry["frame"] for entry in frames]

    wanted = [name.strip() for name in args.phrases.split(",") if name.strip()]
    phrases = [e for e in PHRASES if not wanted or e["phrase"] in wanted]
    if wanted and len(phrases) != len(wanted):
        missing = set(wanted) - {e["phrase"] for e in phrases}
        raise SystemExit(f"评估集里没有这些短语：{sorted(missing)}")

    variants = [name.strip() for name in args.variants.split(",") if name.strip()]
    unknown = set(variants) - set(VARIANTS)
    if unknown:
        raise SystemExit(f"未知变体：{sorted(unknown)}")

    # 用**生产那一份**生效配置（`get_settings()` 读 .env），不在这里另起一套默认值 ——
    # 评估集要量的是生产行为，不是本工具作者记忆里的默认值（§3.1.2 的同一条教训：
    # 默认值 ≠ 生效值）。只覆盖一处：把 key 清空。
    settings = replace(get_settings(), siliconflow_api_key="")
    profiles = _resolve_profiles(settings, phrases)

    (output_dir / "setup.json").write_text(
        json.dumps(
            {
                "manifest": str(Path(args.manifest).resolve()),
                "manifest_sha256": _sha256_file(Path(args.manifest)),
                "frames": [
                    {
                        "frame_id": str(f["frame_id"]),
                        "color_ref": f["color_ref"],
                        "color_sha256": _sha256_file(Path(f["color_ref"])),
                    }
                    for f in frames
                ],
                "phrases": [
                    {"phrase": e["phrase"], "expect": e["expect"], "note": e["note"]}
                    for e in phrases
                ],
                "profiles": {
                    phrase: {k: v for k, v in row.items() if k != "profile"}
                    for phrase, row in profiles.items()
                },
                "variants": {name: VARIANTS[name] for name in variants},
                "overrides": {
                    "siliconflow_api_key": "<清空：让评估集不依赖网络、可复现>",
                },
                "effective": {
                    "eval_use_llm": EVAL_USE_LLM,
                    "target_score_min": TARGET_SCORE_MIN,
                    "grounded_sam_root": settings.grounded_sam_root,
                    "grounded_sam_python": settings.grounded_sam_python,
                    "detection_device": settings.detection_device,
                    "grounding_dino_box_threshold": settings.grounding_dino_box_threshold,
                    "grounding_dino_text_threshold": settings.grounding_dino_text_threshold,
                    "grounding_prompt_llm_expansion_enabled": (
                        settings.grounding_prompt_llm_expansion_enabled
                    ),
                    "static_object_prompts_enabled": settings.static_object_prompts_enabled,
                    "enable_gdino_high_recall": settings.enable_gdino_high_recall,
                    "go2w_result_role_separation": settings.go2w_result_role_separation,
                    "go2w_verify_candidate_scope": settings.go2w_verify_candidate_scope,
                    "go2w_detection_cadence_enabled": settings.go2w_detection_cadence_enabled,
                    "go2w_target_refresh_interval_s": settings.go2w_target_refresh_interval_s,
                    "go2w_anchor_refresh_interval_s": settings.go2w_anchor_refresh_interval_s,
                    "go2w_scene_refresh_interval_s": settings.go2w_scene_refresh_interval_s,
                    "go2w_scene_max_stale_s": settings.go2w_scene_max_stale_s,
                },
                "gpu": BENCH._gpu_state(),
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    if args.dry_run:
        print(json.dumps(
            {p: {k: v for k, v in row.items() if k != "profile"}
             for p, row in profiles.items()},
            ensure_ascii=False, indent=2,
        ))
        print("[dry-run] 只解析了画像与词表，没有加载模型。")
        return 0

    detections = _run_detection(args, settings, frames, phrases, profiles, output_dir)

    rows: list[dict] = []
    detection_rows: list[dict] = []
    for (fid, phrase), slot in detections.items():
        if phrase is None:
            continue
        entry = next(e for e in phrases if e["phrase"] == phrase)
        objects = slot["objects"]
        # 复现性：同样的输入重复若干轮，检测产物必须逐字节相同。不相同就说明
        # 这个评估集**本身**不可复现，下面的指标全部作废 —— 直接抛，不给"看着
        # 差不多"的机会。
        if len(set(slot["object_hashes"])) > 1:
            raise SystemExit(
                f"检测产物跨轮不一致：frame={fid} phrase={phrase} "
                f"hashes={sorted(set(slot['object_hashes']))} —— "
                "评估集不可复现，指标无意义"
            )
        profile_module = profiles[phrase]["profile"]
        detection_rows.append({
            "frame_id": fid, "phrase": phrase,
            "object_count": len(objects),
            "latency_ms": BENCH._summarize(slot["timings_ms"]),
            "objects_sha256": slot["object_hashes"][0],
            "repeats": len(slot["timings_ms"]),
        })
        for variant in variants:
            config = VARIANTS[variant]
            if variant == "baseline":
                decision = _decide_baseline(objects, profile_module)
            else:
                decision = _decide_phase2(
                    objects, profile_module, **{
                        "role_separation": config["go2w_result_role_separation"],
                        "verify_scope": config["go2w_verify_candidate_scope"],
                    },
                )
            decision["verify_is_target_term"] = _label_is_target_term(
                decision, profile_module
            )
            row = {
                "variant": variant,
                "frame_id": fid,
                "phrase": phrase,
                "expect": entry["expect"],
                "resolver_source": profiles[phrase]["resolver_source"],
                "object_count": len(objects),
                "detect_p50_ms": BENCH._summarize(slot["timings_ms"]).get("p50", 0.0),
                **decision,
            }
            row["judgement"] = _judge_row(row, entry["expect"])
            row["missed"] = row["judgement"]["missed"]
            row["false_find"] = row["judgement"]["false_find"]
            row["false_trigger"] = row["judgement"]["false_trigger"]
            row["stolen_slot"] = row["judgement"]["stolen_slot"]
            rows.append(row)

    (output_dir / "quality.jsonl").write_text(
        "\n".join(json.dumps(row, ensure_ascii=False) for row in rows) + "\n",
        encoding="utf-8",
    )
    (output_dir / "detections.jsonl").write_text(
        "\n".join(json.dumps(row, ensure_ascii=False) for row in detection_rows) + "\n",
        encoding="utf-8",
    )

    summary = {
        name: _summarize_variant(rows, name) for name in variants
    }
    summary["cadence_simulation"] = _cadence_simulation(settings)
    summary["verdict"] = _verdict(summary)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
