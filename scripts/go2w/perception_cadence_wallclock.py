#!/usr/bin/env python3
"""阶段二 §5.1.4（审查 P2）的**墙钟实测**：无运动端到端回放。

## 为什么必须重做一遍测量

`quality_eval/summary.json` 里的 `cadence_simulation` 是**释放计数模拟**：它
假设"每轮恰好 1 秒"，于是"场景槽 60 次 → 15 次"看着漂亮，却答不上审查问的
三件事：

1. **前台 + 后台总 DINO 前向次数**。后台 Full Semantic（`analyze_semantic`）
   **故意**要全槽（`slots=None`，见入口里那段 owner 冲突的表态），而前台分频
   后走的是**子集键**，两者在 `_detect_cache` 里不共享条目 ⇒ 分频开着时后台
   那一次从"命中缓存"变成"真打一次前向"。**只数前台就会把这笔加回来的量
   漏掉**，得出"分频白干"或"分频大赚"两个相反的错结论。
2. **场景槽的刷新周期与"每轮还有没有场景物体"**。*这是检测列表里的事，不是地图
   记账*：`scene_objects` 数的是组装出来的列表里 `source_prompt_term` 命中 RAM
   词表的条数。它答的是"分频开着时这一轮的观测里场景物体还在不在"，**不**答
   "地图收到了什么"—— 地图那条链（`spatial_memory` / `semantic_object_gate`）
   本工具一次都没跑（`ReplayNode` 只实现 quick 段用到的那几个方法）。
   报告里不许把它写成"场景对象入账"。
3. **真实墙上时钟**。一次前向在这台机器上要几秒是物理事实，不能拿 1 Hz 当
   时间轴。把时间轴换成实测耗时之后，"每分钟跑了几轮"才是真的。
4. **组装层的槽归属缺陷（补充复审必修 1 的指纹）**。跨槽组装时，被留下用于
   `scene` 的旧全量条目**不能**把它那一帧的 `target` 框继续算成目标候选。
   `stale_target_candidates` 数的是"命中目标词、却盖着**别的帧**的来源戳、
   还留在目标候选池里"的框 —— 走的是真 `classify_roles` + 真 `select_candidate`，
   不是照词表数一遍。**只看 `mixed_frames` 事件不够**：它只说明"观测混了帧"，
   不说明"混进来的旧框被当成目标用了"，后者才是缺陷。
5. **默认开关组合下的锁框（第三次补充复审）**。上面那条指纹量的是**候选视图**，
   而默认配置下复核选框**不读**候选视图（职责分离默认关着，退回全量
   `detections`）。所以 `locked_*` 取的是**本次生效开关下真正被选中的那个框**，
   并记下它出自哪个池（`locked_pool`）与实体 ID（`locked_entity_id`）——
   否则"组装层干净了"会被误读成"默认锁框干净了"。

## 做法：跑生产代码，不另写一份对照实现

* 观测段的决策与组装（`decide_detection_refresh` / `assemble_slot_objects`）
  以及 quick 段本身，直接 exec `run_semantic_exploration.py` 里那段**真语句**
  （锚点与 `tests/test_detection_scheduler.py` 第 6 节同一对）。
* 检测走真的 `GroundedSAMSubprocessDetector`（真 worker、真模型）；`_detect`
  的去重键用生产函数 `detection_cache_key`；前向次数取 **worker 回填的
  `diagnostics.dino_forward_count`**（"生效值"的唯一可信来源），不数调用次数 ——
  高召回重试与 prompt 扩展重试各自会多发一次前向。
* 后台 Full Semantic 用真的 `AsyncSemanticObservationManager`（真 TTL、真
  single-flight、真冷启动阻塞），`analyze` 只保留生产里那条 grounded_sam
  分支的调用形状（`_detect(..., slots=None)`）。

## 回放的是什么（**如实读这一段再用里面的数字**）

* **场景是冻结的**：位姿恒为原点、sector 恒为 0，所以平移/朝向判据在回放里
  **永远不触发**（这本就是分频声称要省的那批轮次）。行走/换房间下的收益如实标
  "未验收"，本工具不伪造。
* **画面逐轮轮转**（2026-10-14 补充复审后改的）：清单里的三帧按下标轮转，每轮
  拷到新路径（生产里 `_cached_image` 也是每帧一个新路径），所以去重键每轮都变。
  **这不是"站着不动"那一幕的复现**：静止场景里画面不该变，这里每轮都在变。
  对"按槽保留旧结果"来说这是**压力测试** —— 它让"目标动过/走开过"这件事真的
  发生，跨槽旧框才有机会暴露。上一版每轮复用同一张 `setup.json` 里写死的那一帧
  （`frame 5465`），于是 §5 里"三张相邻静止帧"的说法**当时并不成立**：那个回放
  只有一张图，而且一张不动的图**根本**暴露不了目标移位。
  某次运行到底喂了几帧、哪几帧，只认 `setup.json` 的 `frames_used` / `frame_count`。
* **只有 quick 段与前台/后台两路检测是真的**：`ReplayNode` 不实现完整
  `observe()`，没有语义地图记账，没有自主搜索，没有复核（verify）。所以本工具
  **不能说**任何关于"入账周期""P2 守住没有""端到端"的话 —— 那三件事分别由
  `tests/test_detection_scheduler.py`（observe → `_feed_detection` 真行为）与
  真机回放负责。

**不启动机器狗运动、不解除任何锁、不重启任何服务。**

用法（必须用**真实模型环境**解释器）：

    /home/mxt/anaconda3/envs/go2_robot_scene_demo/bin/python \\
      scripts/go2w/perception_cadence_wallclock.py \\
      --manifest artifacts/perception_profile_20261013/capture.json \\
      --output-dir artifacts/perception_optimization/20261013_phase2/cadence_wallclock_20261014_abba_v2 \\
      --seconds 90 --order abba --warmup-seconds 20

**先热身再分臂，且跑两对、第二对交换起始臂**（`--order baab`）：
`--warmup-seconds` 那段跑在臂之外，把冷启动（首轮约 18–20 s 的模型加载）
明确记进 `setup.json` 的 `warmup`，不让它落进第一个臂 —— 落在第一个臂上
ABBA 抵消不掉，那个"合计"就同时含了冷启动和分频两件事（补充复审「必修 2」）。
两对**必须串行**跑，并排会互相抢 GPU。

**不会覆盖旧证据**：目标目录非空时直接拒绝，除非显式 `--force`。
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import textwrap
import threading
import time
from collections import OrderedDict
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

ENTRY = ROOT / "scripts" / "go2w" / "run_semantic_exploration.py"

# quick 段的边界锚点。与 `tests/test_detection_scheduler.py` 用同一对：
# 两处都盯同一段源码，改了锚点两边一起红，不会一边绿一边烂。
_QUICK_START = 'profiler.mark("quick_start")'
_QUICK_END = 'quick_latency_ms = (time.perf_counter() - quick_start_perf)'

# 两个臂 = 分频开 / 关。**同一个开关**，不另造实现。
ARMS: dict[str, bool] = {"off": False, "on": True}


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))
    return ordered[index]


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


# --------------------------------------------------------------------------- #
# 把入口文件里那段 quick 真语句取出来执行
# --------------------------------------------------------------------------- #
def load_entry_namespace() -> dict:
    """exec `run_semantic_exploration.py`，拿到它模块级的名字。"""
    import importlib.util

    spec = importlib.util.spec_from_file_location("_rse_wallclock", ENTRY)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return dict(vars(module))


def quick_block_source() -> str:
    text = ENTRY.read_text(encoding="utf-8")
    if _QUICK_START not in text or _QUICK_END not in text:
        raise AssertionError("quick 段的锚点不在源文件里了 —— 本工具需要跟着改")
    start = text.rindex("\n", 0, text.index(_QUICK_START)) + 1
    end = text.rindex("\n", 0, text.index(_QUICK_END)) + 1
    return textwrap.indent(textwrap.dedent(text[start:end]), "    ")


def make_quick_round(entry_ns: dict):
    ns = dict(entry_ns)
    ns["time"] = time
    # **必须把 `cadence_entries` 也还出去**：quick 段里那句
    # `cadence_entries = put_slot_entry(cadence_entries, ...)` 是**局部重绑**，
    # 不还出来调用方手里的永远是空表 —— 于是每轮都像"第一次"，场景槽的旧结果
    # 一个都留不下，回放会得出"分频把场景物体丢了"这个**假**结论（自己踩到过）。
    wrapper = (
        "def _quick_round(node, state, profiler, stable_image, prompt, env, frame_id,\n"
        "                 slots, reuse_detection, stale_assembly, cadence_entries,\n"
        "                 cadence_ttl_by_slot, settings):\n"
        + quick_block_source()
        + "    return objects, assembly, cadence_entries\n"
    )
    exec(wrapper, ns)  # noqa: S102 - 包的是本仓源文件里的真语句
    return ns["_quick_round"]


class _Profiler:
    """只吸收 `mark()`/`record()`；本工具不测分项耗时。"""

    def mark(self, name: str) -> None:
        pass

    def record(self, name: str, start: str) -> None:
        pass


def _gsam_to_dict(obj) -> dict:
    """`DetectedObject` → dict。**必须与 `run_autonomous_loop._gsam_to_dict` 同字段集**。

    那个函数在依赖 rclpy 的文件里，宿主 import 不进来，所以这里重写一份。
    重写就有漂移风险，所以 `tests/test_cadence_wallclock_tool.py` 拿生产函数的
    源码解析出字段名跟这里逐一对齐 —— 生产加了字段而这里没加，判据会红。
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


# --------------------------------------------------------------------------- #
# 回放用的 node：只实现 quick 段与分频真正用到的那几个方法
# --------------------------------------------------------------------------- #
class ReplayNode:
    """`_detect` 逐行照 `run_autonomous_loop.Go2WNode._detect` 的形状做。

    三条不能省：去重键用生产函数 `detection_cache_key`（含**词槽**）、
    串行化锁（真 worker 只有一条管道）、缓存上限 3 与 LRU 淘汰。省掉任何一条，
    "前台与后台共用一次前向"这件事就测不出来了。
    """

    _detector = "grounded_sam"

    def __init__(self, detector, settings, cache_dir: Path) -> None:
        self.detector = detector
        self.settings = settings
        self._cache_dir = cache_dir
        self._lock = threading.Lock()
        self._cache: "OrderedDict[tuple, list[dict]]" = OrderedDict()
        self._ram_lock = threading.Lock()
        self._last_ram_tag_time = 0.0
        self._cached_ram_terms: list[str] = []
        self.last_ram_terms: list[str] = []
        self.events: list[dict] = []
        self.calls: list[dict] = []
        self.counters = {
            "detect_calls": 0,
            "cache_hits": 0,
            "forwards": 0,
            "forwards_foreground": 0,
            "forwards_background": 0,
            "forwards_unreported_calls": 0,
            "terms": 0,
            "detect_wall_s": 0.0,
        }
        # 后台 Full Semantic 在**另一个线程**里调 `_detect`，所以"这一笔算谁
        # 头上"必须按线程分，不能按调用顺序猜。
        self._lane = threading.local()

    # -- 生产里由 node 提供的最小面 ---------------------------------------- #
    def _host_s(self) -> float:
        return time.monotonic()

    def _write(self, payload: dict) -> None:
        self.events.append(dict(payload))

    def lane(self) -> str:
        return getattr(self._lane, "value", "foreground")

    def stable_frame(self, frame_path: str, frame_id: str) -> str:
        """每轮一个新路径 —— 生产里 `_cached_image` 就是这么做的。

        路径不变的话去重键不变，第一轮之后每轮都是缓存命中，整个测量变成
        "在测缓存"。**这是本工具最容易悄悄失效的地方**，所以单独成一个方法。
        """
        target = self._cache_dir / f"bundle_{frame_id}.jpg"
        shutil.copy2(frame_path, target)
        return str(target)

    def _ram_scene_terms(self, image_path: str) -> list[str]:
        """照 `run_autonomous_loop._ram_scene_terms`：按间隔低频问一次 RAM++。"""
        settings = self.settings
        if not settings.enable_ram_plus:
            return []
        now = time.monotonic()
        with self._ram_lock:
            if now - self._last_ram_tag_time < settings.ram_scene_tag_interval_seconds:
                return list(self._cached_ram_terms)
            terms = self.detector.discover_scene_terms(image_path)
            self._last_ram_tag_time = time.monotonic()
            self._cached_ram_terms = list(terms)
            self.last_ram_terms = list(terms)
            return list(terms)

    def _detect(self, image_path: str, prompt: str, env: dict, *,
                frame_id: str = "", slots=None) -> list[dict]:
        from app.detectors.grounded_sam_subprocess import (
            detection_cache_key,
            diagnostics_event_payload,
        )

        key = detection_cache_key(image_path, prompt, slots)
        lane = self.lane()
        with self._lock:
            if key is not None and key in self._cache:
                self._cache.move_to_end(key)
                self.counters["cache_hits"] += 1
                self.calls.append({
                    "frame_id": str(frame_id), "lane": lane,
                    "slots": None if slots is None else sorted(slots),
                    "cache_hit": True, "forwards": 0, "terms": 0,
                    "wall_s": 0.0, "reason": "cache",
                })
                return deepcopy(self._cache[key])
            terms = [t.strip() for t in prompt.replace("。", ".").split(".") if t.strip()]
            started = time.perf_counter()
            objects = self.detector.detect_with_dynamic_terms(
                image_path, prompt, terms, self._ram_scene_terms(image_path),
                frame_id=str(frame_id),
                slots=None if slots is None else frozenset(slots),
            )
            wall_s = time.perf_counter() - started
            payload = [_gsam_to_dict(obj) for obj in objects]
            if key is not None:
                self._cache[key] = deepcopy(payload)
                if len(self._cache) > 3:
                    self._cache.popitem(last=False)

        diag = diagnostics_event_payload(self.detector, frame_id=frame_id)
        attempts = list((diag or {}).get("attempts") or [])
        forwards = 0
        term_total = 0
        reported = 0
        for attempt in attempts:
            stats = attempt.get("diagnostics") or {}
            if "dino_forward_count" in stats:
                reported += 1
                forwards += int(stats.get("dino_forward_count") or 0)
                term_total += int(stats.get("term_count") or 0)
        if not attempts or reported != len(attempts):
            # 没有诊断 == 数不出这一笔。**记为未知**，不猜成 1：猜出来的数字
            # 会混进"总前向次数"里，而那正是本工具唯一的结论依据。
            self.counters["forwards_unreported_calls"] += 1
        self.counters["detect_calls"] += 1
        self.counters["forwards"] += forwards
        self.counters["forwards_foreground" if lane == "foreground"
                      else "forwards_background"] += forwards
        self.counters["terms"] += term_total
        self.counters["detect_wall_s"] += wall_s
        self.calls.append({
            "frame_id": str(frame_id), "lane": lane,
            "slots": None if slots is None else sorted(slots),
            "cache_hit": False, "forwards": forwards, "terms": term_total,
            "wall_s": round(wall_s, 3),
            "attempts": len(attempts),
            "diagnostics_reported": reported,
        })
        return payload


# --------------------------------------------------------------------------- #
# 一个臂：连跑若干秒
# --------------------------------------------------------------------------- #
def make_semantic_manager(analyze, *, node, settings, ttl_seconds: float):
    """后台 Full Semantic 管理器，**时钟域与生产逐字一致**。

    ``now`` 必须是 ``time.time``（epoch），因为 `submit_if_needed` 的传入
    ``capture_timestamp`` 在生产里就是 epoch（[run_semantic_exploration.py:1744]：
    ``time.time()``，或 RGB-D 源的 host_timestamp，那里有合法性窗口检查）。

    管理器把 ``capture_timestamp`` 原样存进 ``SemanticObservation.timestamp_sec``，
    再用 ``age = now() - latest.timestamp_sec`` 判 TTL。**两个域一旦错开，age 变成一个
    巨大的负数，TTL 这条闸门就永远不成立** —— 首帧之后后台再也不会刷新，而所有
    "看起来正常"的症状一个都不会有：没有报错、没有告警，只是 `bg` 那一列静默变成
    "只跑过一次"。本工具第一版就踩了这个（``now=time.monotonic`` 配 epoch 的
    capture_timestamp），结果把**分频开着时后台多付的那笔全量前向**藏掉了 ——
    恰好是审查 P2 要看的那个数。判据 `test_the_background_manager_shares_the_clock_
    of_the_capture_timestamps` 与校准刀 E 就是钉这件事的。
    """
    from app.live_robot.async_semantic_observer import (
        AsyncSemanticObservationManager,
    )

    return AsyncSemanticObservationManager(
        analyze,
        enabled=settings.vlm_runtime_semantic_background_enabled,
        max_inflight=settings.vlm_runtime_semantic_max_inflight,
        ttl_seconds=max(5.0, float(ttl_seconds)),
        translation_refresh_m=settings.vlm_runtime_semantic_translation_refresh_m,
        heading_sector_deg=settings.vlm_runtime_semantic_heading_sector_deg,
        initial_warmup_blocking=settings.vlm_runtime_semantic_initial_warmup_blocking,
        visual_change_enabled=settings.vlm_runtime_semantic_visual_change_enabled,
        visual_change_threshold=None,
        # 见 docstring：这里**不能**用 time.monotonic。
        now=time.time,
        event_sink=node._write,
    )


def run_arm(
    arm: str,
    *,
    node: ReplayNode,
    quick_round,
    settings,
    frames: list[dict],
    prompt: str,
    target_terms: set[str],
    env: dict,
    seconds: float,
    profile=None,
) -> dict:
    from app.live_robot.async_semantic_observer import compute_scene_signature
    from app.perception.detection_scheduler import (
        ALL_SLOTS,
        SLOT_ANCHOR,
        SLOT_SCENE,
        SLOT_TARGET,
        assemble_slot_objects,
        decide_detection_refresh,
    )

    cadence_on = ARMS[arm]
    quick_settings = SimpleNamespace(go2w_detection_cadence_enabled=cadence_on)
    ttl_by_slot = {
        SLOT_TARGET: float(settings.go2w_target_refresh_interval_s),
        SLOT_ANCHOR: float(settings.go2w_anchor_refresh_interval_s),
        SLOT_SCENE: float(settings.go2w_scene_refresh_interval_s),
    }
    cadence_state: dict = {
        "target_s": None, "anchor_s": None, "scene_s": None,
        "place_id": None, "sector": None,
    }
    cadence_entries: list = []
    profiler = _Profiler()

    baseline = dict(node.counters)
    baseline_calls = len(node.calls)
    baseline_events = len(node.events)
    window_started = time.monotonic()

    # 后台 Full Semantic：**真的**管理器、真的 TTL 与 single-flight。
    # `analyze` 只保留生产 grounded_sam 分支的调用形状。
    def analyze(image_path, _profile, *, request_id=None, frame_id=None,
                robot_pose=None):
        node._lane.value = "background"
        try:
            objects = node._detect(image_path, prompt, env,
                                   frame_id=str(frame_id or "semantic_live"),
                                   slots=None)
            return {
                "image_path": image_path, "frame_id": str(frame_id or ""),
                "scene_objects": [dict(item) for item in objects],
                "source": "grounded_sam_subprocess",
                "semantic_source_frame_id": str(frame_id or ""),
            }
        finally:
            node._lane.value = "foreground"

    manager = make_semantic_manager(
        analyze, node=node, settings=settings, ttl_seconds=settings.vlm_runtime_semantic_ttl_seconds
    )

    rounds: list[dict] = []
    deadline = time.monotonic() + float(seconds)
    index = 0
    prefix = f"{arm}{int(time.time()) % 100000}-"
    while time.monotonic() < deadline:
        index += 1
        # **逐轮换图**（2026-10-14 补充复审）。原来每轮复制的是同一张 `color_ref`，
        # 于是"目标移了位/走开了"这件事在回放里根本发生不了，跨槽旧框的问题也就
        # 测不出来。三帧按下标轮转：每臂的序列相同（第 1 轮都是 frame 0），
        # 所以两臂之间仍然可比。
        #
        # 说清楚这是什么：位姿判据在回放里恒不触发（`translation_m=0`、
        # `sector` 固定），**可喂进去的画面每轮都在变** —— 对"按槽保留"来说这是
        # 压力测试，不是生产里"站着不动"那一幕的复现。
        frame = frames[(index - 1) % len(frames)]
        frame_id = f"{prefix}{index:04d}"
        stable_image = node.stable_frame(frame["color_ref"], frame_id)
        started = time.monotonic()
        # 静止：位姿恒定，所以平移/朝向判据永不触发（这本就是分频要省的那批轮次）。
        pose = {"x": 0.0, "y": 0.0, "yaw_rad": 0.0, "yaw_deg": 0.0}
        sector = 0
        # 后台提交点与生产同位置：在 quick 段之前。
        manager.submit_if_needed(
            image_path=stable_image, frame_id=frame_id,
            capture_timestamp=time.time(), timestamp_quality="receive_time",
            robot_pose=pose, target_profile=node.detector.target_profile,
            scene_signature=compute_scene_signature(stable_image),
        )
        decision = decide_detection_refresh(
            now=time.monotonic(),
            last_target_s=cadence_state["target_s"],
            last_anchor_s=cadence_state["anchor_s"],
            last_scene_s=cadence_state["scene_s"],
            ttl_target_s=ttl_by_slot[SLOT_TARGET],
            ttl_anchor_s=ttl_by_slot[SLOT_ANCHOR],
            ttl_scene_s=ttl_by_slot[SLOT_SCENE],
            scene_max_stale_s=float(settings.go2w_scene_max_stale_s),
            place_id=None, last_place_id=None,
            sector=sector, last_sector=cadence_state["sector"],
            translation_m=0.0, translation_refresh_m=0.25,
        )
        slots = decision.slots if cadence_on else None
        stale_assembly = (
            assemble_slot_objects(cadence_entries, now=time.monotonic(),
                                  ttl_by_slot=ttl_by_slot)
            if cadence_on else None
        )
        reuse_detection = bool(
            cadence_on and not slots and stale_assembly is not None
            and stale_assembly.objects
        )
        if cadence_on and not slots and not reuse_detection:
            slots = None

        calls_before = len(node.calls)
        # 事件也要按**本轮**切片：`node.events` 跨臂累积，全局取最后一条
        # `detection_cadence_assembled` 会在分频关着的臂里读到**上一个臂**留下的
        # 那条 —— 于是"混帧轮数"在第一臂是 0、在第四臂变成满格（自己踩到）。
        events_before = len(node.events)
        objects, assembly, cadence_entries = quick_round(
            node, {}, profiler, stable_image, prompt, env, frame_id,
            slots, reuse_detection, stale_assembly, cadence_entries,
            ttl_by_slot, quick_settings,
        )
        round_calls = node.calls[calls_before:]
        round_wall_s = time.monotonic() - started

        # 与生产同一处记账：**只有真跑了才推时间戳**（复用那轮不推）。
        if not reuse_detection:
            refreshed_at = time.monotonic()
            if slots is None or SLOT_TARGET in slots:
                cadence_state["target_s"] = refreshed_at
            if slots is None or SLOT_ANCHOR in slots:
                cadence_state["anchor_s"] = refreshed_at
            if slots is None or SLOT_SCENE in slots:
                cadence_state["scene_s"] = refreshed_at
            cadence_state["sector"] = sector

        # "这是场景物体还是目标物体"按 `source_prompt_term` 分：RAM 词表里的
        # 是场景槽扩出来的词，目标词表里的是目标槽的词。**不按 `detection_slot`
        # 分** —— 分频关着时组装根本不写那个字段，两个臂就没法用同一把尺子比。
        ram_terms = set(node.last_ram_terms or [])
        scene_objects = [
            item for item in objects
            if str(item.get("source_prompt_term") or "") in ram_terms
        ]
        target_like = [
            item for item in objects
            if str(item.get("source_prompt_term") or "") in target_terms
        ]
        # 目标判据走**真链路**（分类 + 选框），不是照词表数一遍（补充复审必修 1）：
        # 要回答的是"这一轮锁在谁身上、它是不是这一帧的框"。`mixed_frames` 只说明
        # "观测混了帧"，不说明"混进来的旧框被当成目标用了" —— 后者才是缺陷。
        verdict = _target_verdict(objects, profile, settings, frame_id)
        rounds.append({
            "arm": arm,
            "round": index,
            "frame_id": frame_id,
            # 6 位小数，不是 3：stub 检测器下一轮只要几百微秒，3 位会把整列舍成
            # 0.0，稳态速率的分母直接变成 0（`steady_forwards_per_s` 报 None）。
            # 真检测器下一轮几秒，两种量级都要装得下。
            "wall_s": round(round_wall_s, 6),
            "slots": None if slots is None else sorted(slots),
            "reused": bool(reuse_detection),
            "scene_slot_refreshed": bool(slots is None or SLOT_SCENE in slots),
            "objects": len(objects),
            "scene_objects": len(scene_objects),
            "target_objects": len(target_like),
            "forwards": sum(int(call["forwards"]) for call in round_calls),
            "foreground_forwards": sum(
                int(call["forwards"]) for call in round_calls
                if call["lane"] == "foreground"
            ),
            "background_forwards": sum(
                int(call["forwards"]) for call in round_calls
                if call["lane"] == "background"
            ),
            "detect_calls": sum(1 for call in round_calls if not call["cache_hit"]),
            "cache_hits": sum(1 for call in round_calls if call["cache_hit"]),
            "mixed_frames": None,
            "frame_index": (index - 1) % len(frames),
            "color_ref": str(frame.get("color_ref") or ""),
            **verdict,
        })
        assembled = [
            e for e in node.events[events_before:]
            if e.get("event") == "detection_cadence_assembled"
        ]
        if assembled:
            rounds[-1]["mixed_frames"] = bool(assembled[-1].get("mixed_frames"))

    cadence_state["place_id"] = None
    # 窗口内的账先在**排空之前**记一份（补充复审必修 2）：原来只有一个总数，
    # 它其实是"窗口内发起的工作 + 窗口后收尾的那部分"。"窗口总前向"这个说法
    # 因此不准确 —— 收尾那几笔是上一轮欠下的，不该算在窗口的算力账上。
    in_window = {key: node.counters[key] - baseline[key] for key in node.counters}
    window_wall_s = time.monotonic() - window_started

    # 等后台线程收尾：不 wait 的话最后一笔前向会落到下一个臂的账上。
    drained = _drain(manager, timeout=120.0)

    arm_events = node.events[baseline_events:]
    arm_calls = node.calls[baseline_calls:]
    counters = {key: node.counters[key] - baseline[key] for key in node.counters}
    return {
        "arm": arm,
        "cadence_enabled": cadence_on,
        "seconds": round(float(seconds), 3),
        "window_wall_s": round(window_wall_s, 3),
        "rounds": len(rounds),
        "background_drained": drained,
        "rounds_per_min": round(len(rounds) / max(1e-9, float(seconds)) * 60.0, 3),
        "counters": counters,
        "counters_in_window": in_window,
        "round_detail": rounds,
        "events": arm_events,
        "calls": arm_calls,
    }


def _target_verdict(objects: list[dict], profile, settings, frame_id: str) -> dict:
    """跑真分类 + 真选框：这一轮的目标候选池干净吗、锁在谁身上。

    ``stale_target_candidates`` 就是本缺陷的指纹：组装出来的物体里，命中目标词
    却来自**别的帧**、还被算作候选的框有几个。目标槽 TTL 为 0（默认）时它必须
    恒为 0 —— 目标槽每轮都重跑，最新一次没报出来的框不该留在候选池里。

    **它和职责分离开关无关**，这是故意的：`classify_roles` 每轮都跑，所以"旧框
    还算不算目标候选"是组装/分类层的结论，正是补充复审必修 1 要钉的东西。

    ``locked_*`` 取的是**本次生效开关下真正被选中的那个框**，并连它从哪个池子
    里选出来（``locked_pool``）与它的实体 ID（``locked_entity_id``）一起留底
    （第三次补充复审：原来 `locked_*` 固定取 `candidates_authoritative=True`
    的候选视图，那是**阶段二的目标配置**、不是默认配置；职责分离默认关着，
    实际走的是退回全量 `detections` 的那条旧路 —— 于是"组装层修好了"被误读成
    "默认锁框干净了"。现在两者不再混：默认组合下 `locked_pool` 会是
    `detections`，锁框是否干净由 `locked_from_current_frame` 如实反映）。

    复用轮（`reused=True`）不算：那一轮本来就没跑检测，观测整体来自旧帧是设计
    如此。汇总时按轮过滤。
    """
    from app.perception.entity_roles import (
        VERIFY_SCOPE_TARGET_CANDIDATES,
        classify_roles,
        select_candidate,
    )

    if profile is None:
        return {
            "target_candidates": None, "stale_target_candidates": None,
            "locked_score": None, "locked_src_frame": None,
            "locked_from_current_frame": None, "selection_fell_back": None,
            "locked_pool": None, "locked_entity_id": None,
            "locked_is_target_candidate": None,
        }
    scope = str(getattr(settings, "go2w_verify_candidate_scope",
                        VERIFY_SCOPE_TARGET_CANDIDATES))
    partition = classify_roles(objects, profile)
    best, used_candidates = select_candidate(
        partition.target_candidates, partition.detections,
        scope=scope,
        candidates_authoritative=bool(
            getattr(settings, "go2w_result_role_separation", False)
        ),
    )
    # **"没盖来源帧"不等于"来自别的帧"**：分频关着时入口走的是
    # `SlotAssembly(objects=list(fresh), ...)`，物体身上根本没有
    # `entity_source_frame_id`（那是按槽组装才写的）。把它当成"过期"会把
    # 分频**关着**的那两个臂整臂染红（本工具第一版就踩了这个，2223 轮）。
    stale = [
        item for item in partition.target_candidates
        if _stamped_elsewhere(item, frame_id)
    ]
    best_frame = "" if best is None else str(best.get("entity_source_frame_id") or "")
    return {
        "target_candidates": len(partition.target_candidates),
        "stale_target_candidates": len(stale),
        "locked_score": None if best is None else round(float(best.get("score") or 0.0), 4),
        "locked_src_frame": best_frame or None,
        # 没盖来源帧 ⇒ 判不了（None），不是 False。只有**确实盖了别的帧**才算
        # "锁在过期框上"。
        "locked_from_current_frame": (
            None if (best is None or not best_frame) else best_frame == str(frame_id)
        ),
        # 本次**生效配置**下复核选框走的是哪条路：True = 退回全量 detections
        # （职责分离关着，也就是旧行为）。这是纯配置事实，不含分数排序，确定。
        "selection_fell_back": not used_candidates,
        # 锁到的那个框**从哪个池子里选出来的** —— 复核要的"真实来源"。
        # 与 `selection_fell_back` 是同一件事的两个说法，但那条是"走了哪条路"、
        # 这条是"这个框的来路"，报告表里直接引后者。
        "locked_pool": (
            "target_candidates" if used_candidates else "detections"
        ) if best is not None else None,
        "locked_entity_id": (
            None if best is None else str(best.get("entity_id") or "") or None
        ),
        # 锁到的那个框**是不是目标词的候选**。必须有这一列，否则默认组合下
        # 两件完全不同的事会挤进同一个读数：①"锁在旧帧的目标框上"（本阶段要修的
        # 缺陷）；②"锁在场景物体上"（任务 3 未生效的已知行为 —— 职责分离关着时
        # `select_candidate` 在全量里取最高分，一把椅子就会赢）。三份视图共用同一批
        # dict，所以这里可以按**对象身份**判，不用重算 ID。
        "locked_is_target_candidate": (
            None if best is None
            else any(item is best for item in partition.target_candidates)
        ),
    }


def _stamped_elsewhere(item: dict, frame_id: str) -> bool:
    """这个物体**确定**来自别的帧吗？（没盖来源帧 ⇒ 不算，判不了就不判）"""
    stamped = str(item.get("entity_source_frame_id") or "")
    return bool(stamped) and stamped != str(frame_id)


def _drain(manager, *, timeout: float) -> bool:
    """等后台在途请求结束。返回是否等干净（超时返回 False，如实记）。

    必须等：后台是 daemon 线程，不等的话最后一个臂的最后一笔前向会记到
    **下一个臂**的账上，两臂的"总前向次数"就都不对了。
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not manager.has_inflight():
            return True
        time.sleep(0.2)
    return False


# --------------------------------------------------------------------------- #
def summarize_arm(result: dict) -> dict:
    rounds = result["round_detail"]
    counters = result["counters"]
    wall = [float(row["wall_s"]) for row in rounds]
    # 「场景槽刷新间隔」= 相邻两次**真的重跑了场景槽**之间隔了多久。用累计墙钟
    # 算，不用轮号差：轮号差还要乘每轮耗时才是周期，而每轮耗时在两个臂之间
    # **本来就不一样** —— 那正好是被测的东西，拿它当尺子会自己抵消掉。
    # 名字说的是**检测列表里的槽刷新**，不是地图入账（见模块 docstring）。
    elapsed = 0.0
    stamps: list[float] = []
    for row in rounds:
        elapsed += float(row["wall_s"])
        if row["scene_slot_refreshed"]:
            stamps.append(elapsed)
    intervals = [round(b - a, 3) for a, b in zip(stamps, stamps[1:])]
    zero_scene = [row["round"] for row in rounds if row["scene_objects"] == 0]

    # 稳态口径（补充复审必修 2）：把每臂**自己那一轮**的冷启动成本摘掉，剩下的
    # 按实际墙钟归一化成"每秒前向"。为什么要归一化而不是直接比总数：两臂的轮数
    # 本来就不一样（省下来的时间会变成更多轮），直接比总数等于把"轮数变化"和
    # "每轮算力"混在一起。速率才是等价的"等长时间窗的总前向"。
    steady = rounds[1:]
    steady_wall = sum(float(row["wall_s"]) for row in steady)
    steady_fg = sum(int(row["foreground_forwards"]) for row in steady)
    steady_bg = sum(int(row["background_forwards"]) for row in steady)
    in_window = result.get("counters_in_window") or counters
    # **只判"这一轮真的重跑了目标槽"的那些轮**：目标槽没刷的那一轮，目标候选本来
    # 就该来自更早那一帧（定时器还没到），把它算成缺陷是假警报。目标槽 TTL 为 0
    # （默认）时，非复用轮本来就都刷目标槽。
    judged = [
        row for row in rounds
        if not row["reused"] and (row["slots"] is None or "target" in row["slots"])
    ]
    stale_rows = [
        row["round"] for row in judged if int(row.get("stale_target_candidates") or 0)
    ]
    # **缺陷指纹（默认组合下的最终症状）**：锁到的框既是**目标词候选**、
    # 又盖着别的帧的来源戳。只算目标词候选是有意的 —— 职责分离关着时
    # `select_candidate` 会在全量里取最高分，锁到一把旧帧的椅子是**任务 3 未生效**
    # 的已知行为，不是本阶段要修的那个缺陷；两件事分开记，谁也不冒充谁。
    locked_off_frame = [
        row["round"] for row in judged
        if row.get("locked_from_current_frame") is False
        and row.get("locked_is_target_candidate") is True
    ]
    # 另一半如实上报（不藏）：锁到场景物体上 —— 职责分离默认关着时的常态。
    locked_on_scene = [
        row["round"] for row in judged
        if row.get("locked_is_target_candidate") is False
    ]
    # 复核选框**走的是哪条路**（纯配置事实）：默认 `GO2W_RESULT_ROLE_SEPARATION`
    # 是 false ⇒ 下游退回全量 detections，组装层新写的目标候选视图**不经过**这条
    # 路。报告要据此说清"组装层修好了"能覆盖到哪些消费方，不能含糊过去。
    fell_back = [
        row["round"] for row in judged if row.get("selection_fell_back") is True
    ]
    return {
        "arm": result["arm"],
        "cadence_enabled": result["cadence_enabled"],
        "seconds": result["seconds"],
        "window_wall_s": result.get("window_wall_s"),
        "rounds": result["rounds"],
        "rounds_per_min": result["rounds_per_min"],
        "round_wall_s": _summarize(wall),
        "first_round_wall_s": wall[0] if wall else None,
        "total_round_wall_s": round(sum(wall), 3),
        # 两个口径都留着：`forwards_*` 是排空后的总数（含窗口后收尾），
        # `forwards_*_in_window` 只算窗口内发起的。差额就是收尾那几笔。
        "forwards_total": counters["forwards"],
        "forwards_total_in_window": in_window["forwards"],
        "forwards_added_by_drain": counters["forwards"] - in_window["forwards"],
        "forwards_foreground": counters["forwards_foreground"],
        "forwards_foreground_in_window": in_window["forwards_foreground"],
        "forwards_background": counters["forwards_background"],
        "forwards_background_in_window": in_window["forwards_background"],
        "forwards_unreported_calls": counters["forwards_unreported_calls"],
        "terms_total": counters["terms"],
        "detect_calls": counters["detect_calls"],
        "cache_hits": counters["cache_hits"],
        "detect_wall_s": round(counters["detect_wall_s"], 3),
        # 稳态：摘掉每臂第一轮之后的"每秒前向"（前/后台分开）。
        "steady_rounds": len(steady),
        "steady_wall_s": round(steady_wall, 3),
        "steady_forwards_per_s": round(
            (steady_fg + steady_bg) / steady_wall, 4
        ) if steady_wall > 0 else None,
        "steady_foreground_forwards_per_s": round(
            steady_fg / steady_wall, 4
        ) if steady_wall > 0 else None,
        "steady_background_forwards_per_s": round(
            steady_bg / steady_wall, 4
        ) if steady_wall > 0 else None,
        "scene_slot_refresh_rounds": sum(
            1 for row in rounds if row["scene_slot_refreshed"]
        ),
        "scene_refresh_interval_s": _summarize(intervals),
        "rounds_without_scene_objects": len(zero_scene),
        "rounds_without_scene_objects_at": zero_scene[:20],
        "reused_rounds": sum(1 for row in rounds if row["reused"]),
        "background_drained": result["background_drained"],
        "mixed_frame_rounds": sum(1 for row in rounds if row["mixed_frames"]),
        # 缺陷指纹：非复用轮里，目标候选有没有来自别帧的框。默认配置下必须是 0。
        "judged_rounds": len(judged),
        # 第一轮是整臂唯一的**全槽**轮（`cadence_state` 每个臂都从空起），所以
        # 这一列就是"这一臂到底有没有见过目标"。没有它，"旧框没留下"与"本来
        # 就没有旧框"（检测器没给出目标框）在汇总表上长得一模一样 —— 阴性结论
        # 必须能自证不是空的（补充复审的纪律）。
        "first_round_target_candidates": (
            rounds[0].get("target_candidates") if rounds else None
        ),
        "rounds_with_stale_target_candidates": len(stale_rows),
        "rounds_with_stale_target_candidates_at": stale_rows[:20],
        "rounds_locked_off_current_frame": len(locked_off_frame),
        "rounds_locked_off_current_frame_at": locked_off_frame[:20],
        # 锁到**场景物体**上的轮数（职责分离关着时的常态，如实上报）。
        "rounds_locked_on_a_scene_object": len(locked_on_scene),
        "rounds_locked_on_a_scene_object_at": locked_on_scene[:20],
        "rounds_selection_fell_back": len(fell_back),
        # 锁到的框**从哪个池子选出来的**（第三次补充复审要的"真实来源"）：
        # 本臂实际生效的开关组合决定了它，重跑一次不必再去猜。
        "locked_pools": sorted({
            str(row["locked_pool"]) for row in rounds
            if row.get("locked_pool")
        }),
        "locked_entity_ids": sorted({
            str(row["locked_entity_id"]) for row in rounds
            if row.get("locked_entity_id")
        })[:20],
        "frames_cycled": len({row["frame_index"] for row in rounds}),
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--manifest",
        default=str(ROOT / "artifacts" / "perception_profile_20261013" / "capture.json"),
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--target", default=None)
    parser.add_argument("--seconds", type=float, default=90.0,
                        help="每个臂回放的**墙上时钟**秒数（不是轮数）")
    parser.add_argument("--order", choices=("ab", "ba", "abba", "baab"), default="abba",
                        help=(
                            "abba = off,on,on,off（两次反向相加抵消 GPU 线性漂移）；"
                            "baab = on,off,off,on，用来**交换起始臂**做重复对照 —— "
                            "冷启动只落在第一个臂上，只跑一个顺序分不清"
                            '"分频省了"与"第一个臂被冷启动坑了"（补充复审必修 2）'
                        ))
    parser.add_argument("--frame-index", type=int, default=None,
                        help="只用清单里的第几帧（默认 None = **全部帧逐轮轮转**）")
    parser.add_argument("--warmup-seconds", type=float, default=20.0,
                        help=(
                            "计时开始**之前**先空跑这么多秒把模型/缓存焐热，这段不计入"
                            "任何臂。0 = 不预热（旧行为，冷启动会全落在第一个臂上）"
                        ))
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    output_dir = Path(args.output_dir).resolve()
    if output_dir.exists() and any(output_dir.iterdir()) and not args.force:
        raise SystemExit(
            f"输出目录非空：{output_dir}\n本工具不覆盖已有证据。换目录或加 --force。"
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    # **曾经在这里建一个空 `objects/`**，而没有任何东西往里写 —— 两轮证据里它
    # 都是空的，看证据的人会以为"逐轮物体在这里"，其实没有（第三次补充复审点名）。
    # 逐轮到底锁了谁、那个框出自哪个池、来自哪一帧，`rounds.jsonl` 已经逐行记着
    # （`locked_score` / `locked_pool` / `locked_src_frame` / `locked_entity_id`），
    # 那才是要留的底。空目录不建了，也不列进证据清单。

    from app.config import get_settings
    from app.detectors.grounded_sam_subprocess import GroundedSAMSubprocessDetector
    from app.reasoning.target_profile import TargetProfileResolver

    settings = get_settings()
    frames = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
    if isinstance(frames, dict):
        frames = frames["frames"]
    frames = [entry.get("frame", entry) for entry in frames]
    # 默认**全部帧逐轮轮转**（补充复审：每轮复制同一张图，目标移动这件事在回放里
    # 根本发生不了，跨槽旧框也就测不出来）。`--frame-index` 仍可把范围压回一帧，
    # 用于和上一次运行逐字对齐。
    if args.frame_index is not None:
        frames = [frames[int(args.frame_index)]]
    if not frames:
        raise SystemExit("清单里一帧都没有")
    target_name = args.target or "绿色垃圾桶"
    profile = TargetProfileResolver().resolve(target_name, use_llm=False)
    detector_terms = list(profile.detector_terms())
    prompt = ". ".join(detector_terms)
    # 目标槽的词集：拿它把"目标物体"与"场景物体"分开 **两个臂都用同一把尺子**。
    target_terms = {term.strip().lower() for term in detector_terms if term.strip()}
    env = os.environ.copy()

    record: dict = {
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "manifest": str(Path(args.manifest).resolve()),
        "frames_used": [str(item.get("color_ref") or "") for item in frames],
        "frame_count": len(frames),
        "frame_index": args.frame_index,
        "target": target_name,
        "prompt": prompt,
        "seconds_per_arm": args.seconds,
        "warmup_seconds": args.warmup_seconds,
        "order": args.order,
        "python": sys.executable,
        "config": {
            key: getattr(settings, key) for key in (
                "detection_device",
                "go2w_detection_cadence_enabled",
                "go2w_target_refresh_interval_s",
                "go2w_anchor_refresh_interval_s",
                "go2w_scene_refresh_interval_s",
                "go2w_scene_max_stale_s",
                # 复核选框的两条开关：`stale_target_candidates` 这个指纹**不受**
                # 它们影响（分类每轮都跑），但"锁在谁身上"完全由它们决定 ——
                # `locked_*` 取的就是这两个开关生效后的那一次选择。生效值必须
                # 落进 setup.json，否则读证据的人没法判断那条路走的是哪一支。
                "go2w_result_role_separation",
                "go2w_verify_candidate_scope",
                "vlm_runtime_semantic_background_enabled",
                "vlm_runtime_semantic_ttl_seconds",
                "ram_scene_tag_interval_seconds",
                "enable_ram_plus",
                "grounding_dino_box_threshold",
                "grounding_dino_text_threshold",
            )
        },
    }

    entry_ns = load_entry_namespace()
    quick_round = make_quick_round(entry_ns)
    detector = GroundedSAMSubprocessDetector(settings=settings, target_profile=profile)
    cache_dir = output_dir / "cached_frames"
    cache_dir.mkdir(exist_ok=True)
    node = ReplayNode(detector, settings, cache_dir)

    sequence = {"ab": ["off", "on"], "ba": ["on", "off"],
                "abba": ["off", "on", "on", "off"],
                "baab": ["on", "off", "off", "on"]}[args.order]
    # **预热**（补充复审必修 2）：模型加载/首次分配那一次只发生在整个会话的第一
    # 个轮次上，ABBA 抵消不掉它 —— 它只会落在一个臂头上（上一版就落在 off#1 的
    # 20.051 s 里）。所以计时前先空跑一段，把冷启动代价**单独量出来**而不是让它
    # 混进某个臂。这段的前向不进任何臂的账：`run_arm` 在**每个臂自己开始时**取
    # 基线，预热臂的计数因此天然落在所有基线之外。
    warmup: dict | None = None
    # **两个累加器必须在 `try` 之前建好**：2026-10-14 加预热块时把 `results = []`
    # 丢在了 `try` 里，于是一个 90 秒的臂跑完之后才在 `results.append` 上炸
    # `NameError` —— 白跑一整臂，而所有单测都是直接调 `run_arm` 的，**一条都不会红**。
    # 这就是"判据没覆盖到的路径"：`main()` 的记账只有真跑才会走到（见 §5 与新加的
    # `test_main_bookkeeping_runs_from_warmup_to_disk`，校准刀 I 就砍这一行）。
    results: list[dict] = []
    try:
        if args.warmup_seconds and args.warmup_seconds > 0:
            print(f"REPLAY_WARMUP start {args.warmup_seconds}s", flush=True)
            warm = run_arm(
                "off", node=node, quick_round=quick_round,
                settings=settings, frames=frames, prompt=prompt,
                target_terms=target_terms, env=env,
                seconds=args.warmup_seconds, profile=profile,
            )
            warmup = {
                "arm": "off",
                "seconds": warm["seconds"],
                "rounds": warm["rounds"],
                "window_wall_s": warm["window_wall_s"],
                "first_round_wall_s": (
                    warm["round_detail"][0]["wall_s"] if warm["round_detail"] else None
                ),
                "forwards": warm["counters"]["forwards"],
                "forwards_foreground": warm["counters"]["forwards_foreground"],
                "forwards_background": warm["counters"]["forwards_background"],
                "note": (
                    "会话冷启动代价：**不计入任何臂**。第一个臂的第一轮仍然会带着"
                    "它自己那点残差（缓存/分配），所以 `first_round_wall_s` 也留着。"
                ),
            }
            print(
                f"REPLAY_WARMUP done rounds={warmup['rounds']} "
                f"forwards={warmup['forwards']} "
                f"first_round={warmup['first_round_wall_s']}s",
                flush=True,
            )
        for position, arm in enumerate(sequence):
            print(f"REPLAY_ARM {position} {arm} start", flush=True)
            result = run_arm(
                arm, node=node, quick_round=quick_round,
                settings=settings, frames=frames, prompt=prompt,
                target_terms=target_terms, env=env, seconds=args.seconds,
                profile=profile,
            )
            result["position"] = position
            results.append(result)
            print(
                f"REPLAY_ARM {position} {arm} rounds={result['rounds']} "
                f"forwards={result['counters']['forwards']} "
                f"fg={result['counters']['forwards_foreground']} "
                f"bg={result['counters']['forwards_background']} "
                f"cache_hits={result['counters']['cache_hits']}",
                flush=True,
            )
    finally:
        detector.close()

    summaries = [summarize_arm(result) for result in results]
    record["warmup"] = warmup
    (output_dir / "rounds.jsonl").write_text(
        "".join(
            json.dumps(row, ensure_ascii=False) + "\n"
            for result in results for row in result["round_detail"]
        ),
        encoding="utf-8",
    )
    (output_dir / "calls.jsonl").write_text(
        "".join(
            json.dumps(call, ensure_ascii=False) + "\n"
            for result in results for call in result["calls"]
        ),
        encoding="utf-8",
    )
    (output_dir / "setup.json").write_text(
        json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    (output_dir / "summary.json").write_text(
        json.dumps(
            {"setup": record, "arms": summaries, "comparison": compare(summaries)},
            ensure_ascii=False, indent=2,
        ),
        encoding="utf-8",
    )
    print("REPLAY_DONE " + json.dumps(compare(summaries), ensure_ascii=False))
    return 0


def compare(summaries: list[dict]) -> dict:
    """两臂对照。ABBA 时先按位置两两相减再相加，抵消 GPU 线性漂移。"""
    by_arm: dict[str, list[dict]] = {}
    for row in summaries:
        by_arm.setdefault(row["arm"], []).append(row)
    if "off" not in by_arm or "on" not in by_arm:
        return {"note": "只有一个臂，没有对照"}
    off = by_arm["off"]
    on = by_arm["on"]
    out: dict = {
        "note": (
            "所有数字都是实测：前向次数取 worker 回填的 dino_forward_count，"
            "墙钟取本机真跑。**回放夹具逐轮在冻结帧之间轮换**（3 张同场景静止帧），"
            "所以这不是「原地不动的静止场景回放」——轮换是为了压出跨帧装配缺陷，"
            "它的代价是每轮的检测输入都在变；机器狗行走/换房间下的节拍不在本对照里。"
        ),
        "off_runs": len(off), "on_runs": len(on),
    }
    if len(off) == len(on) and len(off) >= 1:
        pairs = list(zip(off, on))
        out["paired"] = [
            {
                "off_rounds": a["rounds"], "on_rounds": b["rounds"],
                "off_forwards": a["forwards_total"], "on_forwards": b["forwards_total"],
                "off_fg": a["forwards_foreground"], "on_fg": b["forwards_foreground"],
                "off_bg": a["forwards_background"], "on_bg": b["forwards_background"],
                "off_terms": a["terms_total"], "on_terms": b["terms_total"],
                "off_median_round_s": a["round_wall_s"].get("p50"),
                "on_median_round_s": b["round_wall_s"].get("p50"),
                # 稳态那一对（补充复审必修 2）：等价的"等长时间窗"口径。
                "off_steady_fwd_per_s": a["steady_forwards_per_s"],
                "on_steady_fwd_per_s": b["steady_forwards_per_s"],
                "off_first_round_s": a["first_round_wall_s"],
                "on_first_round_s": b["first_round_wall_s"],
                "off_added_by_drain": a["forwards_added_by_drain"],
                "on_added_by_drain": b["forwards_added_by_drain"],
            }
            for a, b in pairs
        ]
        # **逐对的方向**单独列出来：两对方向相反时，"合计"那个数就是两个反向
        # 变化的和，不能当成稳态结论（补充复审必修 2 点名的正是这件事）。
        total_off = sum(a["forwards_total"] for a in off)
        total_on = sum(b["forwards_total"] for b in on)
        out["totals"] = {
            "off_forwards": total_off, "on_forwards": total_on,
            "delta_pct": round((total_on / total_off - 1.0) * 100.0, 2)
            if total_off else None,
            "note": (
                "这是**含冷启动的合计**。冷启动只落在第一个臂上，ABBA 抵消不掉它；"
                "方向要看 `pair_directions` 与 `steady_*_forwards_per_s`。"
            ),
        }
        directions = []
        for a, b in pairs:
            if not a["forwards_total"]:
                continue
            directions.append(round((b["forwards_total"] / a["forwards_total"] - 1.0) * 100.0, 2))
        out["pair_directions_pct"] = directions
        if len(directions) >= 2:
            signs = {1 if d > 0 else (-1 if d < 0 else 0) for d in directions}
            out["pair_directions_agree"] = len(signs) == 1
        # 缺陷指纹的总账：任何一臂出现非零都是缺陷（默认配置下）。
        out["stale_target_rounds_by_arm"] = {
            row["arm"] + f"#{i}": row["rounds_with_stale_target_candidates"]
            for i, row in enumerate(summaries)
        }
        out["verify_selection_note"] = (
            "「复核选框走哪条路」由 `GO2W_RESULT_ROLE_SEPARATION` 决定，生效值见 "
            "setup.json 的 config。它关着（**默认**）时下游 `select_candidate` 退回"
            "全量 detections —— 那是加这个功能之前的旧行为，`target_candidates`"
            "这条修好的路**不经过**它。各臂实测退回了几轮看 "
            "`rounds_selection_fell_back`，锁到的框出自哪个池看 `locked_pools`。"
            "组装层的指纹 (`stale_target_rounds_by_arm`) 与这条开关无关："
            "`classify_roles` 每轮都跑。**注意**：退回全量不等于退回旧缺陷 —— "
            "入口在退回之前已经用同一把新鲜度尺子过滤过那张表（第三次补充复审必修），"
            "所以 `locked_pools=['detections']` 的臂里 `rounds_locked_off_current_frame` "
            "同样必须是 0，那才是「默认组合下旧框不会冒充当前帧」的实测读数。"
        )
    return out


if __name__ == "__main__":
    raise SystemExit(main())
