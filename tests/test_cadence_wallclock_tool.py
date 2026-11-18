"""`scripts/go2w/perception_cadence_wallclock.py` 的判据。

这个工具是**测量工具**，它自己的错会让报告里的每一个数字都错，而且错得很像
真的（"分频省了 40% 前向"）。所以判据盯的是**测量链**，不是被测的分频：

1. 本地那份 `_gsam_to_dict` 与生产那份字段集一致（重写就有漂移风险）；
2. quick 段的锚点还在（工具 exec 的是生产源码里那段真语句）；
3. **每轮换一个新帧路径**这件事真的发生了 —— 否则第一轮之后全是缓存命中，
   整个测量变成"在测缓存"，而结果看着完全正常；
4. 前向次数取自 worker 诊断、并按**前台/后台两条道**分开记；
5. 两个臂在同一个静止场景下，场景槽的刷新轮数确实不同，而且分频开着时
   每一轮的观测里**仍然有场景物体**（这正是审查 P2 那条要修的）；
6. 后台管理器的**时钟域**与它收到的 `capture_timestamp` 一致 —— 错开会让
   TTL 闸门永久失效，后台前向数被静默低估（本工具第一版就踩了，见下）。

跑的是 stub 检测器：**不加载模型、不起 worker、不碰 GPU**。
"""

from __future__ import annotations

import ast
import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "scripts" / "go2w" / "perception_cadence_wallclock.py"
LOOP = ROOT / "scripts" / "go2w" / "run_autonomous_loop.py"
FRAME = ROOT / "artifacts" / "perception_profile_20261013" / "frames" / "color_5465.jpg"


def _tool():
    spec = importlib.util.spec_from_file_location("_cadence_wallclock_under_test", TOOL)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def tool():
    return _tool()


# --------------------------------------------------------------------------- #
# 1) 本地 `_gsam_to_dict` 不许与生产漂移
# --------------------------------------------------------------------------- #
def _production_gsam_to_dict_keys() -> set[str]:
    """从生产源码里解析出 `_gsam_to_dict` 返回的字典键。"""
    tree = ast.parse(LOOP.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "_gsam_to_dict":
            for inner in ast.walk(node):
                if isinstance(inner, ast.Return) and isinstance(inner.value, ast.Dict):
                    return {key.value for key in inner.value.keys}
    raise AssertionError("生产源码里找不到 `_gsam_to_dict` 的返回字典 —— 判据要跟着改")


def test_the_local_object_mapping_matches_the_production_one(tool):
    """生产加了字段而工具没加 ⇒ 下游看到的物体少一个键，且**不报错**。"""
    stub = SimpleNamespace(
        label="trash bin", label_zh="垃圾桶", category="container",
        source_prompt_term="trash bin", detection_slots=("target", "scene"),
        score=0.9, bbox_2d=[0, 0, 1, 1],
        mask_rle=None, mask_size=None, mask_area_ratio=0.0,
    )
    payload = tool._gsam_to_dict(stub)
    assert set(payload) == _production_gsam_to_dict_keys()
    # 值也要真的透传：只对字段名，写死成 `[]` 也能骗过上面那条，而下游
    # （`classify_roles` 按词槽归属）看的正是**值**。
    assert payload["detection_slots"] == ["target", "scene"]


def test_an_object_without_the_slot_record_maps_to_an_empty_list(tool):
    """旧缓存/quick 路径的物体没有这枚属性 ⇒ 落成空表（"不知道"），不报错。"""
    stub = SimpleNamespace(
        label="desk", label_zh="桌子", category="furniture",
        source_prompt_term="desk", score=0.5, bbox_2d=[0, 0, 1, 1],
        mask_rle=None, mask_size=None, mask_area_ratio=0.0,
    )
    assert tool._gsam_to_dict(stub)["detection_slots"] == []


# --------------------------------------------------------------------------- #
# 2) 锚点
# --------------------------------------------------------------------------- #
def test_the_quick_block_anchors_still_exist(tool):
    source = tool.quick_block_source()
    assert "node._detect(stable_image, prompt, env" in source
    assert "assemble_slot_objects(" in source


def test_the_arm_names_map_to_the_one_real_switch(tool):
    """两个臂必须是**同一个开关**的开与关，不另造实现。"""
    assert tool.ARMS == {"off": False, "on": True}


# --------------------------------------------------------------------------- #
# 3~5) 用 stub 检测器跑真回放循环
# --------------------------------------------------------------------------- #
class _StubDetector:
    """真 `_build_prompts`（拿它算"这次问了几组词"）+ 假 DetectedObject。"""

    def __init__(self, settings, profile) -> None:
        from app.detectors.grounded_sam_subprocess import GroundedSAMSubprocessDetector

        self.target_profile = profile
        self._builder = GroundedSAMSubprocessDetector(settings=settings,
                                                     target_profile=profile)
        self.detect_attempts: list[dict] = []
        self.last_diagnostics = None
        self.last_diagnostics_available = False
        self.closed = False

    def close(self) -> None:
        # `main()` 的 `finally` 会调它；真检测器关的是 worker 子进程。
        self.closed = True

    def discover_scene_terms(self, image_path: str) -> list[str]:
        return ["desk", "office chair"]

    def detect_with_dynamic_terms(self, image_path, target_text, dynamic_terms,
                                  ram_scene_terms=None, *, frame_id="", slots=None):
        fs = None if slots is None else frozenset(slots)
        prompts = self._builder._build_prompts(
            target_text, dynamic_terms, ram_scene_terms, slots=fs
        )
        stats = {"dino_forward_count": len(prompts), "term_count": len(prompts)}
        self.detect_attempts = [{"frame_id": frame_id, "diagnostics": stats}]
        self.last_diagnostics = stats
        self.last_diagnostics_available = True
        # 每个 prompt 组名下挂一个框，其中场景组的框用 RAM 词当 `source_prompt_term`
        # —— 回放循环就是靠这个字段把"场景物体"数出来的。
        out = []
        for text in prompts:
            term = "desk" if "desk" in text else target_text.split(".")[0].strip()
            out.append(SimpleNamespace(
                label=term, label_zh=term, category="object",
                source_prompt_term=term, score=0.5, bbox_2d=[0.1, 0.1, 0.2, 0.2],
                mask_rle=None, mask_size=None, mask_area_ratio=0.0,
            ))
        return out


@pytest.fixture(scope="module")
def replay_env(tmp_path_factory):
    from app.config import get_settings
    from app.reasoning.target_profile import TargetProfileResolver

    settings = get_settings()
    profile = TargetProfileResolver().resolve("绿色垃圾桶", use_llm=False)
    target_terms = {t.strip().lower() for t in profile.detector_terms() if t.strip()}
    return settings, profile, target_terms


class _SettingsOverride:
    """只盖住指定几项、其余转发给真 settings 的代理。

    `Settings` 不是 pydantic 模型（没有 `model_copy`），而且这里要盖的是
    "窗口只有一秒多，4 s 的 TTL 数不出第二次刷新"这种测试期参数，
    **不能**去改生产默认值。
    """

    def __init__(self, base, **overrides) -> None:
        object.__setattr__(self, "_base", base)
        object.__setattr__(self, "_overrides", overrides)

    def __getattr__(self, name):
        overrides = object.__getattribute__(self, "_overrides")
        if name in overrides:
            return overrides[name]
        return getattr(object.__getattribute__(self, "_base"), name)


def _run(tmp_path, tool, *, seconds=0.4, order="ab", same_path=False,
         detector_cls=None, frames=None, patch_ns=None, **overrides):
    settings, profile, target_terms = _replay_env_cached()
    if overrides:
        settings = _SettingsOverride(settings, **overrides)
    ns = tool.load_entry_namespace()
    # 注入点：入口那段真语句用的名字都在这个 dict 里（`quick_round` 是拿它 exec
    # 出来的），所以"把生产函数换成缺陷版"必须在**这里**换 —— 换
    # `app.perception.detection_scheduler` 上的属性没用，入口早就绑好了。
    if patch_ns is not None:
        patch_ns(ns)
    quick_round = tool.make_quick_round(ns)
    detector = (detector_cls or _StubDetector)(settings, profile)
    cache_dir = tmp_path / "cached"
    cache_dir.mkdir(parents=True, exist_ok=True)
    node = tool.ReplayNode(detector, settings, cache_dir)
    if same_path:
        # 注入：每轮都指向同一个文件 —— 去重键于是永远不变。
        fixed = cache_dir / "fixed.jpg"
        import shutil

        shutil.copy2(FRAME, fixed)
        node.stable_frame = lambda frame_path, frame_id: str(fixed)
    frames = [
        {"frame_id": f"f{i}", "color_ref": str(path)}
        for i, path in enumerate(frames or [FRAME])
    ]
    results = []
    # 真按 `--order` 排臂，**且不清 `node.events`**（生产 `main()` 也不清）：
    # 跨臂累积正是"读到别的臂的事件"这类缺陷唯一的暴露方式，清了就等于把它
    # 藏起来（这条一开始就踩过）。
    sequence = {"ab": ["off", "on"], "ba": ["on", "off"],
                "abba": ["off", "on", "on", "off"],
                "baab": ["on", "off", "off", "on"]}[order]
    for arm in sequence:
        results.append(tool.run_arm(
            arm, node=node, quick_round=quick_round, settings=settings,
            frames=frames, prompt=". ".join(profile.detector_terms()),
            target_terms=target_terms, env={}, seconds=seconds,
            profile=profile,
        ))
    return results


_ENV_CACHE: tuple | None = None


def _replay_env_cached():
    global _ENV_CACHE
    if _ENV_CACHE is None:
        from app.config import get_settings
        from app.reasoning.target_profile import TargetProfileResolver

        settings = get_settings()
        profile = TargetProfileResolver().resolve("绿色垃圾桶", use_llm=False)
        terms = {t.strip().lower() for t in profile.detector_terms() if t.strip()}
        _ENV_CACHE = (settings, profile, terms)
    return _ENV_CACHE


def test_the_background_manager_shares_the_clock_of_the_capture_timestamps(tool):
    """**第二容易悄悄失效的地方**：管理器与 capture_timestamp 的**时钟域**。

    管理器把 `capture_timestamp` 原样存进 `SemanticObservation.timestamp_sec`，
    TTL 判据是 `now() - latest.timestamp_sec >= ttl`。工具喂进去的
    `capture_timestamp` 是**epoch**（生产就是 `time.time()`），所以 `now` 也必须
    是 epoch。一旦写成 `time.monotonic`，age 变成一个巨大的负数，**首帧之后后台
    再也不会刷新**：无报错、无告警，只有 `forwards_background` 静默停在"一笔"，
    而这一列正是审查 P2 要看的那笔"分频开着时后台多付的全量前向"（自己踩过）。
    """
    import time as _time

    from app.config import get_settings

    settings = get_settings()
    node = SimpleNamespace(_write=lambda event: None)
    manager = tool.make_semantic_manager(
        lambda *a, **k: {}, node=node, settings=settings, ttl_seconds=12.0
    )
    manager_now = float(manager._now())
    wall_now = _time.time()
    assert abs(manager_now - wall_now) < 60.0, (
        f"管理器的 now()={manager_now:.3f} 与 epoch 的 {wall_now:.3f} 不同域 —— "
        "TTL 闸门会永久失效，后台前向数会被静默低估"
    )


def test_a_fresh_frame_path_per_round_keeps_the_cache_out_of_the_way(tool, tmp_path):
    """**本工具最容易悄悄失效的地方**：路径不变 ⇒ 每轮都命中缓存 ⇒ 测的是缓存。

    所以先证明这条守卫有分辨力：同一个路径跑一遍，缓存命中必须真的出现。
    """
    fresh = _run(tmp_path / "fresh", tool, order="ab")[0]
    assert fresh["counters"]["detect_calls"] >= fresh["rounds"]
    # 冷启动那一次后台提交与前台同帧、同键，**命中是生产的正确行为**
    # （前台已经打过了）。所以这里判的是"没有随轮数增长"，不是"恒为 0"。
    assert fresh["counters"]["cache_hits"] <= 2, (
        f"每轮换了新路径却命中 {fresh['counters']['cache_hits']} 次缓存 —— 测量口径不对"
    )
    same = _run(tmp_path / "same", tool, order="ab", same_path=True)[0]
    assert same["counters"]["cache_hits"] > same["rounds"] // 2, (
        "同一路径也不命中缓存 —— 说明这条守卫没有分辨力，"
        "上面那条'没随轮数增长'就不算证据"
    )


def test_the_forward_count_comes_from_worker_diagnostics(tool, tmp_path):
    """前向次数取 worker 回填的 `dino_forward_count`，不是"调了几次检测"。"""
    off = _run(tmp_path / "count", tool, order="ab")[0]
    assert off["counters"]["detect_calls"] > 1
    assert off["counters"]["forwards"] > off["counters"]["detect_calls"] - 1
    assert off["counters"]["forwards_unreported_calls"] == 0
    assert off["counters"]["terms"] > 0


def test_the_background_lane_is_counted_apart_from_the_foreground(tool, tmp_path):
    """后台 Full Semantic 走 `slots=None`，与前台**不是**同一笔账。

    **不要断言"关着分频时后台一定付了几笔前向"**：关着时分频不生效、前后台
    同一个去重键，谁先拿到锁谁真跑、另一个命中缓存 —— 那是**竞态结果**，不是
    不变量（第一版这么断言，于是这条判据时红时绿）。真正的不变量有两条：
    两条道都出现在 `calls` 里、且两条道的前向数**恰好把总数分完**。
    """
    off, on = _run(tmp_path / "lanes", tool, order="ab", seconds=0.6)
    assert off["counters"]["forwards_foreground"] > 0
    assert {call["lane"] for call in off["calls"]} == {"foreground", "background"}
    for result in (off, on):
        counters = result["counters"]
        assert (
            counters["forwards_foreground"] + counters["forwards_background"]
            == counters["forwards"]
        ), "两条道的前向数没有把总数分完 —— 分道记账漏了"
    # 分频开着时前台用子集键、后台用全量键 ⇒ 后台**不可能**被去重，
    # 这一条是确定的（这也是审查 P2 那笔"加回来的量"）。
    assert on["counters"]["forwards_background"] > 0
    assert on["counters"]["cache_hits"] == 0, (
        "分频开着时后台命中了前台缓存 —— 说明两条道共用了一个去重键"
    )


def test_the_off_arm_refreshes_the_scene_every_round(tool, tmp_path):
    off = _run(tmp_path / "offarm", tool, order="ab")[0]
    assert off["rounds"] > 2
    assert all(row["scene_slot_refreshed"] for row in off["round_detail"])
    assert all(row["slots"] is None for row in off["round_detail"])


def test_the_on_arm_refreshes_the_scene_less_often_but_keeps_it_in_the_observation(
    tool, tmp_path
):
    """审查 P2 的现场：分频开着，场景槽刷得少了，**但每一轮的观测里还有场景物体**。

    这里只证"少刷"与"没丢"；"省了多少墙钟/前向"是实测数字，由工具跑真模型得出。
    """
    results = _run(tmp_path / "onarm", tool, order="ab", seconds=1.2,
                   go2w_scene_refresh_interval_s=0.15)
    off, on = results
    off_summary = tool.summarize_arm(off)
    on_summary = tool.summarize_arm(on)
    assert on_summary["rounds"] > 2
    assert on_summary["scene_slot_refresh_rounds"] < on_summary["rounds"], (
        "分频开着却每轮都刷场景槽 —— 分频没生效"
    )
    assert off_summary["scene_slot_refresh_rounds"] == off_summary["rounds"]
    assert on_summary["rounds_without_scene_objects"] == 0, (
        "只刷目标槽的那几轮把场景物体丢了 —— 整体覆盖的老缺陷回来了"
    )
    assert off_summary["rounds_without_scene_objects"] == 0


def test_the_off_arm_reports_no_mixed_frame_rounds(tool, tmp_path):
    """分频关着时**没有**按槽组装这件事，所以"混帧轮数"必须是 0。

    这条防的是**跨臂串味**：`node.events` 跨臂累积，若全局取最后一条
    `detection_cadence_assembled`，分频关着的臂会读到上一个臂留下的那条 ——
    结果是同一个臂在第一轮跑出 0、在第四轮跑出满格（ABBA 顺序下才暴露）。
    """
    results = _run(tmp_path / "mixed", tool, order="abba", seconds=0.6)
    off = [tool.summarize_arm(row) for row in results if row["arm"] == "off"]
    on = [tool.summarize_arm(row) for row in results if row["arm"] == "on"]
    assert all(row["mixed_frame_rounds"] == 0 for row in off), (
        "分频关着的臂报了混帧轮数 —— 多半是读到了别的臂的事件"
    )
    assert off[0]["mixed_frame_rounds"] == off[-1]["mixed_frame_rounds"]
    # 新调度每次触发都刷新整帧 target/anchor/scene，未触发时整体复用同一次
    # 检测，因此开关两臂都不应再产生混帧组装。
    assert all(row["mixed_frame_rounds"] == 0 for row in on)


def test_the_summary_reports_the_scene_accounting_period(tool, tmp_path):
    """§5.2：场景对象进地图的**周期**要以墙钟报出来，不能只报"刷了几次"。"""
    on = _run(tmp_path / "summary", tool, order="ab", seconds=1.2,
              go2w_scene_refresh_interval_s=0.15)[1]
    summary = tool.summarize_arm(on)
    assert "scene_refresh_interval_s" in summary
    assert summary["scene_refresh_interval_s"]["n"] >= 1
    assert summary["total_round_wall_s"] > 0
    assert summary["rounds_per_min"] > 0


# --------------------------------------------------------------------------- #
# 6) 补充复审（2026-10-14）后的三件事：逐轮换图、窗口/收尾拆账、缺陷指纹
# --------------------------------------------------------------------------- #
def _all_frozen_frames() -> list:
    """清单里的**全部**帧（本仓有 3 张）。文件名写死，缺了就直接红。"""
    frames_dir = FRAME.parent
    found = sorted(frames_dir.glob("color_*.jpg"))
    assert len(found) >= 3, f"冻结帧少于 3 张：{found} —— 多帧回放判据没素材"
    return found


def test_the_replay_cycles_through_every_frame_instead_of_reusing_one(tool, tmp_path):
    """逐轮换图：每轮复制同一张图，目标移动/走开这件事在回放里**发生不了**。

    （复审原文：单张重复图像无法暴露目标移动后的旧框问题。）
    """
    frames = _all_frozen_frames()
    result = _run(tmp_path / "cycle", tool, order="ab", seconds=1.2, frames=frames)[0]
    # 至少要跑到第 3 轮才谈得上轮转。
    assert result["rounds"] >= 3, f"窗口太短，只跑了 {result['rounds']} 轮"
    used = [row["frame_index"] for row in result["round_detail"]]
    assert used == [i % len(frames) for i in range(len(used))], (
        f"帧没有按下标轮转：{used}"
    )
    assert {row["color_ref"] for row in result["round_detail"]} == {
        str(path) for path in frames
    }
    summary = tool.summarize_arm(result)
    assert summary["frames_cycled"] == len(frames)


def test_the_window_total_and_the_post_window_drain_are_accounted_separately(tool, tmp_path):
    """复审必修 2：原来那个"窗口总前向"其实是"窗口内 + 窗口后收尾"。

    两个数分开记，差额就是收尾那几笔 —— 后台在途请求是上一轮欠下的，不该算在
    这个臂的窗口算力账上。
    """
    result = _run(tmp_path / "split", tool, order="ab", seconds=1.0)[0]
    summary = tool.summarize_arm(result)
    assert (
        summary["forwards_added_by_drain"]
        == summary["forwards_total"] - summary["forwards_total_in_window"]
    )
    assert summary["forwards_total_in_window"] <= summary["forwards_total"]
    # 稳态速率：摘掉第一轮之后按实际墙钟归一化 —— 两臂轮数本来就不同，直接比
    # 总数等于把"轮数变化"和"每轮算力"混在一起。
    assert summary["steady_rounds"] == max(0, summary["rounds"] - 1)
    if summary["steady_rounds"]:
        assert summary["steady_forwards_per_s"] is not None
        assert summary["steady_forwards_per_s"] > 0


class _VanishingTargetDetector(_StubDetector):
    """目标框**只在全槽那一轮**出现，之后只跑目标槽的那几轮什么都没有。

    这就是"目标走了"的最小模型：目标槽 TTL 是 0，每轮都重跑，最新一次没报出来
    就是"此刻不在这儿"。
    """

    def detect_with_dynamic_terms(self, image_path, target_text, dynamic_terms,
                                  ram_scene_terms=None, *, frame_id="", slots=None):
        out = super().detect_with_dynamic_terms(
            image_path, target_text, dynamic_terms, ram_scene_terms,
            frame_id=frame_id, slots=slots,
        )
        # "是不是全槽那一轮"用**槽集里有没有 scene** 判，**不能**用"槽集里有没有
        # target"：全槽那一轮本来就含 target，这么写会把第一轮的目标框也删掉，
        # 于是阳性对照里根本没有旧框可陈旧 —— 判据恒报 0，看着像"尺子很干净"
        # （第一版就是这么写的，被 `_pre_fix_assemble` 那条阳性对照当场戳穿）。
        is_full_round = slots is None or "scene" in slots
        if not is_full_round:
            return [item for item in out
                    if str(item.source_prompt_term) != target_text.split(".")[0].strip()]
        return out


def _pre_fix_assemble(entries, *, now, ttl_by_slot=None, order=None):
    """**修之前**的 `assemble_slot_objects`（逐字复刻旧行为）。

    两处：① 入选条目里**全部**物体都并进来（不看这个物体所属的槽是不是还新鲜）；
    ② 同一个去重键**先到先得**（升序遍历，留下的是旧帧那个框）。
    """
    from app.perception.detection_scheduler import ALL_SLOTS, SlotAssembly, _object_key

    wanted = tuple(order or ALL_SLOTS)
    picked = []
    for slot in wanted:
        best = None
        for entry in entries:
            if slot not in entry.slots:
                continue
            if best is None or float(entry.at) > float(best.at):
                best = entry
        if best is None:
            continue
        if not any(best is item for item in picked):
            picked.append(best)
    picked.sort(key=lambda item: float(item.at))
    objects, seen, frames = [], set(), set()
    for entry in picked:
        frame = str(entry.frame_id or "")
        age = max(0.0, float(now) - float(entry.at))
        slots = "+".join(sorted(entry.slots))
        if frame:
            frames.add(frame)
        for obj in entry.objects or ():
            key = _object_key(obj)
            if key in seen:
                continue
            seen.add(key)
            item = dict(obj) if isinstance(obj, dict) else {"label": str(obj)}
            if frame:
                item["entity_source_frame_id"] = frame
            item["detection_slot"] = slots
            item["detection_age_s"] = round(age, 3)
            objects.append(item)
    return SlotAssembly(objects=objects, entries=tuple(picked),
                        frame_id=frames.pop() if len(frames) == 1 else "")


# --------------------------------------------------------------------------- #
# 7) `main()` 自己的记账：预热 → 各臂 → 汇总 → 落盘
# --------------------------------------------------------------------------- #
def test_main_bookkeeping_runs_from_warmup_to_disk(tool, tmp_path, monkeypatch):
    """`main()` 的记账路径必须**真的跑得通**，不是靠读代码看不出问题。

    2026-10-14 真踩到：加预热块时把 `results = []` 丢进了 `try` 里。所有单测都是
    直接调 `run_arm` 的，**一条都不会红**；要等一个 90 秒的真机计时臂跑完、在
    `results.append` 上炸 `NameError` 才现形 —— 白跑一整臂，而且是在唯一一种
    "跑起来很贵"的用法里才现形。所以这里把 `--seconds` 压到几十毫秒、检测器换成
    替身，专门跑 `main()` 自己的那套记账与落盘（预热记录、两臂汇总、三份文件）。
    """
    # 注入点选 **`ReplayNode`**，不选检测器类：`main()` 自己在函数体里
    # `from app.detectors... import GroundedSAMSubprocessDetector`，patch 模块属性会
    # 连 `_StubDetector` 里那个真 `_build_prompts` 持有者也一起换掉 ⇒ 无限递归
    # （踩过）。真检测器的 `__init__` 不起 worker（`self._worker = None`），
    # 构造它是安全的；换掉 node，谁进回放循环就由这里说了算。
    class _StubNode(tool.ReplayNode):
        def __init__(self, detector, settings, cache_dir):
            super().__init__(
                _StubDetector(settings, detector.target_profile), settings, cache_dir
            )

    monkeypatch.setattr(tool, "ReplayNode", _StubNode)
    out = tmp_path / "main"
    monkeypatch.setattr(sys, "argv", [
        "perception_cadence_wallclock.py",
        "--manifest", str(ROOT / "artifacts" / "perception_profile_20261013"
                          / "capture.json"),
        "--output-dir", str(out),
        "--seconds", "0.05", "--warmup-seconds", "0.01", "--order", "ab",
    ])
    assert tool.main() == 0

    record = json.loads((out / "setup.json").read_text(encoding="utf-8"))
    assert record["frame_count"] == 3, "三帧都要进 frames_used —— 否则报告里那句就又是假的"
    assert len(record["frames_used"]) == 3
    # 预热**真的跑过而且真的记了**：只写个 `warmup: null` 会让"冷启动代价单独量"
    # 这句话没有出处。
    assert record["warmup"] is not None
    assert record["warmup"]["rounds"] >= 1
    assert record["warmup"]["first_round_wall_s"] is not None
    # 生效值要落进证据里，读证据的人才知道复核选框走的是哪条路。
    assert "go2w_result_role_separation" in record["config"]
    assert "go2w_verify_candidate_scope" in record["config"]

    summary = json.loads((out / "summary.json").read_text(encoding="utf-8"))
    assert [arm["arm"] for arm in summary["arms"]] == ["off", "on"]
    assert "stale_target_rounds_by_arm" in summary["comparison"]
    # 三份原始流水都要落盘：`rounds.jsonl` 是"每轮到底发生了什么"的唯一凭据。
    for name in ("rounds.jsonl", "calls.jsonl", "summary.json", "setup.json"):
        assert (out / name).is_file(), f"{name} 没落盘"


def test_the_stale_target_fingerprint_is_zero_when_the_target_is_gone(tool, tmp_path):
    """目标走了 ⇒ 非复用轮里**不该**出现"来自别的帧的目标候选"，也不该锁在它上面。"""
    # 取 **on 臂**（`order="ab"` 的第 2 个）：分频关着时入口根本不做按槽组装，
    # 物体身上也不会有来源帧 —— 那个臂上这条判据**没有东西可判**。
    result = _run(tmp_path / "gone", tool, order="ab", seconds=1.2,
                  detector_cls=_VanishingTargetDetector)[1]
    summary = tool.summarize_arm(result)
    assert summary["judged_rounds"] >= 1
    # **先证明这把尺子有东西可判**：全槽那一轮确实给出过目标候选。少了这一条，
    # "0 个陈旧候选"与"检测器压根没给过目标框"分不开 —— 而后者是这条阴性结论
    # 最容易悄悄退化成的那种空（第一版的 `_VanishingTargetDetector` 就是）。
    assert summary["first_round_target_candidates"] >= 1, (
        "全槽那一轮连目标候选都没有 —— 这条阴性结论是空的，不说明任何事"
    )
    assert summary["rounds_with_stale_target_candidates"] == 0, (
        "非复用轮里出现了来自别帧的目标候选 —— 目标已经不在了"
    )
    # 目标槽每轮重跑且都没报出来 ⇒ 候选视图为空 ⇒ 没有可锁的对象。
    assert summary["rounds_locked_off_current_frame"] == 0


def test_unified_full_frame_refresh_prevents_old_partial_assembly_defect(tool, tmp_path):
    """即便注入旧组装器，统一全槽刷新也不会留下旧目标槽。"""
    result = _run(
        tmp_path / "buggy", tool, order="ab", seconds=1.2,
        detector_cls=_VanishingTargetDetector,
        patch_ns=lambda ns: ns.__setitem__("assemble_slot_objects", _pre_fix_assemble),
    )[1]
    summary = tool.summarize_arm(result)
    assert summary["rounds_with_stale_target_candidates"] == 0
    assert summary["rounds_locked_off_current_frame"] == 0
    # "退没退回全量 detections"是**纯配置事实**（职责分离开关），配置不变就
    # 不该在同一个臂里一半一半 —— 这条同时把"默认配置走的是哪条路"钉在证据里，
    # 报告里那句"组装层的修复不经过默认复核路径"才有出处。
    assert summary["rounds_selection_fell_back"] in (0, summary["judged_rounds"]), (
        "同一个臂里复核选框一会儿走候选视图、一会儿退回全量 —— 配置事实不该这样"
    )


# --------------------------------------------------------------------------- #
# 8) `locked_*` 必须来自**生效开关**下真正选中的那个框（第三次补充复审）
# --------------------------------------------------------------------------- #
# 原写法把 `locked_*` 固定成"候选视图里的最高分"（`candidates_authoritative=True`），
# 那是**阶段二的目标配置**、不是默认配置：职责分离默认关着，复核选框实际退回全量
# `detections`。于是回放读出的 `locked_*` 只能证实"候选视图干净"，证不了"默认组合
# 下锁框干净" —— 而"单独开任务 4"正是默认组合。
def _verdict_settings(**overrides):
    from types import SimpleNamespace

    base = {
        "go2w_result_role_separation": False,
        "go2w_verify_candidate_scope": "target_candidates",
    }
    base.update(overrides)
    return SimpleNamespace(**base)


def _cross_frame_assembly(*, scene_score=0.95):
    """一次跨帧组装的产物：旧全量条目的目标框（别的帧）+ 场景物体 + 新帧目标框。

    场景物体刻意给**高分**（默认 0.95）：全量池里它会赢过新帧目标框的 0.20，
    候选池里它根本不参赛。两个池子于是给出**不同的赢家** —— 尺子读的是哪一个，
    一眼就能看出来（两池同赢家的话，"按生效开关取框"这件事根本考不出来）。
    """
    return [
        {"label": "trash bin", "score": 0.90, "bbox_2d": [0.10, 0.10, 0.20, 0.30],
         "source_prompt_term": "trash bin", "detection_slot": "anchor+scene+target",
         "detection_fresh_slots": ["scene"], "entity_source_frame_id": "f1"},
        {"label": "office chair", "score": scene_score,
         "bbox_2d": [0.60, 0.60, 0.80, 0.80],
         "source_prompt_term": "office chair", "detection_slot": "anchor+scene+target",
         "detection_fresh_slots": ["scene"], "entity_source_frame_id": "f1"},
        {"label": "trash bin", "score": 0.20, "bbox_2d": [0.11, 0.11, 0.21, 0.31],
         "source_prompt_term": "trash bin", "detection_slot": "target",
         "detection_fresh_slots": ["target"], "entity_source_frame_id": "f2"},
    ]


def test_the_locked_box_is_read_under_the_effective_switches_not_the_target_config(tool):
    """同一个场子，只翻职责分离开关：锁到的框必须**真的换一个**。

    原来 `locked_*` 固定取候选视图（`candidates_authoritative=True`），那是阶段二的
    目标配置、不是默认配置 —— 于是两臂读数都像"锁在目标候选上"，默认组合下
    实际锁到哪儿根本看不出来。

    **同时**钉住两条：① 尺子读的是**生效**配置（关着 ⇒ `detections`、锁到 0.95 的
    椅子；开着 ⇒ `target_candidates`、锁到 0.20 的目标框）；② 无论走哪条，旧帧那个
    0.90 的框都**不许**被锁 —— 这正是"退回全量"与"退回旧缺陷"的区别。
    """
    _, profile, _ = _replay_env_cached()
    objects = _cross_frame_assembly()
    frame_id = "f2"

    off = tool._target_verdict(
        objects, profile, _verdict_settings(go2w_result_role_separation=False), frame_id
    )
    assert off["locked_pool"] == "detections", (
        f"职责分离关着时锁框应退回全量，实测来自 {off['locked_pool']}"
    )
    assert off["selection_fell_back"] is True
    assert off["locked_score"] == 0.95, (
        f"关着时该锁到全量里最高分的椅子，实测 {off['locked_score']}"
        "（0.20 说明尺子仍按候选视图取的框）"
    )
    assert off["locked_is_target_candidate"] is False
    assert off["locked_from_current_frame"] is False  # 椅子来自 f1

    on = tool._target_verdict(
        objects, profile, _verdict_settings(go2w_result_role_separation=True), frame_id
    )
    assert on["locked_pool"] == "target_candidates"
    assert on["selection_fell_back"] is False
    assert on["locked_score"] == 0.20
    assert on["locked_is_target_candidate"] is True
    assert on["locked_from_current_frame"] is True

    # **本阶段真正要修的那一条**：旧帧的**目标框**（0.90）两条路上都不许赢，
    # 也不许留在目标候选池里。
    assert off["locked_score"] != 0.90 and on["locked_score"] != 0.90
    assert on["stale_target_candidates"] == 0


def test_the_fixture_records_which_pool_a_scene_object_lock_came_from(tool, tmp_path):
    """锁到**场景物体**上时，取证要能看出来那是场景物体、不是目标框。

    在职责分离显式关闭的旧组合下，全量里最高分的椅子会赢，
    所以它必须有自己的、不会被误读成"锁在旧目标框上"的读数。少了这一列，
    两件完全不同的事会挤进同一个 `rounds_locked_off_current_frame`。
    """
    result = _run(tmp_path / "scene_lock", tool, order="ab", seconds=0.8,
                  go2w_result_role_separation=False)[1]
    summary = tool.summarize_arm(result)
    # 两列都得在（`summarize_arm` 的表里缺一列，报告就引不到）。
    assert "rounds_locked_on_a_scene_object" in summary
    assert summary["rounds_locked_on_a_scene_object"] <= summary["judged_rounds"]
    # 缺陷指纹只认目标词候选：锁在场景物体上**不算**这条。
    assert summary["rounds_locked_off_current_frame"] == 0
    # 锁框出自哪个池、实体 ID 也逐臂留底（复审要的"真实来源"）。
    assert summary["locked_pools"], "锁框来源池没有留底"
    assert set(summary["locked_pools"]) <= {"detections", "target_candidates"}

    # Current production default separates target and scene roles. The same
    # counter must honestly be zero there; do not switch production back just
    # to make the legacy measurement fixture produce a nonzero count.
    separated = _run(tmp_path / 'separated', tool, order='ab', seconds=.8,
                     go2w_result_role_separation=True)[1]
    separated_summary=tool.summarize_arm(separated)
    assert separated_summary['rounds_locked_on_a_scene_object']==0
    assert set(separated_summary['locked_pools'])=={'target_candidates'}
