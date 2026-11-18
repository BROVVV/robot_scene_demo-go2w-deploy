#!/usr/bin/env python3
"""复现第四次补充复审的必修：目标槽的**上下文词**用旧帧高分赢下默认选框。

复审原话（`reports/perception_optimization_phase_2_fourth_followup_review_20261014.md`）：
项目现成的「打印机」画像会在 **target 槽**查询 `paper tray`；旧帧的全量检测给出
0.95 的 `paper tray`，新帧只刷目标槽、只给出 0.20 的 `printer`；在"分频开启、
职责分离默认关闭"时，最终锁框仍选中**旧帧的 paper tray**。

**跑的是生产源码本身**：`TargetProfileResolver` 的真画像 + 测试里那套「取入口源
文件里那几段真语句 exec 执行」的辅助函数（`tests/test_detection_scheduler.py`）。
不重写一份"等价的"实现 —— 那正是复审要求排除的复现方式。

框上的 `slots=` 模拟检测器本轮的新行为：它随每个框记 `detection_slots`
（"这个框是哪个槽问出来的"）。旧帧是全槽检测，新帧只刷目标槽。

**边界（如实写，本轮不修）**：场景 2 的**默认**组合（职责分离关）里，旧全量
条目里的**场景**物体（`desk` 0.15）仍会顶上来当锁框、`bool(detections)` 口径
仍报 `PRESENT`。那是任务 3 / 任务 2 的地盘，与本次"上下文词借场景新鲜度留下"
是两件事 —— 本脚本钉的只是**上下文词不再赢**（见末尾的不变式）。

用法（纯 CPU，.venv 即可，不需要 GPU、不需要机器狗）：

    PYTHONPYCACHEPREFIX=/tmp/pyc_repro_cadence .venv/bin/python \
      scripts/go2w/repro_cadence_stale_context_word.py

退出码：0 = 三个场景的四种开关组合都符合预期（旧上下文词一次都没赢）。
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


_TDS = ROOT / "tests" / "test_detection_scheduler.py"
tds = _load("_tds", _TDS)
entry_ns = dict(vars(_load("_rse_under_test", tds.ENTRY)))

from app.reasoning.target_profile import TargetProfileResolver  # noqa: E402

PROFILE = TargetProfileResolver().resolve("打印机", use_llm=False)

# 旧帧（全槽检测）里那个 0.95 的上下文词；新帧（只刷目标槽）给 0.20 的目标词。
TRAY_OLD = tds._obj("paper tray", 0.95, (0.10, 0.10, 0.20, 0.30), slots=("target",))
DESK_OLD = tds._obj("desk", 0.15, (0.60, 0.60, 0.80, 0.80), slots=("scene",))
PRINTER_NEW = tds._obj("printer", 0.20, (0.11, 0.11, 0.21, 0.31), slots=("target",))

# 全场唯一不许出现的东西：旧帧那个上下文词被锁上。
LOCKED: list[str | None] = []


def _round(by_slot, *, role_separation: bool, scope: str):
    """跑一整轮：冷启动全槽 → 新帧只刷目标槽，再走真分类段 + 真锁框段。"""
    node = tds._CadenceNode(by_slot)
    rounds = tds._run_rounds(entry_ns, node, [None, frozenset({"target"})])
    node.feed_calls.clear()
    return tds._run_observe_feed(
        entry_ns, node, rounds[1],
        profile=PROFILE, role_separation=role_separation,
        cadence=True, scope=scope,
    ), node.feed_calls


# 逐格期望：(职责分离, 复核范围) -> (锁框, 三态)
# 四种组合要分开写：`职责分离=True + target_candidates` 下候选池为空时**不锁**
# 才是对的（那比锁上场景物体更好），用一格一个期望表达得出来。
def _report(title: str, by_slot, *, expect):
    print(f"\n=== {title} ===")
    ok = True
    for role_separation in (False, True):
        for scope in ("target_candidates", "all_detections"):
            (candidates, state, best, detections), feed = _round(
                by_slot, role_separation=role_separation, scope=scope
            )
            labels = sorted(item["label"] for item in detections)
            locked = None if best is None else best["label"]
            print(
                f"  职责分离={role_separation!s:5s} 复核范围={scope:17s} "
                f"锁框={locked!s:10s} 三态={state:8s} 选框池={labels}"
            )
            LOCKED.append(locked)
            want = expect[(role_separation, scope)]
            if (locked, state) != want:
                ok = False
                print(f"    **不符合预期**：期望锁框={want[0]} 三态={want[1]}")
    return ok


def main() -> int:
    print("画像:", PROFILE.canonical_name_zh)
    print("  detector_terms   :", PROFILE.detector_terms())
    print("  context_labels_en:", PROFILE.context_labels_en, "（拼进 **target** 槽）")

    results = [
        # ① 旧全量有高分上下文词 → 新目标槽只有低分目标词：四格都锁**新**的框。
        _report(
            "场景 1：旧 paper tray 0.95 + desk 0.15 → 新目标槽只给 printer 0.20",
            {None: [TRAY_OLD, DESK_OLD], ("target",): [PRINTER_NEW]},
            expect={
                (False, "target_candidates"): ("printer", "PRESENT"),
                (False, "all_detections"): ("printer", "PRESENT"),
                (True, "target_candidates"): ("printer", "POSSIBLE"),
                (True, "all_detections"): ("printer", "POSSIBLE"),
            },
        ),
        # ② 旧上下文词 → 新目标槽返回空。上下文词一次都没赢；剩下的差异是
        #    任务 3 在默认组合下尚未生效（见文件头"边界"）——那是另一件事。
        _report(
            "场景 2：旧 paper tray 0.95 + desk 0.15 → 新目标槽**空**",
            {None: [TRAY_OLD, DESK_OLD], ("target",): []},
            expect={
                # 默认组合：候选池口径还是"整份 detections"，场景物体 desk 顶上来。
                (False, "target_candidates"): ("desk", "PRESENT"),
                (False, "all_detections"): ("desk", "PRESENT"),
                # 开了职责分离：候选池里没有目标 ⇒ 不锁、报 ABSENT。这是对的。
                (True, "target_candidates"): (None, "ABSENT"),
                (True, "all_detections"): ("desk", "ABSENT"),
            },
        ),
        # ③ 旧全量**只有**上下文词 → 新目标槽空：四格都不锁、都 ABSENT。
        _report(
            "场景 2b：旧全量只有上下文词 → 新目标槽空",
            {None: [TRAY_OLD], ("target",): []},
            expect={
                (False, "target_candidates"): (None, "ABSENT"),
                (False, "all_detections"): (None, "ABSENT"),
                (True, "target_candidates"): (None, "ABSENT"),
                (True, "all_detections"): (None, "ABSENT"),
            },
        ),
    ]

    print("\n=== 对照：框上**没有**词槽来源记录（本改动之前的行为）===")
    print("  （没有 `detection_slots` ⇒ 按旧行为一律留着 ⇒ 旧 0.95 又赢）")
    _round(
        {None: [tds._obj("paper tray", 0.95, (0.10, 0.10, 0.20, 0.30))],
         ("target",): [tds._obj("printer", 0.20, (0.11, 0.11, 0.21, 0.31))]},
        role_separation=False, scope="target_candidates",
    )
    print("  见 tests/test_detection_scheduler.py 的"
          " `test_the_context_word_is_kept_when_the_box_carries_no_slot_record`")

    # 全场不变式：不管哪种开关组合、哪个场景，旧帧那个上下文词**一次都没赢**。
    # 这条比逐格期望更硬 —— 逐格期望会随任务 3 落地而改，这条不该改。
    dirty = [label for label in LOCKED if label == "paper tray"]
    print(f"\n不变式：{len(LOCKED)} 格锁框里出现 `paper tray` 的次数 = {len(dirty)}")

    ok = all(results) and not dirty
    print("判定:", "全部符合预期" if ok else "**有场景不符合预期**")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.path.insert(0, str(ROOT))
    raise SystemExit(main())
