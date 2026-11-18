"""RAM++ 计划 §42/§43：把「场景语义覆盖」与「目标召回」**分开**度量。

§42 原文：

    这次不能只看"识别多少 object"。必须两个指标：
    scene semantic coverage / target recall

**为什么必须分开。** 两者可以朝相反方向走，而合并成一个数就看不见了：

- 09-16 11:08 那次真机 run（`search_20260916_110826_3f7ac850`）进了 **57 个物体**、
  `navigation_failures=0`，看起来"认得很好"；目标「红色雨伞」**一次都没确认**
  （`verify_attempts=0`），最后 `MAX_PLANNING_CYCLES_REACHED`。
  合并成一个数，这次 run 会得到"中等偏上"的分数 —— 而它其实是**彻底失败**。
- 反过来，目标恰好被确认、而房间里的桌子椅子一个都没进图，也是可能的
  （`scene_objects_light` 那条路与目标候选那条路是分开的，
  见 `app/live_robot/autonomous_explorer.py` 的 `if match.has_candidate:` 闸门）。

所以本脚本**只打印两个数，永不打印它们的平均**。`combine_metrics()` 这个函数
**故意不存在** —— 想合并的人得自己写，那就得自己想清楚为什么。

**§43 的数据集边界（如实写在代码里，不假装有）**：
§43 要的 6 类场景里，**C/D/E/F 四类是 mock 注入场景，已经被单元测试覆盖**，
不需要图片：C→`tests/test_ram_target_priority.py`，D→`tests/test_yellow_chair_vlm_verify.py`，
E/F→`tests/test_target_sweep_trigger.py`。**只有 A（普通办公室）和 B（RAM 属性混杂）
需要真实帧**，而它们需要一个 manifest 里的**人工标注**做分母。
脚本对**没标注的条目不打印数字**（打印 `未标注`），因为 0 分与"没说"是两件事
（与 `webui_map_snapshot_writer.py` 那条「拿不到的一律不编」同一个原则）。

用法：

    /usr/bin/python3 scripts/go2w/eval_ram_semantic_coverage.py \\
        --run outputs/live_runs/explore_go2w_20260916_155531 \\
        --manifest tests/fixtures/ram_semantic/scene_a_office.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def normalize(text: Any) -> str:
    """归一化到可比形态。与 `app/video/target_search.py::normalize_label` 同形。

    刻意**不做**子串匹配：`label_zh` 的子串回退会把 "computer chair" 塌成「电脑」
    （见 `app/detectors/vocabulary.py:349-359`），拿它当度量会把两个不同的东西算成一个。
    度量宁可保守（漏配），不要激进（错配）。
    """
    return " ".join(str(text or "").lower().split()).strip(" .,;:，。；：")


def load_run_labels(run_dir: Path) -> tuple[set[str], list[str]]:
    """读一次 run 的语义地图，返回（归一化 label 集合, 原始 label 列表）。"""
    path = run_dir / "semantic_map.json"
    if not path.exists():
        raise SystemExit(f"ERROR: {path} 不存在 —— 这个目录不是一次 explore/search run")
    payload = json.loads(path.read_text(encoding="utf-8"))
    objects = payload.get("objects") or []
    raw = [str(obj.get("label") or "") for obj in objects if isinstance(obj, dict)]
    raw = [item for item in raw if item.strip()]
    return {normalize(item) for item in raw}, raw


def scene_coverage(
    expected_scene_terms: list[str], run_labels: set[str]
) -> dict[str, Any] | None:
    """§42 指标一：**非目标**场景物体进了多少。

    `expected_scene_terms` 为空时返回 `None`（= 未标注），**不是 0.0**。
    """
    expected = [item for item in expected_scene_terms if str(item).strip()]
    if not expected:
        return None
    hit = [item for item in expected if normalize(item) in run_labels]
    miss = [item for item in expected if normalize(item) not in run_labels]
    return {
        "expected": len(expected),
        "hit": len(hit),
        "miss": miss,
        "ratio": len(hit) / len(expected),
    }


def target_recall(target: str, target_present: bool, run_labels: set[str]) -> dict[str, Any] | None:
    """§42 指标二：目标被认出来没有。

    返回 `{"ratio": ...}`，三种情况：

    - 目标**在场**且出现在地图里 → `1.0`
    - 目标**在场**但地图里没有 → `0.0`（**这是召回失败，不是"没目标"**）
    - 目标**不在场** → `None`（没有分母。召回率对不存在的目标无定义，
      该看的是误报率，那是另一个指标，本脚本不算）
    """
    if not target_present:
        return None
    return {"ratio": 1.0 if normalize(target) in run_labels else 0.0, "target": target}


def evaluate(run_dir: Path, manifest_path: Path) -> dict[str, Any]:
    run_labels, raw_labels = load_run_labels(run_dir)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    return {
        "scene": manifest.get("scene", manifest_path.stem),
        "target": manifest.get("target", ""),
        "frames": manifest.get("frames", []),
        "run_object_count": len(raw_labels),
        "run_labels": sorted(raw_labels),
        "scene_semantic_coverage": scene_coverage(
            manifest.get("expected_scene_terms") or [], run_labels
        ),
        "target_recall": target_recall(
            str(manifest.get("target") or ""),
            bool(manifest.get("target_present")),
            run_labels,
        ),
    }


def _ratio(bucket: dict[str, Any] | None) -> str:
    if bucket is None:
        return "未标注"
    return f"{bucket['hit']}/{bucket['expected']} = {bucket['ratio']:.3f}"


def _recall(bucket: dict[str, Any] | None, target_present: bool) -> str:
    if bucket is None:
        return "不适用（目标不在场，召回率无定义）" if not target_present else "未标注"
    return f"{bucket['ratio']:.3f}" + ("" if bucket["ratio"] else "  ← 目标在场但地图里没有")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path, help="一次 run 的目录")
    parser.add_argument("--manifest", required=True, type=Path, help="§43 标注清单")
    parser.add_argument("--json", action="store_true", help="输出 JSON 而不是表格")
    args = parser.parse_args()

    result = evaluate(args.run, args.manifest)
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0

    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    print(f"== Scene {result['scene']} ==  run={args.run}")
    print(f"   run 语义地图物体数：{result['run_object_count']}")
    print(f"   §42 指标一 scene semantic coverage : {_ratio(result['scene_semantic_coverage'])}")
    if result["scene_semantic_coverage"] not in (None,):
        miss = result["scene_semantic_coverage"]["miss"]
        if miss:
            print(f"        未进图：{'、'.join(miss)}")
    print(
        "   §42 指标二 target recall           : "
        f"{_recall(result['target_recall'], bool(manifest.get('target_present')))}"
    )
    print()
    print("   ⚠ 两项**分开**报告，不合并。理由见本文件 docstring。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
