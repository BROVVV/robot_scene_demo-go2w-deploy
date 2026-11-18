"""RAM++ 计划 §42：`scene semantic coverage` 与 `target recall` 必须是两个数。

§42 原文：

    这次不能只看"识别多少 object"。必须两个指标：
    scene semantic coverage / target recall

**这个文件守的是一对命题，缺一半就没有分辨力：**

1. 一次 run 进了**全部**场景物体、但**没有目标** ⇒ `coverage = 1.0` **且** `recall = 0.0`；
2. 一次 run **只**进了目标、场景物体一个没进 ⇒ `coverage = 0.0` **且** `recall = 1.0`。

只测其中一半的话，一个把目标也算进 coverage 的实现（或把场景物体算进 recall 的实现）
照样能全绿。两条合起来才逼出"这两个数量的不是同一件事"。

**为什么这值得守。** 09-16 11:08 那次真机 run（`search_20260916_110826_3f7ac850`）
进了 **57 个物体**、`navigation_failures=0`、`finish_reason=MAX_PLANNING_CYCLES_REACHED`，
目标「红色雨伞」**一次都没确认**（`verify_attempts=0`）。
合并成一个分数，这次 run 会被评成"中等偏上"，而它其实是**彻底失败**。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_GO2W = PROJECT_ROOT / "scripts" / "go2w"
if str(SCRIPTS_GO2W) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_GO2W))

import eval_ram_semantic_coverage as ev  # noqa: E402


def _run(tmp_path: Path, labels: list[str]) -> Path:
    """造一次最小 run：只放 `semantic_map.json`，脚本也只读它。"""
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    (run_dir / "semantic_map.json").write_text(
        json.dumps({"objects": [{"object_id": f"obj_{i:03d}", "label": label}
                                for i, label in enumerate(labels)]},
                   ensure_ascii=False),
        encoding="utf-8",
    )
    return run_dir


def _manifest(tmp_path: Path, **overrides) -> Path:
    payload = {
        "scene": "T",
        "target": "白色垃圾桶",
        "target_present": True,
        "expected_scene_terms": ["办公椅", "办公桌", "隔断"],
    }
    payload.update(overrides)
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    return path


# --------------------------------------------------------------------------
# §42 的核心：两项指标朝相反方向走
# --------------------------------------------------------------------------

def test_all_scene_objects_but_no_target_is_full_coverage_and_zero_recall(
    tmp_path: Path,
) -> None:
    """进了全部场景物体、没有目标 ⇒ coverage 1.0 **且** recall 0.0。"""
    run = _run(tmp_path, ["办公椅", "办公桌", "隔断"])

    result = ev.evaluate(run, _manifest(tmp_path))

    assert result["scene_semantic_coverage"]["ratio"] == 1.0
    assert result["scene_semantic_coverage"]["miss"] == []
    assert result["target_recall"]["ratio"] == 0.0, (
        "目标在场却没进图，这就是召回失败 —— 不许因为它'看起来认得好'而被 coverage 掩盖"
    )


def test_target_only_with_no_scene_objects_is_zero_coverage_and_full_recall(
    tmp_path: Path,
) -> None:
    """只进了目标、场景物体一个没进 ⇒ coverage 0.0 **且** recall 1.0。

    这是上一条的对照。少了它，一个「把目标也算进 coverage」的实现照样能全绿 ——
    而那种实现会把"只找到目标、房间其他东西全没认出来"报成一次好结果。
    """
    run = _run(tmp_path, ["白色垃圾桶"])

    result = ev.evaluate(run, _manifest(tmp_path))

    assert result["scene_semantic_coverage"]["ratio"] == 0.0
    assert set(result["scene_semantic_coverage"]["miss"]) == {"办公椅", "办公桌", "隔断"}
    assert result["scene_semantic_coverage"]["hit"] == 0
    assert result["target_recall"]["ratio"] == 1.0


# --------------------------------------------------------------------------
# 未标注 ≠ 0 分
# --------------------------------------------------------------------------

def test_an_unannotated_manifest_reports_no_number_instead_of_zero(
    tmp_path: Path,
) -> None:
    """没标注 ⇒ `None`（打印成「未标注」），**不是 0.0**。

    与 `webui_map_snapshot_writer.py:385` 那条「拿不到的一律不编」同一个原则：
    0.0 分和"没说"是两件事。把没标注显示成 0.0，会让"我还没标"看起来像
    "这个管线一个物体都没认出来"。
    """
    run = _run(tmp_path, ["办公椅"])

    result = ev.evaluate(run, _manifest(tmp_path, expected_scene_terms=[]))

    assert result["scene_semantic_coverage"] is None
    assert ev._ratio(result["scene_semantic_coverage"]) == "未标注"


def test_an_absent_target_has_no_recall_denominator(tmp_path: Path) -> None:
    """目标不在场 ⇒ recall 无定义（`None`），不是 0.0。

    召回率的分母是"在场的目标数"。目标不在场时它无定义；该看的是**误报率**，
    那是另一个指标，本脚本不算。报 0.0 会让人以为"漏了"，而实际是"不该有"。
    """
    run = _run(tmp_path, ["办公椅", "办公桌", "隔断"])

    result = ev.evaluate(run, _manifest(tmp_path, target_present=False))

    assert result["target_recall"] is None
    assert result["scene_semantic_coverage"]["ratio"] == 1.0


# --------------------------------------------------------------------------
# 匹配口径
# --------------------------------------------------------------------------

def test_coverage_counts_each_expected_term_once_even_if_seen_many_times(
    tmp_path: Path,
) -> None:
    """同一物体被观测多次（地图里有多个 `obj_*` 同 label）不能把覆盖率顶上去。"""
    run = _run(tmp_path, ["办公椅", "办公椅", "办公椅", "办公桌", "隔断"])

    result = ev.evaluate(run, _manifest(tmp_path))

    assert result["scene_semantic_coverage"]["expected"] == 3
    assert result["scene_semantic_coverage"]["hit"] == 3


def test_matching_is_exact_after_normalization_not_substring(tmp_path: Path) -> None:
    """归一化后**精确**匹配，不做子串匹配。

    `label_zh` 的子串回退会把 "computer chair" 塌成「电脑」
    （`app/detectors/vocabulary.py:349-359`）。度量里用子串匹配，
    会把「电脑」算成命中「电脑椅」，把一个错配报成一次成功。
    度量宁可保守（漏配）也不要激进（错配）。
    """
    run = _run(tmp_path, ["电脑"])

    result = ev.evaluate(run, _manifest(tmp_path, expected_scene_terms=["电脑椅"]))

    assert result["scene_semantic_coverage"]["ratio"] == 0.0, (
        "「电脑」不是「电脑椅」，不许因子串关系算命中"
    )


def test_normalization_tolerates_case_and_whitespace(tmp_path: Path) -> None:
    run = _run(tmp_path, ["White  Trash Bin"])

    result = ev.evaluate(
        run,
        _manifest(tmp_path, target="white trash bin", target_present=True,
                  expected_scene_terms=[]),
    )

    assert result["target_recall"]["ratio"] == 1.0


def test_the_real_fixture_manifest_is_loadable_and_annotated() -> None:
    """§43 的 Scene A 清单必须真的能被读出来，且四个场景词都在。

    这条不跑任何模型、不算任何 run —— 它只保证**那份人工标注没被写坏**。
    配 run 的第一真實測量要等一次"自己标注自己那张帧"的 run（见该目录 README）。
    """
    manifest_path = PROJECT_ROOT / "tests" / "fixtures" / "ram_semantic" / "scene_a_office.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    assert manifest["scene"] == "A"
    assert len(manifest["expected_scene_terms"]) == 4
    for frame in manifest["frames"]:
        assert (PROJECT_ROOT / frame).exists(), f"清单引用的帧不在仓库里：{frame}"
