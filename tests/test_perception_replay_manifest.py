"""回放工具的「清单冻结」语义（独立审查 R3）。

审查要求：*「无显式覆盖时，读取清单中的完整目标与最终词表；明确 CLI 覆盖规则并
记录偏离。区分'字段缺失'和'已冻结为空'。增加非默认目标、目标解析变化及空场景
词表的复跑测试。」*

这些判据原来长在 `perception_replay_benchmark.main()` 里（要起真模型才能跑到），
所以本文件只驱动抽出来的两个纯函数：`_load_manifest()` 与
`resolve_target_and_vocabulary()`。判据本身与 GPU 无关 —— 它问的是"这份清单
冻结了什么、复跑时会不会被静默换掉"。

三个容易写错的点，各自都有测试：

1. **空词表也是冻结**。`ram_tags_en: []` 是合法的「这一帧就是没有场景词」，
   用真值判断（`if item.get("ram_tags_en")`）会把它当成"没冻结"，复跑时又去问
   一次 RAM++，拿到什么算什么 —— 对照的承诺就没了。
2. **目标必须一起冻结**。只传 `--manifest` 复跑时目标会落到 CLI 默认值，
   对最初用别的目标跑出来的清单，这会把目标静默换掉、词表整个变样。
3. **换了目标就不能套旧词表**。清单里的 `target_terms` 是"当时那个目标"的
   词表，目标变了还用它自相矛盾 —— 那时必须现场重算并如实标成 `recomputed`。
"""

from __future__ import annotations

import json
from pathlib import Path

from scripts.go2w.perception_replay_benchmark import (
    DEFAULT_TARGET,
    _load_manifest,
    resolve_target_and_vocabulary,
)

RAM_TAGS = ["bin", "chair", "floor"]


def _write(tmp_path: Path, payload: object) -> Path:
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    return path


def _frozen(**overrides) -> dict:
    """一份"什么都没冻结"的清单描述，逐条覆盖成要测的形态。"""
    base = {
        "has_vocabulary": False,
        "ram_tags": {},
        "target": None,
        "target_terms": None,
        "target_prompt": None,
    }
    base.update(overrides)
    return base


# ---------------------------------------------------------------------------
# 一、清单解析：冻结了什么
# ---------------------------------------------------------------------------


def test_a_capture_json_list_freezes_nothing(tmp_path: Path) -> None:
    """旧格式（顶层 list）：只冻结帧，目标与词表都不在清单里。"""
    path = _write(tmp_path, [{"frame": {"frame_id": "5465", "color_ref": "/x.jpg"}}])
    frames, frozen = _load_manifest(path)
    assert [frame["frame_id"] for frame in frames] == ["5465"]
    assert frozen == _frozen()


def test_an_empty_ram_tag_list_counts_as_frozen(tmp_path: Path) -> None:
    """`ram_tags_en: []` 是"已冻结为空"，不是"字段缺失"。"""
    path = _write(tmp_path, {"frames": [
        {"frame_id": "a", "color_ref": "/a.jpg", "ram_tags_en": []},
    ]})
    _, frozen = _load_manifest(path)
    assert frozen["has_vocabulary"] is True, "空列表必须算冻结，否则复跑会重问 RAM++"
    assert frozen["ram_tags"] == {"a": []}


def test_a_missing_ram_tags_key_is_not_frozen(tmp_path: Path) -> None:
    """同一份清单里，**没有这个键**才是"没冻结"。"""
    path = _write(tmp_path, {"frames": [{"frame_id": "a", "color_ref": "/a.jpg"}]})
    _, frozen = _load_manifest(path)
    assert frozen["has_vocabulary"] is False
    assert frozen["ram_tags"] == {}


def test_one_frozen_frame_is_enough_to_call_the_manifest_frozen(tmp_path: Path) -> None:
    """部分帧冻结也算冻结：有键的那帧照抄，没键的那帧由调用方再补。"""
    path = _write(tmp_path, {"frames": [
        {"frame_id": "a", "color_ref": "/a.jpg", "ram_tags_en": RAM_TAGS},
        {"frame_id": "b", "color_ref": "/b.jpg"},
    ]})
    _, frozen = _load_manifest(path)
    assert frozen["has_vocabulary"] is True
    assert frozen["ram_tags"] == {"a": RAM_TAGS}
    assert "b" not in frozen["ram_tags"]


# ---------------------------------------------------------------------------
# 二、目标解析：CLI > 清单 > 默认值，且偏离要留痕
# ---------------------------------------------------------------------------


def test_the_manifest_target_is_used_when_the_cli_says_nothing() -> None:
    out = resolve_target_and_vocabulary(None, _frozen(target="红色杯子"))
    assert out["record"]["target"] == "红色杯子"
    assert out["record"]["target_source"] == "manifest"
    assert "target_deviation" not in out["record"]


def test_the_builtin_default_is_used_when_neither_says_anything() -> None:
    out = resolve_target_and_vocabulary(None, _frozen())
    assert out["record"]["target"] == DEFAULT_TARGET
    assert out["record"]["target_source"] == "default"


def test_an_explicit_target_wins_and_the_deviation_is_recorded() -> None:
    """显式换了目标必须留痕，否则证据里的词表和清单里的对不上却看着都正常。"""
    out = resolve_target_and_vocabulary("红色杯子", _frozen(target="绿色垃圾桶"))
    record = out["record"]
    assert record["target"] == "红色杯子"
    assert record["target_source"] == "cli"
    assert record["target_deviation"] == {
        "manifest_target": "绿色垃圾桶",
        "cli_target": "红色杯子",
    }


def test_saying_the_same_target_as_the_manifest_is_not_a_deviation() -> None:
    out = resolve_target_and_vocabulary("绿色垃圾桶", _frozen(target="绿色垃圾桶"))
    assert out["record"]["target_source"] == "cli"
    assert "target_deviation" not in out["record"]


# ---------------------------------------------------------------------------
# 三、最终词表：只有目标一致时才认清单里的那份
# ---------------------------------------------------------------------------


def test_a_non_default_target_gets_its_own_terms_not_the_default_ones() -> None:
    """换目标就换词表：`垃圾桶` 与默认目标的 detector_terms 必须不同。

    这一条同时是"目标解析真的在起作用"的阳性对照 —— 如果哪天 `resolve()`
    对任何输入都返回同一份词表，这里会红。
    """
    trash = resolve_target_and_vocabulary("垃圾桶", _frozen())["target_terms"]
    default = resolve_target_and_vocabulary(None, _frozen())["target_terms"]
    assert trash and default
    assert "trash can" in [term.lower() for term in trash]
    assert trash != default


def test_the_frozen_terms_are_used_when_the_target_matches() -> None:
    out = resolve_target_and_vocabulary(None, _frozen(
        target="红色杯子", target_terms=["red cup", "crimson mug"],
        target_prompt="red cup. crimson mug",
    ))
    assert out["target_terms"] == ["red cup", "crimson mug"]
    assert out["target_prompt"] == "red cup. crimson mug"
    assert out["record"]["target_terms_source"] == "manifest"


def test_changing_the_target_drops_the_frozen_terms() -> None:
    """换了目标还套旧词表是自相矛盾的：那时必须重算并标成 recomputed。"""
    out = resolve_target_and_vocabulary("蓝色椅子", _frozen(
        target="红色杯子", target_terms=["red cup"], target_prompt="red cup",
    ))
    assert out["record"]["target_terms_source"] == "recomputed"
    assert out["target_terms"] != ["red cup"]


def test_a_frozen_prompt_survives_even_without_frozen_terms() -> None:
    """清单只冻了 prompt（没有 terms）时，prompt 仍优先，terms 由它拼出来。"""
    out = resolve_target_and_vocabulary(None, _frozen(target="红色杯子"))
    assert out["record"]["target_terms_source"] == "recomputed"
    assert out["target_prompt"] == ". ".join(out["target_terms"])


# ---------------------------------------------------------------------------
# 四、场景词表：冻结为空也要说自己是冻结的
# ---------------------------------------------------------------------------


def test_an_empty_frozen_vocabulary_still_reports_manifest_as_the_source() -> None:
    out = resolve_target_and_vocabulary(None, _frozen(
        has_vocabulary=True, ram_tags={"a": []},
    ))
    assert out["record"]["vocabulary_source"] == "manifest"
    assert out["vocab"] == {"a": []}


def test_without_a_frozen_vocabulary_the_source_says_live_ram() -> None:
    out = resolve_target_and_vocabulary(None, _frozen())
    assert out["record"]["vocabulary_source"] == "live RAM++"
    assert out["vocab"] == {}


def test_the_returned_vocabulary_is_a_copy_not_the_manifest_dict() -> None:
    """复跑要在 vocab 上 setdefault 补帧 —— 不能写回清单里那份描述。"""
    frozen = _frozen(has_vocabulary=True, ram_tags={"a": RAM_TAGS})
    out = resolve_target_and_vocabulary(None, frozen)
    out["vocab"].setdefault("b", ["x"])
    assert frozen["ram_tags"] == {"a": RAM_TAGS}
