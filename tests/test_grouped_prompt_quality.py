"""§5.1.6 多词分组前向：实验分支本身 + 它的五项质量指标。

**这个文件不是"分组更快"的证据** —— 提速只能在真实回放里测
（`artifacts/perception_optimization/20261013_phase2/grouped_prompt/`），
计划书 §3.2 明令不得从代码结构推断性能收益。

它守的是两件在真实回放里**看不清**的事：

1. **分组分支到底做了什么**：一次前向、caption 的拼法与分隔符、回传 phrase 到
   词的关联规则、关联不上的框是**丢掉并计数**而不是猜一个词。
2. **质量指标算法本身对不对**：IoU 配对必须**一对一**（允许多对一的话，一个
   物体被检出两次就能把召回刷成 1.0）、串词怎么算、`0/0` 不能报成 `0.0`。

第 1 条里最关键的是"关联规则"。阶段一删掉旧 phrase 模式的原因是**串词**：
多词一起喂时 `get_phrases_from_posmap` 会把所有超阈值的 token 连成一串。现在走
`remove_combined=True`，`predict()` 内部先用 `[CLS]/[SEP]/.` 算出分隔符位置、
把每个 query 的激活限制在相邻两个分隔符之间 —— 这是"靠 token 跨度挡串词"，
不是"靠阈值调参"。所以判据必须能分辨"关联对了一个词"与"关联到了隔壁的词"。
"""

from __future__ import annotations

import importlib.util
import io
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from app.config import (
    DEFAULT_GO2W_DINO_PROMPT_MODE,
    Settings,
    get_settings,
)
from app.detectors import grounded_sam_worker as worker
from scripts.go2w.perception_replay_benchmark import (
    _box_iou,
    _boxes_from_payload,
    _build_equivalence,
    _build_quality,
    _build_summary,
    _round_combos,
    _round_order,
    match_boxes,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]


# ---------------------------------------------------------------------------
# 阶段一那份最小 torch 替身：直接加载，不复制
#
# 复制一份的话，"`_detect` 用到的 torch 面"就有了两处定义，改一边另一边静默
# 走偏 —— 而这里要跑的正是**同一个生产 `_detect`**。加载失败就是硬错（下面那条
# 断言先红），不会退化成"跳过"。
# ---------------------------------------------------------------------------
def _load_sibling_stubs():
    path = PROJECT_ROOT / "tests" / "test_grounded_sam_equivalent_optimization.py"
    spec = importlib.util.spec_from_file_location("_phase1_dino_stubs", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


STUBS = _load_sibling_stubs()
_T = STUBS._T


class _Torch(STUBS._FakeTorch):
    """阶段一那份替身 + 分组分支多用的两个面。

    `_predict_grouped` 用了 `dtype=torch.long` 的 `tensor()`，以及**用张量当下标**
    （`boxes[index_tensor]`）。阶段一那份 `tensor()` 写死了 `dtype=float`，浮点数组
    当下标会直接抛 `IndexError`；替身不补这一格，测的就不是生产代码走的那条路。
    """

    long = "long"

    @staticmethod
    def tensor(values, dtype=None):
        if dtype is None:
            return STUBS._FakeTorch.tensor(values)
        return _T(np.asarray(values, dtype=np.int64))


def test_the_sibling_stubs_loaded() -> None:
    assert callable(STUBS._make_predict)
    assert callable(STUBS._fake_nms)


# ---------------------------------------------------------------------------
# 一、prompt 文本的拆解与 phrase → term 的关联
# ---------------------------------------------------------------------------

def test_split_terms_keeps_the_two_level_structure() -> None:
    """`||` 分组是槽位、组内 `.` 才是词 —— 压成一层两个计数就分不开了。"""
    prompts, terms = worker._split_terms("trash can. chair || bin")
    assert prompts == ["trash can. chair", "bin"]
    assert terms == ["trash can", "chair", "bin"]


def test_split_terms_ignores_empty_groups() -> None:
    assert worker._split_terms("  ||  || trash can ") == (["trash can"], ["trash can"])
    assert worker._split_terms("") == ([], [])


def test_an_exact_phrase_maps_back_to_its_term() -> None:
    assert worker.associate_phrase_to_term("trash can", ["trash can", "chair"]) == "trash can"
    # predict 回传的 phrase 已经去过点，但词表里的词本身可能带点。
    assert worker.associate_phrase_to_term("trash can", ["trash. can"]) == "trash. can"


def test_a_unique_partial_phrase_maps_back_to_its_term() -> None:
    """`text_threshold` 高一点时 phrase 会只剩词的一部分。唯一命中就认，不唯一就不认。"""
    assert worker.associate_phrase_to_term("trash", ["trash can", "chair"]) == "trash can"


def test_an_ambiguous_phrase_is_refused_rather_than_guessed() -> None:
    """猜错正是阶段一删掉 phrase 模式的那个串词病：把办公椅的框标成椅子，下游
    会当成"目标就在旁边"。宁可丢掉这个框（调用方会**计数**），也不给它安一个词。
    """
    # 逐字相等优先于包含关系：`chair` 恰好是词表里的一个词，就不该判成歧义。
    assert worker.associate_phrase_to_term("chair", ["armchair", "chair"]) == "chair"
    # `chai` 同时是 `armchair` 与 `chair desk` 的子串 ⇒ 两个命中，不猜。
    assert worker.associate_phrase_to_term("chai", ["armchair", "chair desk"]) is None
    assert worker.associate_phrase_to_term("room", ["living room", "room heater"]) is None


def test_an_unrelated_or_empty_phrase_maps_to_nothing() -> None:
    assert worker.associate_phrase_to_term("garbage bin", ["trash can", "chair"]) is None
    assert worker.associate_phrase_to_term("", ["trash can"]) is None
    assert worker.associate_phrase_to_term("   ", ["trash can"]) is None
    assert worker.associate_phrase_to_term("anything", []) is None


# ---------------------------------------------------------------------------
# 二、分组分支的行为：一次前向、caption 形态、丢掉不猜
# ---------------------------------------------------------------------------

def _grouped_predict(calls: list, rows, scores, phrases):
    def predict(*, model, image, caption, box_threshold, text_threshold, device,
                remove_combined=False):
        del model, image, box_threshold, text_threshold, device
        calls.append({"caption": caption, "remove_combined": remove_combined})
        return (_T(np.asarray(rows, dtype=float)),
                _T(np.asarray(scores, dtype=float)),
                list(phrases))

    return predict


def _run_grouped(terms, rows, phrases, scores=None):
    calls: list = []
    stats: dict = {}
    boxes, confidences, labels = worker._predict_all_prompts(
        None, None, " || ".join(terms), 0.12, 0.10, "cpu", _Torch,
        _grouped_predict(calls, rows, scores or [0.9] * len(rows), phrases),
        stats=stats, prompt_mode="grouped",
    )
    return calls, stats, boxes, confidences, labels


def test_grouped_does_exactly_one_forward_for_all_terms() -> None:
    """一次 `predict()` 吃掉所有词 —— 这就是分组省下来的东西。"""
    calls, stats, boxes, _, labels = _run_grouped(
        ["trash can", "chair"],
        [[0.3, 0.5, 0.2, 0.4], [0.7, 0.6, 0.3, 0.5]],
        ["trash can", "chair"],
    )
    assert len(calls) == 1
    assert stats["dino_forward_count"] == 1
    assert stats["term_count"] == 2, "词数仍按词数记，不能被前向次数顶掉"
    assert labels == ["trash can", "chair"]
    assert len(boxes) == 2


def test_grouped_caption_uses_the_period_separator_and_asks_for_token_spans() -> None:
    """分隔符与 `remove_combined` 是这套机制的两半，缺一个就退回串词那条老路。

    `predict()` 只在 `remove_combined=True` 时才算 `sep_idx`（token id
    `[CLS]/[SEP]/.`）并把 phrase 限制在相邻两个分隔符之间。`.` 换成别的字符，
    `sep_idx` 就是空的，phrase 又会跨词连串。
    """
    calls, stats, _, _, _ = _run_grouped(
        ["trash can", "chair"], [[0.3, 0.5, 0.2, 0.4]], ["trash can"],
    )
    assert calls[0]["remove_combined"] is True
    assert calls[0]["caption"] == "trash can . chair ."
    assert stats["grouped_caption"] == "trash can . chair ."


def test_a_box_whose_phrase_matches_no_term_is_dropped_and_counted() -> None:
    """关联不上就丢 —— 且**必须计数**，否则"丢了多少"在日志里查不出来。"""
    calls, stats, boxes, _, labels = _run_grouped(
        ["trash can", "chair"],
        [[0.3, 0.5, 0.2, 0.4], [0.7, 0.6, 0.3, 0.5], [0.1, 0.1, 0.05, 0.05]],
        ["trash can", "chair", "garbage bin"],
    )
    assert len(calls) == 1
    assert labels == ["trash can", "chair"]
    assert len(boxes) == 2, "第三个框的词不在词表里，不该被安上别的词"
    assert stats["grouped_box_count"] == 3
    assert stats["grouped_kept_boxes"] == 2
    assert stats["grouped_unmatched_boxes"] == 1
    assert stats["grouped_empty_boxes"] == 0


def test_an_empty_phrase_is_counted_separately_from_an_unmatched_one() -> None:
    """空串（激活落在 [CLS]/[SEP] 上的老毛病）与"解出了别的词"是两回事。

    合成一个数就分不出"模型没输出词"和"模型输出了一个我不认识的词"。
    """
    _, stats, boxes, _, _ = _run_grouped(
        ["trash can"], [[0.3, 0.5, 0.2, 0.4], [0.4, 0.4, 0.2, 0.2]], ["", "  "],
    )
    assert len(boxes) == 0
    assert stats["grouped_empty_boxes"] == 2
    assert stats["grouped_unmatched_boxes"] == 0


def test_grouped_with_no_terms_does_no_forward_at_all() -> None:
    calls: list = []
    stats: dict = {}
    boxes, _, labels = worker._predict_all_prompts(
        None, None, "   ", 0.12, 0.10, "cpu", _Torch,
        _grouped_predict(calls, [], [], []), stats=stats, prompt_mode="grouped",
    )
    assert calls == []
    assert labels == [] and len(boxes) == 0


def test_per_term_still_forwards_once_per_term() -> None:
    """默认路径逐字不变：词数 == 前向次数，caption 是单词、不带 remove_combined。"""
    calls: list = []

    def predict(*, model, image, caption, box_threshold, text_threshold, device,
                remove_combined=False):
        del model, image, box_threshold, text_threshold, device
        calls.append({"caption": caption, "remove_combined": remove_combined})
        return (_T(np.asarray([[0.3, 0.5, 0.2, 0.4]], dtype=float)),
                _T(np.asarray([0.9])), [])

    stats: dict = {}
    _, _, labels = worker._predict_all_prompts(
        None, None, "trash can || chair", 0.12, 0.10, "cpu", _Torch,
        predict, stats=stats,
    )
    assert [call["caption"] for call in calls] == ["trash can", "chair"]
    assert all(call["remove_combined"] is False for call in calls)
    assert labels == ["trash can", "chair"]
    assert stats["term_count"] == stats["dino_forward_count"] == 2


def test_an_unknown_prompt_mode_falls_back_and_keeps_the_requested_value() -> None:
    """生效值/请求值分两个字段：静默回落会让"我开的是分组"看不出来。"""
    assert worker._resolve_prompt_mode("grouped") == ("grouped", "grouped")
    assert worker._resolve_prompt_mode(" Grouped ") == ("grouped", "grouped")
    assert worker._resolve_prompt_mode("group") == ("per_term", "group")
    assert worker._resolve_prompt_mode(None) == ("per_term", "")


def test_the_default_prompt_mode_is_per_term() -> None:
    """§5.1.6：分组是**待评估的候选**，默认必须还是逐词。"""
    assert DEFAULT_GO2W_DINO_PROMPT_MODE == "per_term"
    assert worker.DEFAULT_PROMPT_MODE == "per_term"
    assert worker.DEFAULT_PROMPT_MODE == DEFAULT_GO2W_DINO_PROMPT_MODE, (
        "worker 与 app 两份字面量必须一致 —— worker 跑在另一个解释器里、"
        "import 不到 app，只能靠这条判据把它们绑在一起"
    )


# ---------------------------------------------------------------------------
# 三、端到端：`options["prompt_mode"]` 真的走到了 DINO
# ---------------------------------------------------------------------------

def _run_detect(prompt_mode: str, terms: list[str]):
    """跑一次**真的** `_detect()`，只替换模型与 `predict`。

    与只读源码的结构守卫相比，这一条能抓到"开关解析对了、却没往后传"——
    本仓库翻过的那类车（入口那一跳没人管）。
    """
    model = STUBS._FakeGroundingDINO()
    sam = STUBS._FakeSAM2()
    stats: dict = {}
    calls: list = []

    def predict(*, model, image, caption, box_threshold, text_threshold, device,
                remove_combined=False):
        model(image, captions=[caption])
        calls.append({"caption": caption, "remove_combined": remove_combined})
        if remove_combined:
            # 三个框、三个 phrase：其中一个 phrase（garbage bin）不在词表里。
            return (_T(np.array([[0.3, 0.5, 0.2, 0.4],
                                 [0.7, 0.6, 0.3, 0.5],
                                 [0.15, 0.2, 0.1, 0.1]], dtype=float)),
                    _T(np.array([0.91, 0.83, 0.40])),
                    ["trash can", "chair", "garbage bin"])
        rows = {"trash can": [[0.3, 0.5, 0.2, 0.4]], "chair": [[0.7, 0.6, 0.3, 0.5]]}
        values = rows.get(caption, [])
        return (_T(np.array(values, dtype=float) if values else np.zeros((0, 4))),
                _T(np.array([0.9] * len(values))),
                [])

    objects = worker._detect(
        SimpleNamespace(), "frame.jpg", " || ".join(terms),
        0.12, 0.10, 100, 0.35,
        model, sam, "cpu", _Torch, np,
        lambda path: (
            np.zeros((STUBS._HEIGHT, STUBS._WIDTH, 3), dtype=np.uint8),
            _T(np.zeros((3, STUBS._HEIGHT, STUBS._WIDTH))),
        ),
        predict, STUBS._fake_box_convert, STUBS._fake_nms,
        options={"prompt_mode": prompt_mode, "request_id": "grouped-test"},
        stats=stats,
    )
    return objects, stats, calls


def test_the_switch_reaches_the_dino_call_through_the_real_detect() -> None:
    objects, stats, calls = _run_detect("grouped", ["trash can", "chair"])

    assert stats["prompt_mode"] == "grouped", "生效值没进 diagnostics"
    assert stats["dino_forward_count"] == 1, "整帧只该有一次前向"
    assert stats["term_count"] == 2
    assert len(calls) == 1 and calls[0]["remove_combined"] is True
    labels = sorted(item["label"] for item in objects)
    assert labels == ["chair", "trash can"], f"关联不上的框漏进了结果：{labels}"
    assert stats["grouped_unmatched_boxes"] == 1


def test_the_default_path_through_the_real_detect_is_unchanged() -> None:
    """不传 `prompt_mode` 时逐字走老路 —— 这条保证默认值是真的没变。"""
    objects, stats, calls = _run_detect("per_term", ["trash can", "chair"])

    assert stats["prompt_mode"] == "per_term"
    assert stats["dino_forward_count"] == stats["term_count"] == 2
    assert [call["caption"] for call in calls] == ["trash can", "chair"]
    assert all(call["remove_combined"] is False for call in calls)
    assert sorted(item["label"] for item in objects) == ["chair", "trash can"]
    assert "grouped_unmatched_boxes" not in stats


def test_the_config_switch_accepts_grouped_and_shouts_on_a_typo(monkeypatch, capsys) -> None:
    """`GO2W_DINO_PROMPT_MODE` 的生效值。写错了要**出声**，不能静默回落。"""
    monkeypatch.delenv("GO2W_DINO_PROMPT_MODE", raising=False)
    get_settings.cache_clear()
    try:
        assert get_settings().go2w_dino_prompt_mode == "per_term"
    finally:
        get_settings.cache_clear()

    monkeypatch.setenv("GO2W_DINO_PROMPT_MODE", "grouped")
    get_settings.cache_clear()
    try:
        assert get_settings().go2w_dino_prompt_mode == "grouped"
    finally:
        get_settings.cache_clear()

    monkeypatch.setenv("GO2W_DINO_PROMPT_MODE", "Group")
    get_settings.cache_clear()
    try:
        assert get_settings().go2w_dino_prompt_mode == "per_term"
        assert "GO2W_DINO_PROMPT_MODE" in capsys.readouterr().err
    finally:
        get_settings.cache_clear()


def test_the_request_body_carries_the_configured_prompt_mode() -> None:
    """`Settings → 请求体`这一跳。只断言 config 的话，"配了但没发出去"照样全绿。

    真正写进 worker stdin 的那一行 JSON 才是契约 —— worker 是另一个解释器，
    它只认请求体里的字段（命令行那份对 `--serve` 是够不着的死路径）。
    """
    from app.detectors import grounded_sam_subprocess as gss

    settings = Settings(
        siliconflow_api_key="", grounded_sam_root="/tmp",
        grounded_sam_python="python", go2w_dino_prompt_mode="grouped",
    )
    detector = gss.GroundedSAMSubprocessDetector(settings)
    detector._ensure_worker = lambda: None                # type: ignore[method-assign]
    detector._worker = SimpleNamespace(stdin=io.StringIO())
    detector._read_response = lambda: {"objects": []}     # type: ignore[method-assign]

    detector._request(image=Path("frame.jpg"), text_prompt="trash can", frame_id="5465")
    written = json.loads(detector._worker.stdin.getvalue().strip())
    assert written["dino_prompt_mode"] == "grouped"
    assert written["frame_id"] == "5465"
    assert detector.last_diagnostics_available is False, (
        "响应里没有 diagnostics 时要明确记成没有，不能把历史值冒充成生效值"
    )


# ---------------------------------------------------------------------------
# 四、质量指标：召回 / 串词 / 重复框 / 分数变化
# ---------------------------------------------------------------------------

def _box(term: str, bbox, score: float = 0.9) -> dict:
    return {"term": term, "bbox": list(bbox), "score": score}


def test_iou_of_identical_and_disjoint_boxes() -> None:
    assert _box_iou([0, 0, 1, 1], [0, 0, 1, 1]) == pytest.approx(1.0)
    assert _box_iou([0, 0, 0.4, 1], [0.6, 0, 1, 1]) == 0.0
    assert _box_iou([], [0, 0, 1, 1]) == 0.0
    assert _box_iou([0, 0, 0.5, 1], [0.5, 0, 1, 1]) == 0.0, "贴边不算相交"


def test_matching_is_one_to_one_so_a_double_detection_cannot_fake_recall() -> None:
    """**这条是召回算法的命门。**

    允许一个参照框被配走两次（多对一）时，一个物体被检出两次就把召回刷成
    1.0 —— 漏掉的那个物体的缺失被藏起来了。重复框必须单列，不能算召回。
    """
    reference = [_box("trash can", [0, 0, 0.5, 0.5])]
    candidate = [
        _box("trash can", [0, 0, 0.5, 0.5], score=0.95),
        _box("trash can", [0.001, 0.001, 0.501, 0.501], score=0.94),
    ]
    result = match_boxes(reference, candidate)
    assert result["matched"] == 1
    assert result["recall"] == pytest.approx(1.0)
    assert result["duplicates"] == 1, "第二个框配到了同一个参照框，是重复框"
    assert result["extra"] == 1


def test_a_box_that_landed_on_the_right_object_with_the_wrong_word_is_cross_talk() -> None:
    """串词的判据：几何配上、**词不一致**。

    `recall`（词也要对）与 `geometry_recall`（只看几何）之差就是这个数 ——
    只报一个召回，串词就隐身了。
    """
    reference = [_box("office chair", [0.5, 0.3, 0.8, 0.75])]
    candidate = [_box("chair", [0.5, 0.3, 0.8, 0.75])]
    result = match_boxes(reference, candidate)
    assert result["geometry_recall"] == pytest.approx(1.0)
    assert result["recall"] == 0.0
    assert result["label_flips"] == 1
    assert result["matches"][0]["label_flip"] is True


def test_a_missing_box_is_missed_not_a_flip() -> None:
    result = match_boxes([_box("trash can", [0, 0, 0.5, 0.5])], [])
    assert result["missed"] == 1
    assert result["matched"] == 0
    assert result["label_flips"] == 0
    assert result["recall"] == 0.0


def test_an_empty_reference_reports_none_instead_of_zero() -> None:
    """一帧里本来就没框（`0/0`）与"一个都没配上"是两件事。"""
    result = match_boxes([], [_box("trash can", [0, 0, 0.5, 0.5])])
    assert result["recall"] is None
    assert result["geometry_recall"] is None
    assert result["extra"] == 1


def test_a_low_iou_overlap_is_not_a_match() -> None:
    """门槛是**生效**的：这一对的 IoU 是 0.333，0.5 那一档配不上、0.3 那一档配得上。"""
    reference = [_box("trash can", [0, 0, 1, 1])]
    candidate = [_box("trash can", [0.5, 0, 1.5, 1])]
    assert _box_iou(reference[0]["bbox"], candidate[0]["bbox"]) == pytest.approx(1 / 3, abs=1e-4)
    assert match_boxes(reference, candidate, iou_threshold=0.5)["matched"] == 0
    assert match_boxes(reference, candidate, iou_threshold=0.3)["matched"] == 1


def test_score_delta_is_reported_for_matched_pairs_only() -> None:
    reference = [_box("trash can", [0, 0, 0.5, 0.5], score=0.90)]
    candidate = [_box("trash can", [0, 0, 0.5, 0.5], score=0.78)]
    result = match_boxes(reference, candidate)
    assert result["score_delta_max"] == pytest.approx(0.12)
    assert result["score_delta_mean"] == pytest.approx(0.12)


def test_boxes_are_read_from_the_producer_payload_shape() -> None:
    """`term` 必须取 `source_prompt_term`（"这个框是哪个词生成的"），不是 `label`。"""
    payload = json.dumps({"objects": [
        {"label": "trash can", "label_zh": "垃圾桶", "score": 0.9,
         "bbox_2d": [0, 0, 1, 1], "source_prompt_term": "trash can"},
    ]})
    parsed = _boxes_from_payload(payload)
    assert parsed[0]["term"] == "trash can"
    assert parsed[0]["bbox"] == [0, 0, 1, 1]
    assert parsed[0]["score"] == pytest.approx(0.9)
    assert _boxes_from_payload("not json") == []
    assert _boxes_from_payload(json.dumps({"objects": []})) == []


def _objects(term_boxes) -> str:
    return json.dumps({"objects": [
        {"label": term, "source_prompt_term": term, "score": score, "bbox_2d": bbox}
        for term, bbox, score in term_boxes
    ]})


_REFERENCE = [("trash can", [0, 0, 0.5, 0.5], 0.9), ("chair", [0.5, 0.5, 1, 1], 0.8)]


def _quality_fixture(candidate_boxes: list) -> dict:
    """一帧一实现：逐词那侧固定，分组那侧由调用方给。"""
    objects_by_key = {}
    for repeat in (0, 1):
        objects_by_key[("inprocess", "5465", "optimized", "per_term", repeat)] = (
            _objects(_REFERENCE)
        )
        objects_by_key[("inprocess", "5465", "optimized", "grouped", repeat)] = (
            _objects(candidate_boxes)
        )
    return objects_by_key


def test_identical_results_adopt_and_are_reported_as_such() -> None:
    quality = _build_quality(
        _quality_fixture(_REFERENCE), prompt_modes=["per_term", "grouped"]
    )
    assert quality["skipped_pairs"] == 0
    assert quality["reference_prompt_mode"] == "per_term", "参照必须是现行路径那一侧"
    total = quality["totals"]["grouped"]
    assert total["repeats"] == 2
    assert (total["missed"], total["label_flips"], total["duplicates"]) == (0, 0, 0)
    assert quality["verdict"]["grouped"]["adopt"] is True
    assert "per_term" not in quality["totals"], "参照模式不该出现在候选侧的汇总里"
    # 延迟不在这里算：数据源是 timings 而不是对象 JSON，两处都算会长出两个数。
    assert "latency" not in quality and "detect_ms" not in total


def test_a_lost_box_rejects_adoption() -> None:
    """§3.2：分组不得新增漏检。丢掉一个物体就不能判采用。"""
    lost = [("trash can", [0, 0, 0.5, 0.5], 0.9)]
    quality = _build_quality(_quality_fixture(lost), prompt_modes=["per_term", "grouped"])
    total = quality["totals"]["grouped"]
    assert total["missed"] == 2, "两轮各漏一个"
    assert total["recall"] == pytest.approx(0.5)
    assert quality["verdict"]["grouped"]["adopt"] is False


def test_cross_talk_rejects_adoption_even_when_nothing_is_lost() -> None:
    """框都在、位置都对，只有词串了 —— 这正是阶段一删掉 phrase 模式的那个病。"""
    flipped = [("chair", [0, 0, 0.5, 0.5], 0.9), ("trash can", [0.5, 0.5, 1, 1], 0.8)]
    quality = _build_quality(_quality_fixture(flipped), prompt_modes=["per_term", "grouped"])
    total = quality["totals"]["grouped"]
    assert total["missed"] == 0, "几何上一个都没漏"
    assert total["label_flips"] == 4, "两轮各两个词串了"
    assert quality["verdict"]["grouped"]["adopt"] is False


def test_a_reference_repeat_without_a_candidate_is_skipped_not_counted_as_zero() -> None:
    """配不齐的轮次要跳过并**计数**，不能静默按"零个框"计进召回。"""
    objects_by_key = _quality_fixture(_REFERENCE)
    del objects_by_key[("inprocess", "5465", "optimized", "grouped", 1)]
    quality = _build_quality(objects_by_key, prompt_modes=["per_term", "grouped"])
    assert quality["skipped_pairs"] == 1
    assert quality["totals"]["grouped"]["repeats"] == 1


# ---------------------------------------------------------------------------
# 五、延迟：两根正交的轴不能合并成一个数
# ---------------------------------------------------------------------------

def _timing(mode="inprocess", implementation="optimized", prompt_mode="per_term",
            frame_id="5465", repeat=0, detect_ms=100.0, prompt="trash can. chair") -> dict:
    return {
        "mode": mode, "implementation": implementation, "prompt_mode": prompt_mode,
        "frame_id": frame_id, "repeat": repeat, "warmup": False, "order_index": 0,
        "ram_ms": 1.0, "detect_ms": detect_ms, "total_ms": 2.0,
        "object_count": 0, "mask_count": 0, "prompt": prompt,
        "ram_tags": [], "live_ram_tags": [], "objects_sha256": "x", "diagnostics": {},
    }


def test_the_two_prompt_modes_never_land_in_the_same_bucket() -> None:
    """桶键不带 `prompt_mode` 的话，"逐词和分组混在一起"的均值就出厂了。"""
    timings = [_timing(prompt_mode="per_term", detect_ms=100.0),
               _timing(prompt_mode="grouped", detect_ms=40.0)]
    summary = _build_summary(timings, ["optimized"], {})
    assert summary["by_mode_and_implementation"][
        "inprocess/optimized/per_term/detect"]["mean"] == pytest.approx(100.0)
    assert summary["by_mode_and_implementation"][
        "inprocess/optimized/grouped/detect"]["mean"] == pytest.approx(40.0)

    delta = summary["prompt_mode_delta"]["inprocess/optimized/per_term_vs_grouped"]
    assert delta["per_term_mean_ms"] == pytest.approx(100.0)
    assert delta["grouped_mean_ms"] == pytest.approx(40.0)
    assert delta["reduction_pct"] == pytest.approx(60.0)


def test_speedup_is_keyed_by_prompt_mode_too() -> None:
    """baseline/optimized 是另一根轴，两根轴交叉处各报一个提速。"""
    timings = [
        _timing(implementation="baseline", prompt_mode="grouped", detect_ms=90.0),
        _timing(implementation="optimized", prompt_mode="grouped", detect_ms=30.0),
    ]
    summary = _build_summary(timings, ["baseline", "optimized"], {})
    assert summary["speedup"]["inprocess/grouped/detect_mean"]["reduction_pct"] == (
        pytest.approx(66.67, abs=0.01)
    )


# ---------------------------------------------------------------------------
# 六、等价性报告要按 prompt 模式分支
# ---------------------------------------------------------------------------

def test_the_prompt_identity_assertion_is_scoped_to_one_prompt_mode() -> None:
    """**这条就是计划书点名的那个坑。**

    分组模式下送去 DINO 的 caption 与逐词**本就不一样**（一条多词 caption vs
    N 条单词 caption）。把两种模式的行塞进同一个键去比"词表必须逐字相同"，
    分组那一侧会被无条件判死 —— 那不是发现缺陷，那是判据写错了。

    同一模式内这条断言**一点没放松**：同一模式的两次运行词表不同，照样红。
    """
    timings = [_timing(prompt_mode="per_term"), _timing(prompt_mode="grouped")]
    equivalence = _build_equivalence(["optimized"], {}, timings)
    assert equivalence["all_prompts_identical"] is True
    assert "inprocess/per_term/5465" in equivalence["prompt_identical"]
    assert "inprocess/grouped/5465" in equivalence["prompt_identical"]

    # 两种模式跑了**不同的词表**（操作失误，或有意做词表对照）时同样不能判死：
    # 这条断言管的是"同一模式内两次运行的词表是否逐字相同"，跨模式那点差异
    # 由信息项报给报告作者。键里少一个 `prompt_mode` 就会把这里判红。
    per_mode_vocab = [
        _timing(prompt_mode="per_term", prompt="trash can. chair"),
        _timing(prompt_mode="grouped", prompt="trash can. chair. bin"),
    ]
    mixed = _build_equivalence(["optimized"], {}, per_mode_vocab)
    assert mixed["all_prompts_identical"] is True
    assert mixed["prompt_text_same_across_prompt_modes"]["inprocess/5465"] is False

    # 同一模式内词表不一致 ⇒ 判红。断言一点没放松。
    dirty = [_timing(prompt_mode="grouped", prompt="trash can. chair"),
             _timing(prompt_mode="grouped", prompt="trash can. chair. bin")]
    assert _build_equivalence(["optimized"], {}, dirty)["all_prompts_identical"] is False


def test_the_cross_mode_prompt_note_is_informational_only() -> None:
    """跨模式那条只是信息项：输入文本相同而实际 caption 条数不同，是本实验的定义。"""
    timings = [_timing(prompt_mode="per_term"), _timing(prompt_mode="grouped")]
    equivalence = _build_equivalence(["optimized"], {}, timings)
    assert equivalence["prompt_text_same_across_prompt_modes"]["inprocess/5465"] is True
    assert "prompt_mode_note" in equivalence


# ---------------------------------------------------------------------------
# 七、对照轴的顺序
# ---------------------------------------------------------------------------

def test_one_prompt_mode_reproduces_the_phase_one_round_order_exactly() -> None:
    """只跑一种模式时投影回实现轴，必须与 `_round_order` **逐项相同**。

    否则同一份数据会有两个顺序，阶段一那份证据就不再可比（计划书要求复用
    `_round_order`）。
    """
    implementations = ["baseline", "optimized"]
    for order in ("alternate", "random"):
        combos = _round_combos(order, implementations, ["per_term"], 4)
        assert [[name for name, _ in round_] for round_ in combos] == _round_order(
            order, implementations, 4
        )
    assert _round_combos("alternate", implementations, ["per_term"], 2) == [
        [("baseline", "per_term"), ("optimized", "per_term")],
        [("optimized", "per_term"), ("baseline", "per_term")],
    ]


def test_two_axes_alternate_every_repeat_and_keep_every_combo() -> None:
    rounds = _round_combos("alternate", ["optimized"], ["per_term", "grouped"], 3)
    assert rounds[0] == [("optimized", "per_term"), ("optimized", "grouped")]
    assert rounds[1] == [("optimized", "grouped"), ("optimized", "per_term")]
    assert rounds[2] == rounds[0], "ABBA：奇偶交替才抵得掉 GPU 的线性漂移"
    assert sorted(rounds[0]) == sorted(rounds[1])


def test_the_two_axis_random_order_is_reproducible() -> None:
    """两轴随机必须**同种子可复现**，否则对照结果没法被别人重跑。"""
    implementations = ["baseline", "optimized"]
    first = _round_combos("random", implementations, ["per_term", "grouped"], 3)
    second = _round_combos("random", implementations, ["per_term", "grouped"], 3)
    assert first == second
    every_combo = sorted(_round_combos(
        "alternate", implementations, ["per_term", "grouped"], 1
    )[0])
    for round_ in first:
        assert sorted(round_) == every_combo, "每一轮都要把四个组合各跑一次"


def test_the_quality_artifact_is_only_written_when_both_modes_ran() -> None:
    """只跑一种模式时不写 `quality.json`：空壳会被读成"对照过了，没问题"。"""
    source = (
        PROJECT_ROOT / "scripts" / "go2w" / "perception_replay_benchmark.py"
    ).read_text(encoding="utf-8")
    assert "if len(prompt_modes) > 1:" in source
    assert '"quality.json"' in source
