"""阶段二质量评估集 runner 的判据（计划书 §3.2 / §5.1）。

评估集本身是**证据**（`artifacts/` 被 .gitignore 忽略），但**跑它的工具是交付物**
（阶段一第三、四次复审的教训：脚本不在版本控制里，报告要人跑就跑不起来）。
所以这些判据全部对着 `scripts/go2w/perception_quality_eval.py` 跑。

三层：

1. **不漂移**：runner 里的 `DetectedObject → dict` 字段集必须与生产那份
   `_gsam_to_dict` 逐字相同 —— 差一个字段，评估集测的就不是生产行为。
2. **变体语义**：`baseline` 必须复现阶段二**之前**的真实语义（二分 + 全量
   argmax），`phase2` 必须复现任务 2/3 的语义。这是整个对照的立足点：
   变体错一个，结论就是假的。
3. **判据算术**：`_judge_row` / `_summarize_variant` / `_verdict` 的账要能手工复核。

**不需要 GPU**：这里只跑判决层与纯函数，检测那一段在 GPU 环境里跑。
"""

from __future__ import annotations

import ast
import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
EVAL_PATH = ROOT / "scripts" / "go2w" / "perception_quality_eval.py"
LOOP_PATH = ROOT / "scripts" / "go2w" / "run_autonomous_loop.py"


def _load_eval():
    spec = importlib.util.spec_from_file_location("_quality_eval_under_test", EVAL_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def ev():
    return _load_eval()


def _profile(canonical="椅子", terms=("chair",), anchors=()):
    from app.reasoning.target_profile import TargetProfile

    return TargetProfile(
        raw_query=canonical,
        canonical_name_zh=canonical,
        primary_labels_en=list(terms),
        relation_anchor_labels_en=list(anchors),
    )


def _obj(term, score, label=None, bbox=(0.1, 0.1, 0.3, 0.3)):
    return {
        "label": label or term,
        "label_zh": term,
        "category": term,
        "source_prompt_term": term,
        "score": score,
        "bbox_2d": list(bbox),
        "mask_rle": None,
        "mask_size": None,
        "mask_area_ratio": None,
    }


# --------------------------------------------------------------------------- #
# 1) 字段集不许漂移
# --------------------------------------------------------------------------- #
def _dict_keys_from_source(path: Path, func_name: str) -> set[str]:
    """取出一个「return {字面量}」函数的键集合。"""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == func_name:
            for stmt in ast.walk(node):
                if isinstance(stmt, ast.Return) and isinstance(stmt.value, ast.Dict):
                    return {
                        key.value
                        for key in stmt.value.keys
                        if isinstance(key, ast.Constant)
                    }
    raise AssertionError(f"{func_name} 不在 {path.name} 里 —— 判据锚点失效")


def test_the_eval_dict_has_exactly_the_production_fields() -> None:
    """`_detected_object_to_dict` 必须是生产 `_gsam_to_dict` 的**同字段**副本。

    这里刻意不 import 生产那份（它在 `run_autonomous_loop.py` 里，那个模块顶层
    就 import rclpy，两个解释器都没有），所以改用 AST 比对字段名 ——
    生产加了一个字段而评估集没跟上，评估集就会在**少一个字段**的数据上判
    `classify_roles`，那是静默失真、不是报错。
    """
    eval_keys = _dict_keys_from_source(EVAL_PATH, "_detected_object_to_dict")
    prod_keys = _dict_keys_from_source(LOOP_PATH, "_gsam_to_dict")
    assert eval_keys == prod_keys, (
        f"评估集多出 {sorted(eval_keys - prod_keys)}，"
        f"少了 {sorted(prod_keys - eval_keys)}"
    )


# --------------------------------------------------------------------------- #
# 2) 变体语义 = 阶段二前后的真实语义
# --------------------------------------------------------------------------- #
def test_baseline_reproduces_the_pre_phase2_binary_and_global_argmax(ev) -> None:
    """阶段二之前就是 `bool(detections)` + 在全量上取最高分。

    这条是**对照的立足点**：baseline 一旦被写成"我记忆里的旧版"，对照就没有
    意义。所以断言的是旧版**真实**的两处行为：有东西就 PRESENT；最高的那个
    （哪怕它是场景物体）进复核。
    """
    objects = [_obj("chair", 0.95), _obj("trash bin", 0.32)]
    row = ev._decide_baseline(objects, _profile())
    assert row["target_state"] == "PRESENT", "二分口径：有检测就是 PRESENT"
    assert row["verify_selected"] is True
    assert row["verify_label"] == "chair", "旧路径取全量最高分，椅子抢占"
    assert row["candidate_count"] is None, "旧路径没有『候选』这个概念"


def test_baseline_says_present_even_when_only_scene_objects_exist(ev) -> None:
    """目标不存在、画面里只有别的物体 ⇒ 旧路径仍然报 PRESENT 并进复核。

    这正是 §3.2「不得新增假'找到'」要修的那一类，baseline 必须**如实**
    表现出这个缺陷，否则对照会显得"两边一样好"。
    """
    objects = [_obj("chair", 0.7), _obj("bottle", 0.4)]
    row = ev._decide_baseline(objects, _profile(canonical="灭火器", terms=("fire extinguisher",)))
    assert row["target_state"] == "PRESENT"
    assert row["verify_selected"] is True


def test_phase2_restricts_the_verification_slot_to_target_candidates(ev) -> None:
    """任务 3 的核心断言：高分椅子不得抢占复核。"""
    objects = [_obj("chair", 0.95), _obj("trash bin", 0.32)]
    row = ev._decide_phase2(
        objects, _profile(canonical="垃圾桶", terms=("trash bin",)),
        role_separation=True, verify_scope="target_candidates",
    )
    assert row["verify_label"] == "trash bin"
    assert row["verify_score"] == pytest.approx(0.32)
    assert row["candidate_count"] == 1


def test_phase2_refuses_to_verify_when_no_target_candidate_exists(ev) -> None:
    """§3.2 第 1 条：场景物体不作为复核对象，不因此判目标存在。"""
    objects = [_obj("chair", 0.9), _obj("bottle", 0.5)]
    row = ev._decide_phase2(
        objects, _profile(canonical="灭火器", terms=("fire extinguisher",)),
        role_separation=True, verify_scope="target_candidates",
    )
    assert row["verify_selected"] is False
    assert row["target_state"] == "ABSENT"
    assert row["candidate_count"] == 0
    assert row["scene_count"] == 2


def test_phase2_keeps_the_relation_anchor_visible(ev) -> None:
    """锚点既进 relation_anchors、又进场景物体（它真的是要看进图的东西）。"""
    objects = [_obj("chair", 0.8), _obj("trash bin", 0.6)]
    row = ev._decide_phase2(
        objects,
        _profile(canonical="椅子", terms=("chair",), anchors=("trash bin",)),
        role_separation=True, verify_scope="target_candidates",
    )
    assert row["anchor_count"] == 1
    assert row["scene_count"] == 1
    assert row["verify_label"] == "chair"


def test_the_disabled_scope_falls_back_to_every_detection(ev) -> None:
    """`role_separation=False` 时即使给了 target_candidates 也要退回旧口径 ——
    这就是 `GO2W_RESULT_ROLE_SEPARATION=false` 的回退路径。"""
    objects = [_obj("chair", 0.95), _obj("trash bin", 0.32)]
    row = ev._decide_phase2(
        objects, _profile(canonical="垃圾桶", terms=("trash bin",)),
        role_separation=False, verify_scope="all_detections",
    )
    assert row["verify_label"] == "chair"


# --------------------------------------------------------------------------- #
# 3) 判据算术
# --------------------------------------------------------------------------- #
def test_judgement_flags_a_false_find_only_on_absent_rows(ev) -> None:
    absent_row = {"verify_selected": True, "target_state": "PRESENT",
                  "verify_is_target_term": False}
    flags = ev._judge_row(absent_row, "absent")
    assert flags["false_find"] is True
    assert flags["false_trigger"] is True

    present_row = {"verify_selected": True, "target_state": "PRESENT",
                   "verify_is_target_term": True}
    flags = ev._judge_row(present_row, "present")
    assert flags["false_find"] is False
    assert flags["missed"] is False


def test_judgement_flags_a_missed_positive(ev) -> None:
    row = {"verify_selected": False, "target_state": "ABSENT",
           "verify_is_target_term": False}
    flags = ev._judge_row(row, "present")
    assert flags["missed"] is True


def test_the_target_term_check_uses_the_prompt_term_not_the_label(ev) -> None:
    """`source_prompt_term` 才是"谁生成了这个框"；label 是中文显示契约。"""
    profile = _profile(canonical="椅子", terms=("chair",))
    hit = {"verify_term": "chair", "verify_label": "椅子"}
    miss = {"verify_term": "bottle", "verify_label": "椅子"}
    assert ev._label_is_target_term(hit, profile) is True
    assert ev._label_is_target_term(miss, profile) is False


def _row(variant, expect, **kw):
    base = {
        "variant": variant, "expect": expect,
        "verify_selected": False, "target_state": "ABSENT",
        "verify_is_target_term": False, "missed": False,
        "false_find": False, "false_trigger": False, "stolen_slot": False,
    }
    base.update(kw)
    return base


def test_the_summary_counts_only_the_matching_variant(ev) -> None:
    rows = [
        _row("baseline", "absent", verify_selected=True, false_find=True),
        _row("phase2", "absent", verify_selected=False),
        _row("phase2", "present", verify_selected=True, verify_is_target_term=True),
    ]
    summary = ev._summarize_variant(rows, "phase2")
    assert summary["rows"] == 2
    assert summary["false_find_count"] == 0
    assert summary["missed_positive_count"] == 0
    assert summary["verify_hits_target_term_rate"] == 1.0


def test_the_verdict_rejects_a_variant_that_adds_a_false_find(ev) -> None:
    """§3.2：新增假"找到"就是不许 —— 判决必须是 reject，不是"差不多"。"""
    summary = {
        "baseline": {"false_find_count": 0, "missed_positive_count": 0,
                     "false_trigger_count": 0, "stolen_slot_count": 2},
        "phase2": {"false_find_count": 1, "missed_positive_count": 0,
                   "false_trigger_count": 0, "stolen_slot_count": 0},
    }
    verdict = ev._verdict(summary)
    assert verdict["verdict"] == "reject"
    assert verdict["checks"]["no_new_false_find"] is False
    assert verdict["checks"]["no_new_stolen_slot"] is True


def test_the_verdict_adopts_when_nothing_regresses(ev) -> None:
    summary = {
        "baseline": {"false_find_count": 1, "missed_positive_count": 0,
                     "false_trigger_count": 1, "stolen_slot_count": 2},
        "phase2": {"false_find_count": 0, "missed_positive_count": 0,
                   "false_trigger_count": 0, "stolen_slot_count": 1},
    }
    assert ev._verdict(summary)["verdict"] == "adopt"


# --------------------------------------------------------------------------- #
# 4) 评估集定义与分频模拟
# --------------------------------------------------------------------------- #
def test_the_frozen_phrase_table_is_well_formed(ev) -> None:
    allowed = {"present", "absent", "relation"}
    assert len(ev.PHRASES) >= 6, "计划书的评估集是 6 条短语"
    for entry in ev.PHRASES:
        assert set(entry) >= {"phrase", "expect", "note"}
        assert entry["expect"] in allowed, entry
        assert entry["note"].strip(), "每条都要写清它覆盖什么"


def test_the_eval_set_covers_both_a_present_and_an_absent_target(ev) -> None:
    """§3.2 的两条门槛各自都要有素材，否则门槛无从判。"""
    expects = {entry["expect"] for entry in ev.PHRASES}
    assert "present" in expects and "absent" in expects


def test_the_relation_phrase_carries_a_frozen_profile_with_an_anchor(ev) -> None:
    """`use_llm=False` 的 fallback 解析器**不产锚点词**（见 `_fallback_profile`）。

    所以关系那条必须自带冻结画像；否则评估集里根本没有锚点那条路径，
    "验证了 relation_anchors"就是空话。
    """
    relation = [e for e in ev.PHRASES if e["expect"] == "relation"]
    assert relation, "评估集里没有关系短语"
    for entry in relation:
        override = entry.get("profile_override") or {}
        assert override.get("relation_anchor_labels_en"), entry["phrase"]


def test_cadence_simulation_reports_slot_refreshes_not_just_forwards(ev) -> None:
    """分频省的是**词数**，默认 TTL 下前向轮次**不会**减少。

    这条判据刻意同时钉住两件事，因为它们混在一起最容易得出错误结论：

    * 目标槽 TTL=0 ⇒ 开着分频也是每轮都刷（`forward_cycles` 两边都是 60）；
    * 场景槽被节流 ⇒ 场景词从 60 次降到个位数。

    把这两条化简成一个"前向次数"，就会得出"分频没用"——那是口径错了。
    """
    from app.config import Settings

    settings = Settings(siliconflow_api_key="", grounding_prompt_llm_expansion_enabled=False)
    result = ev._cadence_simulation(settings)
    off = result["cadence_off"]
    on = result["cadence_on_default_ttls"]
    assert off["slot_refreshes"] == {"target": 60, "anchor": 60, "scene": 60}
    assert on["forward_cycles"] == 60, "目标槽 TTL=0：前向轮次不该减少"
    assert on["slot_refreshes"]["scene"] < off["slot_refreshes"]["scene"]
    assert on["term_refreshes"] < off["term_refreshes"]
    assert "模拟" in result["note"]


def test_a_throttled_target_ttl_is_what_actually_skips_forwards(ev) -> None:
    """要真的**一次前向都不发**（复用上一轮产物），得把目标槽 TTL 也调起来。

    这是 `reuse_detection` 那条路的模拟证据：默认 TTL 下它永远不会触发。
    """
    throttled = ev._cadence_timeline(
        cycles=60, period_s=1.0,
        ttl_target_s=2.0, ttl_anchor_s=2.0, ttl_scene_s=4.0, scene_max_stale_s=20.0,
    )
    assert throttled["idle_cycles"] > 0
    assert throttled["forward_cycles"] < 60


def test_the_frozen_frames_still_exist_so_this_file_cannot_silently_rot() -> None:
    frames_dir = ROOT / "artifacts" / "perception_profile_20261013" / "frames"
    if not frames_dir.is_dir():
        pytest.skip("冻结素材不在本机 —— 标 skip 而不是假装通过")
    assert len(sorted(frames_dir.glob("color_*.jpg"))) >= 3


# --------------------------------------------------------------------------- #
# 5) 工具不许往只读基线里写
# --------------------------------------------------------------------------- #
def test_the_runner_never_writes_into_the_readonly_baseline() -> None:
    """`artifacts/perception_profile_20261013/` 是阶段的只读证据。"""
    text = EVAL_PATH.read_text(encoding="utf-8")
    for line in text.splitlines():
        if "perception_profile_20261013" not in line:
            continue
        assert "write_text" not in line, line
        assert "mkdir" not in line, line


def test_setup_json_records_effective_values_not_defaults() -> None:
    """§3.1.2：报告里要写**生效值**，所以 setup.json 得先把它们落下来。"""
    text = EVAL_PATH.read_text(encoding="utf-8")
    assert '"effective"' in text
    for key in (
        "go2w_result_role_separation",
        "go2w_verify_candidate_scope",
        "go2w_detection_cadence_enabled",
    ):
        assert key in text, key


def test_the_dry_run_path_does_not_touch_the_model(ev, tmp_path) -> None:
    """`--dry-run` 必须在**没有 GPU**的地方也能验证评估集本身。"""
    argv = [
        "perception_quality_eval.py",
        "--output-dir", str(tmp_path / "out"),
        "--dry-run",
    ]
    old = sys.argv
    sys.argv = argv
    try:
        assert ev.main() == 0
    finally:
        sys.argv = old
    setup = json.loads((tmp_path / "out" / "setup.json").read_text(encoding="utf-8"))
    assert setup["effective"]["eval_use_llm"] is False
    assert len(setup["phrases"]) >= 6
