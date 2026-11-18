"""轻量化改造计划 §20 的 ``sam2_min_box_score``：只给过了分的框跑 SAM2。

审计报告 `reports/go2w_plan_completion_audit_20260915.md:66` 的原话：
「其中 ``sam2_min_box_score`` 缺席是真浪费：**SAM2 现在对 NMS 后每一个 box 都跑**，
正是计划要优化掉的开销」。计划书 §20 给的取值是 ``0.35``。

**这个文件守的是三件不同的事，别混着看：**

1. **配置管线**（行为级）：常量/字段/env/请求 —— 真能跑通。
2. **取值的校准**（行为级，读真机产出）：0.35 是**量出来的**，不是从计划书抄下来
   就完事。真机数据决定了它必须坐在一条空带里。
3. **worker 里的过滤本身**（源码级契约）：worker 是模型进程，`_detect` 要真
   GroundingDINO + 真 SAM2 + 真 torch 才跑得动，本机跑不了。所以这一段的断言是
   源码级的 —— 沿用本仓库对 ROS/模型节点的既有分工（见
   `tests/test_go2w_plan16_events.py` 的开头自陈）。**它们的边界要说清**：能证明
   「过滤写在了正确的位置、下标没被搞乱」，**不能**证明真机上掩码确实是这么分的。
   后者靠实机跑一次检测看 mask_area_ratio 是否还落在合理范围。

**为什么下标对齐值得单独立一条断言。** 把 ``_predict_masks(predictor, image,
xyxy[passing], ...)`` 的返回值直接当 ``masks`` 用，是这次改动最容易犯的错：第 k 个
掩码会贴到第 k 个**框**上而不是第 k 个**过关的框**上。低分框一多，掩码整体错位，
而错位的掩码**不会抛异常、不会报错**，只会把 bbox 悄悄精修到旁边那个物体身上 ——
下游看到的仍是一个"有掩码的物体"。这是本次改动唯一能造成静默错误的地方。
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from app.config import (
    DEFAULT_GROUNDING_DINO_BOX_THRESHOLD,
    DEFAULT_SAM2_MIN_BOX_SCORE,
    Settings,
    get_settings,
)

ROOT = Path(__file__).resolve().parents[1]
WORKER = ROOT / "app" / "detectors" / "grounded_sam_worker.py"
SUBPROCESS = ROOT / "app" / "detectors" / "grounded_sam_subprocess.py"
# 真机 detector_score 的唯一来源（2026-09-16 实测：36 条非空全在这里）。
MEMORY = ROOT / "data" / "memory" / "observational_memory.jsonl"


# --- 1. 配置管线 ---------------------------------------------------------

def test_default_threshold_matches_the_plan() -> None:
    assert DEFAULT_SAM2_MIN_BOX_SCORE == 0.35, "计划书 §20 给的取值就是 0.35"
    assert Settings(siliconflow_api_key="").sam2_min_box_score == DEFAULT_SAM2_MIN_BOX_SCORE


def test_env_override_reaches_the_settings(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("SAM2_MIN_BOX_SCORE", "0.5")
    get_settings.cache_clear()
    try:
        assert get_settings().sam2_min_box_score == 0.5
    finally:
        get_settings.cache_clear()


def test_the_gate_is_not_dead_by_construction() -> None:
    """门槛必须**高于** GroundingDINO 基础门槛，否则这道门永远不会拦下任何框。

    不高于它 = 所有框都过了基础门槛 → 所有框都跑 SAM2 = 旧行为，§20 这条优化
    静默失效。这正是本仓库反复出现的那类病（「默认值 ≠ 生效值」）：配置项在、
    名字对、也不报错，只是从不生效。

    断的是**生效值**而不是两个常量：基础门槛会被 .env 覆盖（实测默认 0.10、
    当前生效 0.12），只比常量会漏掉"覆盖之后门槛反超"的情况。
    """
    settings = get_settings()
    assert settings.sam2_min_box_score > settings.grounding_dino_box_threshold, (
        f"SAM2 门槛 {settings.sam2_min_box_score} 不高于 GroundingDINO 基础门槛 "
        f"{settings.grounding_dino_box_threshold} —— 每个框都会跑 SAM2，"
        "§20 的优化从未生效"
    )
    assert DEFAULT_SAM2_MIN_BOX_SCORE > DEFAULT_GROUNDING_DINO_BOX_THRESHOLD


# --- 2. 真机取值的校准 ---------------------------------------------------

def _real_scores() -> list[float]:
    if not MEMORY.exists():
        pytest.skip(f"真机产出不在：{MEMORY}（数据丢了不能假装通过）")
    scores: list[float] = []
    for line in MEMORY.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        stack = [record]
        while stack:
            node = stack.pop()
            if isinstance(node, dict):
                for key, value in node.items():
                    if key == "detector_score" and isinstance(value, (int, float)):
                        scores.append(float(value))
                    elif isinstance(value, (dict, list)):
                        stack.append(value)
            elif isinstance(node, list):
                stack.extend(node)
    if len(scores) < 20:
        pytest.skip(f"真机样本只有 {len(scores)} 条，够不上校准（判据没有分辨力）")
    return scores


def test_the_threshold_sits_in_a_gap_not_in_the_data() -> None:
    """0.35 是**量出来的**：真机分布里它两侧都是空的。

    2026-09-16 实测（`data/memory/observational_memory.jsonl`，36 条非空）：

    - 杂质带最高 **0.2478**（另两条 0.1357 / 0.1406），共 3/36 = 8% 被挡在门外
    - 过关的最低分 **0.8000**
    - 落在 [0.30, 0.60] 的样本：**0 条**

    所以门槛坐在一条 0.248→0.800 的空带里，**选 0.35 还是 0.5 结果一样** ——
    这不是"拿精度换速度"。

    两侧余量都断言，因为两侧的坏法不同：门槛太低 = 门是死的（照样给垃圾框跑
    SAM2）；门槛太高 = **真物体会丢掩码**（降到 2D 语义，3D 定位质量静默下滑）。
    """
    scores = _real_scores()
    threshold = get_settings().sam2_min_box_score
    blocked = [s for s in scores if s < threshold]
    passed = [s for s in scores if s >= threshold]
    assert blocked, (
        "真机数据里一个低于门槛的框都没有 —— 这道门在真机上从未拦下任何东西，"
        "优化是否生效无从判断"
    )
    assert passed, f"门槛 {threshold} 把所有真机检测都挡住了"
    junk_top, real_bottom = max(blocked), min(passed)
    assert threshold - junk_top >= 0.05, (
        f"门槛 {threshold} 贴着杂质带顶部 {junk_top:.4f}。"
        "注意：**这条红了不一定是回归**，可能是真机出现了一个分数刚好卡在门槛上的框 —— "
        "那正是要重新校准 sam2_min_box_score 的信号（它到底是垃圾还是真物体？）"
    )
    assert real_bottom - threshold >= 0.20, (
        f"门槛 {threshold} 距最弱的真机检测 {real_bottom:.4f} 太近 —— "
        "往上一点就会有真实物体丢掉掩码，而丢掩码不会报错，只会让 3D 定位变差"
    )


# --- 3. worker 里的过滤（源码级契约） ------------------------------------

def test_masks_stay_index_aligned_with_all_boxes() -> None:
    """**本次改动唯一能造成静默错误的地方**：槽位表必须与 ``xyxy`` 等长。

    组装循环是 ``masks[index]``，而 index 走的是**全体** NMS 后的框。所以槽位表
    要预先铺满 None 再往里填；一旦只放"过关的那几个"，掩码就会整体错位。
    """
    src = WORKER.read_text(encoding="utf-8")
    slot = "mask_slots: list = [None] * len(xyxy)"
    assert src.count(slot) == 1, "槽位表的构造方式变了，断言前提已失效"
    gate = src.index(slot)
    tail = src[gate:gate + 1600]
    assert "masks[index] if masks is not None else None" in src, (
        "组装循环不再按全体框的下标取掩码 —— 下标对齐的前提变了"
    )
    assert "masks = mask_slots" in tail, "算完的槽位表没有交回 masks"
    # 只把过关的子集喂给 SAM2（省的就是这一段的钱）
    assert "_predict_masks(sam2_predictor, image_source, xyxy[scored]" in tail, (
        "喂给 SAM2 的不是过关子集 —— 要么全跑了（没省下开销），"
        "要么传的是别的切片（下标对不上）"
    )
    # SAM2 整段失败时 `_predict_masks` 返回 None，此时槽位表应当**原样留空**
    assert "if predicted is not None:" in tail, (
        "没判 _predict_masks 的 None 返回 —— SAM2 抛异常时会把 None 当可迭代对象解包"
    )
    # zip(scored, ...) 而不是 zip(range(...), ...)：前者才把掩码贴回它自己的框
    assert "for slot, mask in zip(scored, predicted):" in tail
    # `>=`：门槛上的框算**通过**（`>` 会把它踢掉，与 §20「通过基础门槛」的措辞不符）
    assert ">= float(sam2_min_box_score)" in tail


def test_the_gate_runs_after_nms() -> None:
    """§20 原话：「只有通过 GroundingDINO 基础门槛的 bbox 才跑 SAM2。当前已有
    NMS，继续复用。」—— 复用 NMS 的意思就是门开在 NMS **之后**。

    开在 NMS 之前，被 NMS 抑制掉的重叠框也会白跑一次 SAM2，省下来的开销大部分
    又还回去了。
    """
    src = WORKER.read_text(encoding="utf-8")
    body = src[src.index("def _detect("):src.index("def _build_ram_plus")]
    assert body.index("keep = nms(") < body.index("mask_slots: list = [None]")


def test_zero_threshold_is_not_swallowed_by_an_or_fallback() -> None:
    """``0.0`` 是合法且有意义的值（关门 = 旧行为），``or`` 会把它顶成默认值。

    这里刻意**不是**照抄上面几个阈值的 ``args.x or 0.35`` 写法：那套写法在
    ``box_threshold``/``text_threshold`` 上无害（0 对它们本来就不是合法值），
    照抄到这里就会把"关掉这道门"变成"开到 0.35"。
    """
    src = WORKER.read_text(encoding="utf-8")
    assert 'parser.add_argument("--sam2-min-box-score", type=float, default=0.35)' in src, (
        "argparse 不再带 default —— 落回 None 后要么崩，要么得靠 `or` 兜底"
    )
    assert "sam2_min_box_score or " not in src, (
        "出现了 `or` 兜底 —— 显式传 0.0 会被顶成默认值，门关不掉"
    )
    assert 'float(request.get("sam2_min_box_score", args.sam2_min_box_score))' in src


def test_the_subprocess_carries_the_threshold_in_the_request() -> None:
    """配置要从 settings 走到 worker。判据是请求体里那一行本身，不是命令行。

    **只走请求、不走命令行**是有意的：worker 侧是
    ``request.get(key, args.key)``，请求永远优先，所以命令行那份对 --serve
    路径是够不着的死路径（看着像生效、其实从不生效）。worker 的 argparse
    默认值已经等于 settings 默认值，手动 ``--image`` 那条路照样有门。
    """
    src = SUBPROCESS.read_text(encoding="utf-8")
    assert '"sam2_min_box_score": self.settings.sam2_min_box_score,' in src, (
        "常驻 worker 的请求里没有带门槛 —— worker 会用自己的 argparse 默认值，"
        "settings 里改了这个值不会生效"
    )
    assert "--sam2-min-box-score" not in src, (
        "又加了一条命令行参数 —— 请求优先，那份是死路径"
    )
