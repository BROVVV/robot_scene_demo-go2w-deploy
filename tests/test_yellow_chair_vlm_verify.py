"""§44.5：两把椅子都送验，**只有属性对得上的那把**成为目标。

计划书原文：

    mock：2 chair candidates
    VLM：chair A attribute false / chair B attribute true
    最终只 B 为目标

**为什么这条是真缺口。** 现有 verify 相关测试（`tests/test_step_search_runner.py`、
`tests/test_autonomous_explorer.py`、`tests/test_score_fusion.py`）**全是单候选** ——
也就是说"只有 B 成为目标"这句话在它们那里根本没有第二个候选可以对照。
多候选路径只在 `app/video/semantic_verifier.py:62` 那个 `for obj in frame_candidates`
循环里，而那里恰好是"漏验一个"或"混用同一张裁剪图"最容易发生的地方。

**这个文件守的是一对命题，缺一半就没有分辨力：**

1. 属性不符的那把椅子**被验了**（`attempted == 2`）但**没被确认**；
2. 两把都属性相符时**两把都确认**（对照）。

只测第 1 条的话，一个"每个 frame 只验最高分那一个候选"的实现照样让
`matched_bboxes == [B]` 成立 —— 那不是 §44.5 要的语义，那是把多候选悄悄降级成单候选，
代价是"最高分的不是目标、第二高分的才是"这种情况永远选不中。

**诚实边界**：`CropVerifier.verify_crop` 被打桩，所以这里证明的是**循环与判定逻辑**
（谁被送验、谁的结论被留下、裁剪图是不是各用各的），不证明真 VLM 会对一张黄色椅子
返回 `is_target=True`。后者要真模型，见 `scripts/go2w/verify_ram_target_verify.py`
的 `_yellow_chair_case`（那条是离线验收脚本，不是 pytest）。
"""

from __future__ import annotations

from pathlib import Path

import pytest
from PIL import Image

from app.config import Settings
from app.detectors.crop_verifier import CropVerifier
from app.reasoning.target_profile import TargetProfile
from app.video.models import FrameAnalysisResult
from app.video.semantic_verifier import verify_video_candidates

# 属性不符的椅子（普通办公椅）与属性相符的椅子（黄色）。
CHAIR_A = {
    "object_id": "det_001", "label": "office chair", "label_zh": "办公椅",
    "bbox": [0.10, 0.10, 0.30, 0.60], "confidence": 0.90,
}
CHAIR_B = {
    "object_id": "det_002", "label": "office chair", "label_zh": "办公椅",
    "bbox": [0.55, 0.12, 0.75, 0.62], "confidence": 0.80,
}


def _profile() -> TargetProfile:
    """「黄色椅子」——颜色是本次 verify 唯一要区分的属性。"""
    return TargetProfile(
        raw_query="找黄色椅子",
        canonical_name_zh="黄色椅子",
        primary_labels_en=["chair", "office chair"],
        aliases_en=["seat"],
        colors=["yellow"],
    )


def _frame(image_path: str, objects: list[dict]) -> FrameAnalysisResult:
    return FrameAnalysisResult(
        frame_id=0, timestamp_sec=1.0, image_path=image_path,
        annotated_frame_path=None, scene_summary="", objects=objects, relations=[],
    )


def _image(tmp_path: Path) -> str:
    path = tmp_path / "frame.png"
    Image.new("RGB", (200, 200), (128, 128, 128)).save(path)
    return str(path)


def _settings(api_key: str = "test-key") -> Settings:
    """`CropVerifier.available` 只看 key 在不在；有 key 才会走到打桩的 verify。"""
    return Settings(siliconflow_api_key=api_key)


def _by_id(frame: FrameAnalysisResult, object_id: str) -> dict:
    """从**帧里**取候选（`verify_video_candidates` 是 `obj.update(...)` 原地改的）。

    不能读模块级的 `CHAIR_A`：帧里放的是 `dict(CHAIR_A)` 的副本，改的是副本。
    """
    for obj in frame.objects:
        if obj.get("object_id") == object_id:
            return obj
    raise AssertionError(f"帧里没有 {object_id}")


def _stub_verify(monkeypatch: pytest.MonkeyPatch, verdicts: dict[str, dict]) -> list[dict]:
    """按**裁剪文件名里的 object_id** 决定这次验的是哪把椅子，并记下每次调用。

    用文件名而不是调用次序：候选先按 confidence 降序排过，用次序会把
    "排序变了"和"验错对象了"混成同一个失败。文件名是 `verify_video_candidates`
    自己拼的（`video_crops/video_f{frame_id}_{object_id}.jpg`），拿它当键等于
    顺带钉住了"每个候选各存各的裁剪图"。
    """
    calls: list[dict] = []

    def fake(self, crop_path, target_profile, candidate_label, context=None):
        calls.append({
            "crop": str(crop_path), "label": candidate_label, "context": context,
        })
        for object_id, verdict in verdicts.items():
            if object_id in str(crop_path):
                return dict(verdict)
        raise AssertionError(f"打桩没覆盖这张裁剪图：{crop_path}")

    monkeypatch.setattr(CropVerifier, "verify_crop", fake)
    return calls


# 属性相符 / 不符的两份 VLM 结论。四项子分都写全，因为 `fuse_score` 会把
# detector / vlm / attribute / context 加权平均（`app/vision/score_fusion.py:22-35`），
# 只给 is_target 不给分数的话融合分照样低，确认不了。
YELLOW = {
    "is_target": True, "target_match_score": 0.95, "attribute_score": 0.95,
    "category_score": 0.95, "context_score": 0.90, "evidence": ["椅面是黄色"],
}
NOT_YELLOW = {
    "is_target": False, "target_match_score": 0.60, "attribute_score": 0.05,
    "category_score": 0.95, "context_score": 0.50,
    "rejection_reason": "attribute_mismatch:not_yellow",
    "evidence": ["椅面是灰色"],
}


def test_only_the_attribute_matching_chair_becomes_the_target(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """§44.5 正面：A 属性不符、B 相符 ⇒ 只 B 进 `matched_bboxes`。"""
    calls = _stub_verify(monkeypatch, {"det_001": NOT_YELLOW, "det_002": YELLOW})
    frame = _frame(_image(tmp_path), [dict(CHAIR_A), dict(CHAIR_B)])

    report = verify_video_candidates(
        [frame], "黄色椅子", _profile(), tmp_path / "out", settings=_settings(),
    )

    assert report["attempted"] == 2, (
        "两把椅子都必须被送去 verify —— 只验一把的话「只有 B 是目标」是白捡的"
    )
    assert len(calls) == 2
    verification = frame.metadata["semantic_verification"]
    assert verification["is_present"] is True
    assert verification["matched_bboxes"] == [CHAIR_B["bbox"]], (
        f"只有属性相符的那把该成为目标，实际 {verification['matched_bboxes']}"
    )


def test_the_rejected_chair_is_recorded_with_a_reason_not_silently_dropped(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """被否掉的椅子必须留下 `decision` 与非空 `rejection_reason`。

    "没成为目标"有两种：**验过、被否**与**压根没送验**。前者可复盘（知道模型
    说了什么），后者只会在图上表现为"这个物体不存在"。这条把两者的区别钉住 ——
    顺带钉住 §44.5 的 A 是前者。
    """
    _stub_verify(monkeypatch, {"det_001": NOT_YELLOW, "det_002": YELLOW})
    frame = _frame(_image(tmp_path), [dict(CHAIR_A), dict(CHAIR_B)])

    verify_video_candidates(
        [frame], "黄色椅子", _profile(), tmp_path / "out", settings=_settings(),
    )

    chair_a = _by_id(frame, "det_001")
    chair_b = _by_id(frame, "det_002")
    assert chair_a["decision"] != "confirmed"
    assert chair_a["rejection_reason"] == "attribute_mismatch:not_yellow"
    assert chair_a["final_score"] < chair_b["final_score"], (
        "属性不符的那把融合分必须更低 —— 否则分数融合没把属性分算进去"
    )
    assert chair_a["crop_verify"]["is_target"] is False
    assert chair_b["crop_verify"]["is_target"] is True


def test_each_candidate_gets_its_own_crop_file(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """两个候选的裁剪图必须是**两个不同文件**，且文件真的落盘了。

    共用一张（比如把 `crop_path` 拼在循环外）会让 VLM 两次看到同一张图 ——
    于是两把椅子得到同一个结论，"只有 B 是目标"退化成"先验的那把是目标"。
    文件名里带 object_id 是实现的约定，这条同时断言文件存在，排除"路径对但没写"。
    """
    calls = _stub_verify(monkeypatch, {"det_001": NOT_YELLOW, "det_002": YELLOW})
    frame = _frame(_image(tmp_path), [dict(CHAIR_A), dict(CHAIR_B)])

    verify_video_candidates(
        [frame], "黄色椅子", _profile(), tmp_path / "out", settings=_settings(),
    )

    crops = [call["crop"] for call in calls]
    assert len(set(crops)) == 2, f"两个候选共用了裁剪图：{crops}"
    assert all(Path(crop).is_file() for crop in crops), "裁剪图必须真的写到磁盘上"
    assert "det_001" in crops[0] or "det_001" in crops[1]


def test_both_chairs_become_targets_when_both_match_attributes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """**对照**：两把都是黄的 ⇒ `matched_bboxes` 必须有两个。

    没有这条，上一条的 `== [CHAIR_B["bbox"]]` 可以由一个"永远只确认最后一个
    候选"的实现满足 —— 那种实现会把"两把黄椅子"报成一把，而搜到一把就停的
    逻辑就会漏掉另一把。
    """
    _stub_verify(monkeypatch, {"det_001": YELLOW, "det_002": YELLOW})
    frame = _frame(_image(tmp_path), [dict(CHAIR_A), dict(CHAIR_B)])

    verify_video_candidates(
        [frame], "黄色椅子", _profile(), tmp_path / "out", settings=_settings(),
    )

    matched = frame.metadata["semantic_verification"]["matched_bboxes"]
    assert sorted(map(tuple, matched)) == sorted(
        [tuple(CHAIR_A["bbox"]), tuple(CHAIR_B["bbox"])]
    ), f"两把黄椅子都该被确认，实际 {matched}"


def test_neither_chair_becomes_a_target_when_neither_matches(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """**反向对照**：两把都不是黄的 ⇒ 一个目标都没有，`is_present` 为 False。

    这是"这一帧没有黄色椅子"的正确表达。少这条的话，一个 `is_present` 恒 True
    的实现能同时通过上面两条 —— 而恒 True 意味着**每一帧都在报"看见目标了"**，
    搜索会停在一把灰椅子前面宣布完成。
    """
    _stub_verify(monkeypatch, {"det_001": NOT_YELLOW, "det_002": NOT_YELLOW})
    frame = _frame(_image(tmp_path), [dict(CHAIR_A), dict(CHAIR_B)])

    verify_video_candidates(
        [frame], "黄色椅子", _profile(), tmp_path / "out", settings=_settings(),
    )

    verification = frame.metadata["semantic_verification"]
    assert verification["matched_bboxes"] == []
    assert verification["is_present"] is False


def _relation_profile() -> TargetProfile:
    return TargetProfile(
        raw_query="挂着书包的椅子",
        canonical_name_zh="挂着书包的椅子",
        primary_labels_en=["chair", "office chair"],
        relation_constraints=["backpack on chair"],
        relation_anchor_labels_en=["backpack"],
        relation_anchor_labels_zh=["书包"],
        context_labels_en=["backpack"],
        context_labels_zh=["书包"],
    )


def _context_by_chair(calls: list[dict]) -> dict[str, list[dict]]:
    return {
        ("det_001" if "det_001" in call["crop"] else "det_002"): call["context"]["context_objects"]
        for call in calls
    }


def test_each_chair_gets_the_backpack_nearest_to_itself(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """§44.6 的同帧 A/B 对照：每个候选拿到的是**离自己最近**的那个 anchor。

    既有测试分两处各测了一半（`test_video_semantic_verifier.py` 里「有 anchor」
    与「没 anchor」各一条），**但不在同一帧里**。同帧双候选才有分辨力：把
    `_verification_region` 提到循环外、或把上一轮的 `context_objects` 留在变量里，
    两个椅子就会拿到同一个书包 —— 而那时 VLM 拿到的是不属于它的并集框。

    **⚠ 与计划书 §44.6 字面要求的偏离，如实记在这里。** §44.6 写的是
    「chair A 不应该有该 context」。实测不是这样：`_verification_region`
    （`app/video/semantic_verifier.py:151-225`）里**没有距离门**，它的选法是
    `min(matches, key=(is_anchor, distance))` —— 只要帧里有 anchor，
    **每一个**候选都会被挂上离它最近的那一个。A 拿到的是离 A 最近的书包，
    B 拿到的是离 B 最近的书包，两者都在场。

    这是**设计选择而非疏漏**（函数自己的 docstring：*"Include the nearest
    explicit relation anchor without weakening the gate"*）：几何层不做
    「书包是否真的在椅子上」的判断，它只保证关系证据**在裁剪框里**，
    真正的空间判断交给 VLM（`_verification_prompt` 会把
    `relation_constraints` 与 context 一起写进 prompt）。这条测试因此钉的是
    **每个候选各取最近**这件事，而不是"A 的 context 为空"。
    """
    calls = _stub_verify(monkeypatch, {"det_001": NOT_YELLOW, "det_002": YELLOW})
    near_a = {
        "object_id": "det_010", "label": "backpack", "label_zh": "书包",
        "bbox": [0.02, 0.02, 0.12, 0.20], "confidence": 0.9,
    }
    near_b = {
        "object_id": "det_011", "label": "backpack", "label_zh": "书包",
        "bbox": [0.62, 0.05, 0.72, 0.22], "confidence": 0.9,
    }
    frame = _frame(_image(tmp_path), [dict(CHAIR_A), dict(CHAIR_B), near_a, near_b])

    verify_video_candidates(
        [frame], "挂着书包的椅子", _relation_profile(), tmp_path / "out",
        settings=_settings(),
    )

    contexts = _context_by_chair(calls)
    assert [item["object_id"] for item in contexts["det_001"]] == ["det_010"], (
        "A 必须拿到离 A 近的那个书包"
    )
    assert [item["object_id"] for item in contexts["det_002"]] == ["det_011"], (
        "B 必须拿到离 B 近的那个书包 —— 两个椅子拿到同一个就是循环外的变量没清"
    )

    # 裁剪框必须是**并集**，且严格大于候选自己的框。
    # §15.3 的原话是「§15.3 的裁剪框要靠它并集」：只把书包写进 context 而不撑框，
    # VLM 拿到的那张图里根本没有书包，`relation_constraints` 就成了 prompt 里
    # 一句无凭无据的话 —— 判"椅子到底挂没挂书包"时它只能瞎猜。
    by_chair = {
        ("det_001" if "det_001" in call["crop"] else "det_002"): call for call in calls
    }
    for chair_id, anchor_id, anchor_bbox in (
        ("det_001", "det_010", near_a["bbox"]),
        ("det_002", "det_011", near_b["bbox"]),
    ):
        context = by_chair[chair_id]["context"]
        candidate_bbox = context["candidate_bbox"]
        verification_bbox = context["verification_bbox"]
        assert verification_bbox == [
            min(candidate_bbox[0], anchor_bbox[0]),
            min(candidate_bbox[1], anchor_bbox[1]),
            max(candidate_bbox[2], anchor_bbox[2]),
            max(candidate_bbox[3], anchor_bbox[3]),
        ], f"{chair_id} 的裁剪框必须是候选框与 {anchor_id} 框的并集"
        assert verification_bbox != candidate_bbox, "并集后一定要比紧框大"


def test_a_frame_with_no_relation_anchor_leaves_every_crop_tight(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """帧里根本没有书包 ⇒ 两把椅子的 context 都为空、裁剪框都不许被撑大。

    与上一条配对：上一条证明"有 anchor 时各取最近"，这条证明"没有时不许凭空
    并集"。少了这条，一个「无论如何都把帧里第一个 context 物体并进去」的实现
    也能通过上一条（两个书包都在场，它总能挑到一个）。
    """
    calls = _stub_verify(monkeypatch, {"det_001": NOT_YELLOW, "det_002": YELLOW})
    desk = {
        "object_id": "det_020", "label": "desk", "label_zh": "桌子",
        "bbox": [0.30, 0.70, 0.90, 0.95], "confidence": 0.9,
    }
    frame = _frame(_image(tmp_path), [dict(CHAIR_A), dict(CHAIR_B), desk])

    verify_video_candidates(
        [frame], "挂着书包的椅子", _relation_profile(), tmp_path / "out",
        settings=_settings(),
    )

    for call in calls:
        assert call["context"]["context_objects"] == [], (
            f"帧里没有书包，{call['crop']} 不该有 context"
        )
        assert call["context"]["verification_bbox"] == call["context"]["candidate_bbox"], (
            "没有 anchor 时裁剪框必须保持紧框（不许被桌子撑大）"
        )


def test_a_positional_context_label_alone_does_not_widen_the_crop(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """**只有 `context_labels` 而没有 `relation_constraints`** 时不许并集。

    这条钉的是 `_verification_region` 开头那个 `if not
    target_profile.relation_constraints: return bbox, []` 的**前置守卫**，
    而它此前全仓无人覆盖：`test_video_semantic_verifier.py` 里那条紧框测试用的
    profile 连 `context_terms()` 都是空的，走的是更靠后的 `if not context_terms`
    守卫，根本到不了这里。

    两种 profile 的区别是有语义的：
    - 「**挂着书包的**椅子」——书包是**关系证据**，必须并进框里让 VLM 看见；
    - 「**桌上的**手机」——桌只是**位置提示**，把它并进框会递给 VLM 一张
      同时包含桌子和手机的图，而 §15.2 的注释记录的正是这个失败模式：
      「实测里 VLM 只裁到玻璃门就宣布确认」。
    """
    calls = _stub_verify(monkeypatch, {"det_001": NOT_YELLOW, "det_002": YELLOW})
    table = {
        "object_id": "det_030", "label": "table", "label_zh": "桌子",
        "bbox": [0.05, 0.80, 0.95, 0.99], "confidence": 0.9,
    }
    frame = _frame(_image(tmp_path), [dict(CHAIR_A), dict(CHAIR_B), table])
    profile = TargetProfile(
        raw_query="桌上的椅子",
        canonical_name_zh="桌上的椅子",
        primary_labels_en=["chair", "office chair"],
        # 有关键位置提示词，但**没有**关系约束。
        context_labels_en=["table"],
        context_labels_zh=["桌子"],
    )

    verify_video_candidates(
        [frame], "桌上的椅子", profile, tmp_path / "out", settings=_settings(),
    )

    assert calls, "两把椅子都该被验到"
    for call in calls:
        assert call["context"]["context_objects"] == [], (
            "位置提示词不是关系证据，不该进 context"
        )
        assert call["context"]["verification_bbox"] == call["context"]["candidate_bbox"], (
            "没有关系约束时裁剪框必须保持紧框"
        )


# --------------------------------------------------------------------------
# §44.9 RAM 普通物体不进 VLM crop verify
# --------------------------------------------------------------------------

# RAM+DINO 在一帧里给出的普通场景物体，一个都不是目标。
ORDINARY_OBJECTS = [
    {"object_id": "det_201", "label": "office chair", "label_zh": "办公椅",
     "bbox": [0.05, 0.10, 0.20, 0.60], "confidence": 0.9},
    {"object_id": "det_202", "label": "table", "label_zh": "桌子",
     "bbox": [0.25, 0.30, 0.55, 0.70], "confidence": 0.9},
    {"object_id": "det_203", "label": "monitor", "label_zh": "显示器",
     "bbox": [0.60, 0.15, 0.75, 0.45], "confidence": 0.9},
    {"object_id": "det_204", "label": "plant", "label_zh": "盆栽",
     "bbox": [0.80, 0.40, 0.95, 0.90], "confidence": 0.9},
]

FIRE_EXTINGUISHER = {
    "object_id": "det_205", "label": "fire extinguisher", "label_zh": "灭火器",
    "bbox": [0.35, 0.05, 0.48, 0.35], "confidence": 0.85,
}


def _fire_extinguisher_profile() -> TargetProfile:
    return TargetProfile(
        raw_query="找灭火器",
        canonical_name_zh="灭火器",
        primary_labels_en=["fire extinguisher"],
        aliases_en=["extinguisher"],
    )


def _counting_verify(monkeypatch: pytest.MonkeyPatch) -> list[str]:
    """记下每一次 crop verify 请求，并且**不返回结论** —— 这几条只数次数。"""
    calls: list[str] = []

    def fake(self, crop_path, target_profile, candidate_label, context=None):
        calls.append(str(crop_path))
        return dict(YELLOW)

    monkeypatch.setattr(CropVerifier, "verify_crop", fake)
    return calls


def test_ordinary_scene_objects_are_never_sent_to_the_crop_verifier(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """§44.9：RAM+DINO 给的普通物体**一个都不许**触发 VLM crop verify。

    这不是省 API 钱的优化，是**判据污染**：`verify_video_candidates` 一旦把桌子
    和盆栽也送去问「这是不是灭火器」，每一条都会答"不是"，于是
    `frame.metadata["semantic_verification"]` 里塞满无关的 rejection，真正被否掉
    的灭火器候选淹在里面 —— 「验过被否」与「压根没送验」再也分不开，而那正是
    §44.5 那条测试在守的区分。

    两个方向都要钉：`attempted == 0`，且**没有任何一个候选被送出去**
    （`attempted` 是实现自报的计数，只断言它就等于用实现证明实现）。
    """
    calls = _counting_verify(monkeypatch)
    frame = _frame(_image(tmp_path), [dict(obj) for obj in ORDINARY_OBJECTS])

    result = verify_video_candidates(
        [frame], "灭火器", _fire_extinguisher_profile(),
        tmp_path / "out", settings=_settings(),
    )

    assert calls == [], f"普通物体被送去问 VLM 了：{calls}"
    assert result["attempted"] == 0
    # 一个候选都没送，就不该在帧上留下语义结论 —— 下游读这个键当证据。
    assert "semantic_verification" not in frame.metadata


def test_ordinary_objects_still_reach_the_semantic_map(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """§44.9 的另一半：普通 object **仍然进入 semantic map**。

    这是上一条的对照面。只测「0 次调用」的话，一个在过滤阶段就把 `frame.objects`
    清空（或把普通物体标成已验过）的实现同样得 0 —— 而它的后果是语义地图里没有
    桌子、没有盆栽，整张拓扑只剩目标节点。判据分开的代价不能是丢掉这些物体。
    """
    calls = _counting_verify(monkeypatch)
    frame = _frame(_image(tmp_path), [dict(obj) for obj in ORDINARY_OBJECTS])

    verify_video_candidates(
        [frame], "灭火器", _fire_extinguisher_profile(),
        tmp_path / "out", settings=_settings(),
    )

    assert calls == []
    assert len(frame.objects) == len(ORDINARY_OBJECTS)
    # 逐个字段比，不是比个数：`obj.update(...)` 是原地改的，只数个数看不出
    # 「留下了但被追加了 decision / 被改写了 bbox」。
    for expected in ORDINARY_OBJECTS:
        assert _by_id(frame, expected["object_id"]) == expected


def test_the_same_frame_does_send_a_real_target_candidate_to_the_verifier(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """对照：同一帧里放一个真目标 ⇒ 验证器**必须**被调一次。

    没有这一条，上面两条的「0」没有分辨力 —— 一个永远不调用 `verify_crop` 的
    实现（比如过滤条件写成了恒假）能让它们照样全绿，而搜索会彻底失去 VLM 复核。
    """
    calls = _counting_verify(monkeypatch)
    frame = _frame(
        _image(tmp_path),
        [dict(obj) for obj in ORDINARY_OBJECTS] + [dict(FIRE_EXTINGUISHER)],
    )

    verify_video_candidates(
        [frame], "灭火器", _fire_extinguisher_profile(),
        tmp_path / "out", settings=_settings(),
    )

    assert len(calls) == 1, f"灭火器候选必须被验，实际调用：{calls}"
    assert FIRE_EXTINGUISHER["object_id"] in calls[0], "验的必须是灭火器那张裁剪图"
