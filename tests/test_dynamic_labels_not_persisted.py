"""轻量化计划 §26：person/dog/cat 不写入持久语义地图，但当前检测仍要显示。

**为什么需要注入校准。** "人没有出现在地图里"这个结论本身是**阴性**的 ——
它可能成立，也可能只是因为测试根本没喂到那条持久化路径。所以除了正向断言，
这里还有一条反向用例：把过滤函数换成恒等函数，断言"人"**确实**会进地图。
没有这一条，正向断言是空的（仓库硬规矩：没证明判据有分辨力的"没检出"不算数）。

**为什么要中文标签。** 计划书原文给的是英文 `{"person","dog","cat"}`，但这
条流水线的 label 实测全是中文（见下面 REAL_OBSERVED_LABELS，取自
`artifacts/slam_validation/semanticD_20260910_170332/events.jsonl`）。照抄
英文集合会得到一个永远匹配不上的死过滤器 —— 这条测试就是拦它的。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.perception.depth_object_localizer import ObjectSpatialObservation  # noqa: E402
from app.spatial.dynamic_labels import (  # noqa: E402
    DYNAMIC_LABELS,
    drop_dynamic_objects,
    is_dynamic_object,
)
from app.spatial.semantic_navigation_graph import SemanticNavigationGraph  # noqa: E402

# 真机 events.jsonl 里实际出现过的标签（2026-09-16 实测）。一个都不能被误伤 ——
# 把人过滤掉是 §26 的要求，把垃圾桶、办公椅一起过滤掉就是 bug。
REAL_OBSERVED_LABELS = (
    "垃圾桶", "桌子", "办公椅", "电脑主机", "背包", "电脑显示器", "风扇",
)

DYNAMIC_CASES = (
    # 计划书原集合
    "person", "dog", "cat",
    # 这条中文流水线实际会产出的等价标签
    "人", "行人", "人物", "人员", "狗", "小狗", "犬", "猫", "小猫",
)

KEPT_CASES = REAL_OBSERVED_LABELS + (
    "椅子", "饮水机", "沙发", "纸箱", "门", "窗户", "消防栓", "盆栽",
)


@pytest.mark.parametrize("label", DYNAMIC_CASES)
def test_dynamic_labels_are_dynamic(label: str) -> None:
    assert is_dynamic_object({"label": label}), f"{label} 应被判为动态物体"


@pytest.mark.parametrize("label", KEPT_CASES)
def test_kept_labels_are_not_dynamic(label: str) -> None:
    """反向控制：这些必须**不被**判为动态，否则过滤器就是误伤。"""
    assert not is_dynamic_object({"label": label}), f"{label} 被误判成动态物体了"


def test_plan_literal_set_is_a_subset() -> None:
    """计划书写死的三个英文词必须仍然在集合里（不许为了中文把它删掉）。"""
    assert {"person", "dog", "cat"} <= DYNAMIC_LABELS


def test_label_zh_takes_priority_over_label() -> None:
    """label 是占位英文、label_zh 才是真标签时，以 label_zh 为准。"""
    assert is_dynamic_object({"label": "object", "label_zh": "人"})
    assert not is_dynamic_object({"label": "person", "label_zh": "垃圾桶"})


def test_accepts_both_dicts_and_objects() -> None:
    """下游两种形态都有：worker 里是对象，graph 里可能是 dict。"""
    obj = ObjectSpatialObservation(object_id="a", label="人")
    assert is_dynamic_object(obj)
    assert is_dynamic_object({"label": "人"})
    assert not is_dynamic_object(ObjectSpatialObservation(object_id="b", label="桌子"))


def test_drop_preserves_order_and_does_not_mutate() -> None:
    src = [{"label": "桌子"}, {"label": "人"}, {"label": "椅子"}, {"label": "狗"}]
    kept = drop_dynamic_objects(src)
    assert [o["label"] for o in kept] == ["桌子", "椅子"]
    assert len(src) == 4, "原列表不该被改动"


def _observation(label: str, index: int) -> ObjectSpatialObservation:
    return ObjectSpatialObservation(
        object_id=f"obj_{index}",
        label=label,
        map_xyz=(float(index), 0.0, 0.0),
        spatial_quality="RGBD_METRIC",
        confidence=0.9,
    )


def _feed(graph: SemanticNavigationGraph, labels: tuple[str, ...]) -> None:
    """走真实的 update_observation 路径，喂两次（confirm 需要两次观测）。"""
    for round_index in (1, 2):
        objects = [_observation(label, i) for i, label in enumerate(labels)]
        graph.update_observation(
            observation_id=f"obs_{round_index}",
            heading_sector=0,
            scene_objects=[{"label": label} for label in labels],
            scene_relations=[],
            pose=None,
            spatial_objects=objects,
            timestamp=float(round_index),
        )


def _persisted_labels(graph: SemanticNavigationGraph) -> set[str]:
    return {entry.label for entry in graph.object_map.objects.values()}


def test_dynamic_object_is_not_persisted() -> None:
    """端到端正向：同一次观测里，椅子进地图，人进不去。"""
    graph = SemanticNavigationGraph()
    _feed(graph, ("办公椅", "人"))
    labels = _persisted_labels(graph)
    assert "办公椅" in labels, f"椅子本该进地图，实得 {labels}"
    assert "人" not in labels, f"人不该进持久地图，实得 {labels}"


def test_injection_calibration_without_filter_the_person_persists(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """**注入校准 —— 本条测试的要点。**

    把过滤函数换成恒等函数，"人"必须**出现**在地图里。这证明上一条断言不是
    空的：路径确实喂到了，是过滤器在起作用。若把 dynamic_labels 的过滤逻辑
    删掉，这一条仍然会通过（它本来就没过滤），但上一条会失败 —— 两条一起才
    说明"过滤器真的在拦"。
    """
    monkeypatch.setattr(
        "app.spatial.semantic_navigation_graph.drop_dynamic_objects",
        lambda objects: list(objects),
    )
    graph = SemanticNavigationGraph()
    _feed(graph, ("办公椅", "人"))
    labels = _persisted_labels(graph)
    assert "人" in labels, (
        "注入校准失败：不过滤时人也没进地图 —— 说明上一条正向用例是空的，"
        f"根本没喂到持久化路径。实得 {labels}"
    )


def test_worker_call_site_also_filters() -> None:
    """worker 侧是**第二个**持久化入口，不能只改 graph 那一处。"""
    src = (ROOT / "scripts" / "go2w" / "run_semantic_exploration.py").read_text(
        encoding="utf-8"
    )
    assert "drop_dynamic_objects(localized)" in src, (
        "worker 喂给 semantic_map.update_with_associations 之前没有过滤动态物体"
    )


def test_display_path_does_not_use_the_filtered_list() -> None:
    """§26 的另一半："仍然可以显示当前检测"。

    WebUI 的当前检测事件走 `observation.scene_objects`，只要它没被换成过滤后的
    localized 列表，人就还能被看见 —— 只是不会被当成永久地标。
    """
    src = (ROOT / "scripts" / "go2w" / "run_semantic_exploration.py").read_text(
        encoding="utf-8"
    )
    marker = src.index('"event": "semantic_frame_objects_localized"')
    block = src[marker - 800 : marker]
    assert "for scene_object in observation.scene_objects" in block, (
        "语义帧物体事件不再遍历 observation.scene_objects —— 过滤可能已经"
        "波及显示路径，人/狗/猫会从 WebUI 上消失"
    )
