"""轻量化计划 §26：动态物体不写入持久语义地图。

person / dog / cat 这类物体是动态的，写进持久地图就会变成永久 landmark ——
下次走到同一地点，地图上还站着一个早就不在那儿的人。

⚠ **计划书原文给的集合是英文 `{"person", "dog", "cat"}`，照抄会做出一个
永远匹配不上的死过滤器。** 这条流水线的 label 实际是中文：2026-09-16 实测
`artifacts/slam_validation/semanticD_20260910_170332/events.jsonl` 里的取值
是 垃圾桶 / 桌子 / 办公椅 / 电脑主机 / 背包 / 电脑显示器 / 风扇，一个英文
都没有。所以中英都收，并且 `label` 与 `label_zh` 两个字段都看。

**显示不受影响**：过滤发生在"喂给持久地图之前"，WebUI 的当前检测走的是
另一条数据流（`observation.scene_objects`）。这与 §26 的"仍然可以显示当前
检测"一致。

**也不做人/宠物的运动分类算法**（§26 明令）；这里只认标签，不做任何跟踪
或速度判断。
"""

from __future__ import annotations

from typing import Any

# 计划书的英文原集合 + 这条中文流水线实际会产出的等价标签。
# 刻意不含"宠物"这类二级词："宠物笼"是家具、不是动态物体，宁漏不误伤。
DYNAMIC_LABELS = frozenset(
    {
        # person
        "person",
        "people",
        "human",
        "pedestrian",
        "man",
        "woman",
        "人",
        "行人",
        "人物",
        "人员",
        "人类",
        "男人",
        "女人",
        # dog
        "dog",
        "puppy",
        "狗",
        "小狗",
        "犬",
        "宠物狗",
        # cat
        "cat",
        "kitten",
        "猫",
        "小猫",
        "宠物猫",
        "animal",
        "动物",
        "bird",
        "鸟",
        "pet",
        "宠物",
    }
)


def _label_of(obj: Any) -> str:
    """取标签：dict 与对象两种形态都吃，label_zh 优先于 label。

    与 `geometric_relation_extractor._label` 取的字段一致（那里也是
    label_zh -> label -> name），免得同一条流水线里两处对"标签是什么"
    有不同答案。
    """
    if isinstance(obj, dict):
        raw = obj.get("label_zh") or obj.get("label") or obj.get("name")
    else:
        raw = (
            getattr(obj, "label_zh", None)
            or getattr(obj, "label", None)
            or getattr(obj, "name", None)
        )
    return str(raw or "").strip().lower()


def is_dynamic_object(obj: Any) -> bool:
    """这个观测是不是 §26 说的动态物体。"""
    mobility = obj.get("mobility_class") if isinstance(obj, dict) else getattr(obj, "mobility_class", None)
    eligible = obj.get("topology_eligible") if isinstance(obj, dict) else getattr(obj, "topology_eligible", None)
    if mobility == "dynamic" or eligible is False:
        return True
    return _label_of(obj) in DYNAMIC_LABELS


def drop_dynamic_objects(objects: list[Any]) -> list[Any]:
    """过滤掉动态物体，保持原有顺序。返回新列表，不改入参。"""
    return [item for item in objects if not is_dynamic_object(item)]
