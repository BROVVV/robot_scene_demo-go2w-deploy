"""Detection vocabulary for indoor robot scenes."""

from __future__ import annotations


TERM_ZH = {
    "object": "物体",
    "item": "物品",
    "base": "底座",
    "black base": "黑色底座",
    "green object": "绿色物体",
    "green item": "绿色物品",
    "chair": "椅子",
    "office chair": "办公椅",
    "clothing": "衣服",
    "yellow clothing": "黄色衣服",
    "coat": "外套",
    "jacket": "夹克",
    "shirt": "上衣",
    "desk": "桌子",
    "table": "桌子",
    "cabinet": "柜子",
    "drawer": "抽屉",
    "monitor": "显示器",
    "computer monitor": "显示器",
    "computer case": "电脑主机",
    "computer": "电脑",
    "robot": "机器人设备",
    "machine": "设备",
    "person": "人",
    "box": "箱子",
    "basket": "篮子",
    "trash bin": "垃圾桶",
    "shoe": "鞋",
    "bottle": "瓶子",
    "cup": "水杯",
    "cable": "线缆",
    "shelf": "架子",
    "door": "门",
    "wall": "墙",
    "floor": "地面",
    "backpack": "书包",
    "phone": "手机",
    "toilet": "厕所",
    "restroom": "厕所",
    "bathroom": "厕所",
    "washroom": "厕所",
    "lavatory": "厕所",
    "toilet sign": "厕所标识",
    "restroom sign": "厕所标识",
    "bathroom sign": "厕所标识",
    "sign": "指示牌",
    "fire extinguisher": "灭火器",
    "portable extinguisher": "灭火器",
    "fire suppression unit": "灭火器",
}

TERM_CATEGORY = {
    "object": "unknown",
    "item": "unknown",
    "base": "structure",
    "black base": "structure",
    "green object": "unknown",
    "green item": "unknown",
    "chair": "furniture",
    "office chair": "furniture",
    "desk": "furniture",
    "table": "furniture",
    "cabinet": "furniture",
    "drawer": "furniture",
    "shelf": "furniture",
    "clothing": "clothing",
    "yellow clothing": "clothing",
    "coat": "clothing",
    "jacket": "clothing",
    "shirt": "clothing",
    "monitor": "electronics",
    "computer monitor": "electronics",
    "computer case": "electronics",
    "computer": "electronics",
    "phone": "electronics",
    "robot": "robot",
    "machine": "equipment",
    "person": "person",
    "box": "container",
    "basket": "container",
    "trash bin": "container",
    "bottle": "container",
    "cup": "container",
    "shoe": "personal_item",
    "cable": "cable",
    "door": "structure",
    "wall": "structure",
    "floor": "structure",
    "backpack": "bag",
    "toilet": "place",
    "restroom": "place",
    "bathroom": "place",
    "washroom": "place",
    "lavatory": "place",
    "toilet sign": "sign",
    "restroom sign": "sign",
    "bathroom sign": "sign",
    "sign": "sign",
    "fire extinguisher": "safety_equipment",
    "portable extinguisher": "safety_equipment",
    "fire suppression unit": "safety_equipment",
}

BASE_TERMS = [
    "chair",
    "office chair",
    "clothing",
    "yellow clothing",
    "coat",
    "jacket",
    "shirt",
    "desk",
    "table",
    "cabinet",
    "drawer",
    "monitor",
    "computer monitor",
    "computer case",
    "computer",
    "robot",
    "machine",
    "person",
    "box",
    "basket",
    "trash bin",
    "shoe",
    "bottle",
    "cup",
    "cable",
    "shelf",
    "door",
    "backpack",
    "phone",
    "sign",
]

TARGET_TERM_RULES = {
    "绿色": ["green object", "green item", "object", "item"],
    "黑色": ["black base", "base", "object", "item"],
    "底座": ["base", "black base"],
    "物体": ["object", "item"],
    "黄衣服": ["yellow clothing", "clothing", "coat", "jacket", "shirt"],
    "衣服": ["clothing", "coat", "jacket", "shirt"],
    "椅子": ["chair", "office chair"],
    "凳子": ["chair"],
    "桌": ["desk", "table"],
    "手机": ["phone"],
    "显示器": ["monitor", "computer monitor"],
    "主机": ["computer case", "computer"],
    "电脑": ["computer", "computer monitor", "computer case"],
    "箱": ["box"],
    "篮": ["basket"],
    "鞋": ["shoe"],
    "杯": ["cup"],
    "瓶": ["bottle"],
    "包": ["backpack"],
    "人": ["person"],
    "机器人": ["robot", "machine"],
    "灭火器": [
        "fire extinguisher",
        "portable extinguisher",
        "fire suppression unit",
    ],
    "消防器材": [
        "fire extinguisher",
        "fire equipment",
        "emergency equipment",
    ],
    "厕所": [
        "toilet",
        "restroom",
        "bathroom",
        "washroom",
        "lavatory",
        "toilet sign",
        "restroom sign",
        "bathroom sign",
        "sign",
        "door",
    ],
    "洗手间": [
        "toilet",
        "restroom",
        "bathroom",
        "washroom",
        "toilet sign",
        "restroom sign",
        "sign",
        "door",
    ],
    "卫生间": [
        "toilet",
        "restroom",
        "bathroom",
        "washroom",
        "toilet sign",
        "restroom sign",
        "sign",
        "door",
    ],
}


def build_detection_terms(
    target_text: str,
    max_terms: int = 36,
    dynamic_terms: list[str] | None = None,
) -> list[str]:
    terms: list[str] = []
    terms.extend(dynamic_terms or [])
    for key, values in TARGET_TERM_RULES.items():
        if key in target_text:
            terms.extend(values)

    terms.extend(BASE_TERMS)
    deduped: list[str] = []
    seen: set[str] = set()
    for term in terms:
        if term in seen:
            continue
        seen.add(term)
        deduped.append(term)
        if len(deduped) >= max_terms:
            break
    return deduped


def build_detection_prompts(
    target_text: str,
    max_terms: int = 36,
    dynamic_terms: list[str] | None = None,
    context_terms: list[str] | None = None,
    include_base_terms: bool = True,
) -> list[str]:
    target_terms: list[str] = list(dynamic_terms or [])
    for key, values in TARGET_TERM_RULES.items():
        if key in target_text:
            target_terms.extend(values)
    target_terms = _dedupe(target_terms)

    prompts: list[str] = []
    if target_terms:
        prompts.append(terms_to_prompt(target_terms[:16]))

    if context_terms:
        prompts.append(terms_to_prompt(_dedupe(context_terms)[:12]))

    if include_base_terms:
        base_terms = build_detection_terms(
            target_text,
            max_terms=max_terms,
            dynamic_terms=dynamic_terms,
        )
        prompts.append(terms_to_prompt(base_terms))
    return _dedupe(prompts)


def terms_to_prompt(terms: list[str]) -> str:
    return ". ".join(terms) + "."


def label_zh(label: str) -> str:
    normalized = _normalize_label(label)
    matched = _match_known_term(normalized, TERM_ZH)
    return TERM_ZH.get(matched, normalized)


def category_for_label(label: str) -> str:
    normalized = _normalize_label(label)
    matched = _match_known_term(normalized, TERM_CATEGORY)
    return TERM_CATEGORY.get(matched, "unknown")


def color_for_label(label: str) -> str | None:
    normalized = _normalize_label(label)
    if "yellow" in normalized:
        return "yellow"
    if "black" in normalized:
        return "black"
    if "white" in normalized:
        return "white"
    if "green" in normalized:
        return "green"
    return None


def _normalize_label(label: str) -> str:
    return label.lower().strip().strip(".")


def _match_known_term(label: str, mapping: dict[str, str]) -> str:
    if label in mapping:
        return label
    matches = [
        term
        for term in mapping
        if term in label or all(part in label for part in term.split())
    ]
    if not matches:
        return label
    return max(matches, key=len)


def _dedupe(values: list[str]) -> list[str]:
    deduped: list[str] = []
    seen: set[str] = set()
    for value in values:
        if value in seen:
            continue
        seen.add(value)
        deduped.append(value)
    return deduped
