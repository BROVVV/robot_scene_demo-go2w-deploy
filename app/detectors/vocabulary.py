"""Detection vocabulary for indoor robot scenes."""

from __future__ import annotations

import re

# 未知英文标签的中文显示兜底。它是**显示**兜底，绝不能当实体关联键 ——
# 否则 bin / barrel / stool 这些互不相干的未知类别会被并成同一个实体
# （计划书 §3）。`association_label()` 就是为这条存在的。
LABEL_ZH_FALLBACK = "未分类物体"

_CHINESE_RE = re.compile(r"[一-鿿]")


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
    "cardboard box": "纸箱",
    "carton": "纸箱",
    "tree trunk": "树干",
    "pillar": "立柱",
    "door frame": "门框",
    "parked car": "停放车辆",
    "sofa": "沙发",
    "basket": "篮子",
    "trash bin": "垃圾桶",
    "trash can": "垃圾桶",
    "waste bin": "垃圾桶",
    "garbage bin": "垃圾桶",
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
    # 2026-09-17：RAM++ 开放词表实测命中、而静态词典没覆盖的一批词。
    # 少一个，WebUI 三维框上就直接漏一个英文（计划书 §2.2）。
    # `bin` 的静态值只是兜底：RAM++ 同次返回的 tags_zh 优先于这里
    # （见 `resolve_label_zh`），所以 RAM 说是「容器」就显示「容器」。
    "bin": "垃圾桶",
    "barrel": "桶",
    "stool": "凳子",
    "lid": "盖子",
    "hook": "挂钩",
    "curtain": "窗帘",
    "speaker": "音箱",
    "plug": "插头",
    "pipe": "管道",
    "pipeline": "管道",
    "industrial pipe": "工业管道",
    "green slime": "绿色史莱姆泥",
    "slime": "史莱姆泥",
    "liquid leak": "液体泄漏",
    "fluid spill": "液体泄漏",
    "simulated liquid leak": "疑似管道液体泄漏",
    # YOLOE prompt-free 常见实体与本机实测长尾类别。
    "movie poster": "电影海报",
    "art print": "装饰画",
    "luggage": "行李",
    "bag": "包",
    "handbag": "手提包",
    "locker": "储物柜",
    "safe": "保险柜",
    "camera": "相机",
    "glasses": "眼镜",
    "pillow": "枕头",
    "cushion": "靠垫",
    "blanket": "毛毯",
    "towel": "毛巾",
    "bed": "床",
    "mattress": "床垫",
    "wood-burning stove": "燃木炉",
    "weight": "重物",
    "arcade machine": "街机",
    "baby bottle": "奶瓶",
    "elevator car": "电梯轿厢",
    "container": "容器",
    "traffic cone": "交通锥",
    "pallet": "托盘",
    "truck": "卡车",
    "car": "汽车",
    "tree": "树",
    "lamp post": "路灯",
    "fire hydrant": "消防栓",
    "electrical cabinet": "电气柜",
    "equipment cabinet": "设备柜",
    "storage tank": "储罐",
    "pipe support": "管道支架",
    "valve": "阀门",
    "pump": "泵",
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
    "cardboard box": "container",
    "carton": "container",
    "tree trunk": "structure",
    "pillar": "structure",
    "door frame": "structure",
    "parked car": "vehicle",
    "sofa": "furniture",
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
    "bin": "container",
    "barrel": "container",
    "lid": "container",
    "stool": "furniture",
    "hook": "hardware",
    "curtain": "furnishing",
    "speaker": "electronics",
    "plug": "electronics",
    "pipe": "structure",
    "pipeline": "structure",
    "industrial pipe": "structure",
    "green slime": "hazard",
    "slime": "hazard",
    "liquid leak": "hazard",
    "fluid spill": "hazard",
    "simulated liquid leak": "hazard",
    "movie poster": "sign",
    "art print": "furnishing",
    "luggage": "personal_item",
    "bag": "bag",
    "handbag": "bag",
    "locker": "furniture",
    "safe": "container",
    "camera": "electronics",
    "glasses": "personal_item",
    "pillow": "furnishing",
    "cushion": "furnishing",
    "blanket": "furnishing",
    "towel": "furnishing",
    "bed": "furniture",
    "mattress": "furniture",
    "wood-burning stove": "equipment",
    "weight": "object",
    "arcade machine": "equipment",
    "baby bottle": "container",
    "elevator car": "structure",
    "container": "container",
    "traffic cone": "safety_equipment",
    "pallet": "container",
    "truck": "vehicle",
    "car": "vehicle",
    "tree": "structure",
    "lamp post": "structure",
    "fire hydrant": "safety_equipment",
    "electrical cabinet": "equipment",
    "equipment cabinet": "equipment",
    "storage tank": "equipment",
    "pipe support": "structure",
    "valve": "equipment",
    "pump": "equipment",
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


RAM_SCENE_STOPLIST = {
    # 场景/画幅词，不是可定位的独立物体
    "indoor",
    "outdoor",
    "room",
    "home",
    "background",
    "image",
    "photo",
    # 动作词
    "sit",
    "stand",
    "lay",
    "hang",
    # 单独出现的纯颜色词（TargetProfile 里的颜色不受影响，只过滤 RAM scene tags）
    "gray",
    "grey",
    "green",
    "yellow",
    "black",
    "white",
    "red",
    "blue",
    # 材质词
    "wood",
    "metal",
    "plastic",
    # 2026-09-17：真机实测漏到三维物体框里的三类非独立物体词
    # （purple / sparkle / corner，见计划书 §2.1）。它们都是 RAM++ 扩出来的，
    # 目标画像自己的颜色属性走 TARGET_TERM_RULES，不经过这里，所以不误杀。
    "purple",
    "sparkle",
    "corner",
}


def normalize_scene_tag(tag: object) -> str:
    return " ".join(str(tag).lower().split()).strip(" .,;")


def _select_ram_scene_tags(
    tags_en: list[object], max_terms: int
) -> list[tuple[int, str]]:
    """过滤规则唯一实现，返回 `(原下标, 归一化英文词)`。

    只做规则过滤，不引入 NLP 模型、不做 embedding 相似度：
    小 stoplist + 1~4 个英文单词 + 归一化去重 + 截断。
    不在 `TERM_ZH` / `BASE_TERMS` 里的词**一律保留** —— RAM++ 的意义就是扩大
    开放物体覆盖，按封闭词表过滤等于白装。

    保留原下标是为了让中文侧能对回去（`tags_en` 与 `tags_zh` 同序同长）。
    """
    selected: list[tuple[int, str]] = []
    seen: set[str] = set()
    for index, raw_tag in enumerate(tags_en):
        normalized = normalize_scene_tag(raw_tag)
        if not normalized or normalized in RAM_SCENE_STOPLIST:
            continue
        if not 1 <= len(normalized.split()) <= 4:
            continue
        if normalized in seen:
            continue
        seen.add(normalized)
        selected.append((index, normalized))
        if len(selected) >= max_terms:
            break
    return selected


def filter_ram_scene_terms(tags: list[str], max_terms: int = 12) -> list[str]:
    """把 RAM++ 的 scene tags 收敛成能喂给 GroundingDINO 的物体词。"""
    return [term for _index, term in _select_ram_scene_tags(tags, max_terms)]


def filter_ram_scene_term_pairs(
    tags_en: list[object],
    tags_zh: list[object],
    max_terms: int = 12,
) -> list[tuple[str, str]]:
    """`filter_ram_scene_terms` 的配对版：过滤后仍保留 en -> zh 的对应关系。

    RAM++ 一次同时返回 `tags_en` 与 `tags_zh`（worker 的 `_tag`），两边同序同长。
    只消费 `tags_en` 会把 RAM 已经算好的中文丢掉，于是下游只能查静态词典、
    查不到就漏英文（计划书 §2.2）。这里把配对保住，供
    `GroundedSAMSubprocessDetector` 生成 `DetectedObject` 时查。

    长度不一致时**不猜**：只对两边都存在的下标做配对，多余的英文词中文侧留空
    （调用方回落到静态词典）。中文侧为空串的英文词同样保留 —— 丢中文只是少一条
    优先来源，丢词却是丢了 RAM++ 的开放词表覆盖。
    """
    zh_list = list(tags_zh)
    pairs: list[tuple[str, str]] = []
    for index, term in _select_ram_scene_tags(tags_en, max_terms):
        zh = normalize_scene_tag(zh_list[index]) if index < len(zh_list) else ""
        pairs.append((term, zh))
    return pairs


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


# 不该画成三维物体框的标签：纯颜色/属性词、视觉效果词、画面区域与背景词。
# 它们会通过 RAM++ 的开放词表混进检测结果（2026-09-17 实测上屏的有
# purple / sparkle / corner / 地面）。中英都收，因为同一条流水线上游是英文、
# 下游实体图的 label 是中文（见 app/spatial/dynamic_labels.py 的同类教训）。
#
# **刻意不含 "物体"/"object"/"item"**：它们是目标搜索的合法兜底词
# （TARGET_TERM_RULES 里「绿色」「物体」都映射到它们），拦掉会伤到目标链路。
NON_OBJECT_LABELS = frozenset(
    {
        # 纯颜色（RAM++ 把颜色当 tag 返回，且常与真实物体同帧）
        "purple", "紫色",
        "pink", "粉色", "粉红",
        "brown", "棕色",
        "orange", "橙色",
        # 视觉效果 / 材质属性
        "sparkle", "闪光", "反光",
        "glossy", "光泽",
        "shadow", "阴影",
        "reflection", "倒影",
        # 画面区域 / 背景
        "corner", "角落",
        "floor", "地面", "地板",
        "wall", "墙", "墙壁",
        "ceiling", "天花板",
        "ground", "background", "背景",
        "room", "房间",
        "indoor", "outdoor", "室外", "室内",
        "image", "photo", "画面",
        "sky", "天空",

        # YOLOE prompt-free 的 4k+ 固化词表里混有场景、媒体概念和物体零件。
        # 这些标签可以作为逐帧取证，但不应成为持久语义地图节点。
        "video", "video clip", "movie scene", "animation", "screenshot",
        "tv sitcom", "tv show", "television show",
        "text", "caption", "watermark", "logo",
        # 动作/状态不是可独立定位的物体。prompt-free 词表会直接产出这类标签。
        "sew", "sit", "stand", "lay", "hang", "walk", "run", "jump",
        "drive", "ride", "eat", "drink", "sleep", "cook", "read", "write",
        "watch", "play", "work", "open", "close",
        "elevator lobby", "lobby", "hallway", "corridor", "stairwell",
        "bank vault", "airplane window", "film premiere",
        "electric chair",
        "hinge", "door handle", "cabinet handle", "drawer handle",
        "doorknob", "knob", "camera lens", "zoom lens",
        "screw", "bolt", "nut", "washer",
        "face", "head", "hand", "arm", "leg", "foot", "eye", "ear",
        "mouth", "nose",
        # 动态动物与过小/瞬态视觉目标不作为长期拓扑地标。
        "american shorthair", "glass bead", "soap bubble", "christmas ball",
        "patient", "newborn", "newborn baby", "flip",
    }
)

# 同一类零部件/场景在 YOLOE 词表里有大量复合写法，例如 ``metal hinge``、
# ``front door handle``。仅列 exact label 会漏掉它们，因此只对边界明确的英文词元
# 做补充过滤；不使用模糊子串，避免 ``headboard`` 被 ``head`` 误伤。
NON_OBJECT_ENGLISH_TOKENS = frozenset(
    {
        "hinge", "handle", "doorknob", "knob", "lens", "window",
        "screw", "bolt", "washer",
        "lobby", "hallway", "corridor", "stairwell",
        "sitcom", "show", "scene", "premiere",
    }
)


# 复合写法里的分隔符。RAM++ 的中文配对会把同义词串成一条（实测「地板/地面」），
# 而中文化那一步会把实体图的 label 换成这个复合串，原始英文 `floor` 就没了 ——
# 整串去比 NON_OBJECT_LABELS 必然漏，于是一整块地面被画成三维物体框。
_COMPOUND_LABEL_SEPARATORS = re.compile(r"[/、,，|]")


def is_non_object_label(*labels: object) -> bool:
    """这些标签里有没有「非独立物体」的。

    调用方把原始英文与中文显示名一起传进来：同一个词在两条路上可能是
    英文（`floor`）也可能是中文（`地面`），只看一个必然漏。

    同一个词还可能以**复合写法**出现（`地板/地面`、`墙/墙壁`），所以整串比不中时
    再按分隔符拆开逐段比。拆分只在这个判断里做：`_normalize_label` 同时是关联
    键的归一化入口，在那里加拆分会改掉实体的关联键。
    """
    for label in labels:
        if label is None:
            continue
        text = str(label)
        normalized = _normalize_label(text)
        if normalized in NON_OBJECT_LABELS:
            return True
        if set(re.findall(r"[a-z]+", normalized)).intersection(
            NON_OBJECT_ENGLISH_TOKENS
        ):
            return True
        for part in _COMPOUND_LABEL_SEPARATORS.split(text):
            part = part.strip()
            if part and _normalize_label(part) in NON_OBJECT_LABELS:
                return True
    return False


def _resolve_zh(
    raw_label: object,
    runtime_zh: object = None,
    target_zh: object = None,
) -> str | None:
    """按优先级解析中文名，解析不出来返回 None（不兜底）。

    优先级（计划书 §6 阶段 A）：
        明确目标画像中文名 > RAM++ 同次 en->zh 配对 > 静态 TERM_ZH
        > 原文本身已经是中文
    只有 `resolve_label_zh` 会加兜底；`association_label` 要能区分
    「知道中文」和「不知道」，所以本体不许兜底。
    """
    if target_zh and str(target_zh).strip():
        return str(target_zh).strip()

    raw = str(raw_label or "").strip()
    candidate = str(runtime_zh or "").strip()
    # 兜底串不是知识，喂回来也不能算命中。
    if candidate and candidate != LABEL_ZH_FALLBACK and _CHINESE_RE.search(candidate):
        return candidate

    normalized = _normalize_label(raw)
    if normalized in TERM_ZH:
        return TERM_ZH[normalized]
    matched = _match_known_term(normalized, TERM_ZH)
    if matched != normalized:
        return TERM_ZH[matched]
    if raw and _CHINESE_RE.search(raw):
        return raw
    return None


def resolve_label_zh(
    raw_label: object,
    *,
    runtime_zh: object = None,
    target_zh: object = None,
    fallback: str = LABEL_ZH_FALLBACK,
) -> str:
    """唯一的中文显示名入口。**未知英文不许作为最终值漏出去。**

    `runtime_zh` 是 RAM++ 同一次调用返回的中文配对（`tags_zh`），
    `target_zh` 是目标画像里已经明确的中文名。
    """
    return _resolve_zh(raw_label, runtime_zh, target_zh) or fallback


def resolve_display_label(
    raw_label: object,
    *,
    runtime_zh: object = None,
    target_zh: object = None,
) -> str:
    """Return the best Chinese UI label or the explicit unknown sentinel.

    The raw detector label remains available in ``label`` for association and
    audit.  Runtime UIs that launch asynchronous VLM review may decorate the
    sentinel as a pending state, but this vocabulary layer must keep one stable
    value so unknown objects remain eligible for that review.
    """
    return _resolve_zh(raw_label, runtime_zh, target_zh) or LABEL_ZH_FALLBACK


def association_label(
    raw_label: object,
    *,
    runtime_zh: object = None,
    target_zh: object = None,
) -> str:
    """实体硬关联用的**规范标签**：知道中文就用中文，不知道就用原文。

    刻意不返回 `LABEL_ZH_FALLBACK`：两个互不相干的未知英文类别若共用
    「未分类物体」当关联键，会被 `SemanticObjectMap` 并成同一个实体
    （计划书 §3）。显示名走 `resolve_label_zh`，两条路分开。
    """
    resolved = _resolve_zh(raw_label, runtime_zh, target_zh)
    if resolved:
        return resolved
    return _normalize_label(str(raw_label or "")) or "object"


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
