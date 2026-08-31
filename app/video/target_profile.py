"""Resolve arbitrary natural-language search requests into an open-vocabulary profile."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import re
from typing import TYPE_CHECKING, Any

from app.config import Settings, get_settings
from app.utils.json_utils import extract_json_from_text
from app.vision.vocab import COLOR_TERMS, COMMON_OBJECT_VOCAB

if TYPE_CHECKING:
    from app.task_understanding.schemas import GroundingPromptPlan


@dataclass(frozen=True)
class TargetProfile:
    raw_query: str
    canonical_name_zh: str
    target_type: str = "object"
    primary_labels_en: list[str] = field(default_factory=list)
    aliases_zh: list[str] = field(default_factory=list)
    aliases_en: list[str] = field(default_factory=list)
    attributes: list[str] = field(default_factory=list)
    colors: list[str] = field(default_factory=list)
    materials: list[str] = field(default_factory=list)
    affordances: list[str] = field(default_factory=list)
    relation_constraints: list[str] = field(default_factory=list)
    context_labels_en: list[str] = field(default_factory=list)
    context_labels_zh: list[str] = field(default_factory=list)
    likely_regions_zh: list[str] = field(default_factory=list)
    negative_terms: list[str] = field(default_factory=list)
    possible_confusions: list[str] = field(default_factory=list)
    search_hint_zh: str = ""
    resolver_source: str = "fallback"
    resolver_error: str | None = None
    target_category: str = "unknown"
    grounding_strategy: str = "unknown"
    direct_detection_supported: bool = True
    requires_proxy_objects: bool = False
    requires_scene_confirmation: bool = False
    requires_state_verification: bool = False
    grounding_prompt_plan: GroundingPromptPlan | None = None

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        if self.grounding_prompt_plan is not None:
            payload["grounding_prompt_plan"] = asdict(self.grounding_prompt_plan)
        payload.update(
            {
                "raw_target": self.raw_query,
                "core_entities": self.core_entities,
                "zh_terms": self.zh_terms,
                "en_terms": self.en_terms,
                "spatial_constraints": self.relation_constraints,
                "context_objects": self.context_terms(),
                "grounding_prompt": self.grounding_prompt,
            }
        )
        return payload

    @property
    def raw_target(self) -> str:
        return self.raw_query

    @property
    def core_entities(self) -> list[str]:
        return _dedupe([self.canonical_name_zh, *self.primary_labels_en[:2]])

    @property
    def zh_terms(self) -> list[str]:
        return _dedupe([self.canonical_name_zh, *self.aliases_zh])

    @property
    def en_terms(self) -> list[str]:
        return _dedupe([*self.primary_labels_en, *self.aliases_en])

    @property
    def grounding_prompt(self) -> str:
        return build_grounding_prompt(self)

    def direct_terms(self) -> list[str]:
        return _dedupe(
            [
                self.canonical_name_zh,
                *self.primary_labels_en,
                *self.aliases_zh,
                *self.aliases_en,
            ]
        )

    def detector_terms(self, max_terms: int = 40) -> list[str]:
        attribute_combinations = [
            f"{color} {term}"
            for color in self.colors
            for term in self.primary_labels_en[:3]
        ]
        terms = [
            *self.primary_labels_en,
            *attribute_combinations,
            *self.aliases_en,
        ]
        return _dedupe([_clean_detector_term(item) for item in terms if item])[:max_terms]

    def context_terms(self) -> list[str]:
        return _dedupe([*self.context_labels_en, *self.context_labels_zh])

    def prompt_context(self) -> str:
        return (
            "自然语言目标已解析为以下目标画像，请严格按画像判断，不要只做字面匹配："
            f"核心目标={self.canonical_name_zh}；类型={self.target_type}；"
            f"英文开放词表={self.primary_labels_en + self.aliases_en}；"
            f"属性={self.attributes}；关系约束={self.relation_constraints}；"
            f"上下文线索={self.context_labels_zh + self.context_labels_en}。"
            "只有画面证据满足核心目标及关键属性/关系时才设 is_present=true；"
            "matched_indices 只列真正构成目标的物体。"
        )


class TargetProfileResolver:
    """Use one cheap text-only model call, with a deterministic fallback."""

    def __init__(
        self,
        settings: Settings | None = None,
        client: Any | None = None,
    ) -> None:
        self.settings = settings or get_settings()
        self.client = client

    def resolve(self, query: str, use_llm: bool = True) -> TargetProfile:
        query = query.strip()
        if not query:
            raise ValueError("Target query must not be empty.")
        fallback = _fallback_profile(query)
        if not use_llm or not self.settings.siliconflow_api_key:
            return fallback
        try:
            from openai import OpenAI

            client = self.client or OpenAI(
                api_key=self.settings.siliconflow_api_key,
                base_url=self.settings.siliconflow_base_url,
                timeout=self.settings.siliconflow_timeout_seconds,
            )
            response = client.chat.completions.create(
                model=self.settings.reasoning_model,
                messages=[
                    {"role": "system", "content": _TARGET_PROFILE_SYSTEM_PROMPT},
                    {"role": "user", "content": _target_profile_user_prompt(query)},
                ],
                temperature=0.0,
                max_tokens=min(1200, self.settings.siliconflow_max_tokens),
            )
            content = _response_text(response)
            payload = extract_json_from_text(content)
            return _profile_from_payload(query, payload)
        except Exception as exc:
            return TargetProfile(
                **{
                    **asdict(fallback),
                    "resolver_error": f"{type(exc).__name__}: {exc}",
                }
            )


_TARGET_PROFILE_SYSTEM_PROMPT = """你是具身导航目标解析器。把任意自然语言搜索请求转成开放词表视觉目标画像。
只输出 JSON，不要 Markdown。不要假设目标已经出现，也不要生成坐标或路径。
英文标签必须是视觉检测器可识别的简短名词短语；上下文只能作为线索，不能冒充直接目标。"""


def _target_profile_user_prompt(query: str) -> str:
    return f"""搜索请求：{query}

返回：
{{
  "canonical_name_zh": "核心目标的简洁中文名称",
  "target_type": "object|place|person|text_sign|attribute_relation",
  "primary_labels_en": ["最直接的英文视觉类别，1到5个"],
  "aliases_zh": ["中文同义表达"],
  "aliases_en": ["英文同义类别"],
  "attributes": ["颜色、材质、状态、用途等约束"],
  "colors": ["颜色英文词"],
  "materials": ["材质"],
  "affordances": ["用途或可供性"],
  "relation_constraints": ["on table、next to door 等关系约束"],
  "context_labels_en": ["可能提示目标位置但不能证明目标存在的英文物体类别"],
  "context_labels_zh": ["对应中文上下文物体"],
  "likely_regions_zh": ["适合继续检查的语义区域"],
  "negative_terms": ["明确不应混淆为目标的类别"],
  "possible_confusions": ["外观相似的混淆物"],
  "search_hint_zh": "一句谨慎的搜索建议"
}}

规则：
- 去掉“找到、寻找、帮我找、带我去、看一下”等任务动词，保留完整目标语义。
- “红色把手的白色柜门”不能退化成“门”，要保留颜色、部件和所属关系。
- “能打印 A3 的设备”核心类别应包含 printer，属性保留 A3。
- 地点目标要给出直接标识类别和入口类别，同时把普通门/走廊放入 context，而非 direct。
- 不限于示例类别，可生成任何合理的开放词表英文短语。"""


def _profile_from_payload(query: str, payload: dict[str, Any]) -> TargetProfile:
    fallback = _fallback_profile(query)
    canonical = str(payload.get("canonical_name_zh") or fallback.canonical_name_zh).strip()
    primary = _string_list(payload.get("primary_labels_en"))
    aliases_en = _string_list(payload.get("aliases_en"))
    aliases_zh = _string_list(payload.get("aliases_zh"))
    return TargetProfile(
        raw_query=query,
        canonical_name_zh=canonical,
        target_type=str(payload.get("target_type") or "object"),
        primary_labels_en=primary or fallback.primary_labels_en,
        aliases_zh=_dedupe([canonical, *aliases_zh]),
        aliases_en=aliases_en,
        attributes=_string_list(payload.get("attributes")),
        colors=_string_list(payload.get("colors")),
        materials=_string_list(payload.get("materials")),
        affordances=_string_list(payload.get("affordances")),
        relation_constraints=_string_list(payload.get("relation_constraints")),
        context_labels_en=_string_list(payload.get("context_labels_en")),
        context_labels_zh=_string_list(payload.get("context_labels_zh")),
        likely_regions_zh=_string_list(payload.get("likely_regions_zh")),
        negative_terms=_string_list(payload.get("negative_terms")),
        possible_confusions=_string_list(payload.get("possible_confusions")),
        search_hint_zh=str(payload.get("search_hint_zh") or "").strip(),
        resolver_source="llm",
    )


def _fallback_profile(query: str) -> TargetProfile:
    cleaned = query.strip()
    prefixes = [
        "请帮我找到",
        "请帮我找",
        "帮我找到",
        "帮我找",
        "带我去",
        "寻找",
        "找到",
        "找一下",
        "找",
        "去",
    ]
    for prefix in prefixes:
        if cleaned.startswith(prefix) and len(cleaned) > len(prefix):
            cleaned = cleaned[len(prefix) :].strip(" ：:，,。")
            break
    colors = [
        english
        for chinese, english in COLOR_TERMS.items()
        if chinese in cleaned
    ]
    known = _known_profile(cleaned)
    if known is not None:
        return TargetProfile(
            raw_query=query, resolver_source="fallback",
            **{**known, "colors": colors},
        )
    if "打印" in cleaned or "A3" in cleaned.upper():
        return TargetProfile(
            raw_query=query,
            canonical_name_zh="A3打印机" if "A3" in cleaned.upper() else "打印机",
            target_type="object",
            primary_labels_en=["printer", "copier", "multifunction printer"],
            aliases_zh=["打印机", "打印设备"],
            aliases_en=["office printer", "office machine"],
            attributes=["supports A3 paper"] if "A3" in cleaned.upper() else [],
            affordances=["printing"],
            context_labels_en=["paper tray", "office desk"],
            context_labels_zh=["纸盒", "办公桌"],
            possible_confusions=["scanner"],
            resolver_source="fallback",
        )
    if "柜门" in cleaned:
        return TargetProfile(
            raw_query=query,
            canonical_name_zh=cleaned,
            primary_labels_en=["cabinet door", "door panel"],
            aliases_zh=["柜门", "柜子", "门板", "把手"],
            aliases_en=["cabinet", "handle", "cupboard door"],
            attributes=[
                item
                for item in ["white" if "白" in cleaned else "", "red handle" if "红" in cleaned and "把手" in cleaned else ""]
                if item
            ],
            colors=colors,
            affordances=["openable", "storage"],
            context_labels_en=["cabinet", "drawer", "wall"],
            context_labels_zh=["柜子", "抽屉", "墙"],
            negative_terms=["room door", "main door"],
            possible_confusions=["drawer", "refrigerator door", "room door"],
            resolver_source="fallback",
        )
    for chinese, terms in COMMON_OBJECT_VOCAB.items():
        if chinese in cleaned:
            return TargetProfile(
                raw_query=query,
                canonical_name_zh=chinese,
                primary_labels_en=terms[:4],
                aliases_zh=[chinese],
                aliases_en=terms[1:],
                colors=colors,
                resolver_source="fallback",
            )
    english_terms = [cleaned] if cleaned.isascii() else []
    return TargetProfile(
        raw_query=query,
        canonical_name_zh=cleaned or query,
        primary_labels_en=english_terms,
        aliases_zh=[cleaned] if cleaned else [query],
        aliases_en=english_terms,
        resolver_source="fallback",
    )


def _known_profile(cleaned: str) -> dict[str, Any] | None:
    known = {
        "手机": (["phone", "cell phone", "mobile phone", "smartphone"], ["桌子", "沙发", "床", "充电线"], ["table", "sofa", "bed", "charger"]),
        "电视": (
            ["television", "TV", "tv screen", "monitor", "display", "flat screen", "large black screen", "screen-like rectangle"],
            ["沙发", "电视柜", "客厅墙面", "遥控器", "娱乐柜"],
            ["sofa", "TV stand", "living room wall", "remote control", "entertainment cabinet"],
        ),
        "电视机": (
            ["television", "TV", "tv screen", "monitor", "display", "flat screen", "large black screen", "screen-like rectangle"],
            ["沙发", "电视柜", "客厅墙面", "遥控器", "娱乐柜"],
            ["sofa", "TV stand", "living room wall", "remote control", "entertainment cabinet"],
        ),
        "水杯": (["cup", "mug", "water bottle"], ["桌子", "厨房台面"], ["table", "countertop"]),
        "钥匙": (["key", "keychain"], ["桌子", "门", "鞋柜"], ["table", "door", "shoe cabinet"]),
        "灭火器": (
            ["fire extinguisher", "portable extinguisher"],
            ["墙边", "消防箱", "走廊"],
            ["wall", "fire cabinet", "corridor"],
        ),
        "消防器材": (
            ["fire extinguisher", "fire equipment", "emergency equipment"],
            ["墙边", "消防箱", "走廊"],
            ["wall", "fire cabinet", "corridor"],
        ),
        "厕所": (["toilet sign", "restroom sign", "bathroom sign"], ["门", "指示牌", "走廊"], ["door", "sign", "corridor"]),
        "洗手间": (["toilet sign", "restroom sign", "bathroom sign"], ["门", "指示牌", "走廊"], ["door", "sign", "corridor"]),
        "卫生间": (["toilet sign", "restroom sign", "bathroom sign"], ["门", "指示牌", "走廊"], ["door", "sign", "corridor"]),
    }
    for term, (labels, context_zh, context_en) in known.items():
        if term in cleaned:
            canonical = (
                "厕所"
                if term in {"洗手间", "卫生间"}
                else "电视"
                if term == "电视机"
                else term
            )
            return {
                "canonical_name_zh": canonical,
                "target_type": "place" if canonical == "厕所" else "object",
                "primary_labels_en": labels,
                "aliases_zh": [term, canonical],
                "aliases_en": labels,
                "context_labels_zh": context_zh,
                "context_labels_en": context_en,
                "likely_regions_zh": ["客厅", "影音区"] if canonical == "电视" else [],
                "negative_terms": (
                    ["window", "mirror", "poster", "painting", "whiteboard"]
                    if canonical == "电视"
                    else []
                ),
                "search_hint_zh": "",
            }
    return None


def build_grounding_prompt(profile: TargetProfile, max_terms: int = 40) -> str:
    """Build an English-first, period-delimited open-vocabulary prompt."""
    terms = profile.detector_terms(max_terms=max_terms)
    terms.extend(profile.context_labels_en)
    cleaned = _dedupe([_clean_detector_term(item) for item in terms if item])
    return ". ".join(cleaned[:max_terms]) + ("." if cleaned else "")


def _response_text(response: Any) -> str:
    content = response.choices[0].message.content
    if not isinstance(content, str) or not content.strip():
        raise ValueError("Target profile response was empty.")
    return content.strip()


def _string_list(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return _dedupe([str(item).strip() for item in value if str(item).strip()])


def _clean_detector_term(value: str) -> str:
    return " ".join(
        re.sub(r"[^a-z0-9\u4e00-\u9fff]+", " ", value.lower()).split()
    )


def _dedupe(values: list[str]) -> list[str]:
    result: list[str] = []
    seen: set[str] = set()
    for value in values:
        key = value.lower().strip()
        if not key or key in seen:
            continue
        seen.add(key)
        result.append(value.strip())
    return result
