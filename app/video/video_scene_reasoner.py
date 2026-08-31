"""Scene-centric reasoning for every sampled first-person video frame."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

from app.video.models import FrameAnalysisResult
from app.video.target_context_rules import TARGET_CONTEXT_RULES
from app.video.target_profile import TargetProfile
from app.video.target_search import canonical_target, normalize_label


@dataclass
class VideoFrameContext:
    video_id: str
    frame_id: int
    timestamp_sec: float
    image_path: str
    target: str
    detector_result: dict[str, Any] | None = None
    previous_memory_context: list[dict[str, Any]] | None = None
    previous_frame_summary: str | None = None


@dataclass
class FrameSceneReasoningResult:
    video_id: str
    frame_id: int
    timestamp_sec: float
    scene_understanding: dict[str, Any]
    landmarks: list[dict[str, Any]]
    objects: list[dict[str, Any]]
    regions: list[dict[str, Any]]
    target_evidence: dict[str, Any]
    negative_evidence: list[str]
    psg_hypotheses: list[dict[str, Any]]
    memory_update: dict[str, Any]
    reasoning_summary: dict[str, Any]
    raw_model_output: dict[str, Any] | None = None
    image_path: str = ""
    annotated_frame_path: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class VideoSceneReasoner:
    """Convert detector/LLM frame output into stable navigation memory."""

    def __init__(
        self,
        target: str,
        target_profile: TargetProfile,
        enable_negative_evidence: bool = True,
        always_write_memory: bool = True,
    ) -> None:
        self.target = target
        self.profile = target_profile
        self.enable_negative_evidence = enable_negative_evidence
        self.always_write_memory = always_write_memory

    def reason(
        self,
        video_id: str,
        frame: FrameAnalysisResult,
        memory_context: dict[str, Any] | None = None,
        previous_frame_summary: str | None = None,
    ) -> FrameSceneReasoningResult:
        room_type, room_confidence = _infer_room_type(frame)
        landmarks = _extract_landmarks(frame.objects)
        regions = _extract_regions(frame.objects)
        direct_objects = [
            obj for obj in frame.objects if bool(obj.get("is_target_candidate"))
        ]
        contextual_objects = _contextual_objects(frame.objects, self.target, self.profile)
        directly_found = bool(direct_objects)
        candidate_found = bool(contextual_objects)
        target_evidence = {
            "target": self.target,
            "directly_found": directly_found,
            "candidate_found": candidate_found,
            "direct_evidence": [
                {
                    "object_id": obj.get("object_id"),
                    "name": obj.get("label_zh") or obj.get("label"),
                    "position": obj.get("image_position"),
                    "confidence": obj.get("confidence", 0.0),
                }
                for obj in direct_objects
            ],
            "indirect_evidence": [
                f"画面出现与“{self.target}”相关的环境线索："
                f"{obj.get('label_zh') or obj.get('label')}。"
                for obj in contextual_objects
            ],
            "best_candidate": (
                max(direct_objects, key=lambda item: float(item.get("confidence", 0.0)))
                if direct_objects
                else None
            ),
        }
        negative = (
            _negative_evidence(self.target, room_type, regions, candidate_found)
            if self.enable_negative_evidence and not directly_found
            else []
        )
        hypotheses = _build_hypotheses(
            target=self.target,
            profile=self.profile,
            room_type=room_type,
            directly_found=directly_found,
            contextual_objects=contextual_objects,
            negative_evidence=negative,
            memory_context=memory_context or {},
        )
        scene_summary = frame.scene_summary.strip() or (
            f"当前帧为疑似{_room_zh(room_type)}，记录了可见环境但没有可靠自由文本摘要。"
        )
        visual_quality = "low" if frame.error else "medium"
        importance = "high" if directly_found else "medium" if candidate_found else "low"
        should_write = not bool(frame.error) and bool(
            scene_summary or frame.objects or landmarks or regions
        )
        if self.always_write_memory and not frame.error:
            should_write = True
        landmark_names = [item["name"] for item in landmarks]
        tags = _dedupe(
            [
                room_type,
                *landmark_names,
                *(["target_found"] if directly_found else ["negative_target_evidence"]),
                *(["traversable"] if regions else []),
            ]
        )
        conclusion = (
            f"当前帧直接观察到“{self.target}”，应保存正证据和环境上下文。"
            if directly_found
            else f"当前帧未直接观察到“{self.target}”，但应保存环境与负目标证据。"
        )
        evidence = [
            f"场景摘要：{scene_summary}",
            (
                f"稳定参照物包括：{'、'.join(landmark_names)}。"
                if landmark_names
                else "没有识别出高置信度稳定参照物。"
            ),
        ]
        return FrameSceneReasoningResult(
            video_id=video_id,
            frame_id=frame.frame_id,
            timestamp_sec=frame.timestamp_sec,
            image_path=frame.image_path,
            annotated_frame_path=frame.annotated_frame_path,
            scene_understanding={
                "room_type": room_type,
                "room_type_confidence": room_confidence,
                "scene_summary": scene_summary,
                "visual_quality": visual_quality,
                "egocentric_view": _egocentric_view(frame.objects, regions),
            },
            landmarks=landmarks,
            objects=[dict(item) for item in frame.objects],
            regions=regions,
            target_evidence=target_evidence,
            negative_evidence=negative,
            psg_hypotheses=hypotheses,
            memory_update={
                "should_write": should_write,
                "memory_kind": (
                    "target_observation" if directly_found else "environment_observation"
                ),
                "importance": importance,
                "summary": (
                    f"{scene_summary} "
                    + (
                        f"已发现目标“{self.target}”。"
                        if directly_found
                        else f"未发现目标“{self.target}”。"
                    )
                ).strip(),
                "tags": tags,
            },
            reasoning_summary={
                "evidence": evidence,
                "negative_evidence": negative,
                "hypotheses": [item["hypothesis"] for item in hypotheses],
                "conclusion": conclusion,
                "confidence": round(
                    max(0.45, min(0.92, (room_confidence + 0.65) / 2)), 3
                ),
            },
            raw_model_output={
                "scene_summary": frame.scene_summary,
                "metadata": frame.metadata,
            },
            metadata={"previous_frame_summary": previous_frame_summary},
        )


def _infer_room_type(frame: FrameAnalysisResult) -> tuple[str, float]:
    text = " ".join(
        [
            frame.scene_summary,
            *[
                str(obj.get("label", "")) + " " + str(obj.get("label_zh", ""))
                for obj in frame.objects
            ],
        ]
    ).lower()
    rules = [
        ("corridor", ["corridor", "hallway", "走廊", "过道"]),
        ("entrance", ["entrance", "foyer", "玄关", "门口", "鞋柜"]),
        ("living_room", ["living room", "sofa", "coffee table", "客厅", "沙发", "茶几"]),
        ("bedroom", ["bedroom", "bed", "床", "卧室", "床头柜"]),
        ("kitchen", ["kitchen", "sink", "countertop", "厨房", "水槽", "灶台"]),
        ("office", ["office", "monitor", "computer", "办公", "显示器", "打印机"]),
        ("bathroom", ["bathroom", "toilet", "卫生间", "厕所", "洗手间"]),
        ("stairway", ["stairs", "staircase", "楼梯"]),
    ]
    for room_type, terms in rules:
        if any(term in text for term in terms):
            return room_type, 0.74
    labels = {normalize_label(str(obj.get("label", ""))) for obj in frame.objects}
    if "door" in labels and ("wall" in labels or "floor" in labels):
        return "corridor_or_entrance", 0.55
    return "unknown", 0.35


def _extract_landmarks(objects: list[dict[str, Any]]) -> list[dict[str, Any]]:
    stable_terms = {
        "door", "doorway", "wall", "floor", "table", "desk", "cabinet", "sofa",
        "bed", "window", "stairs", "staircase", "outlet", "门", "门框", "墙",
        "地面", "地板", "桌子", "书桌", "柜子", "沙发", "床", "窗", "楼梯", "插座",
    }
    landmarks = []
    for obj in objects:
        names = {normalize_label(str(obj.get("label", ""))), normalize_label(str(obj.get("label_zh", "")))}
        if not any(
            term == name or term in name
            for term in stable_terms
            for name in names
            if name
        ):
            continue
        landmarks.append(
            {
                "name": obj.get("label_zh") or obj.get("label") or "landmark",
                "type": "stable_landmark",
                "position": obj.get("image_position", "unknown"),
                "confidence": float(obj.get("confidence", 0.5)),
                "object_id": obj.get("object_id"),
            }
        )
    return landmarks


def _extract_regions(objects: list[dict[str, Any]]) -> list[dict[str, Any]]:
    regions: list[dict[str, Any]] = []
    for obj in objects:
        name = normalize_label(
            f"{obj.get('label', '')} {obj.get('label_zh', '')}"
        )
        if any(term in name for term in ["floor", "ground", "地面", "地板", "过道"]):
            regions.append(
                {
                    "name": "front_floor",
                    "type": "traversable_region",
                    "status": "free",
                    "position": obj.get("image_position", "lower_center"),
                    "confidence": min(0.85, float(obj.get("confidence", 0.5))),
                }
            )
        if any(term in name for term in ["door", "doorway", "门", "门框"]):
            regions.append(
                {
                    "name": "doorway_region",
                    "type": "transition_region",
                    "status": "uncertain",
                    "position": obj.get("image_position", "center"),
                    "confidence": min(0.8, float(obj.get("confidence", 0.5))),
                }
            )
    return _unique_dicts(regions, "name")


def _contextual_objects(
    objects: list[dict[str, Any]], target: str, profile: TargetProfile
) -> list[dict[str, Any]]:
    rule = TARGET_CONTEXT_RULES.get(canonical_target(target), {})
    terms = [
        *profile.context_terms(),
        *rule.get("likely_objects", []),
    ]
    normalized = [normalize_label(item) for item in terms if normalize_label(item)]
    matches = []
    for obj in objects:
        label = normalize_label(
            f"{obj.get('label', '')} {obj.get('label_zh', '')}"
        )
        if label and any(term in label or label in term for term in normalized):
            matches.append(obj)
    return matches


def _negative_evidence(
    target: str,
    room_type: str,
    regions: list[dict[str, Any]],
    candidate_found: bool,
) -> list[str]:
    evidence = [f"当前关键帧没有直接观察到目标“{target}”。"]
    if regions:
        evidence.append(f"当前可见区域中未发现明确的“{target}”外观证据。")
    if not candidate_found:
        evidence.append(
            f"当前{_room_zh(room_type)}缺少与“{target}”强相关的候选参照物。"
        )
    return evidence


def _build_hypotheses(
    target: str,
    profile: TargetProfile,
    room_type: str,
    directly_found: bool,
    contextual_objects: list[dict[str, Any]],
    negative_evidence: list[str],
    memory_context: dict[str, Any],
) -> list[dict[str, Any]]:
    if directly_found:
        return [
            {
                "hypothesis": f"目标“{target}”位于当前观察区域，应以该帧作为后续复查锚点。",
                "supporting_evidence": ["当前帧存在直接视觉目标证据。"],
                "relation_type": "target_found_in",
                "confidence": 0.9,
            }
        ]
    hypotheses = [
        {
            "hypothesis": (
                f"目标“{target}”未出现在当前{_room_zh(room_type)}的可见范围内。"
            ),
            "supporting_evidence": negative_evidence,
            "relation_type": "target_not_found_in",
            "confidence": 0.68 if room_type != "unknown" else 0.55,
        }
    ]
    if contextual_objects:
        names = _dedupe(
            [
                str(obj.get("label_zh") or obj.get("label"))
                for obj in contextual_objects
            ]
        )
        hypotheses.append(
            {
                "hypothesis": (
                    f"当前区域出现{'、'.join(names)}，与“{target}”存在上下文关联，建议复查。"
                ),
                "supporting_evidence": [f"检测到上下文参照物：{'、'.join(names)}。"],
                "relation_type": "should_revisit",
                "confidence": min(0.82, 0.58 + 0.05 * len(names)),
            }
        )
    else:
        hypotheses.append(
            {
                "hypothesis": (
                    f"当前区域对“{target}”搜索价值较低，但应保留为已搜索负证据。"
                ),
                "supporting_evidence": negative_evidence,
                "relation_type": "low_priority_for_target",
                "confidence": 0.62,
            }
        )
    likely_places = (
        profile.likely_regions_zh
        or memory_context.get("target_prior", {}).get("likely_places", [])
        or TARGET_CONTEXT_RULES.get(canonical_target(target), {}).get("likely_regions", [])
    )
    if likely_places:
        hypotheses.append(
            {
                "hypothesis": (
                    f"后续应优先搜索{'、'.join(likely_places[:5])}等区域。"
                ),
                "supporting_evidence": ["目标位置先验与历史记忆检索结果。"],
                "relation_type": "high_priority_for_target",
                "confidence": 0.66,
            }
        )
    return hypotheses


def _egocentric_view(
    objects: list[dict[str, Any]], regions: list[dict[str, Any]]
) -> dict[str, str]:
    directions: dict[str, list[str]] = {"left": [], "front": [], "right": []}
    for obj in objects:
        position = str(obj.get("image_position", ""))
        label = str(obj.get("label_zh") or obj.get("label") or "物体")
        if "left" in position:
            directions["left"].append(label)
        elif "right" in position:
            directions["right"].append(label)
        else:
            directions["front"].append(label)
    return {
        "front": "、".join(directions["front"][:5]) or "前方信息有限",
        "left": "、".join(directions["left"][:5]) or "左侧信息有限",
        "right": "、".join(directions["right"][:5]) or "右侧信息有限",
        "floor": (
            "检测到可通行地面候选"
            if any(item["type"] == "traversable_region" for item in regions)
            else "地面通行性不确定"
        ),
        "far_region": "仅凭单目视频帧无法可靠判断远端连通性",
    }


def _room_zh(room_type: str) -> str:
    return {
        "corridor": "走廊",
        "entrance": "玄关",
        "living_room": "客厅",
        "bedroom": "卧室",
        "kitchen": "厨房",
        "office": "办公室",
        "bathroom": "卫生间",
        "stairway": "楼梯区域",
        "corridor_or_entrance": "走廊或玄关",
        "unknown": "未知区域",
    }.get(room_type, room_type)


def _dedupe(values: list[str]) -> list[str]:
    result = []
    seen = set()
    for value in values:
        key = value.strip().lower()
        if key and key not in seen:
            result.append(value.strip())
            seen.add(key)
    return result


def _unique_dicts(items: list[dict[str, Any]], key: str) -> list[dict[str, Any]]:
    result = []
    seen = set()
    for item in items:
        value = item.get(key)
        if value in seen:
            continue
        seen.add(value)
        result.append(item)
    return result
