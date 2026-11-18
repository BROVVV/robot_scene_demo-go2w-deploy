"""Pure-domain support for passive target watching during teleoperated mapping.

This module deliberately has no ROS, FastAPI, detector-process, or network
dependency.  It owns the request/state contract, prompt budgeting, candidate
gating and deterministic target-profile post-processing used by both the web
process and the semantic sidecar.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time
from typing import Any, Iterable, Mapping, Sequence


SCHEMA_VERSION = 1
WATCH_STATES = frozenset({
    "IDLE", "PARSING", "ARMED", "CANDIDATE", "VERIFYING",
    "CONFIRMED", "REJECTED", "UNKNOWN", "DEGRADED",
})
VERIFY_MODES = frozenset({"object", "attribute", "relation", "diffuse_hazard"})


@dataclass(frozen=True)
class TargetWatchSpec:
    raw_query: str
    canonical_name_zh: str
    target_type: str
    target_terms_en: tuple[str, ...]
    target_aliases_zh: tuple[str, ...] = ()
    attributes: tuple[str, ...] = ()
    colors: tuple[str, ...] = ()
    relation_constraints: tuple[str, ...] = ()
    anchor_terms_en: tuple[str, ...] = ()
    negative_terms: tuple[str, ...] = ()
    possible_confusions: tuple[str, ...] = ()
    verification_mode: str = "object"
    resolver_source: str = "fallback"
    resolver_error: str | None = None

    def __post_init__(self) -> None:
        if not self.raw_query.strip() or len(self.raw_query) > 500:
            raise ValueError("target query must contain 1..500 characters")
        if self.verification_mode not in VERIFY_MODES:
            raise ValueError(f"unsupported verification mode: {self.verification_mode}")
        if not self.target_terms_en:
            raise ValueError("target_terms_en must not be empty")

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        for name in (
            "target_terms_en", "target_aliases_zh", "attributes", "colors",
            "relation_constraints", "anchor_terms_en", "negative_terms",
            "possible_confusions",
        ):
            value[name] = list(value[name])
        return value

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "TargetWatchSpec":
        return cls(
            raw_query=str(value.get("raw_query") or "").strip(),
            canonical_name_zh=str(value.get("canonical_name_zh") or "").strip(),
            target_type=str(value.get("target_type") or "object"),
            target_terms_en=_terms(value.get("target_terms_en")),
            target_aliases_zh=_terms(value.get("target_aliases_zh")),
            attributes=_terms(value.get("attributes")),
            colors=_terms(value.get("colors")),
            relation_constraints=_terms(value.get("relation_constraints")),
            anchor_terms_en=_terms(value.get("anchor_terms_en")),
            negative_terms=_terms(value.get("negative_terms")),
            possible_confusions=_terms(value.get("possible_confusions")),
            verification_mode=str(value.get("verification_mode") or "object"),
            resolver_source=str(value.get("resolver_source") or "fallback"),
            resolver_error=(
                str(value["resolver_error"]) if value.get("resolver_error") else None
            ),
        )


@dataclass(frozen=True)
class TargetCandidate:
    candidate_id: str
    target_generation: int
    frame_id: str
    source_object_id: str
    matched_term: str
    detection_slots: tuple[str, ...]
    detector_score: float
    bbox_2d: tuple[float, float, float, float]
    mask_available: bool
    anchor_candidate_ids: tuple[str, ...] = ()
    verification_bbox: tuple[float, float, float, float] | None = None
    camera_xyz: tuple[float, float, float] | None = None
    map_xyz: tuple[float, float, float] | None = None
    spatial_quality: str | None = None
    provenance: dict[str, Any] = field(default_factory=dict)
    capture_timestamp: float = 0.0
    evidence_path: str | None = None
    persistent_object_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        for key in (
            "detection_slots", "bbox_2d", "anchor_candidate_ids",
            "verification_bbox", "camera_xyz", "map_xyz",
        ):
            if value.get(key) is not None:
                value[key] = list(value[key])
        return value


@dataclass(frozen=True)
class DetectionBatch:
    raw_detections: tuple[Any, ...]
    mapping_detections: tuple[Any, ...]
    target_candidates: tuple[Any, ...]
    target_anchors: tuple[Any, ...]
    prompt_terms: tuple[str, ...]
    slot_terms: dict[str, tuple[str, ...]]
    diagnostics: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class TargetWatchConfig:
    enabled: bool = True
    auto_hazard_enabled: bool = True
    max_target_terms: int = 6
    max_anchor_terms: int = 4
    reserve_scene_terms: int = 3
    min_score_yoloe_seg: float = 0.20
    min_score_grounded_sam: float = 0.12
    max_bbox_area_ratio: float = 0.80
    max_per_frame: int = 2
    same_candidate_cooldown_s: float = 20.0
    relation_max_center_distance_ratio: float = 0.65
    relation_crop_margin_ratio: float = 0.08
    verify_enabled: bool = True
    socket_path: str = "runtime/go2w/siliconflow_vlm.sock"
    max_inflight: int = 1
    max_pending: int = 1
    client_timeout_s: float = 12.0
    max_calls_per_minute: int = 3
    confirmation_min_confidence: float = 0.70
    cache_ttl_s: float = 120.0
    stop_after_first_object_confirmation: bool = True
    continue_after_hazard_confirmation: bool = True
    evidence_max_files: int = 30
    evidence_jpeg_quality: int = 92

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any] | None) -> "TargetWatchConfig":
        value = dict(raw or {})
        auto_hazard = dict(value.get("auto_hazard") or {})
        candidate = dict(value.get("candidate") or {})
        score = dict(candidate.get("min_score") or {})
        verify = dict(value.get("verify") or {})
        evidence = dict(value.get("evidence") or {})
        result = cls(
            enabled=bool(value.get("enabled", True)),
            auto_hazard_enabled=bool(auto_hazard.get("enabled", True)),
            max_target_terms=int(value.get("max_target_terms", 6)),
            max_anchor_terms=int(value.get("max_anchor_terms", 4)),
            reserve_scene_terms=int(value.get("reserve_scene_terms", 3)),
            min_score_yoloe_seg=float(score.get("yoloe_seg", 0.20)),
            min_score_grounded_sam=float(score.get("grounded_sam", 0.12)),
            max_bbox_area_ratio=float(candidate.get("max_bbox_area_ratio", 0.80)),
            max_per_frame=int(candidate.get("max_per_frame", 2)),
            same_candidate_cooldown_s=float(candidate.get("same_candidate_cooldown_s", 20)),
            relation_max_center_distance_ratio=float(candidate.get("relation_max_center_distance_ratio", 0.65)),
            relation_crop_margin_ratio=float(candidate.get("relation_crop_margin_ratio", 0.08)),
            verify_enabled=bool(verify.get("enabled", True)),
            socket_path=str(verify.get("socket_path") or "runtime/go2w/siliconflow_vlm.sock"),
            max_inflight=int(verify.get("max_inflight", 1)),
            max_pending=int(verify.get("max_pending", 1)),
            client_timeout_s=float(verify.get("client_timeout_s", 12)),
            max_calls_per_minute=int(verify.get("max_calls_per_minute", 3)),
            confirmation_min_confidence=float(verify.get("confirmation_min_confidence", 0.70)),
            cache_ttl_s=float(verify.get("cache_ttl_s", 120)),
            stop_after_first_object_confirmation=bool(verify.get("stop_after_first_object_confirmation", True)),
            continue_after_hazard_confirmation=bool(verify.get("continue_after_hazard_confirmation", True)),
            evidence_max_files=int(evidence.get("max_files", 30)),
            evidence_jpeg_quality=int(evidence.get("jpeg_quality", 92)),
        )
        result.validate()
        return result

    def validate(self) -> None:
        if not 1 <= self.max_target_terms <= 20:
            raise ValueError("target_watch.max_target_terms must be 1..20")
        if not 0 <= self.max_anchor_terms <= 20:
            raise ValueError("target_watch.max_anchor_terms must be 0..20")
        if not 0 <= self.reserve_scene_terms <= 20:
            raise ValueError("target_watch.reserve_scene_terms must be 0..20")
        if not (0 <= self.min_score_yoloe_seg <= 1 and 0 <= self.min_score_grounded_sam <= 1):
            raise ValueError("target_watch candidate scores must be in [0,1]")
        if not 0 < self.max_bbox_area_ratio <= 1:
            raise ValueError("target_watch.max_bbox_area_ratio must be in (0,1]")
        if not 1 <= self.max_per_frame <= 8:
            raise ValueError("target_watch.max_per_frame must be 1..8")
        if self.max_inflight != 1 or not 0 <= self.max_pending <= 1:
            raise ValueError("target_watch verify is bounded to max_inflight=1, max_pending=0..1")
        if not 1 <= self.max_calls_per_minute <= 60:
            raise ValueError("target_watch.max_calls_per_minute must be 1..60")
        if not 0 <= self.confirmation_min_confidence <= 1:
            raise ValueError("target_watch.confirmation_min_confidence must be in [0,1]")
        if not 1 <= self.evidence_max_files <= 200:
            raise ValueError("target_watch.evidence.max_files must be 1..200")
        if not 40 <= self.evidence_jpeg_quality <= 100:
            raise ValueError("target_watch.evidence.jpeg_quality must be 40..100")

    def candidate_score(self, backend: str) -> float:
        return (
            self.min_score_yoloe_seg
            if backend == "yoloe_seg" else self.min_score_grounded_sam
        )


class CandidateCooldown:
    def __init__(self, seconds: float) -> None:
        self.seconds = max(0.0, float(seconds))
        self._seen: dict[str, float] = {}

    def permits(self, key: str, *, now: float | None = None) -> bool:
        when = time.monotonic() if now is None else float(now)
        previous = self._seen.get(str(key))
        if previous is not None and when - previous < self.seconds:
            return False
        self._seen[str(key)] = when
        self._seen = {k: v for k, v in self._seen.items() if when - v < max(60.0, self.seconds * 4)}
        return True

    def clear(self) -> None:
        self._seen.clear()


class MinuteBudget:
    def __init__(self, limit: int) -> None:
        self.limit = max(1, int(limit))
        self._calls: list[float] = []

    def permits(self, *, now: float | None = None) -> bool:
        when = time.monotonic() if now is None else float(now)
        self._calls = [value for value in self._calls if when - value < 60.0]
        if len(self._calls) >= self.limit:
            return False
        self._calls.append(when)
        return True

    def remaining(self, *, now: float | None = None) -> int:
        when = time.monotonic() if now is None else float(now)
        self._calls = [value for value in self._calls if when - value < 60.0]
        return max(0, self.limit - len(self._calls))


def spec_from_profile(query: str, profile: Any) -> TargetWatchSpec:
    """Post-process TargetProfile into target/anchor roles with safe fallbacks."""
    query = str(query).strip()
    deterministic = deterministic_spec(query)
    primary = _terms(getattr(profile, "primary_labels_en", ()))
    aliases = _terms(getattr(profile, "aliases_en", ()))
    anchors = _terms(getattr(profile, "relation_anchor_labels_en", ()))
    relations = _terms(getattr(profile, "relation_constraints", ()))
    attrs = _terms(getattr(profile, "attributes", ()))
    colors = _terms(getattr(profile, "colors", ()))
    target_terms = _dedupe((*deterministic.target_terms_en, *primary, *aliases))
    anchors = _dedupe((*deterministic.anchor_terms_en, *anchors))
    relations = _dedupe((*deterministic.relation_constraints, *relations))
    attrs = _dedupe((*deterministic.attributes, *attrs))
    colors = _dedupe((*deterministic.colors, *colors))
    mode = deterministic.verification_mode
    if mode == "object":
        if relations or anchors:
            mode = "relation"
        elif attrs or colors:
            mode = "attribute"
    return TargetWatchSpec(
        raw_query=query,
        canonical_name_zh=str(getattr(profile, "canonical_name_zh", "") or deterministic.canonical_name_zh),
        target_type=str(getattr(profile, "target_type", "") or deterministic.target_type),
        target_terms_en=target_terms,
        target_aliases_zh=_dedupe((
            *deterministic.target_aliases_zh,
            *_terms(getattr(profile, "aliases_zh", ())),
        )),
        attributes=attrs,
        colors=colors,
        relation_constraints=relations,
        anchor_terms_en=anchors,
        negative_terms=_dedupe((
            *deterministic.negative_terms,
            *_terms(getattr(profile, "negative_terms", ())),
        )),
        possible_confusions=_dedupe((
            *deterministic.possible_confusions,
            *_terms(getattr(profile, "possible_confusions", ())),
        )),
        verification_mode=mode,
        resolver_source=str(getattr(profile, "resolver_source", "fallback")),
        resolver_error=getattr(profile, "resolver_error", None),
    )


def deterministic_spec(query: str) -> TargetWatchSpec:
    text = str(query).strip()
    if not text or len(text) > 500:
        raise ValueError("target query must contain 1..500 characters")
    if "饮水机" in text and any(word in text for word in ("凳", "椅")):
        return TargetWatchSpec(
            text, text, "attribute_relation", ("stool", "chair"),
            ("凳子", "椅子"), relation_constraints=("next to water dispenser",),
            anchor_terms_en=("water dispenser",), verification_mode="relation",
        )
    if "挂" in text and "书包" in text and "椅" in text:
        return TargetWatchSpec(
            text, text, "attribute_relation", ("chair", "office chair"),
            ("椅子",), relation_constraints=("backpack hanging on chair",),
            anchor_terms_en=("backpack",), verification_mode="relation",
        )
    if "椅" in text:
        colors = ("black",) if "黑" in text else ()
        attrs = ("black appearance",) if colors else ()
        terms = ("chair", "office chair", "black chair") if colors else ("chair", "office chair")
        return TargetWatchSpec(
            text, text, "attribute_relation" if colors else "object", terms,
            ("椅子",), attributes=attrs, colors=colors,
            possible_confusions=("stool", "sofa"),
            verification_mode="attribute" if colors else "object",
        )
    if any(word in text for word in ("烟雾", "烟", "雾")):
        return TargetWatchSpec(
            text, "烟雾", "hazard", ("smoke", "visible smoke", "steam"),
            ("烟雾",), attributes=("visible airborne plume",),
            possible_confusions=("fog", "grey wall", "cloud"),
            verification_mode="diffuse_hazard",
        )
    if any(word in text for word in ("泄漏", "漏液", "液体")):
        return TargetWatchSpec(
            text, "泄漏的液体", "hazard",
            ("liquid leak", "fluid spill", "puddle"), ("泄漏液体", "漏液"),
            attributes=("wet spilled liquid",), anchor_terms_en=("pipe", "valve"),
            possible_confusions=("floor reflection", "shadow"),
            verification_mode="diffuse_hazard",
        )
    if "史莱姆" in text or "slime" in text.casefold():
        return TargetWatchSpec(
            text, "史莱姆泥", "hazard", ("slime", "green slime"), ("史莱姆",),
            attributes=("viscous fluorescent material",), colors=("green",),
            anchor_terms_en=("pipe",), possible_confusions=("green object", "yellow wall"),
            verification_mode="diffuse_hazard",
        )
    english = (text.casefold(),) if text.isascii() else ("object",)
    return TargetWatchSpec(text, text, "object", english, (text,), verification_mode="object")


def automatic_slime_hazard_spec() -> TargetWatchSpec:
    """Built-in VLM review profile for the controlled slime leak simulation.

    This profile is deliberately independent from the operator-entered target.
    The detector/hsv enhancer remains the high-recall gate; VLM is only called
    after one of these concrete candidate labels has been observed.
    """
    return TargetWatchSpec(
        raw_query="自动危险源复核：管道上的史莱姆泥或模拟液体泄漏",
        canonical_name_zh="疑似管道史莱姆泥泄漏污染源",
        target_type="hazard",
        target_terms_en=(
            "simulated liquid leak",
            "simulated liquid leak candidate",
            "green slime",
            "slime",
            "liquid leak",
            "fluid spill",
        ),
        target_aliases_zh=("史莱姆泥", "模拟泄漏", "管道漏液", "污染源"),
        attributes=(
            "viscous or wet material attached to or escaping from a pipe",
            "fluorescent slime-like contamination",
        ),
        anchor_terms_en=("pipe", "pipeline", "industrial pipe", "valve"),
        negative_terms=("toy", "decoration"),
        possible_confusions=(
            "green object",
            "plant",
            "floor reflection",
            "yellow wall",
        ),
        verification_mode="diffuse_hazard",
        resolver_source="built_in_auto_hazard",
    )


def active_verification_specs(
    operator_spec: TargetWatchSpec | None,
    *,
    auto_hazard_enabled: bool,
) -> tuple[tuple[TargetWatchSpec, bool], ...]:
    """Return candidate profiles in priority order.

    Automatic hazards go first so an unrelated operator target cannot consume
    a small per-frame candidate budget before a leak candidate is reviewed.
    The boolean marks profiles that are independent of the target-watch input.
    """
    profiles: list[tuple[TargetWatchSpec, bool]] = []
    if auto_hazard_enabled:
        profiles.append((automatic_slime_hazard_spec(), True))
    if operator_spec is not None:
        profiles.append((operator_spec, False))
    return tuple(profiles)


def target_profile_from_spec(spec: TargetWatchSpec) -> Any:
    """Build the existing TargetProfile type without making it this module's ABI."""
    from app.reasoning.target_profile import TargetProfile

    return TargetProfile(
        raw_query=spec.raw_query,
        canonical_name_zh=spec.canonical_name_zh,
        target_type=spec.target_type,
        primary_labels_en=list(spec.target_terms_en),
        aliases_zh=list(spec.target_aliases_zh),
        attributes=list(spec.attributes),
        colors=list(spec.colors),
        relation_constraints=list(spec.relation_constraints),
        relation_anchor_labels_en=list(spec.anchor_terms_en),
        negative_terms=list(spec.negative_terms),
        possible_confusions=list(spec.possible_confusions),
        resolver_source=spec.resolver_source,
        resolver_error=spec.resolver_error,
        requires_state_verification=spec.verification_mode != "object",
        requires_scene_confirmation=spec.verification_mode in {"relation", "diffuse_hazard"},
    )


def schedule_terms(
    spec: TargetWatchSpec | None,
    scene_terms: Sequence[str],
    hazard_terms: Sequence[str],
    *,
    max_terms: int,
    max_target_terms: int,
    max_anchor_terms: int,
    reserve_scene_terms: int,
) -> tuple[list[str], dict[str, tuple[str, ...]], list[str]]:
    """Apply target > anchor > related hazard > global hazard > scene priority."""
    budget = max(1, int(max_terms))
    target = list(spec.target_terms_en[:max_target_terms]) if spec else []
    anchors = list(spec.anchor_terms_en[:max_anchor_terms]) if spec else []
    related_hazard = list(spec.target_terms_en) if spec and spec.verification_mode == "diffuse_hazard" else []
    target = list(_dedupe(target))
    anchors = [x for x in _dedupe(anchors) if x.casefold() not in {v.casefold() for v in target}]
    hazards = [x for x in _dedupe((*related_hazard, *hazard_terms))
               if x.casefold() not in {v.casefold() for v in (*target, *anchors)}]
    scene = [x for x in _dedupe(scene_terms)
             if x.casefold() not in {v.casefold() for v in (*target, *anchors, *hazards)}]

    selected_scene = scene[: min(len(scene), max(0, reserve_scene_terms))]
    result = list(_dedupe((*target, *anchors, *hazards)))[: max(0, budget - len(selected_scene))]
    result.extend(selected_scene[: max(0, budget - len(result))])
    if len(result) < budget:
        result.extend(scene[len(selected_scene):][: budget - len(result)])
    result = result[:budget]
    chosen = {x.casefold() for x in result}
    slots = {
        "target": tuple(x for x in target if x.casefold() in chosen),
        "anchor": tuple(x for x in anchors if x.casefold() in chosen),
        "scene": tuple(x for x in (*hazards, *scene) if x.casefold() in chosen),
    }
    truncated = [x for x in _dedupe((*target, *anchors, *hazards, *scene)) if x.casefold() not in chosen]
    return result, slots, truncated


def candidate_id(frame_id: str, index: int, prompt_term: str) -> str:
    digest = hashlib.sha256(
        f"{frame_id}\0{index}\0{prompt_term.casefold()}".encode("utf-8")
    ).hexdigest()[:12]
    return f"twc_{digest}"


def candidate_key(candidate: Mapping[str, Any], generation: int) -> str:
    if candidate.get("persistent_object_id"):
        return f"{generation}:obj:{candidate['persistent_object_id']}"
    xyz = candidate.get("map_xyz")
    if isinstance(xyz, (list, tuple)) and len(xyz) >= 2:
        return f"{generation}:map:{round(float(xyz[0]), 1)}:{round(float(xyz[1]), 1)}"
    bbox = candidate.get("bbox_2d") or candidate.get("bbox") or ()
    rounded = ":".join(str(round(float(v), 1)) for v in bbox) if len(bbox) == 4 else "unknown"
    return f"{generation}:box:{candidate.get('matched_term') or candidate.get('label')}:{rounded}"


def bbox_area_ratio(value: Any) -> float:
    try:
        x1, y1, x2, y2 = (float(v) for v in value)
    except (TypeError, ValueError):
        return math.inf
    if not all(math.isfinite(v) for v in (x1, y1, x2, y2)) or x2 <= x1 or y2 <= y1:
        return math.inf
    return (x2 - x1) * (y2 - y1)


def matches_terms(label: Any, terms: Iterable[str]) -> bool:
    value = " ".join(str(label or "").casefold().split())
    return bool(value) and any(
        (term := " ".join(str(raw).casefold().split()))
        and (term == value or term in value or value in term)
        for raw in terms
    )


def atomic_write_json(path: str | Path, payload: Mapping[str, Any]) -> None:
    destination = Path(path).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(dict(payload), ensure_ascii=False, indent=2, allow_nan=False)
    fd, temporary = tempfile.mkstemp(prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, destination)
    finally:
        try:
            Path(temporary).unlink()
        except OSError:
            pass


def load_json_object(path: str | Path) -> dict[str, Any] | None:
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def empty_state(mapping_session_id: Any, *, detector_backend: str | None = None) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "mapping_session_id": mapping_session_id,
        "generation": 0,
        "revision": 0,
        "enabled": False,
        "state": "IDLE",
        "raw_query": "",
        "profile": None,
        "detector_backend": detector_backend,
        "candidate_count": 0,
        "last_candidate": None,
        "verification": None,
        "confirmed_matches": [],
        "statistics": {
            "frames_evaluated": 0,
            "candidates_generated": 0,
            "verify_started": 0,
            "verify_confirmed": 0,
            "verify_rejected": 0,
            "verify_unknown": 0,
            "verify_rate_limited": 0,
            "stale_results_discarded": 0,
        },
        "last_error": None,
        "updated_at": time.time(),
    }


def validate_request(value: Mapping[str, Any], *, session_id: Any | None = None) -> dict[str, Any]:
    if int(value.get("schema_version", -1)) != SCHEMA_VERSION:
        raise ValueError("unsupported target-watch schema_version")
    if session_id is not None and str(value.get("mapping_session_id")) != str(session_id):
        raise ValueError("mapping_session_id mismatch")
    generation = int(value.get("generation", -1))
    if generation < 0:
        raise ValueError("generation must be non-negative")
    command = str(value.get("command") or "")
    if command not in {"set", "clear"}:
        raise ValueError("command must be set or clear")
    enabled = bool(value.get("enabled", False))
    profile = value.get("profile")
    if command == "set":
        if not enabled or not isinstance(profile, dict):
            raise ValueError("set request requires enabled profile")
        spec = TargetWatchSpec.from_dict({**profile, "raw_query": value.get("raw_query")})
        profile = spec.to_dict()
    else:
        enabled = False
        profile = None
    return {
        "schema_version": SCHEMA_VERSION,
        "mapping_session_id": value.get("mapping_session_id"),
        "generation": generation,
        "enabled": enabled,
        "command": command,
        "raw_query": str(value.get("raw_query") or "")[:500],
        "profile": profile,
        "submitted_at": float(value.get("submitted_at") or time.time()),
    }


def _terms(value: Any) -> tuple[str, ...]:
    if not isinstance(value, (list, tuple)):
        return ()
    return _dedupe(str(item).strip() for item in value if str(item).strip())


def _dedupe(values: Iterable[str]) -> tuple[str, ...]:
    result: list[str] = []
    seen: set[str] = set()
    for value in values:
        text = str(value).strip()
        key = text.casefold()
        if text and key not in seen:
            result.append(text)
            seen.add(key)
    return tuple(result)
