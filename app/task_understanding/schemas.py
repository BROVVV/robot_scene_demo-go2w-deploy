"""Schemas for LLM-first natural-language task understanding."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class TaskIntent(str, Enum):
    LOCATE_OBJECT = "locate_object"
    LOCATE_PERSON = "locate_person"
    FIND_ROOM = "find_room"
    LOCATE_AREA = "locate_area"
    INSPECT_AREA = "inspect_area"
    CHECK_DOOR_STATE = "check_door_state"
    PATROL_AREA = "patrol_area"
    CHECK_PASSABLE_AREA = "check_passable_area"
    SEARCH_SEMANTIC_TARGET = "search_semantic_target"
    FOLLOW_TARGET = "follow_target"
    APPROACH_TARGET = "approach_target"
    REPORT_STATUS = "report_status"
    MIXED = "mixed"
    NON_NAVIGATION = "non_navigation"
    UNSUPPORTED = "unsupported"
    UNKNOWN = "unknown"


class TargetCategory(str, Enum):
    OBJECT = "object"
    PERSON = "person"
    ROOM = "room"
    DOOR = "door"
    CONTAINER = "container"
    AREA = "area"
    FLOOR = "floor"
    CORRIDOR = "corridor"
    SCENE_REGION = "scene_region"
    SEMANTIC_TARGET = "semantic_target"
    UNKNOWN = "unknown"


@dataclass
class GroundingPromptPlan:
    raw_task: str = ""
    primary_intent: str = "unknown"

    target_name_zh: str = ""
    target_name_en: str = ""
    target_category: str = "unknown"

    grounding_strategy: str = "unknown"

    direct_terms_en: list[str] = field(default_factory=list)
    proxy_object_terms_en: list[str] = field(default_factory=list)
    context_anchor_terms_en: list[str] = field(default_factory=list)
    state_terms_en: list[str] = field(default_factory=list)
    negative_terms_en: list[str] = field(default_factory=list)

    grounding_prompt: str = ""
    prompt_source: str = "unknown"
    prompt_reason_zh: str = ""

    is_valid_for_grounding_dino: bool = False
    requires_proxy_objects: bool = False
    requires_scene_confirmation: bool = False
    requires_state_verification: bool = False

    retry_count: int = 0
    warnings: list[str] = field(default_factory=list)
    raw_llm_response: dict[str, Any] = field(default_factory=dict)


class SubtaskType(str, Enum):
    LOCATE_OBJECT = "locate_object"
    LOCATE_PERSON = "locate_person"
    FIND_ROOM = "find_room"
    LOCATE_AREA = "locate_area"
    INSPECT_DOOR_STATE = "inspect_door_state"
    PATROL_AREA = "patrol_area"
    INSPECT_AREA = "inspect_area"
    OBSERVE = "observe"
    OBSERVE_AREA = "observe_area"
    CHECK_PASSABLE_AREA = "check_passable_area"
    SEARCH_TARGET = "search_target"
    APPROACH_TARGET = "approach_target"
    NAVIGATE_TO_VIEWPOINT = "navigate_to_viewpoint"
    STOP_AND_REPORT = "stop_and_report"
    OPEN_CONTAINER = "open_container"
    OPEN_DOOR = "open_door"
    PICK_UP_OBJECT = "pick_up_object"
    MOVE_OBJECT = "move_object"
    DELIVER_OBJECT = "deliver_object"
    MANIPULATE_OBJECT = "manipulate_object"
    PHYSICAL_HARM = "physical_harm"
    PHYSICAL_ASSAULT = "physical_assault"
    DAMAGE_OBJECT = "damage_object"
    DAMAGE_PROPERTY = "damage_property"
    CHASE_OR_RAM = "chase_or_ram"
    PRIVACY_INVASIVE_ACTION = "privacy_invasive_action"
    NON_NAVIGATION = "non_navigation"
    OTHER_UNSUPPORTED = "other_unsupported"
    UNKNOWN = "unknown"


class SafetyFlag(str, Enum):
    PHYSICAL_HARM_REQUEST = "physical_harm_request"
    MANIPULATION_REQUIRED = "manipulation_required"
    NON_NAVIGATION_REQUEST = "non_navigation_request"
    PRIVACY_SENSITIVE_REQUEST = "privacy_sensitive_request"
    LOW_CONFIDENCE_TASK_PARSE = "low_confidence_task_parse"
    LLM_TASK_UNDERSTANDING_UNAVAILABLE = "llm_task_understanding_unavailable"
    LLM_TASK_VERIFICATION_FAILED = "llm_task_verification_failed"
    ATTACK_OR_ASSAULT = "attack_or_assault"
    CHASE_OR_RAM_PERSON = "chase_or_ram_person"
    PROPERTY_DAMAGE = "property_damage"
    UNSUPPORTED_MANIPULATION = "unsupported_manipulation"
    PRIVACY_RISK = "privacy_risk"
    UNKNOWN_RISK = "unknown_risk"


@dataclass
class TaskTarget:
    name_zh: str = ""
    name_en: str = ""
    category: str | TargetCategory = TargetCategory.UNKNOWN
    attributes: list[str] = field(default_factory=list)
    relations: list[str] = field(default_factory=list)

    @property
    def raw_text(self) -> str:
        return self.name_zh or self.name_en

    @property
    def area_hint(self) -> str | None:
        return None

    @property
    def person_identifier(self) -> str | None:
        category = self.category.value if isinstance(self.category, TargetCategory) else str(self.category)
        return self.name_zh if category == TargetCategory.PERSON.value else None


@dataclass
class TaskArea:
    name_zh: str = ""
    category: str | TargetCategory = TargetCategory.UNKNOWN


@dataclass
class RequestedSubtask:
    id: str
    type: str | SubtaskType
    object: str = ""
    recipient_or_target: str = ""
    state_query: str = ""
    is_navigation_relevant: bool = False
    requires_visual_grounding: bool = True
    requires_manipulation: bool = False
    requires_physical_contact: bool = False
    requires_harmful_action: bool = False
    semantic_role: str = "main_goal"
    llm_reason: str = ""
    requested_by_user: bool = True
    allowed_by_capability: bool | None = None
    allowed_by_safety: bool | None = None
    blocked_reason: str | None = None

    @property
    def navigation_relevant(self) -> bool:
        return self.is_navigation_relevant

    @property
    def description(self) -> str:
        return self.llm_reason or self.object or self.recipient_or_target or str(self.type)


@dataclass
class SafetyAssessment:
    contains_physical_harm_request: bool = False
    contains_manipulation_request: bool = False
    contains_privacy_sensitive_request: bool = False
    contains_non_navigation_request: bool = False
    risk_level: str = "none"
    reason: str = ""


@dataclass
class ExecutionRecommendation:
    can_execute_fully: bool = False
    can_execute_navigation_part: bool = False
    allowed_navigation_subtasks: list[str] = field(default_factory=list)
    unsupported_subtasks: list[str] = field(default_factory=list)
    user_feedback_zh: str = ""


@dataclass
class TaskParseConfidence:
    intent: float = 0.0
    safety: float = 0.0
    target: float = 0.0


@dataclass
class ParsedTask:
    raw_task: str
    language: str = "zh"
    task_summary: str = ""
    primary_intent: str | TaskIntent = TaskIntent.UNKNOWN
    target: TaskTarget = field(default_factory=TaskTarget)
    area: TaskArea = field(default_factory=TaskArea)
    requested_subtasks: list[RequestedSubtask] = field(default_factory=list)
    safety_assessment: SafetyAssessment = field(default_factory=SafetyAssessment)
    execution_recommendation: ExecutionRecommendation = field(
        default_factory=ExecutionRecommendation
    )
    confidence: TaskParseConfidence = field(default_factory=TaskParseConfidence)
    initial_visibility_state: str = "unknown"
    raw_llm_response: dict[str, Any] = field(default_factory=dict)
    parser_source: str = "llm"
    notes: list[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.initial_visibility_state = "unknown"

    @property
    def subtasks(self) -> list[RequestedSubtask]:
        return self.requested_subtasks

    @property
    def navigation_relevant(self) -> bool:
        return any(item.is_navigation_relevant for item in self.requested_subtasks)

    @property
    def requires_visual_grounding(self) -> bool:
        return any(
            item.requires_visual_grounding and item.is_navigation_relevant
            for item in self.requested_subtasks
        )


@dataclass
class ActionabilityResult:
    fully_executable: bool
    navigation_part_executable: bool
    allowed_subtasks: list[RequestedSubtask] = field(default_factory=list)
    blocked_subtasks: list[RequestedSubtask] = field(default_factory=list)
    safety_flags: list[SafetyFlag] = field(default_factory=list)
    blocked_reasons: list[str] = field(default_factory=list)
    user_feedback_zh: str = ""
    execution_constraints: dict[str, Any] = field(default_factory=dict)

    @property
    def user_feedback(self) -> str:
        return self.user_feedback_zh


@dataclass
class NavigationTask:
    executable: bool
    navigation_goal: str = "unknown"
    target: TaskTarget = field(default_factory=TaskTarget)
    area: TaskArea = field(default_factory=TaskArea)
    subtasks: list[RequestedSubtask] = field(default_factory=list)
    constraints: dict[str, Any] = field(default_factory=dict)
    user_feedback_zh: str = ""
    source_raw_task: str = ""
    intent: str | TaskIntent = TaskIntent.UNKNOWN
    blocked_subtasks: list[RequestedSubtask] = field(default_factory=list)
    requires_visual_grounding: bool = True
    initial_visibility_state: str = "unknown"

    def __post_init__(self) -> None:
        self.initial_visibility_state = "unknown"

    @property
    def allowed_subtasks(self) -> list[RequestedSubtask]:
        return self.subtasks

    @property
    def execution_constraints(self) -> dict[str, Any]:
        return self.constraints

    @property
    def user_feedback(self) -> str:
        return self.user_feedback_zh


ParsedSubtask = RequestedSubtask
TargetSpec = TaskTarget
CapabilityGateResult = ActionabilityResult
