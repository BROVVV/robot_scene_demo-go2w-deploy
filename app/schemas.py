"""Pydantic models for scene analysis results."""

from __future__ import annotations

from enum import Enum
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field


Confidence = float


class StrictBaseModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class EvidenceSource(str, Enum):
    LLM_VISION = "llm_vision"
    GROUNDED_SAM = "grounded_sam"
    SAM2_MASK = "sam2_mask"
    CROP_VERIFY = "crop_verify"
    GEOMETRIC_RELATION = "geometric_relation"
    LLM_SITUATED_REASONING = "llm_situated_reasoning"
    EPISODIC_MEMORY = "episodic_memory"
    HUMAN_FEEDBACK = "human_feedback"
    NEGATIVE_EVIDENCE = "negative_evidence"


class NodeObservationStatus(str, Enum):
    OBSERVED = "observed"
    INFERRED = "inferred"
    UNREACHABLE = "unreachable"
    REJECTED = "rejected"


class Actionability(str, Enum):
    ROBOT_EXECUTABLE = "robot_executable"
    ROBOT_VIEWPOINT_ONLY = "robot_viewpoint_only"
    NEEDS_REOBSERVATION = "needs_reobservation"
    NEEDS_HUMAN = "needs_human"
    UNSAFE_OR_IMPOSSIBLE = "unsafe_or_impossible"


class QuadrupedActionPrimitive(str, Enum):
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    MOVE_FORWARD_SHORT = "move_forward_short"
    MOVE_BACKWARD_SHORT = "move_backward_short"
    STOP_AND_REOBSERVE = "stop_and_reobserve"
    CENTER_VIEW_ON_REGION = "center_view_on_region"
    SCAN_LEFT_TO_RIGHT = "scan_left_to_right"
    RETURN_TO_LAST_SAFE_POSE = "return_to_last_safe_pose"
    ASK_HUMAN_FOR_OCCLUDED_AREA = "ask_human_for_occluded_area"
    MARK_UNREACHABLE = "mark_unreachable"


class RobotCapabilityContract(StrictBaseModel):
    platform: str = "quadruped"
    can_move_base: bool = True
    can_rotate_in_place: bool = True
    can_reobserve: bool = True
    can_change_camera_pitch: bool = False
    can_manipulate: bool = False
    can_open_container: bool = False
    can_pick_objects: bool = False
    can_inspect_inside_closed_container: bool = False
    can_crouch_or_look_down: bool = False
    allowed_primitives: list[QuadrupedActionPrimitive] = Field(default_factory=list)
    forbidden_action_phrases_zh: list[str] = Field(default_factory=list)
    max_forward_step_m: float = Field(default=0.5, gt=0.0)
    max_executable_distance_m: float = Field(default=0.5, gt=0.0)
    platform_obstacle_avoidance_assumed: bool = True
    execution_assumption: str = "platform_obstacle_avoidance_assumed"
    require_stop_after_each_motion: bool = True
    require_reobserve_after_each_motion: bool = True


def default_quadruped_capability(
    *,
    max_forward_step_m: float = 0.5,
    max_executable_distance_m: float | None = None,
    platform_obstacle_avoidance_assumed: bool = True,
    can_manipulate: bool = False,
    can_open_container: bool = False,
    can_look_down: bool = False,
) -> RobotCapabilityContract:
    return RobotCapabilityContract(
        can_manipulate=can_manipulate,
        can_open_container=can_open_container,
        can_pick_objects=can_manipulate,
        can_inspect_inside_closed_container=can_open_container,
        can_crouch_or_look_down=can_look_down,
        max_forward_step_m=max_forward_step_m,
        max_executable_distance_m=(
            max_executable_distance_m
            if max_executable_distance_m is not None
            else max_forward_step_m
        ),
        platform_obstacle_avoidance_assumed=platform_obstacle_avoidance_assumed,
        execution_assumption=(
            "platform_obstacle_avoidance_assumed"
            if platform_obstacle_avoidance_assumed
            else "strict_safe_no_platform_obstacle_avoidance"
        ),
        allowed_primitives=list(QuadrupedActionPrimitive),
        forbidden_action_phrases_zh=[
            "打开",
            "翻找",
            "拿起",
            "移动物体",
            "低头观察表面",
            "低头观察桌面",
            "钻到",
            "伸手",
            "机械臂",
            "拉开抽屉",
            "打开柜门",
            "翻包",
            "检查周边30-80厘米",
            "检查周边 30-80 厘米",
        ],
    )


DEFAULT_QUADRUPED_CAPABILITY = default_quadruped_capability()


class Position(StrictBaseModel):
    horizontal: Literal["left", "center", "right"]
    vertical: Literal["front", "middle", "back"]
    relative_to_robot: str
    estimated_distance_m: float | None = None


class BoundingBox2D(StrictBaseModel):
    x1: float = Field(ge=0.0, le=1.0)
    y1: float = Field(ge=0.0, le=1.0)
    x2: float = Field(ge=0.0, le=1.0)
    y2: float = Field(ge=0.0, le=1.0)


class SceneObject(StrictBaseModel):
    id: str
    name: str
    name_zh: str
    category: str
    color: str | None = None
    attributes: list[str] = Field(default_factory=list)
    visible: bool
    position: Position
    bbox_2d: BoundingBox2D
    confidence: Confidence = Field(ge=0.0, le=1.0)
    detector_score: Confidence | None = Field(default=None, ge=0.0, le=1.0)
    text_score: Confidence | None = Field(default=None, ge=0.0, le=1.0)
    mask_area_ratio: Confidence | None = Field(default=None, ge=0.0, le=1.0)
    crop_path: str | None = None
    crop_verify: dict[str, Any] | None = None
    final_score: Confidence | None = Field(default=None, ge=0.0, le=1.0)
    decision: Literal["confirmed", "candidate", "rejected"] | None = None
    rejection_reason: str | None = None
    explanation: str | None = None


class SceneRelation(StrictBaseModel):
    source_id: str
    target_id: str
    relation_type: Literal[
        "left_of",
        "right_of",
        "in_front_of",
        "behind",
        "on",
        "under",
        "above",
        "below",
        "in",
        "near",
        "far",
        "contains",
        "occluding",
    ]
    description_zh: str
    estimated_distance_m: float | None = None
    confidence: Confidence = Field(ge=0.0, le=1.0)


class TopologyNode(StrictBaseModel):
    id: str
    label: str
    object_id: str | None = None


class TopologyEdge(StrictBaseModel):
    source_id: str
    target_id: str
    relation_type: str
    label: str | None = None
    relation_id: str | None = None


class TopologyGraph(StrictBaseModel):
    nodes: list[TopologyNode] = Field(default_factory=list)
    edges: list[TopologyEdge] = Field(default_factory=list)


class TargetDecision(StrictBaseModel):
    target_text: str
    is_present: bool
    matched_object_ids: list[str] = Field(default_factory=list)
    match_reason_zh: str
    confidence: Confidence = Field(ge=0.0, le=1.0)


class RouteStep(StrictBaseModel):
    step_id: int = Field(ge=1)
    action: Literal[
        "move_forward",
        "move_backward",
        "turn_left",
        "turn_right",
        "stop",
    ]
    distance_m: float | None = None
    turn_angle_deg: float | None = None
    description_zh: str


class RoutePlan(StrictBaseModel):
    route_type: Literal["approach_visible_target", "explore_likely_location"]
    summary_zh: str
    steps: list[RouteStep] = Field(default_factory=list)
    safety_notes_zh: list[str] = Field(default_factory=list)


class Ros2Vector3(StrictBaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class Ros2Twist(StrictBaseModel):
    linear: Ros2Vector3 = Field(default_factory=Ros2Vector3)
    angular: Ros2Vector3 = Field(default_factory=Ros2Vector3)


class Ros2MotionCommand(StrictBaseModel):
    command_id: str
    route_step_id: int | None = None
    source_action: Literal[
        "move_forward",
        "move_backward",
        "turn_left",
        "turn_right",
        "stop",
    ]
    topic: str = "/cmd_vel"
    message_type: str = "geometry_msgs/msg/Twist"
    twist: Ros2Twist
    duration_sec: float = Field(gt=0.0)
    distance_m: float | None = None
    turn_angle_deg: float | None = None
    description_zh: str
    interruptible_by_platform: bool = False
    platform_obstacle_avoidance_assumed: bool = False
    requires_stop_after_motion: bool = True
    observe_while_moving: bool = False


def default_navigation_execution_constraints() -> dict[str, Any]:
    return {
        "contact_allowed": False,
        "manipulation_allowed": False,
        "harmful_action_allowed": False,
        "chase_or_ram_allowed": False,
        "requires_stop_after_navigation": True,
        "stop_distance_policy": "safe_distance",
        "max_speed_policy": "normal_or_conservative",
    }


class Ros2MotionPlan(StrictBaseModel):
    plan_id: str
    generated_at: str
    dry_run: bool = True
    topic: str = "/cmd_vel"
    message_type: str = "geometry_msgs/msg/Twist"
    frame_id: str = "base_link"
    route_type: str
    route_summary_zh: str
    command_rate_hz: float = Field(gt=0.0)
    commands: list[Ros2MotionCommand] = Field(default_factory=list)
    safety_notes_zh: list[str] = Field(default_factory=list)
    integration_notes_zh: list[str] = Field(default_factory=list)
    execution_constraints: dict[str, Any] = Field(
        default_factory=default_navigation_execution_constraints
    )
    platform_obstacle_avoidance_assumed: bool = False
    dynamic_motion_horizon_enabled: bool = False
    motion_horizon_profile: str = "strict_safe"
    motion_horizon_decision: dict[str, Any] | None = None


class MotionHorizonDecision(StrictBaseModel):
    enabled: bool = True
    profile: str = "platform_assisted_auto"
    platform_obstacle_avoidance_assumed: bool = True
    scene_type: str = "unknown"
    task_phase: str = "search"
    motion_policy: str = "platform_assisted_fallback"
    recommended_distance_m: float = Field(default=1.0, ge=0.0)
    max_allowed_distance_m: float = Field(default=1.0, ge=0.0)
    original_requested_distance_m: float | None = None
    clipped_distance_m: float | None = None
    llm_recommended_horizon_m: float | None = None
    requires_stop_after_motion: bool = True
    observe_while_moving: bool = False
    soft_observe_interval_sec: float | None = None
    shorten_reason: str | None = None
    decision_reason_zh: str = ""
    source: str = "rule"
    confidence: Confidence = Field(default=0.6, ge=0.0, le=1.0)


class SceneAnalysisResult(StrictBaseModel):
    scene_summary_zh: str
    objects: list[SceneObject] = Field(default_factory=list)
    relations: list[SceneRelation] = Field(default_factory=list)
    topology: TopologyGraph
    target_decision: TargetDecision
    route_plan: RoutePlan
    target_profile: dict[str, Any] | None = None
    detection_config: dict[str, Any] | None = None
    candidate_summary: dict[str, int] | None = None
    candidate_objects: list[dict[str, Any]] = Field(default_factory=list)


class ObservedAnchor(StrictBaseModel):
    object_id: str
    label_zh: str
    label_en: str | None = None
    bbox: list[float] = Field(min_length=4, max_length=4)
    image_region: str
    confidence: Confidence = Field(ge=0.0, le=1.0)
    stable: bool = False
    source: EvidenceSource


class ObservedSceneFacts(StrictBaseModel):
    scene_summary_zh: str
    target_text: str
    target_profile: dict[str, Any] | None = None
    room_type_guess: str | None = None
    visible_anchors: list[ObservedAnchor] = Field(default_factory=list)
    visible_relations: list[dict[str, Any]] = Field(default_factory=list)
    target_observed: bool
    target_evidence: list[str] = Field(default_factory=list)
    negative_evidence: list[str] = Field(default_factory=list)
    searched_regions: list[str] = Field(default_factory=list)


class LLMSearchHypothesis(StrictBaseModel):
    hypothesis_id: str
    target_name: str
    status: NodeObservationStatus = NodeObservationStatus.INFERRED
    candidate_region_zh: str
    candidate_region_type: str
    image_region_hint: str | None = None
    supporting_visible_anchor_ids: list[str] = Field(default_factory=list)
    supporting_visible_anchor_names: list[str] = Field(default_factory=list)
    human_like_rationale_zh: str
    expected_visual_cues_zh: list[str] = Field(default_factory=list)
    suggested_detector_prompts_en: list[str] = Field(default_factory=list)
    suggested_verification_question_zh: str
    confidence: Confidence = Field(ge=0.0, le=1.0)
    uncertainty_zh: str
    actionability: Actionability
    quadruped_view_strategy: list[QuadrupedActionPrimitive] = Field(default_factory=list)
    safety_notes_zh: list[str] = Field(default_factory=list)
    memory_sources: list[str] = Field(default_factory=list)
    should_not_mark_found: bool = True
    max_executable_distance_m: float | None = None
    execution_assumption: str | None = None


class LLMReasoningRequest(StrictBaseModel):
    target_text: str
    target_profile: dict[str, Any] | None = None
    observed_facts: ObservedSceneFacts
    retrieved_episodes: list[dict[str, Any]] = Field(default_factory=list)
    capability_contract: RobotCapabilityContract
    max_hypotheses: int = Field(default=5, ge=1, le=20)
    language: str = "zh"


class LLMReasoningResult(StrictBaseModel):
    scene_interpretation_zh: str
    target_search_logic_zh: str
    hypotheses: list[LLMSearchHypothesis] = Field(default_factory=list)
    global_uncertainty_zh: str
    recommended_next_observation_zh: str
    no_target_found_policy_zh: str
    recommended_motion_horizon_m: float | None = None
    motion_horizon_reason_zh: str | None = None
    motion_profile_hint: str | None = None
    requires_stop_after_motion: bool = True
    reasoning_available: bool = True
    error_message: str | None = None


class ViewpointStep(StrictBaseModel):
    step_id: str
    primitive: QuadrupedActionPrimitive
    target_image_region: str | None = None
    reason_zh: str
    expected_information_gain: Confidence = Field(ge=0.0, le=1.0)
    safety_level: str
    requires_reobserve_after: bool = True
    motion_horizon_m: float | None = None
    motion_policy: str | None = None
    requires_stop_observation: bool = True
    platform_obstacle_avoidance_assumed: bool = False


class QuadrupedSearchPlan(StrictBaseModel):
    plan_id: str
    target_text: str
    target_found: bool
    plan_type: str
    steps: list[ViewpointStep] = Field(default_factory=list)
    non_executable_notes_zh: list[str] = Field(default_factory=list)
    should_export_ros2_dry_run: bool = True
    motion_horizon_decision: MotionHorizonDecision | None = None


class ReasonedPSGNode(StrictBaseModel):
    node_id: str
    label_zh: str
    node_type: str
    observation_status: NodeObservationStatus
    source: EvidenceSource
    confidence: Confidence = Field(ge=0.0, le=1.0)
    bbox: list[float] | None = None
    image_region_hint: str | None = None
    evidence_summary_zh: str
    actionability: Actionability | None = None
    generated_by: str
    display_style: str = "solid"
    border_color: str = "#4b5563"
    fill_color: str = "#ffffff"
    motion_horizon_m: float | None = None
    motion_policy: str | None = None
    requires_stop_observation: bool | None = None
    platform_obstacle_avoidance_assumed: bool | None = None


class ReasonedPSGEdge(StrictBaseModel):
    source_id: str
    target_id: str
    relation_type: str
    observation_status: NodeObservationStatus
    source: EvidenceSource
    confidence: Confidence = Field(ge=0.0, le=1.0)
    evidence_summary_zh: str
    line_style: str = "solid"


class ReasonedPredictiveSceneGraph(StrictBaseModel):
    nodes: list[ReasonedPSGNode] = Field(default_factory=list)
    edges: list[ReasonedPSGEdge] = Field(default_factory=list)


class SpatialExperienceMemory(StrictBaseModel):
    memory_id: str
    created_at: str
    target_text: str
    target_normalized: str
    scene_type: str | None = None
    visible_anchor_labels: list[str] = Field(default_factory=list)
    hypothesis_region_zh: str
    hypothesis_rationale_zh: str
    action_taken: list[str] = Field(default_factory=list)
    outcome: Literal["found", "not_found", "not_verified", "human_needed"]
    visual_evidence_summary_zh: str
    negative_evidence_zh: list[str] = Field(default_factory=list)
    confidence_after_outcome: Confidence = Field(ge=0.0, le=1.0)
    image_fingerprint: str | None = None


class EnvironmentRoom(StrictBaseModel):
    room_id: str
    name: str | None = None
    room_type: str | None = None
    floor_id: str | None = None
    location_hint: str | None = None
    confidence: Confidence = Field(ge=0.0, le=1.0)


class EnvironmentDoor(StrictBaseModel):
    door_id: str
    label: str | None = None
    connected_room_id: str | None = None
    floor_id: str | None = None
    location_hint: str | None = None
    state: Literal["open", "closed", "unknown"] = "unknown"
    confidence: Confidence = Field(ge=0.0, le=1.0)


class RoomTypePrior(StrictBaseModel):
    room_type: str
    common_objects: list[str] = Field(default_factory=list)
    likely_layout: list[str] = Field(default_factory=list)
    confidence: Confidence = Field(ge=0.0, le=1.0)


class ObjectLocationPrior(StrictBaseModel):
    object_name: str
    likely_locations: list[str] = Field(default_factory=list)
    unlikely_locations: list[str] = Field(default_factory=list)
    related_objects: list[str] = Field(default_factory=list)
    confidence: Confidence = Field(ge=0.0, le=1.0)


class EnvironmentKnowledge(StrictBaseModel):
    building_id: str | None = None
    floor_id: str | None = None
    known_rooms: list[EnvironmentRoom] = Field(default_factory=list)
    known_doors: list[EnvironmentDoor] = Field(default_factory=list)
    corridor_layout: str | None = None
    room_type_priors: list[RoomTypePrior] = Field(default_factory=list)
    object_location_priors: list[ObjectLocationPrior] = Field(default_factory=list)
    last_updated_at: str | None = None
    confidence: Confidence = Field(ge=0.0, le=1.0)


class ObservationRecord(StrictBaseModel):
    observation_id: str
    timestamp: str
    image_id: str | None = None
    frame_id: str | None = None
    location_hint: str | None = None
    visible_objects: list[SceneObject] = Field(default_factory=list)
    visible_relations: list[SceneRelation] = Field(default_factory=list)
    detected_doors: list[EnvironmentDoor] = Field(default_factory=list)
    room_state: dict[str, str | int | float | bool | None] = Field(default_factory=dict)
    task_context: str | None = None
    confidence: Confidence = Field(ge=0.0, le=1.0)


class KnowledgeItem(StrictBaseModel):
    id: str
    knowledge_type: Literal[
        "environment_layout",
        "room_type_prior",
        "object_location_prior",
        "observation",
        "task_memory",
    ]
    content_zh: str
    source: str
    confidence: Confidence = Field(ge=0.0, le=1.0)
    metadata: dict[str, str | int | float | bool | None] = Field(default_factory=dict)


class RobotTask(StrictBaseModel):
    task_id: str
    raw_text: str
    task_type: Literal[
        "find_object",
        "count_objects",
        "inspect_area",
        "check_door_state",
        "find_room",
        "navigate_to_location",
        "verify_condition",
        "summarize_scene",
        "compare_states",
    ]
    target_object: str | None = None
    target_location: str | None = None
    target_room: str | None = None
    scope: str | None = None
    constraints: list[str] = Field(default_factory=list)
    parsed_slots: dict[str, str | int | float | bool | None] = Field(default_factory=dict)
    confidence: Confidence = Field(ge=0.0, le=1.0)


class SceneHypothesis(StrictBaseModel):
    hypothesis_id: str
    target: str
    possible_location: str
    supporting_evidence: list[str] = Field(default_factory=list)
    contradicting_evidence: list[str] = Field(default_factory=list)
    knowledge_sources: list[str] = Field(default_factory=list)
    probability: Confidence = Field(ge=0.0, le=1.0)
    verification_action: str
    status: Literal["proposed", "verified", "rejected", "unknown"] = "proposed"


class PredictiveSceneGraphNode(StrictBaseModel):
    id: str
    label: str
    node_type: Literal[
        "observed_object",
        "inferred_object",
        "place_node",
        "container_node",
        "task_target_node",
    ]
    object_id: str | None = None
    visible: bool
    confidence: Confidence = Field(ge=0.0, le=1.0)
    reason_zh: str | None = None
    source_ids: list[str] = Field(default_factory=list)


class PredictiveSceneGraphEdge(StrictBaseModel):
    source_id: str
    target_id: str
    edge_type: Literal[
        "visible_relation",
        "spatial_prior",
        "functional_prior",
        "containment_prior",
        "navigation_relation",
        "verification_relation",
    ]
    label: str | None = None
    confidence: Confidence = Field(ge=0.0, le=1.0)
    reason_zh: str | None = None
    source_ids: list[str] = Field(default_factory=list)


class PredictiveSceneGraph(StrictBaseModel):
    nodes: list[PredictiveSceneGraphNode] = Field(default_factory=list)
    edges: list[PredictiveSceneGraphEdge] = Field(default_factory=list)
    observed_node_ids: list[str] = Field(default_factory=list)
    inferred_node_ids: list[str] = Field(default_factory=list)
    uncertainty_notes: list[str] = Field(default_factory=list)
    recommended_verification_targets: list[str] = Field(default_factory=list)


class TaskPlanStep(StrictBaseModel):
    step_id: int = Field(ge=1)
    action_type: Literal[
        "observe",
        "move",
        "turn",
        "inspect",
        "count",
        "verify",
        "summarize",
        "stop",
    ]
    target: str | None = None
    description_zh: str
    expected_result: str | None = None
    depends_on: list[int] = Field(default_factory=list)
    confidence: Confidence = Field(ge=0.0, le=1.0)


class CountState(StrictBaseModel):
    counted_object_ids: list[str] = Field(default_factory=list)
    possible_duplicates: list[list[str]] = Field(default_factory=list)
    uncertain_regions: list[str] = Field(default_factory=list)
    recommended_next_viewpoints: list[str] = Field(default_factory=list)


class TaskPlan(StrictBaseModel):
    plan_type: Literal[
        "find_object",
        "count_objects",
        "inspect_area",
        "check_door_state",
        "navigate_to_location",
        "general",
    ]
    summary_zh: str
    steps: list[TaskPlanStep] = Field(default_factory=list)
    fallback_steps: list[TaskPlanStep] = Field(default_factory=list)
    success_conditions: list[str] = Field(default_factory=list)
    uncertainty_notes: list[str] = Field(default_factory=list)
    count_state: CountState | None = None


class KnowledgeUpdate(StrictBaseModel):
    update_id: str
    update_type: Literal["new", "confirmed", "conflict", "expired", "ignored"]
    knowledge_type: Literal[
        "environment_layout",
        "room_type_prior",
        "object_location_prior",
        "temporary_state",
        "task_memory",
    ]
    content_zh: str
    source_observation_id: str | None = None
    stable: bool
    confidence: Confidence = Field(ge=0.0, le=1.0)


class CandidateFact(StrictBaseModel):
    fact_id: str
    fact_type: Literal[
        "environment_layout",
        "room_type_prior",
        "object_location_prior",
        "temporary_state",
        "task_memory",
    ]
    content_zh: str
    source_object_ids: list[str] = Field(default_factory=list)
    stable: bool
    confidence: Confidence = Field(ge=0.0, le=1.0)
    metadata: dict[str, str | int | float | bool | None] = Field(default_factory=dict)


class KnowledgeAwareSceneResult(StrictBaseModel):
    base_scene: SceneAnalysisResult
    deprecated_name: bool = True
    new_concept: str = "llm_generated_commonsense_with_observation_memory"
    static_kb_used: bool = False
    handcrafted_priors_used: bool = False
    parsed_task: RobotTask
    retrieved_knowledge: list[KnowledgeItem] = Field(default_factory=list)
    predictive_scene_graph: PredictiveSceneGraph
    hypotheses: list[SceneHypothesis] = Field(default_factory=list)
    reasoning_summary_zh: str
    task_plan: TaskPlan
    knowledge_updates: list[KnowledgeUpdate] = Field(default_factory=list)
    final_answer_zh: str
    observed_facts: ObservedSceneFacts | None = None
    llm_reasoning: LLMReasoningResult | None = None
    reasoned_predictive_scene_graph: ReasonedPredictiveSceneGraph | None = None
    quadruped_search_plan: QuadrupedSearchPlan | None = None
    actionability_notes_zh: list[str] = Field(default_factory=list)
    retrieved_experiences: list[dict[str, Any]] = Field(default_factory=list)
    experience_writes: list[dict[str, Any]] = Field(default_factory=list)
    visual_grounding_report: dict[str, Any] | None = None
    motion_horizon_decision: MotionHorizonDecision | None = None
