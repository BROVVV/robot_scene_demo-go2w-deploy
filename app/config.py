"""Application configuration loaded from environment variables."""

from __future__ import annotations

import os
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

try:
    from dotenv import load_dotenv
except ModuleNotFoundError:  # ROS system Python may intentionally omit dotenv.
    def load_dotenv(*_args, **_kwargs):
        return False


DEFAULT_SILICONFLOW_BASE_URL = "https://api.siliconflow.cn/v1"
# Keep the legacy model as the vision default.  Text planning and image
# understanding deliberately have separate configuration contracts.
DEFAULT_SILICONFLOW_MODEL = "Qwen/Qwen3-VL-8B-Instruct"
DEFAULT_SILICONFLOW_REASONING_MODEL = "deepseek-ai/DeepSeek-V4-Flash"
DEFAULT_SILICONFLOW_VISION_MODEL = DEFAULT_SILICONFLOW_MODEL
DEFAULT_OUTPUT_DIR = "outputs"
DEFAULT_SILICONFLOW_TIMEOUT_SECONDS = 25.0
DEFAULT_SILICONFLOW_MAX_TOKENS = 4096
DEFAULT_SILICONFLOW_REASONING_TIMEOUT_SECONDS = 40.0
DEFAULT_SILICONFLOW_REASONING_MAX_TOKENS = 2048
DEFAULT_SILICONFLOW_REASONING_EFFORT = "high"
DEFAULT_IMAGE_MAX_SIDE = 1280
DEFAULT_IMAGE_DETAIL = "high"
DEFAULT_ENABLE_LOW_OBJECT_RETRY = True
DEFAULT_MIN_OBJECTS_FOR_COMPLEX_SCENE = 6
DEFAULT_ENABLE_IMAGE_PREPROCESS = True
DEFAULT_IMAGE_PREPROCESS_SHARPEN = False
DEFAULT_IMAGE_PREPROCESS_DENOISE = False
DEFAULT_ENABLE_TILED_DETECTION = False
DEFAULT_TILE_SIZE = 960
DEFAULT_TILE_OVERLAP = 160
DEFAULT_ENABLE_TARGET_PROFILE = True
DEFAULT_TARGET_PROFILE_LANGUAGE = "zh_en"
DEFAULT_TARGET_PROFILE_MAX_TERMS = 40
DEFAULT_ENABLE_TARGET_SYNONYM_EXPANSION = True
DEFAULT_ENABLE_CONTEXT_OBJECT_EXPANSION = True
DEFAULT_DETECTION_BACKEND = "llm"
DEFAULT_GROUNDED_SAM_ROOT = "/root/gpufree-data/Grounded-SAM-2"
DEFAULT_GROUNDED_SAM_PYTHON = "python"
DEFAULT_GROUNDED_SAM_PYTHONPATH = (
    "/root/gpufree-data/Grounded-SAM-2:"
    "/root/gpufree-data/Grounded-SAM-2/grounding_dino"
)
DEFAULT_GROUNDING_DINO_CONFIG = "grounding_dino/groundingdino/config/GroundingDINO_SwinT_OGC.py"
DEFAULT_GROUNDING_DINO_CHECKPOINT = "gdino_checkpoints/groundingdino_swint_ogc.pth"
DEFAULT_GROUNDING_DINO_BOX_THRESHOLD = 0.12
DEFAULT_GROUNDING_DINO_TEXT_THRESHOLD = 0.10
DEFAULT_GROUNDING_DINO_HIGH_RECALL_BOX_THRESHOLD = 0.10
DEFAULT_GROUNDING_DINO_HIGH_RECALL_TEXT_THRESHOLD = 0.08
DEFAULT_ENABLE_GDINO_HIGH_RECALL = True
DEFAULT_ENABLE_SAM2 = True
DEFAULT_SAM2_CONFIG = "configs/sam2.1/sam2.1_hiera_t.yaml"
DEFAULT_SAM2_CHECKPOINT = "checkpoints/sam2.1_hiera_tiny.pt"
DEFAULT_MAX_DETECTED_OBJECTS = 100
DEFAULT_DETECTION_DEVICE = "auto"
DEFAULT_DETECTOR_TIMEOUT_SECONDS = 60.0
DEFAULT_ENABLE_SAM2_MASK_FEATURES = True
DEFAULT_ENABLE_CROP_VERIFY = True
DEFAULT_CROP_VERIFY_BACKEND = "llm"
DEFAULT_CROP_VERIFY_MAX_CANDIDATES = 40
DEFAULT_CROP_VERIFY_EXPAND_RATIO = 1.35
DEFAULT_CROP_VERIFY_MIN_SCORE = 0.55
DEFAULT_CROP_VERIFY_TARGET_SCORE = 0.70
DEFAULT_CROP_VERIFY_TIMEOUT_SECONDS = 60.0
DEFAULT_CROP_VERIFY_SAVE_CROPS = True
DEFAULT_CROP_VERIFY_OUTPUT_DIR = "outputs/crops"
DEFAULT_ENABLE_SCORE_FUSION = True
DEFAULT_FUSION_WEIGHT_DETECTOR = 0.30
DEFAULT_FUSION_WEIGHT_VLM = 0.45
DEFAULT_FUSION_WEIGHT_ATTRIBUTE = 0.15
DEFAULT_FUSION_WEIGHT_CONTEXT = 0.10
DEFAULT_FINAL_TARGET_SCORE_THRESHOLD = 0.65
DEFAULT_FINAL_CANDIDATE_SCORE_THRESHOLD = 0.45
DEFAULT_VIDEO_SAMPLE_FPS = 3.0
DEFAULT_VIDEO_MAX_FRAMES = 300
DEFAULT_VIDEO_ENABLE_TRACKING = True
DEFAULT_VIDEO_TRACK_IOU_THRESHOLD = 0.35
DEFAULT_VIDEO_TRACK_MAX_MISSING_FRAMES = 8
DEFAULT_VIDEO_TRACK_MIN_HITS = 2
DEFAULT_VIDEO_TARGET_CONFIRM_MIN_FRAMES = 3
DEFAULT_VIDEO_TARGET_CONFIRM_SCORE = 0.65
DEFAULT_VIDEO_ENABLE_TRACK_LEVEL_VOTING = True
DEFAULT_VIDEO_VERIFY_EVERY_N_FRAMES = 2
DEFAULT_VIDEO_SAVE_CANDIDATE_CROPS = True
DEFAULT_EVAL_IOU_THRESHOLD = 0.5
DEFAULT_EVAL_OUTPUT_DIR = "outputs/eval"
DEFAULT_VIDEO_ENABLE_SCENE_MEMORY = True
DEFAULT_VIDEO_MODE_DEFAULT = "target_search"
DEFAULT_VIDEO_ENABLE_SCENE_MAPPING = False
DEFAULT_VIDEO_FULL_SCENE_MAP_ENABLED = False
DEFAULT_VIDEO_USE_SCENE_MAP_FOR_SEARCH = True
DEFAULT_VIDEO_ALLOW_SCENE_MAP_ONLY = False
DEFAULT_VIDEO_TARGET_SEARCH_REQUIRED_WHEN_TARGET_PRESENT = True
DEFAULT_VIDEO_SCENE_MAPPING_REUSE_TARGET_FRAMES = True
DEFAULT_VIDEO_SCENE_MAPPING_REUSE_OBJECT_TRACKS = True
DEFAULT_VIDEO_ALWAYS_WRITE_MEMORY = True
DEFAULT_VIDEO_ENABLE_VIDEO_PSG = True
DEFAULT_VIDEO_ENABLE_NAVIGATION_TOPOLOGY = False
DEFAULT_VIDEO_TOPOLOGY_ANNOTATE_TARGET_SEARCH = True
DEFAULT_VIDEO_TOPOLOGY_ADD_TARGET_CANDIDATE_NODES = True
DEFAULT_VIDEO_TOPOLOGY_ADD_TARGET_SEARCH_SCORES = True
DEFAULT_VIDEO_PSG_MAX_PREDICTED_NODES = 30
DEFAULT_VIDEO_PSG_CONFIDENCE_THRESHOLD = 0.45
DEFAULT_VIDEO_TOPOLOGY_OBSERVED_ONLY = False
DEFAULT_VIDEO_SAVE_FRAME_OBSERVATIONS = True
DEFAULT_VIDEO_ENABLE_NEGATIVE_EVIDENCE = True
DEFAULT_VIDEO_ENABLE_REASONING_REPORT = True
DEFAULT_VIDEO_MEMORY_STORE_PATH = "data/memory/video_spatial_memory.jsonl"
DEFAULT_VIDEO_MIN_MEMORY_IMPORTANCE = "low"
DEFAULT_VIDEO_LLM_FRAME_INTERVAL_SEC = 2.0
DEFAULT_VIDEO_PSG_SUMMARY_INTERVAL_SEC = 5.0
DEFAULT_VIDEO_MAX_MEMORY_ENTRIES_PER_VIDEO = 200
DEFAULT_VIDEO_MEMORY_DEDUP_SIMILARITY = 0.86
DEFAULT_VIDEO_ENABLE_MEMORY_RETRIEVAL = True
DEFAULT_VIDEO_MEMORY_RETRIEVAL_TOP_K = 10
DEFAULT_VIDEO_SCENE_REASONER_BACKEND = "llm"
DEFAULT_VIDEO_FORCE_JSON_OUTPUT = True
DEFAULT_LIVE_SEARCH_SEMANTIC_REASONING_ENABLED = False
DEFAULT_LIVE_SEARCH_REASONER_BACKEND = "legacy"
DEFAULT_LIVE_SEARCH_REASONER_MODE = "shadow"
DEFAULT_LIVE_SEARCH_REASONER_MIN_CONFIDENCE = 0.55
DEFAULT_LIVE_SEARCH_REASONER_ALLOW_FORWARD = False
DEFAULT_LIVE_SEARCH_REASONER_MAX_TURN_DEG = 30.0
DEFAULT_LIVE_SEARCH_REASONER_MIN_REPLAN_SECONDS = 5.0
DEFAULT_LIVE_SEARCH_REASONER_SCENE_TTL_SECONDS = 10.0
DEFAULT_VLM_RUNTIME_TRANSPORT = "daemon"
DEFAULT_VLM_RUNTIME_FALLBACK_TO_SUBPROCESS = True
DEFAULT_VLM_RUNTIME_ALLOW_API_CONCURRENCY = True
DEFAULT_VLM_RUNTIME_QUICK_MAX_TOKENS = 1024
DEFAULT_VLM_RUNTIME_VERIFY_MAX_TOKENS = 256
DEFAULT_VLM_RUNTIME_SEMANTIC_MAX_TOKENS = 1536
# The robot deployment allows SiliconFlow requests up to 60 s.  A shorter
# outer Quick deadline kills healthy responses and triggers duplicate fallback
# requests while the daemon is still working.
DEFAULT_VLM_RUNTIME_QUICK_TIMEOUT_SECONDS = 65.0
DEFAULT_VLM_RUNTIME_VERIFY_TIMEOUT_SECONDS = 20.0
DEFAULT_VLM_RUNTIME_SEMANTIC_TIMEOUT_SECONDS = 45.0
DEFAULT_VLM_RUNTIME_SEMANTIC_BACKGROUND_ENABLED = True
DEFAULT_VLM_RUNTIME_SEMANTIC_INITIAL_WARMUP_BLOCKING = False
DEFAULT_VLM_RUNTIME_SEMANTIC_MAX_INFLIGHT = 1
DEFAULT_VLM_RUNTIME_SEMANTIC_TTL_SECONDS = 12.0
DEFAULT_VLM_RUNTIME_SEMANTIC_TRANSLATION_REFRESH_M = 0.30
DEFAULT_VLM_RUNTIME_SEMANTIC_HEADING_SECTOR_DEG = 30.0
DEFAULT_VLM_RUNTIME_SEMANTIC_VISUAL_CHANGE_ENABLED = True
DEFAULT_VLM_RUNTIME_PLANNER_SEMANTIC_SOFT_STALE_SECONDS = 15.0
DEFAULT_VLM_RUNTIME_PLANNER_SEMANTIC_HARD_STALE_SECONDS = 45.0
DEFAULT_VLM_RUNTIME_VERIFY_SAME_FRAME_MAX_CALLS = 1
DEFAULT_LIVE_SEARCH_GRAPH_MATCH_PARTIAL_THRESHOLD = 0.30
DEFAULT_LIVE_SEARCH_GRAPH_MATCH_STRONG_THRESHOLD = 0.72
DEFAULT_LIVE_SEARCH_NEGATIVE_MEMORY_ENABLED = True
DEFAULT_LIVE_SEARCH_NEGATIVE_MEMORY_TTL_SECONDS = 300.0
DEFAULT_LIVE_SEARCH_REASONER_USE_PSG = True
DEFAULT_LIVE_SEARCH_REASONER_USE_OBSERVATION_MEMORY = True
DEFAULT_LIVE_SEARCH_REASONER_USE_LLM_SITUATED_PRIOR = True
DEFAULT_LIVE_SEARCH_REASONER_DEBUG_OUTPUT_DIR = "outputs/live_reasoning"
DEFAULT_VIDEO_NAVIGATION_ENABLED = True
DEFAULT_VIDEO_NAVIGATION_MODE = "visual_preview"
DEFAULT_VIDEO_POSE_BACKEND = "auto"
DEFAULT_VIDEO_POSE_ALLOW_RELATIVE = True
DEFAULT_VIDEO_POSE_REQUIRE_METRIC_FOR_NAV2 = True
DEFAULT_VIDEO_NAVIGATION_AUTO_PLAN = True
DEFAULT_VIDEO_NAVIGATION_AUTO_EXPLORATION = True
DEFAULT_VIDEO_NAVIGATION_MAX_FRAMES = 300
DEFAULT_VIDEO_NAVIGATION_FRAME_SAMPLE_INTERVAL = 5
DEFAULT_VIDEO_NAVIGATION_MIN_TRACK_CONFIDENCE = 0.5
DEFAULT_VIDEO_NAVIGATION_TARGET_OBSERVATION_DISTANCE = 1.5
DEFAULT_VIDEO_NAVIGATION_ENABLE_FRONTIER_EXPLORATION = True
DEFAULT_VIDEO_NAVIGATION_EXPLORATION_MAX_CANDIDATES = 8
DEFAULT_VIDEO_NAVIGATION_EXPLORATION_MIN_INFORMATION_GAIN = 0.2
DEFAULT_VIDEO_NAVIGATION_ALLOW_NAV2_FROM_METRIC_VIDEO = False
DEFAULT_VISUAL_NAV_EXECUTION_ENABLED = False
DEFAULT_ENABLE_LLM_SITUATED_REASONING = True
DEFAULT_ENABLE_LLM_REASONING_MEMORY = True
DEFAULT_LLM_REASONING_MAX_HYPOTHESES = 5
DEFAULT_LLM_REASONING_TIMEOUT_SECONDS = 40.0
DEFAULT_LLM_REASONING_TEMPERATURE = 0.2
DEFAULT_LLM_REASONING_REQUIRE_ACTIONABILITY_GATE = True
DEFAULT_LLM_REASONING_REQUIRE_VISUAL_GATE = True
DEFAULT_QUADRUPED_MAX_FORWARD_STEP_M = 0.5
DEFAULT_QUADRUPED_CAN_MANIPULATE = False
DEFAULT_QUADRUPED_CAN_OPEN_CONTAINER = False
DEFAULT_QUADRUPED_CAN_LOOK_DOWN = False
DEFAULT_LLM_EXPERIENCE_MEMORY_PATH = "data/memory/llm_spatial_experience.jsonl"
DEFAULT_ENABLE_LLM_DYNAMIC_VISUAL_RETRY = True
DEFAULT_LLM_DYNAMIC_VISUAL_RETRY_MAX_TERMS = 12
DEFAULT_PLATFORM_OBSTACLE_AVOIDANCE_ASSUMED = True
DEFAULT_ENABLE_DYNAMIC_MOTION_HORIZON = True
DEFAULT_MOTION_HORIZON_PROFILE = "platform_assisted_auto"
DEFAULT_MOTION_STRICT_SAFE_MAX_STEP_M = 0.5
DEFAULT_MOTION_PLATFORM_INDOOR_DEFAULT_STEP_M = 1.2
DEFAULT_MOTION_PLATFORM_INDOOR_MAX_STEP_M = 2.0
DEFAULT_MOTION_PLATFORM_OPEN_DEFAULT_STEP_M = 3.0
DEFAULT_MOTION_PLATFORM_OPEN_MAX_STEP_M = 5.0
DEFAULT_MOTION_ABSOLUTE_MAX_STEP_M = 6.0
DEFAULT_MOTION_TARGET_CONFIRM_MAX_STEP_M = 0.8
DEFAULT_MOTION_PLATFORM_FALLBACK_STEP_M = 1.5
DEFAULT_MOTION_DEFAULT_STOP_AND_REOBSERVE = True
DEFAULT_MOTION_ENABLE_OBSERVE_WHILE_MOVING = False
DEFAULT_MOTION_SOFT_OBSERVE_INTERVAL_SEC = 1.0
DEFAULT_MOTION_SHORTEN_ON_TARGET_CANDIDATE = True
DEFAULT_MOTION_ALLOW_LLM_RECOMMENDED_HORIZON = True
DEFAULT_MOTION_LLM_HORIZON_WEIGHT = 0.6
DEFAULT_STATIC_KNOWLEDGE_BASE_ENABLED = False
DEFAULT_HANDWRITTEN_OBJECT_PRIORS_ENABLED = False
DEFAULT_HANDWRITTEN_LOCATION_PRIORS_ENABLED = False
DEFAULT_HANDWRITTEN_ROOM_PRIORS_ENABLED = False
DEFAULT_STATIC_OBJECT_PROMPTS_ENABLED = False
DEFAULT_ALLOW_HANDCRAFTED_SEARCH_RULES = False
DEFAULT_LLM_COMMONSENSE_PRIOR_ENABLED = True
DEFAULT_LLM_PRIOR_GENERATION_MODE = "runtime"
DEFAULT_LLM_PRIOR_REQUIRE_REASON = True
DEFAULT_LLM_PRIOR_ALLOW_SEARCH_HINTS = True
DEFAULT_LLM_PRIOR_ALLOW_DETECTOR_PROMPTS = True
DEFAULT_LLM_PRIOR_CAN_CONFIRM_TARGET = False
DEFAULT_LLM_PRIOR_MAX_HYPOTHESES = 8
DEFAULT_LLM_PRIOR_MAX_DETECTOR_PROMPTS = 12
DEFAULT_LLM_PRIOR_OUTPUT_LANGUAGE = "zh"
DEFAULT_EVIDENCE_GATING_ENABLED = True
DEFAULT_VIDEO_TARGET_CONFIRMATION_BY_CONTEXT_ONLY = False
DEFAULT_TARGET_CONFIRMATION_REQUIRE_VISUAL_EVIDENCE = True
DEFAULT_TARGET_CONFIRMATION_REQUIRE_BBOX = True
DEFAULT_TARGET_CONFIRMATION_REQUIRE_CROP_VERIFY = True
DEFAULT_TARGET_CONFIRMATION_REQUIRE_MASK = False
DEFAULT_TARGET_CONFIRMATION_MIN_SCORE = 0.72
DEFAULT_TARGET_INFERRED_IS_NOT_FOUND = True
DEFAULT_VIDEO_SEARCH_RANKER_ENABLED = True
DEFAULT_VIDEO_SEARCH_RANKER_HIGH_SCORE_THRESHOLD = 0.70
DEFAULT_VIDEO_SEARCH_RANKER_CONTEXT_ONLY_CAN_CONFIRM = False
DEFAULT_OBSERVATION_MEMORY_ENABLED = True
DEFAULT_OBSERVATION_MEMORY_STORE_PATH = "data/memory/observational_memory.jsonl"
DEFAULT_OBSERVATION_MEMORY_WRITE_VISUAL_ONLY = True
DEFAULT_OBSERVATION_MEMORY_ALLOW_LLM_SUMMARY = True
DEFAULT_OBSERVATION_MEMORY_LLM_SUMMARY_AS_HYPOTHESIS = True
DEFAULT_OBSERVATION_MEMORY_RETRIEVAL_TOP_K = 10
DEFAULT_OBSERVATION_MEMORY_REQUIRE_PROVENANCE = True
DEFAULT_PRIOR_USAGE_AUDIT_ENABLED = True
DEFAULT_PRIOR_USAGE_REPORT_PATH = "outputs/prior_usage_report.json"
DEFAULT_GROUNDING_PROMPT_LLM_EXPANSION_ENABLED = True
DEFAULT_GROUNDING_PROMPT_REQUIRE_NON_EMPTY = True
DEFAULT_GROUNDING_PROMPT_FAIL_FAST_ON_EMPTY = True
DEFAULT_GROUNDING_PROMPT_RETRY_ON_EMPTY = True
DEFAULT_GROUNDING_PROMPT_MAX_RETRIES = 1
DEFAULT_GROUNDING_PROMPT_MAX_TERMS = 24
DEFAULT_GROUNDING_PROMPT_MIN_TERMS = 3
DEFAULT_GROUNDING_PROMPT_DEBUG_OUTPUT = "outputs/grounding_prompt_plan.json"
DEFAULT_GROUNDING_PROMPT_RETRY_DEBUG_OUTPUT = "outputs/grounding_prompt_retry_plan.json"

# ---- PandarXT-16 and dual-LiDAR safety (all fail-closed by default) ----
DEFAULT_PANDARXT16_ENABLED = False
DEFAULT_PANDARXT16_DIAGNOSTIC_PREPROCESS_ENABLED = False
DEFAULT_PANDARXT16_ZERO_RETURN_MAX_M = 0.05
DEFAULT_PANDARXT16_CLOCK_REQUIRE_VALIDATED_FOR_METRIC = True

DEFAULT_DUAL_LIDAR_SAFETY_ENABLED = False
DEFAULT_DUAL_LIDAR_REQUIRE_VALIDATED_EXTRINSICS = True
DEFAULT_DUAL_LIDAR_UNKNOWN_IS_CLEAR = False
DEFAULT_DUAL_LIDAR_MAX_EVIDENCE_AGE_SECONDS = 0.5

# Current operator-confirmed whole-machine geometry (2026-08-13). The highest
# point is the fixed PandarXT-16 protective frame, not a loose cable, and the
# machine is NOT scheduled for re-measurement.
DEFAULT_GO2W_CURRENT_LENGTH_M = 0.70
DEFAULT_GO2W_CURRENT_WIDTH_M = 0.43
DEFAULT_GO2W_CURRENT_HEIGHT_M = 0.70
DEFAULT_GO2W_CURRENT_HIGHEST_POINT = "pandarxt16_protective_frame"
DEFAULT_GO2W_CURRENT_GEOMETRY_CONFIG = "configs/go2w/current_hardware_geometry.yaml"
DEFAULT_GO2W_CURRENT_STATE_CONFIG = "configs/go2w/current_hardware_state.yaml"
DEFAULT_GO2W_PANDAR_PREPROCESS_CONFIG = "configs/go2w/hesai_pandarxt16_preprocess.yaml"


class SettingsError(RuntimeError):
    """Raised when required runtime configuration is missing."""


@dataclass(frozen=True)
class Settings:
    siliconflow_api_key: str
    siliconflow_base_url: str = DEFAULT_SILICONFLOW_BASE_URL
    # Deprecated compatibility field.  New callers must use vision_model or
    # reasoning_model so the two model IDs cannot be coupled accidentally.
    siliconflow_model: str = DEFAULT_SILICONFLOW_MODEL
    siliconflow_reasoning_model: str = DEFAULT_SILICONFLOW_REASONING_MODEL
    siliconflow_vision_model: str | None = None
    output_dir: str = DEFAULT_OUTPUT_DIR
    siliconflow_timeout_seconds: float = DEFAULT_SILICONFLOW_TIMEOUT_SECONDS
    siliconflow_max_tokens: int = DEFAULT_SILICONFLOW_MAX_TOKENS
    siliconflow_reasoning_timeout_seconds: float = DEFAULT_SILICONFLOW_REASONING_TIMEOUT_SECONDS
    siliconflow_reasoning_max_tokens: int = DEFAULT_SILICONFLOW_REASONING_MAX_TOKENS
    siliconflow_reasoning_effort: str = DEFAULT_SILICONFLOW_REASONING_EFFORT
    image_max_side: int = DEFAULT_IMAGE_MAX_SIDE
    image_detail: str = DEFAULT_IMAGE_DETAIL
    enable_low_object_retry: bool = DEFAULT_ENABLE_LOW_OBJECT_RETRY
    min_objects_for_complex_scene: int = DEFAULT_MIN_OBJECTS_FOR_COMPLEX_SCENE
    enable_image_preprocess: bool = DEFAULT_ENABLE_IMAGE_PREPROCESS
    image_preprocess_sharpen: bool = DEFAULT_IMAGE_PREPROCESS_SHARPEN
    image_preprocess_denoise: bool = DEFAULT_IMAGE_PREPROCESS_DENOISE
    enable_tiled_detection: bool = DEFAULT_ENABLE_TILED_DETECTION
    tile_size: int = DEFAULT_TILE_SIZE
    tile_overlap: int = DEFAULT_TILE_OVERLAP
    enable_target_profile: bool = DEFAULT_ENABLE_TARGET_PROFILE
    target_profile_language: str = DEFAULT_TARGET_PROFILE_LANGUAGE
    target_profile_max_terms: int = DEFAULT_TARGET_PROFILE_MAX_TERMS
    enable_target_synonym_expansion: bool = DEFAULT_ENABLE_TARGET_SYNONYM_EXPANSION
    enable_context_object_expansion: bool = DEFAULT_ENABLE_CONTEXT_OBJECT_EXPANSION
    detection_backend: str = DEFAULT_DETECTION_BACKEND
    grounded_sam_root: str = DEFAULT_GROUNDED_SAM_ROOT
    grounded_sam_python: str = DEFAULT_GROUNDED_SAM_PYTHON
    grounded_sam_pythonpath: str = DEFAULT_GROUNDED_SAM_PYTHONPATH
    grounding_dino_config: str = DEFAULT_GROUNDING_DINO_CONFIG
    grounding_dino_checkpoint: str = DEFAULT_GROUNDING_DINO_CHECKPOINT
    grounding_dino_box_threshold: float = DEFAULT_GROUNDING_DINO_BOX_THRESHOLD
    grounding_dino_text_threshold: float = DEFAULT_GROUNDING_DINO_TEXT_THRESHOLD
    grounding_dino_high_recall_box_threshold: float = DEFAULT_GROUNDING_DINO_HIGH_RECALL_BOX_THRESHOLD
    grounding_dino_high_recall_text_threshold: float = DEFAULT_GROUNDING_DINO_HIGH_RECALL_TEXT_THRESHOLD
    enable_gdino_high_recall: bool = DEFAULT_ENABLE_GDINO_HIGH_RECALL
    enable_sam2: bool = DEFAULT_ENABLE_SAM2
    enable_sam2_mask_features: bool = DEFAULT_ENABLE_SAM2_MASK_FEATURES
    sam2_config: str = DEFAULT_SAM2_CONFIG
    sam2_checkpoint: str = DEFAULT_SAM2_CHECKPOINT
    max_detected_objects: int = DEFAULT_MAX_DETECTED_OBJECTS
    detection_device: str = DEFAULT_DETECTION_DEVICE
    detector_timeout_seconds: float = DEFAULT_DETECTOR_TIMEOUT_SECONDS
    enable_crop_verify: bool = DEFAULT_ENABLE_CROP_VERIFY
    crop_verify_backend: str = DEFAULT_CROP_VERIFY_BACKEND
    crop_verify_max_candidates: int = DEFAULT_CROP_VERIFY_MAX_CANDIDATES
    crop_verify_expand_ratio: float = DEFAULT_CROP_VERIFY_EXPAND_RATIO
    crop_verify_min_score: float = DEFAULT_CROP_VERIFY_MIN_SCORE
    crop_verify_target_score: float = DEFAULT_CROP_VERIFY_TARGET_SCORE
    crop_verify_timeout_seconds: float = DEFAULT_CROP_VERIFY_TIMEOUT_SECONDS
    crop_verify_save_crops: bool = DEFAULT_CROP_VERIFY_SAVE_CROPS
    crop_verify_output_dir: str = DEFAULT_CROP_VERIFY_OUTPUT_DIR
    enable_score_fusion: bool = DEFAULT_ENABLE_SCORE_FUSION
    fusion_weight_detector: float = DEFAULT_FUSION_WEIGHT_DETECTOR
    fusion_weight_vlm: float = DEFAULT_FUSION_WEIGHT_VLM
    fusion_weight_attribute: float = DEFAULT_FUSION_WEIGHT_ATTRIBUTE
    fusion_weight_context: float = DEFAULT_FUSION_WEIGHT_CONTEXT
    final_target_score_threshold: float = DEFAULT_FINAL_TARGET_SCORE_THRESHOLD
    final_candidate_score_threshold: float = DEFAULT_FINAL_CANDIDATE_SCORE_THRESHOLD
    video_sample_fps: float = DEFAULT_VIDEO_SAMPLE_FPS
    video_max_frames: int = DEFAULT_VIDEO_MAX_FRAMES
    video_enable_tracking: bool = DEFAULT_VIDEO_ENABLE_TRACKING
    video_track_iou_threshold: float = DEFAULT_VIDEO_TRACK_IOU_THRESHOLD
    video_track_max_missing_frames: int = DEFAULT_VIDEO_TRACK_MAX_MISSING_FRAMES
    video_track_min_hits: int = DEFAULT_VIDEO_TRACK_MIN_HITS
    video_target_confirm_min_frames: int = DEFAULT_VIDEO_TARGET_CONFIRM_MIN_FRAMES
    video_target_confirm_score: float = DEFAULT_VIDEO_TARGET_CONFIRM_SCORE
    video_enable_track_level_voting: bool = DEFAULT_VIDEO_ENABLE_TRACK_LEVEL_VOTING
    video_verify_every_n_frames: int = DEFAULT_VIDEO_VERIFY_EVERY_N_FRAMES
    video_save_candidate_crops: bool = DEFAULT_VIDEO_SAVE_CANDIDATE_CROPS
    eval_iou_threshold: float = DEFAULT_EVAL_IOU_THRESHOLD
    eval_output_dir: str = DEFAULT_EVAL_OUTPUT_DIR
    video_enable_scene_memory: bool = DEFAULT_VIDEO_ENABLE_SCENE_MEMORY
    video_mode_default: str = DEFAULT_VIDEO_MODE_DEFAULT
    video_enable_scene_mapping: bool = DEFAULT_VIDEO_ENABLE_SCENE_MAPPING
    video_full_scene_map_enabled: bool = DEFAULT_VIDEO_FULL_SCENE_MAP_ENABLED
    video_use_scene_map_for_search: bool = DEFAULT_VIDEO_USE_SCENE_MAP_FOR_SEARCH
    video_allow_scene_map_only: bool = DEFAULT_VIDEO_ALLOW_SCENE_MAP_ONLY
    video_target_search_required_when_target_present: bool = (
        DEFAULT_VIDEO_TARGET_SEARCH_REQUIRED_WHEN_TARGET_PRESENT
    )
    video_scene_mapping_reuse_target_frames: bool = (
        DEFAULT_VIDEO_SCENE_MAPPING_REUSE_TARGET_FRAMES
    )
    video_scene_mapping_reuse_object_tracks: bool = (
        DEFAULT_VIDEO_SCENE_MAPPING_REUSE_OBJECT_TRACKS
    )
    video_always_write_memory: bool = DEFAULT_VIDEO_ALWAYS_WRITE_MEMORY
    video_enable_video_psg: bool = DEFAULT_VIDEO_ENABLE_VIDEO_PSG
    video_enable_navigation_topology: bool = DEFAULT_VIDEO_ENABLE_NAVIGATION_TOPOLOGY
    video_topology_annotate_target_search: bool = (
        DEFAULT_VIDEO_TOPOLOGY_ANNOTATE_TARGET_SEARCH
    )
    video_topology_add_target_candidate_nodes: bool = (
        DEFAULT_VIDEO_TOPOLOGY_ADD_TARGET_CANDIDATE_NODES
    )
    video_topology_add_target_search_scores: bool = (
        DEFAULT_VIDEO_TOPOLOGY_ADD_TARGET_SEARCH_SCORES
    )
    video_psg_max_predicted_nodes: int = DEFAULT_VIDEO_PSG_MAX_PREDICTED_NODES
    video_psg_confidence_threshold: float = DEFAULT_VIDEO_PSG_CONFIDENCE_THRESHOLD
    video_topology_observed_only: bool = DEFAULT_VIDEO_TOPOLOGY_OBSERVED_ONLY
    video_save_frame_observations: bool = DEFAULT_VIDEO_SAVE_FRAME_OBSERVATIONS
    video_enable_negative_evidence: bool = DEFAULT_VIDEO_ENABLE_NEGATIVE_EVIDENCE
    video_enable_reasoning_report: bool = DEFAULT_VIDEO_ENABLE_REASONING_REPORT
    video_memory_store_path: str = DEFAULT_VIDEO_MEMORY_STORE_PATH
    video_min_memory_importance: str = DEFAULT_VIDEO_MIN_MEMORY_IMPORTANCE
    video_llm_frame_interval_sec: float = DEFAULT_VIDEO_LLM_FRAME_INTERVAL_SEC
    video_psg_summary_interval_sec: float = DEFAULT_VIDEO_PSG_SUMMARY_INTERVAL_SEC
    video_max_memory_entries_per_video: int = DEFAULT_VIDEO_MAX_MEMORY_ENTRIES_PER_VIDEO
    video_memory_dedup_similarity: float = DEFAULT_VIDEO_MEMORY_DEDUP_SIMILARITY
    video_enable_memory_retrieval: bool = DEFAULT_VIDEO_ENABLE_MEMORY_RETRIEVAL
    video_memory_retrieval_top_k: int = DEFAULT_VIDEO_MEMORY_RETRIEVAL_TOP_K
    video_scene_reasoner_backend: str = DEFAULT_VIDEO_SCENE_REASONER_BACKEND
    video_force_json_output: bool = DEFAULT_VIDEO_FORCE_JSON_OUTPUT
    video_navigation_enabled: bool = DEFAULT_VIDEO_NAVIGATION_ENABLED
    video_navigation_mode: str = DEFAULT_VIDEO_NAVIGATION_MODE
    video_pose_backend: str = DEFAULT_VIDEO_POSE_BACKEND
    video_pose_allow_relative: bool = DEFAULT_VIDEO_POSE_ALLOW_RELATIVE
    video_pose_require_metric_for_nav2: bool = (
        DEFAULT_VIDEO_POSE_REQUIRE_METRIC_FOR_NAV2
    )
    video_navigation_auto_plan: bool = DEFAULT_VIDEO_NAVIGATION_AUTO_PLAN
    video_navigation_auto_exploration: bool = (
        DEFAULT_VIDEO_NAVIGATION_AUTO_EXPLORATION
    )
    video_navigation_max_frames: int = DEFAULT_VIDEO_NAVIGATION_MAX_FRAMES
    video_navigation_frame_sample_interval: int = (
        DEFAULT_VIDEO_NAVIGATION_FRAME_SAMPLE_INTERVAL
    )
    video_navigation_min_track_confidence: float = (
        DEFAULT_VIDEO_NAVIGATION_MIN_TRACK_CONFIDENCE
    )
    video_navigation_target_observation_distance: float = (
        DEFAULT_VIDEO_NAVIGATION_TARGET_OBSERVATION_DISTANCE
    )
    video_navigation_enable_frontier_exploration: bool = (
        DEFAULT_VIDEO_NAVIGATION_ENABLE_FRONTIER_EXPLORATION
    )
    video_navigation_exploration_max_candidates: int = (
        DEFAULT_VIDEO_NAVIGATION_EXPLORATION_MAX_CANDIDATES
    )
    video_navigation_exploration_min_information_gain: float = (
        DEFAULT_VIDEO_NAVIGATION_EXPLORATION_MIN_INFORMATION_GAIN
    )
    video_navigation_allow_nav2_from_metric_video: bool = (
        DEFAULT_VIDEO_NAVIGATION_ALLOW_NAV2_FROM_METRIC_VIDEO
    )
    visual_nav_execution_enabled: bool = DEFAULT_VISUAL_NAV_EXECUTION_ENABLED
    enable_llm_situated_reasoning: bool = DEFAULT_ENABLE_LLM_SITUATED_REASONING
    enable_llm_reasoning_memory: bool = DEFAULT_ENABLE_LLM_REASONING_MEMORY
    llm_reasoning_max_hypotheses: int = DEFAULT_LLM_REASONING_MAX_HYPOTHESES
    llm_reasoning_timeout_seconds: float = DEFAULT_LLM_REASONING_TIMEOUT_SECONDS
    llm_reasoning_temperature: float = DEFAULT_LLM_REASONING_TEMPERATURE
    llm_reasoning_require_actionability_gate: bool = (
        DEFAULT_LLM_REASONING_REQUIRE_ACTIONABILITY_GATE
    )
    llm_reasoning_require_visual_gate: bool = DEFAULT_LLM_REASONING_REQUIRE_VISUAL_GATE
    quadruped_max_forward_step_m: float = DEFAULT_QUADRUPED_MAX_FORWARD_STEP_M
    quadruped_can_manipulate: bool = DEFAULT_QUADRUPED_CAN_MANIPULATE
    quadruped_can_open_container: bool = DEFAULT_QUADRUPED_CAN_OPEN_CONTAINER
    quadruped_can_look_down: bool = DEFAULT_QUADRUPED_CAN_LOOK_DOWN
    llm_experience_memory_path: str = DEFAULT_LLM_EXPERIENCE_MEMORY_PATH
    enable_llm_dynamic_visual_retry: bool = DEFAULT_ENABLE_LLM_DYNAMIC_VISUAL_RETRY
    llm_dynamic_visual_retry_max_terms: int = (
        DEFAULT_LLM_DYNAMIC_VISUAL_RETRY_MAX_TERMS
    )
    platform_obstacle_avoidance_assumed: bool = (
        DEFAULT_PLATFORM_OBSTACLE_AVOIDANCE_ASSUMED
    )
    enable_dynamic_motion_horizon: bool = DEFAULT_ENABLE_DYNAMIC_MOTION_HORIZON
    motion_horizon_profile: str = DEFAULT_MOTION_HORIZON_PROFILE
    motion_strict_safe_max_step_m: float = DEFAULT_MOTION_STRICT_SAFE_MAX_STEP_M
    motion_platform_indoor_default_step_m: float = (
        DEFAULT_MOTION_PLATFORM_INDOOR_DEFAULT_STEP_M
    )
    motion_platform_indoor_max_step_m: float = DEFAULT_MOTION_PLATFORM_INDOOR_MAX_STEP_M
    motion_platform_open_default_step_m: float = DEFAULT_MOTION_PLATFORM_OPEN_DEFAULT_STEP_M
    motion_platform_open_max_step_m: float = DEFAULT_MOTION_PLATFORM_OPEN_MAX_STEP_M
    motion_absolute_max_step_m: float = DEFAULT_MOTION_ABSOLUTE_MAX_STEP_M
    motion_target_confirm_max_step_m: float = DEFAULT_MOTION_TARGET_CONFIRM_MAX_STEP_M
    motion_platform_fallback_step_m: float = DEFAULT_MOTION_PLATFORM_FALLBACK_STEP_M
    motion_default_stop_and_reobserve: bool = DEFAULT_MOTION_DEFAULT_STOP_AND_REOBSERVE
    motion_enable_observe_while_moving: bool = (
        DEFAULT_MOTION_ENABLE_OBSERVE_WHILE_MOVING
    )
    motion_soft_observe_interval_sec: float = DEFAULT_MOTION_SOFT_OBSERVE_INTERVAL_SEC
    motion_shorten_on_target_candidate: bool = DEFAULT_MOTION_SHORTEN_ON_TARGET_CANDIDATE
    motion_allow_llm_recommended_horizon: bool = (
        DEFAULT_MOTION_ALLOW_LLM_RECOMMENDED_HORIZON
    )
    motion_llm_horizon_weight: float = DEFAULT_MOTION_LLM_HORIZON_WEIGHT
    static_knowledge_base_enabled: bool = DEFAULT_STATIC_KNOWLEDGE_BASE_ENABLED
    handwritten_object_priors_enabled: bool = DEFAULT_HANDWRITTEN_OBJECT_PRIORS_ENABLED
    handwritten_location_priors_enabled: bool = DEFAULT_HANDWRITTEN_LOCATION_PRIORS_ENABLED
    handwritten_room_priors_enabled: bool = DEFAULT_HANDWRITTEN_ROOM_PRIORS_ENABLED
    static_object_prompts_enabled: bool = DEFAULT_STATIC_OBJECT_PROMPTS_ENABLED
    allow_handcrafted_search_rules: bool = DEFAULT_ALLOW_HANDCRAFTED_SEARCH_RULES
    llm_commonsense_prior_enabled: bool = DEFAULT_LLM_COMMONSENSE_PRIOR_ENABLED
    llm_prior_generation_mode: str = DEFAULT_LLM_PRIOR_GENERATION_MODE
    llm_prior_require_reason: bool = DEFAULT_LLM_PRIOR_REQUIRE_REASON
    llm_prior_allow_search_hints: bool = DEFAULT_LLM_PRIOR_ALLOW_SEARCH_HINTS
    llm_prior_allow_detector_prompts: bool = DEFAULT_LLM_PRIOR_ALLOW_DETECTOR_PROMPTS
    llm_prior_can_confirm_target: bool = DEFAULT_LLM_PRIOR_CAN_CONFIRM_TARGET
    llm_prior_max_hypotheses: int = DEFAULT_LLM_PRIOR_MAX_HYPOTHESES
    llm_prior_max_detector_prompts: int = DEFAULT_LLM_PRIOR_MAX_DETECTOR_PROMPTS
    llm_prior_output_language: str = DEFAULT_LLM_PRIOR_OUTPUT_LANGUAGE
    evidence_gating_enabled: bool = DEFAULT_EVIDENCE_GATING_ENABLED
    video_target_confirmation_by_context_only: bool = (
        DEFAULT_VIDEO_TARGET_CONFIRMATION_BY_CONTEXT_ONLY
    )
    target_confirmation_require_visual_evidence: bool = (
        DEFAULT_TARGET_CONFIRMATION_REQUIRE_VISUAL_EVIDENCE
    )
    target_confirmation_require_bbox: bool = DEFAULT_TARGET_CONFIRMATION_REQUIRE_BBOX
    target_confirmation_require_crop_verify: bool = (
        DEFAULT_TARGET_CONFIRMATION_REQUIRE_CROP_VERIFY
    )
    target_confirmation_require_mask: bool = DEFAULT_TARGET_CONFIRMATION_REQUIRE_MASK
    target_confirmation_min_score: float = DEFAULT_TARGET_CONFIRMATION_MIN_SCORE
    target_inferred_is_not_found: bool = DEFAULT_TARGET_INFERRED_IS_NOT_FOUND
    video_search_ranker_enabled: bool = DEFAULT_VIDEO_SEARCH_RANKER_ENABLED
    video_search_ranker_high_score_threshold: float = (
        DEFAULT_VIDEO_SEARCH_RANKER_HIGH_SCORE_THRESHOLD
    )
    video_search_ranker_context_only_can_confirm: bool = (
        DEFAULT_VIDEO_SEARCH_RANKER_CONTEXT_ONLY_CAN_CONFIRM
    )
    observation_memory_enabled: bool = DEFAULT_OBSERVATION_MEMORY_ENABLED
    observation_memory_store_path: str = DEFAULT_OBSERVATION_MEMORY_STORE_PATH
    observation_memory_write_visual_only: bool = DEFAULT_OBSERVATION_MEMORY_WRITE_VISUAL_ONLY
    observation_memory_allow_llm_summary: bool = DEFAULT_OBSERVATION_MEMORY_ALLOW_LLM_SUMMARY
    observation_memory_llm_summary_as_hypothesis: bool = (
        DEFAULT_OBSERVATION_MEMORY_LLM_SUMMARY_AS_HYPOTHESIS
    )
    observation_memory_retrieval_top_k: int = DEFAULT_OBSERVATION_MEMORY_RETRIEVAL_TOP_K
    observation_memory_require_provenance: bool = (
        DEFAULT_OBSERVATION_MEMORY_REQUIRE_PROVENANCE
    )
    prior_usage_audit_enabled: bool = DEFAULT_PRIOR_USAGE_AUDIT_ENABLED
    prior_usage_report_path: str = DEFAULT_PRIOR_USAGE_REPORT_PATH
    grounding_prompt_llm_expansion_enabled: bool = (
        DEFAULT_GROUNDING_PROMPT_LLM_EXPANSION_ENABLED
    )
    grounding_prompt_require_non_empty: bool = DEFAULT_GROUNDING_PROMPT_REQUIRE_NON_EMPTY
    grounding_prompt_fail_fast_on_empty: bool = (
        DEFAULT_GROUNDING_PROMPT_FAIL_FAST_ON_EMPTY
    )
    grounding_prompt_retry_on_empty: bool = DEFAULT_GROUNDING_PROMPT_RETRY_ON_EMPTY
    grounding_prompt_max_retries: int = DEFAULT_GROUNDING_PROMPT_MAX_RETRIES
    grounding_prompt_max_terms: int = DEFAULT_GROUNDING_PROMPT_MAX_TERMS
    grounding_prompt_min_terms: int = DEFAULT_GROUNDING_PROMPT_MIN_TERMS
    grounding_prompt_debug_output: str = DEFAULT_GROUNDING_PROMPT_DEBUG_OUTPUT
    grounding_prompt_retry_debug_output: str = (
        DEFAULT_GROUNDING_PROMPT_RETRY_DEBUG_OUTPUT
    )
    live_search_semantic_reasoning_enabled: bool = (
        DEFAULT_LIVE_SEARCH_SEMANTIC_REASONING_ENABLED
    )
    live_search_reasoner_backend: str = DEFAULT_LIVE_SEARCH_REASONER_BACKEND
    live_search_reasoner_mode: str = DEFAULT_LIVE_SEARCH_REASONER_MODE
    live_search_reasoner_min_confidence: float = (
        DEFAULT_LIVE_SEARCH_REASONER_MIN_CONFIDENCE
    )
    live_search_reasoner_allow_forward: bool = (
        DEFAULT_LIVE_SEARCH_REASONER_ALLOW_FORWARD
    )
    live_search_reasoner_max_turn_deg: float = DEFAULT_LIVE_SEARCH_REASONER_MAX_TURN_DEG
    live_search_reasoner_min_replan_seconds: float = (
        DEFAULT_LIVE_SEARCH_REASONER_MIN_REPLAN_SECONDS
    )
    live_search_reasoner_scene_ttl_seconds: float = (
        DEFAULT_LIVE_SEARCH_REASONER_SCENE_TTL_SECONDS
    )
    vlm_runtime_transport: str = DEFAULT_VLM_RUNTIME_TRANSPORT
    vlm_runtime_fallback_to_subprocess: bool = DEFAULT_VLM_RUNTIME_FALLBACK_TO_SUBPROCESS
    vlm_runtime_allow_api_concurrency: bool = DEFAULT_VLM_RUNTIME_ALLOW_API_CONCURRENCY
    vlm_runtime_quick_max_tokens: int = DEFAULT_VLM_RUNTIME_QUICK_MAX_TOKENS
    vlm_runtime_verify_max_tokens: int = DEFAULT_VLM_RUNTIME_VERIFY_MAX_TOKENS
    vlm_runtime_semantic_max_tokens: int = DEFAULT_VLM_RUNTIME_SEMANTIC_MAX_TOKENS
    vlm_runtime_quick_timeout_seconds: float = DEFAULT_VLM_RUNTIME_QUICK_TIMEOUT_SECONDS
    vlm_runtime_verify_timeout_seconds: float = DEFAULT_VLM_RUNTIME_VERIFY_TIMEOUT_SECONDS
    vlm_runtime_semantic_timeout_seconds: float = DEFAULT_VLM_RUNTIME_SEMANTIC_TIMEOUT_SECONDS
    vlm_runtime_semantic_background_enabled: bool = DEFAULT_VLM_RUNTIME_SEMANTIC_BACKGROUND_ENABLED
    vlm_runtime_semantic_initial_warmup_blocking: bool = DEFAULT_VLM_RUNTIME_SEMANTIC_INITIAL_WARMUP_BLOCKING
    vlm_runtime_semantic_max_inflight: int = DEFAULT_VLM_RUNTIME_SEMANTIC_MAX_INFLIGHT
    vlm_runtime_semantic_ttl_seconds: float = DEFAULT_VLM_RUNTIME_SEMANTIC_TTL_SECONDS
    vlm_runtime_semantic_translation_refresh_m: float = DEFAULT_VLM_RUNTIME_SEMANTIC_TRANSLATION_REFRESH_M
    vlm_runtime_semantic_heading_sector_deg: float = DEFAULT_VLM_RUNTIME_SEMANTIC_HEADING_SECTOR_DEG
    vlm_runtime_semantic_visual_change_enabled: bool = DEFAULT_VLM_RUNTIME_SEMANTIC_VISUAL_CHANGE_ENABLED
    vlm_runtime_planner_semantic_soft_stale_seconds: float = DEFAULT_VLM_RUNTIME_PLANNER_SEMANTIC_SOFT_STALE_SECONDS
    vlm_runtime_planner_semantic_hard_stale_seconds: float = DEFAULT_VLM_RUNTIME_PLANNER_SEMANTIC_HARD_STALE_SECONDS
    vlm_runtime_verify_same_frame_max_calls: int = DEFAULT_VLM_RUNTIME_VERIFY_SAME_FRAME_MAX_CALLS
    live_search_graph_match_partial_threshold: float = (
        DEFAULT_LIVE_SEARCH_GRAPH_MATCH_PARTIAL_THRESHOLD
    )
    live_search_graph_match_strong_threshold: float = (
        DEFAULT_LIVE_SEARCH_GRAPH_MATCH_STRONG_THRESHOLD
    )
    live_search_negative_memory_enabled: bool = (
        DEFAULT_LIVE_SEARCH_NEGATIVE_MEMORY_ENABLED
    )
    live_search_negative_memory_ttl_seconds: float = (
        DEFAULT_LIVE_SEARCH_NEGATIVE_MEMORY_TTL_SECONDS
    )
    live_search_reasoner_use_psg: bool = DEFAULT_LIVE_SEARCH_REASONER_USE_PSG
    live_search_reasoner_use_observation_memory: bool = (
        DEFAULT_LIVE_SEARCH_REASONER_USE_OBSERVATION_MEMORY
    )
    live_search_reasoner_use_llm_situated_prior: bool = (
        DEFAULT_LIVE_SEARCH_REASONER_USE_LLM_SITUATED_PRIOR
    )
    live_search_reasoner_debug_output_dir: str = (
        DEFAULT_LIVE_SEARCH_REASONER_DEBUG_OUTPUT_DIR
    )
    pandarxt16_enabled: bool = DEFAULT_PANDARXT16_ENABLED
    pandarxt16_diagnostic_preprocess_enabled: bool = (
        DEFAULT_PANDARXT16_DIAGNOSTIC_PREPROCESS_ENABLED
    )
    pandarxt16_zero_return_max_m: float = DEFAULT_PANDARXT16_ZERO_RETURN_MAX_M
    pandarxt16_clock_require_validated_for_metric: bool = (
        DEFAULT_PANDARXT16_CLOCK_REQUIRE_VALIDATED_FOR_METRIC
    )
    dual_lidar_safety_enabled: bool = DEFAULT_DUAL_LIDAR_SAFETY_ENABLED
    dual_lidar_require_validated_extrinsics: bool = (
        DEFAULT_DUAL_LIDAR_REQUIRE_VALIDATED_EXTRINSICS
    )
    dual_lidar_unknown_is_clear: bool = DEFAULT_DUAL_LIDAR_UNKNOWN_IS_CLEAR
    dual_lidar_max_evidence_age_seconds: float = (
        DEFAULT_DUAL_LIDAR_MAX_EVIDENCE_AGE_SECONDS
    )
    go2w_current_length_m: float = DEFAULT_GO2W_CURRENT_LENGTH_M
    go2w_current_width_m: float = DEFAULT_GO2W_CURRENT_WIDTH_M
    go2w_current_height_m: float = DEFAULT_GO2W_CURRENT_HEIGHT_M
    go2w_current_highest_point: str = DEFAULT_GO2W_CURRENT_HIGHEST_POINT
    go2w_current_geometry_config: str = DEFAULT_GO2W_CURRENT_GEOMETRY_CONFIG
    go2w_current_state_config: str = DEFAULT_GO2W_CURRENT_STATE_CONFIG
    go2w_pandar_preprocess_config: str = DEFAULT_GO2W_PANDAR_PREPROCESS_CONFIG

    @property
    def vision_model(self) -> str:
        """The model used for image/scene understanding.

        ``SILICONFLOW_MODEL`` remains a compatibility fallback for existing
        test fixtures and local launchers, but is never used by text planners.
        """
        return self.siliconflow_vision_model or self.siliconflow_model

    @property
    def reasoning_model(self) -> str:
        """The model used for text task understanding and planning."""
        return self.siliconflow_reasoning_model


def _project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _env_value(name: str, default: str | None = None) -> str | None:
    value = os.getenv(name)
    if value is None or value.strip() == "":
        return default
    return value.strip()


def _env_float(name: str, default: float) -> float:
    value = _env_value(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _env_int(name: str, default: int) -> int:
    value = _env_value(name)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        return default


def _env_bool(name: str, default: bool) -> bool:
    value = _env_value(name)
    if value is None:
        return default
    normalized = value.lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    return default


def _env_bool_alias(primary: str, legacy: str, default: bool) -> bool:
    return _env_bool(primary, _env_bool(legacy, default))


@lru_cache(maxsize=1)
def get_settings() -> Settings:
    """Load settings from `.env` and process environment variables."""

    load_dotenv(_project_root() / ".env")

    legacy_model = _env_value("SILICONFLOW_MODEL", DEFAULT_SILICONFLOW_MODEL)
    vision_model = _env_value("SILICONFLOW_VISION_MODEL", legacy_model)
    return Settings(
        siliconflow_api_key=_env_value("SILICONFLOW_API_KEY", ""),
        siliconflow_base_url=_env_value(
            "SILICONFLOW_BASE_URL", DEFAULT_SILICONFLOW_BASE_URL
        ),
        siliconflow_model=legacy_model,
        siliconflow_reasoning_model=_env_value(
            "SILICONFLOW_REASONING_MODEL", DEFAULT_SILICONFLOW_REASONING_MODEL
        ),
        siliconflow_vision_model=vision_model,
        output_dir=_env_value("OUTPUT_DIR", DEFAULT_OUTPUT_DIR),
        siliconflow_timeout_seconds=_env_float(
            "SILICONFLOW_TIMEOUT_SECONDS", DEFAULT_SILICONFLOW_TIMEOUT_SECONDS
        ),
        siliconflow_max_tokens=_env_int(
            "SILICONFLOW_MAX_TOKENS", DEFAULT_SILICONFLOW_MAX_TOKENS
        ),
        siliconflow_reasoning_timeout_seconds=_env_float(
            "SILICONFLOW_REASONING_TIMEOUT_SECONDS",
            DEFAULT_SILICONFLOW_REASONING_TIMEOUT_SECONDS,
        ),
        siliconflow_reasoning_max_tokens=_env_int(
            "SILICONFLOW_REASONING_MAX_TOKENS",
            DEFAULT_SILICONFLOW_REASONING_MAX_TOKENS,
        ),
        siliconflow_reasoning_effort=_env_value(
            "SILICONFLOW_REASONING_EFFORT", DEFAULT_SILICONFLOW_REASONING_EFFORT
        ),
        image_max_side=_env_int("IMAGE_MAX_SIDE", DEFAULT_IMAGE_MAX_SIDE),
        image_detail=_env_value("IMAGE_DETAIL", DEFAULT_IMAGE_DETAIL),
        enable_low_object_retry=_env_bool(
            "ENABLE_LOW_OBJECT_RETRY", DEFAULT_ENABLE_LOW_OBJECT_RETRY
        ),
        min_objects_for_complex_scene=_env_int(
            "MIN_OBJECTS_FOR_COMPLEX_SCENE", DEFAULT_MIN_OBJECTS_FOR_COMPLEX_SCENE
        ),
        enable_image_preprocess=_env_bool(
            "ENABLE_IMAGE_PREPROCESS", DEFAULT_ENABLE_IMAGE_PREPROCESS
        ),
        image_preprocess_sharpen=_env_bool(
            "IMAGE_PREPROCESS_SHARPEN", DEFAULT_IMAGE_PREPROCESS_SHARPEN
        ),
        image_preprocess_denoise=_env_bool(
            "IMAGE_PREPROCESS_DENOISE", DEFAULT_IMAGE_PREPROCESS_DENOISE
        ),
        enable_tiled_detection=_env_bool(
            "ENABLE_TILED_DETECTION", DEFAULT_ENABLE_TILED_DETECTION
        ),
        tile_size=_env_int("TILE_SIZE", DEFAULT_TILE_SIZE),
        tile_overlap=_env_int("TILE_OVERLAP", DEFAULT_TILE_OVERLAP),
        enable_target_profile=_env_bool(
            "ENABLE_TARGET_PROFILE", DEFAULT_ENABLE_TARGET_PROFILE
        ),
        target_profile_language=_env_value(
            "TARGET_PROFILE_LANGUAGE", DEFAULT_TARGET_PROFILE_LANGUAGE
        ),
        target_profile_max_terms=_env_int(
            "TARGET_PROFILE_MAX_TERMS", DEFAULT_TARGET_PROFILE_MAX_TERMS
        ),
        enable_target_synonym_expansion=_env_bool(
            "ENABLE_TARGET_SYNONYM_EXPANSION",
            DEFAULT_ENABLE_TARGET_SYNONYM_EXPANSION,
        ),
        enable_context_object_expansion=_env_bool(
            "ENABLE_CONTEXT_OBJECT_EXPANSION",
            DEFAULT_ENABLE_CONTEXT_OBJECT_EXPANSION,
        ),
        detection_backend=_env_value("DETECTION_BACKEND", DEFAULT_DETECTION_BACKEND),
        grounded_sam_root=_env_value("GROUNDED_SAM_ROOT", DEFAULT_GROUNDED_SAM_ROOT),
        grounded_sam_python=_env_value(
            "GROUNDED_SAM_PYTHON", DEFAULT_GROUNDED_SAM_PYTHON
        ),
        grounded_sam_pythonpath=_env_value(
            "GROUNDED_SAM_PYTHONPATH", DEFAULT_GROUNDED_SAM_PYTHONPATH
        ),
        grounding_dino_config=_env_value(
            "GROUNDING_DINO_CONFIG", DEFAULT_GROUNDING_DINO_CONFIG
        ),
        grounding_dino_checkpoint=_env_value(
            "GROUNDING_DINO_CHECKPOINT", DEFAULT_GROUNDING_DINO_CHECKPOINT
        ),
        grounding_dino_box_threshold=_env_float(
            "GROUNDING_DINO_BOX_THRESHOLD", DEFAULT_GROUNDING_DINO_BOX_THRESHOLD
        ),
        grounding_dino_text_threshold=_env_float(
            "GROUNDING_DINO_TEXT_THRESHOLD", DEFAULT_GROUNDING_DINO_TEXT_THRESHOLD
        ),
        grounding_dino_high_recall_box_threshold=_env_float(
            "GROUNDING_DINO_HIGH_RECALL_BOX_THRESHOLD",
            DEFAULT_GROUNDING_DINO_HIGH_RECALL_BOX_THRESHOLD,
        ),
        grounding_dino_high_recall_text_threshold=_env_float(
            "GROUNDING_DINO_HIGH_RECALL_TEXT_THRESHOLD",
            DEFAULT_GROUNDING_DINO_HIGH_RECALL_TEXT_THRESHOLD,
        ),
        enable_gdino_high_recall=_env_bool(
            "ENABLE_GDINO_HIGH_RECALL", DEFAULT_ENABLE_GDINO_HIGH_RECALL
        ),
        enable_sam2=_env_bool("ENABLE_SAM2", DEFAULT_ENABLE_SAM2),
        enable_sam2_mask_features=_env_bool(
            "ENABLE_SAM2_MASK_FEATURES", DEFAULT_ENABLE_SAM2_MASK_FEATURES
        ),
        sam2_config=_env_value("SAM2_CONFIG", DEFAULT_SAM2_CONFIG),
        sam2_checkpoint=_env_value("SAM2_CHECKPOINT", DEFAULT_SAM2_CHECKPOINT),
        max_detected_objects=_env_int("MAX_DETECTED_OBJECTS", DEFAULT_MAX_DETECTED_OBJECTS),
        detection_device=_env_value("DETECTION_DEVICE", DEFAULT_DETECTION_DEVICE),
        detector_timeout_seconds=_env_float(
            "DETECTOR_TIMEOUT_SECONDS", DEFAULT_DETECTOR_TIMEOUT_SECONDS
        ),
        enable_crop_verify=_env_bool(
            "ENABLE_CROP_VERIFY", DEFAULT_ENABLE_CROP_VERIFY
        ),
        crop_verify_backend=_env_value(
            "CROP_VERIFY_BACKEND", DEFAULT_CROP_VERIFY_BACKEND
        ),
        crop_verify_max_candidates=_env_int(
            "CROP_VERIFY_MAX_CANDIDATES", DEFAULT_CROP_VERIFY_MAX_CANDIDATES
        ),
        crop_verify_expand_ratio=_env_float(
            "CROP_VERIFY_EXPAND_RATIO", DEFAULT_CROP_VERIFY_EXPAND_RATIO
        ),
        crop_verify_min_score=_env_float(
            "CROP_VERIFY_MIN_SCORE", DEFAULT_CROP_VERIFY_MIN_SCORE
        ),
        crop_verify_target_score=_env_float(
            "CROP_VERIFY_TARGET_SCORE", DEFAULT_CROP_VERIFY_TARGET_SCORE
        ),
        crop_verify_timeout_seconds=_env_float(
            "CROP_VERIFY_TIMEOUT_SECONDS", DEFAULT_CROP_VERIFY_TIMEOUT_SECONDS
        ),
        crop_verify_save_crops=_env_bool(
            "CROP_VERIFY_SAVE_CROPS", DEFAULT_CROP_VERIFY_SAVE_CROPS
        ),
        crop_verify_output_dir=_env_value(
            "CROP_VERIFY_OUTPUT_DIR", DEFAULT_CROP_VERIFY_OUTPUT_DIR
        ),
        enable_score_fusion=_env_bool(
            "ENABLE_SCORE_FUSION", DEFAULT_ENABLE_SCORE_FUSION
        ),
        fusion_weight_detector=_env_float(
            "FUSION_WEIGHT_DETECTOR", DEFAULT_FUSION_WEIGHT_DETECTOR
        ),
        fusion_weight_vlm=_env_float(
            "FUSION_WEIGHT_VLM", DEFAULT_FUSION_WEIGHT_VLM
        ),
        fusion_weight_attribute=_env_float(
            "FUSION_WEIGHT_ATTRIBUTE", DEFAULT_FUSION_WEIGHT_ATTRIBUTE
        ),
        fusion_weight_context=_env_float(
            "FUSION_WEIGHT_CONTEXT", DEFAULT_FUSION_WEIGHT_CONTEXT
        ),
        final_target_score_threshold=_env_float(
            "FINAL_TARGET_SCORE_THRESHOLD", DEFAULT_FINAL_TARGET_SCORE_THRESHOLD
        ),
        final_candidate_score_threshold=_env_float(
            "FINAL_CANDIDATE_SCORE_THRESHOLD",
            DEFAULT_FINAL_CANDIDATE_SCORE_THRESHOLD,
        ),
        video_sample_fps=_env_float("VIDEO_SAMPLE_FPS", DEFAULT_VIDEO_SAMPLE_FPS),
        video_max_frames=_env_int("VIDEO_MAX_FRAMES", DEFAULT_VIDEO_MAX_FRAMES),
        video_enable_tracking=_env_bool(
            "VIDEO_ENABLE_TRACKING", DEFAULT_VIDEO_ENABLE_TRACKING
        ),
        video_track_iou_threshold=_env_float(
            "VIDEO_TRACK_IOU_THRESHOLD", DEFAULT_VIDEO_TRACK_IOU_THRESHOLD
        ),
        video_track_max_missing_frames=_env_int(
            "VIDEO_TRACK_MAX_MISSING_FRAMES",
            DEFAULT_VIDEO_TRACK_MAX_MISSING_FRAMES,
        ),
        video_track_min_hits=_env_int(
            "VIDEO_TRACK_MIN_HITS", DEFAULT_VIDEO_TRACK_MIN_HITS
        ),
        video_target_confirm_min_frames=_env_int(
            "VIDEO_TARGET_CONFIRM_MIN_FRAMES",
            DEFAULT_VIDEO_TARGET_CONFIRM_MIN_FRAMES,
        ),
        video_target_confirm_score=_env_float(
            "VIDEO_TARGET_CONFIRM_SCORE", DEFAULT_VIDEO_TARGET_CONFIRM_SCORE
        ),
        video_enable_track_level_voting=_env_bool(
            "VIDEO_ENABLE_TRACK_LEVEL_VOTING",
            DEFAULT_VIDEO_ENABLE_TRACK_LEVEL_VOTING,
        ),
        video_verify_every_n_frames=_env_int(
            "VIDEO_VERIFY_EVERY_N_FRAMES", DEFAULT_VIDEO_VERIFY_EVERY_N_FRAMES
        ),
        video_save_candidate_crops=_env_bool(
            "VIDEO_SAVE_CANDIDATE_CROPS", DEFAULT_VIDEO_SAVE_CANDIDATE_CROPS
        ),
        eval_iou_threshold=_env_float(
            "EVAL_IOU_THRESHOLD", DEFAULT_EVAL_IOU_THRESHOLD
        ),
        eval_output_dir=_env_value("EVAL_OUTPUT_DIR", DEFAULT_EVAL_OUTPUT_DIR),
        video_enable_scene_memory=_env_bool(
            "VIDEO_ENABLE_SCENE_MEMORY", DEFAULT_VIDEO_ENABLE_SCENE_MEMORY
        ),
        video_mode_default=_env_value("VIDEO_MODE_DEFAULT", DEFAULT_VIDEO_MODE_DEFAULT),
        video_enable_scene_mapping=_env_bool_alias(
            "VIDEO_ENABLE_SCENE_MAPPING_DEFAULT",
            "VIDEO_ENABLE_SCENE_MAPPING",
            DEFAULT_VIDEO_ENABLE_SCENE_MAPPING,
        ),
        video_full_scene_map_enabled=_env_bool(
            "VIDEO_FULL_SCENE_MAP_ENABLED", DEFAULT_VIDEO_FULL_SCENE_MAP_ENABLED
        ),
        video_use_scene_map_for_search=_env_bool_alias(
            "VIDEO_USE_SCENE_MAP_FOR_SEARCH_DEFAULT",
            "VIDEO_USE_SCENE_MAP_FOR_SEARCH",
            DEFAULT_VIDEO_USE_SCENE_MAP_FOR_SEARCH,
        ),
        video_allow_scene_map_only=_env_bool_alias(
            "VIDEO_ALLOW_SCENE_MAP_ONLY_DEBUG",
            "VIDEO_ALLOW_SCENE_MAP_ONLY",
            DEFAULT_VIDEO_ALLOW_SCENE_MAP_ONLY,
        ),
        video_target_search_required_when_target_present=_env_bool(
            "VIDEO_TARGET_SEARCH_REQUIRED_WHEN_TARGET_PRESENT",
            DEFAULT_VIDEO_TARGET_SEARCH_REQUIRED_WHEN_TARGET_PRESENT,
        ),
        video_scene_mapping_reuse_target_frames=_env_bool(
            "VIDEO_SCENE_MAPPING_REUSE_TARGET_FRAMES",
            DEFAULT_VIDEO_SCENE_MAPPING_REUSE_TARGET_FRAMES,
        ),
        video_scene_mapping_reuse_object_tracks=_env_bool(
            "VIDEO_SCENE_MAPPING_REUSE_OBJECT_TRACKS",
            DEFAULT_VIDEO_SCENE_MAPPING_REUSE_OBJECT_TRACKS,
        ),
        video_always_write_memory=_env_bool(
            "VIDEO_ALWAYS_WRITE_MEMORY", DEFAULT_VIDEO_ALWAYS_WRITE_MEMORY
        ),
        video_enable_video_psg=_env_bool(
            "VIDEO_ENABLE_VIDEO_PSG", DEFAULT_VIDEO_ENABLE_VIDEO_PSG
        ),
        video_enable_navigation_topology=_env_bool_alias(
            "VIDEO_ENABLE_NAVIGATION_TOPOLOGY_DEFAULT",
            "VIDEO_ENABLE_NAVIGATION_TOPOLOGY",
            DEFAULT_VIDEO_ENABLE_NAVIGATION_TOPOLOGY,
        ),
        video_topology_annotate_target_search=_env_bool(
            "VIDEO_TOPOLOGY_ANNOTATE_TARGET_SEARCH",
            DEFAULT_VIDEO_TOPOLOGY_ANNOTATE_TARGET_SEARCH,
        ),
        video_topology_add_target_candidate_nodes=_env_bool(
            "VIDEO_TOPOLOGY_ADD_TARGET_CANDIDATE_NODES",
            DEFAULT_VIDEO_TOPOLOGY_ADD_TARGET_CANDIDATE_NODES,
        ),
        video_topology_add_target_search_scores=_env_bool(
            "VIDEO_TOPOLOGY_ADD_TARGET_SEARCH_SCORES",
            DEFAULT_VIDEO_TOPOLOGY_ADD_TARGET_SEARCH_SCORES,
        ),
        video_psg_max_predicted_nodes=_env_int(
            "VIDEO_PSG_MAX_PREDICTED_NODES",
            DEFAULT_VIDEO_PSG_MAX_PREDICTED_NODES,
        ),
        video_psg_confidence_threshold=_env_float(
            "VIDEO_PSG_CONFIDENCE_THRESHOLD",
            DEFAULT_VIDEO_PSG_CONFIDENCE_THRESHOLD,
        ),
        video_topology_observed_only=_env_bool(
            "VIDEO_TOPOLOGY_OBSERVED_ONLY",
            DEFAULT_VIDEO_TOPOLOGY_OBSERVED_ONLY,
        ),
        video_save_frame_observations=_env_bool(
            "VIDEO_SAVE_FRAME_OBSERVATIONS",
            DEFAULT_VIDEO_SAVE_FRAME_OBSERVATIONS,
        ),
        video_enable_negative_evidence=_env_bool(
            "VIDEO_ENABLE_NEGATIVE_EVIDENCE", DEFAULT_VIDEO_ENABLE_NEGATIVE_EVIDENCE
        ),
        video_enable_reasoning_report=_env_bool(
            "VIDEO_ENABLE_REASONING_REPORT", DEFAULT_VIDEO_ENABLE_REASONING_REPORT
        ),
        video_memory_store_path=_env_value(
            "VIDEO_MEMORY_STORE_PATH", DEFAULT_VIDEO_MEMORY_STORE_PATH
        ),
        video_min_memory_importance=_env_value(
            "VIDEO_MIN_MEMORY_IMPORTANCE", DEFAULT_VIDEO_MIN_MEMORY_IMPORTANCE
        ),
        video_llm_frame_interval_sec=_env_float(
            "VIDEO_LLM_FRAME_INTERVAL_SEC", DEFAULT_VIDEO_LLM_FRAME_INTERVAL_SEC
        ),
        video_psg_summary_interval_sec=_env_float(
            "VIDEO_PSG_SUMMARY_INTERVAL_SEC", DEFAULT_VIDEO_PSG_SUMMARY_INTERVAL_SEC
        ),
        video_max_memory_entries_per_video=_env_int(
            "VIDEO_MAX_MEMORY_ENTRIES_PER_VIDEO",
            DEFAULT_VIDEO_MAX_MEMORY_ENTRIES_PER_VIDEO,
        ),
        video_memory_dedup_similarity=_env_float(
            "VIDEO_MEMORY_DEDUP_SIMILARITY", DEFAULT_VIDEO_MEMORY_DEDUP_SIMILARITY
        ),
        video_enable_memory_retrieval=_env_bool(
            "VIDEO_ENABLE_MEMORY_RETRIEVAL", DEFAULT_VIDEO_ENABLE_MEMORY_RETRIEVAL
        ),
        video_memory_retrieval_top_k=_env_int(
            "VIDEO_MEMORY_RETRIEVAL_TOP_K", DEFAULT_VIDEO_MEMORY_RETRIEVAL_TOP_K
        ),
        video_scene_reasoner_backend=_env_value(
            "VIDEO_SCENE_REASONER_BACKEND", DEFAULT_VIDEO_SCENE_REASONER_BACKEND
        ),
        video_force_json_output=_env_bool(
            "VIDEO_FORCE_JSON_OUTPUT", DEFAULT_VIDEO_FORCE_JSON_OUTPUT
        ),
        video_navigation_enabled=_env_bool(
            "VIDEO_NAVIGATION_ENABLED", DEFAULT_VIDEO_NAVIGATION_ENABLED
        ),
        video_navigation_mode=_env_value(
            "VIDEO_NAVIGATION_MODE", DEFAULT_VIDEO_NAVIGATION_MODE
        ),
        video_pose_backend=_env_value("VIDEO_POSE_BACKEND", DEFAULT_VIDEO_POSE_BACKEND),
        video_pose_allow_relative=_env_bool(
            "VIDEO_POSE_ALLOW_RELATIVE", DEFAULT_VIDEO_POSE_ALLOW_RELATIVE
        ),
        video_pose_require_metric_for_nav2=_env_bool(
            "VIDEO_POSE_REQUIRE_METRIC_FOR_NAV2",
            DEFAULT_VIDEO_POSE_REQUIRE_METRIC_FOR_NAV2,
        ),
        video_navigation_auto_plan=_env_bool(
            "VIDEO_NAVIGATION_AUTO_PLAN", DEFAULT_VIDEO_NAVIGATION_AUTO_PLAN
        ),
        video_navigation_auto_exploration=_env_bool(
            "VIDEO_NAVIGATION_AUTO_EXPLORATION",
            DEFAULT_VIDEO_NAVIGATION_AUTO_EXPLORATION,
        ),
        video_navigation_max_frames=_env_int(
            "VIDEO_NAVIGATION_MAX_FRAMES", DEFAULT_VIDEO_NAVIGATION_MAX_FRAMES
        ),
        video_navigation_frame_sample_interval=_env_int(
            "VIDEO_NAVIGATION_FRAME_SAMPLE_INTERVAL",
            DEFAULT_VIDEO_NAVIGATION_FRAME_SAMPLE_INTERVAL,
        ),
        video_navigation_min_track_confidence=_env_float(
            "VIDEO_NAVIGATION_MIN_TRACK_CONFIDENCE",
            DEFAULT_VIDEO_NAVIGATION_MIN_TRACK_CONFIDENCE,
        ),
        video_navigation_target_observation_distance=_env_float(
            "VIDEO_NAVIGATION_TARGET_OBSERVATION_DISTANCE",
            DEFAULT_VIDEO_NAVIGATION_TARGET_OBSERVATION_DISTANCE,
        ),
        video_navigation_enable_frontier_exploration=_env_bool(
            "VIDEO_NAVIGATION_ENABLE_FRONTIER_EXPLORATION",
            DEFAULT_VIDEO_NAVIGATION_ENABLE_FRONTIER_EXPLORATION,
        ),
        video_navigation_exploration_max_candidates=_env_int(
            "VIDEO_NAVIGATION_EXPLORATION_MAX_CANDIDATES",
            DEFAULT_VIDEO_NAVIGATION_EXPLORATION_MAX_CANDIDATES,
        ),
        video_navigation_exploration_min_information_gain=_env_float(
            "VIDEO_NAVIGATION_EXPLORATION_MIN_INFORMATION_GAIN",
            DEFAULT_VIDEO_NAVIGATION_EXPLORATION_MIN_INFORMATION_GAIN,
        ),
        video_navigation_allow_nav2_from_metric_video=_env_bool(
            "VIDEO_NAVIGATION_ALLOW_NAV2_FROM_METRIC_VIDEO",
            DEFAULT_VIDEO_NAVIGATION_ALLOW_NAV2_FROM_METRIC_VIDEO,
        ),
        visual_nav_execution_enabled=_env_bool(
            "VISUAL_NAV_EXECUTION_ENABLED", DEFAULT_VISUAL_NAV_EXECUTION_ENABLED
        ),
        enable_llm_situated_reasoning=_env_bool(
            "ENABLE_LLM_SITUATED_REASONING",
            DEFAULT_ENABLE_LLM_SITUATED_REASONING,
        ),
        enable_llm_reasoning_memory=_env_bool(
            "ENABLE_LLM_REASONING_MEMORY", DEFAULT_ENABLE_LLM_REASONING_MEMORY
        ),
        llm_reasoning_max_hypotheses=_env_int(
            "LLM_REASONING_MAX_HYPOTHESES",
            DEFAULT_LLM_REASONING_MAX_HYPOTHESES,
        ),
        llm_reasoning_timeout_seconds=_env_float(
            "LLM_REASONING_TIMEOUT_SECONDS",
            DEFAULT_LLM_REASONING_TIMEOUT_SECONDS,
        ),
        llm_reasoning_temperature=_env_float(
            "LLM_REASONING_TEMPERATURE", DEFAULT_LLM_REASONING_TEMPERATURE
        ),
        llm_reasoning_require_actionability_gate=_env_bool(
            "LLM_REASONING_REQUIRE_ACTIONABILITY_GATE",
            DEFAULT_LLM_REASONING_REQUIRE_ACTIONABILITY_GATE,
        ),
        llm_reasoning_require_visual_gate=_env_bool(
            "LLM_REASONING_REQUIRE_VISUAL_GATE",
            DEFAULT_LLM_REASONING_REQUIRE_VISUAL_GATE,
        ),
        quadruped_max_forward_step_m=_env_float(
            "QUADRUPED_MAX_FORWARD_STEP_M", DEFAULT_QUADRUPED_MAX_FORWARD_STEP_M
        ),
        quadruped_can_manipulate=_env_bool(
            "QUADRUPED_CAN_MANIPULATE", DEFAULT_QUADRUPED_CAN_MANIPULATE
        ),
        quadruped_can_open_container=_env_bool(
            "QUADRUPED_CAN_OPEN_CONTAINER", DEFAULT_QUADRUPED_CAN_OPEN_CONTAINER
        ),
        quadruped_can_look_down=_env_bool(
            "QUADRUPED_CAN_LOOK_DOWN", DEFAULT_QUADRUPED_CAN_LOOK_DOWN
        ),
        llm_experience_memory_path=_env_value(
            "LLM_EXPERIENCE_MEMORY_PATH", DEFAULT_LLM_EXPERIENCE_MEMORY_PATH
        ),
        enable_llm_dynamic_visual_retry=_env_bool(
            "ENABLE_LLM_DYNAMIC_VISUAL_RETRY",
            DEFAULT_ENABLE_LLM_DYNAMIC_VISUAL_RETRY,
        ),
        llm_dynamic_visual_retry_max_terms=_env_int(
            "LLM_DYNAMIC_VISUAL_RETRY_MAX_TERMS",
            DEFAULT_LLM_DYNAMIC_VISUAL_RETRY_MAX_TERMS,
        ),
        platform_obstacle_avoidance_assumed=_env_bool(
            "PLATFORM_OBSTACLE_AVOIDANCE_ASSUMED",
            DEFAULT_PLATFORM_OBSTACLE_AVOIDANCE_ASSUMED,
        ),
        enable_dynamic_motion_horizon=_env_bool(
            "ENABLE_DYNAMIC_MOTION_HORIZON",
            DEFAULT_ENABLE_DYNAMIC_MOTION_HORIZON,
        ),
        motion_horizon_profile=_env_value(
            "MOTION_HORIZON_PROFILE",
            DEFAULT_MOTION_HORIZON_PROFILE,
        ),
        motion_strict_safe_max_step_m=_env_float(
            "MOTION_STRICT_SAFE_MAX_STEP_M",
            DEFAULT_MOTION_STRICT_SAFE_MAX_STEP_M,
        ),
        motion_platform_indoor_default_step_m=_env_float(
            "MOTION_PLATFORM_INDOOR_DEFAULT_STEP_M",
            DEFAULT_MOTION_PLATFORM_INDOOR_DEFAULT_STEP_M,
        ),
        motion_platform_indoor_max_step_m=_env_float(
            "MOTION_PLATFORM_INDOOR_MAX_STEP_M",
            DEFAULT_MOTION_PLATFORM_INDOOR_MAX_STEP_M,
        ),
        motion_platform_open_default_step_m=_env_float(
            "MOTION_PLATFORM_OPEN_DEFAULT_STEP_M",
            DEFAULT_MOTION_PLATFORM_OPEN_DEFAULT_STEP_M,
        ),
        motion_platform_open_max_step_m=_env_float(
            "MOTION_PLATFORM_OPEN_MAX_STEP_M",
            DEFAULT_MOTION_PLATFORM_OPEN_MAX_STEP_M,
        ),
        motion_absolute_max_step_m=_env_float(
            "MOTION_ABSOLUTE_MAX_STEP_M",
            DEFAULT_MOTION_ABSOLUTE_MAX_STEP_M,
        ),
        motion_target_confirm_max_step_m=_env_float(
            "MOTION_TARGET_CONFIRM_MAX_STEP_M",
            DEFAULT_MOTION_TARGET_CONFIRM_MAX_STEP_M,
        ),
        motion_platform_fallback_step_m=_env_float(
            "MOTION_PLATFORM_FALLBACK_STEP_M",
            DEFAULT_MOTION_PLATFORM_FALLBACK_STEP_M,
        ),
        motion_default_stop_and_reobserve=_env_bool(
            "MOTION_DEFAULT_STOP_AND_REOBSERVE",
            DEFAULT_MOTION_DEFAULT_STOP_AND_REOBSERVE,
        ),
        motion_enable_observe_while_moving=_env_bool(
            "MOTION_ENABLE_OBSERVE_WHILE_MOVING",
            DEFAULT_MOTION_ENABLE_OBSERVE_WHILE_MOVING,
        ),
        motion_soft_observe_interval_sec=_env_float(
            "MOTION_SOFT_OBSERVE_INTERVAL_SEC",
            DEFAULT_MOTION_SOFT_OBSERVE_INTERVAL_SEC,
        ),
        motion_shorten_on_target_candidate=_env_bool(
            "MOTION_SHORTEN_ON_TARGET_CANDIDATE",
            DEFAULT_MOTION_SHORTEN_ON_TARGET_CANDIDATE,
        ),
        motion_allow_llm_recommended_horizon=_env_bool(
            "MOTION_ALLOW_LLM_RECOMMENDED_HORIZON",
            DEFAULT_MOTION_ALLOW_LLM_RECOMMENDED_HORIZON,
        ),
        motion_llm_horizon_weight=_env_float(
            "MOTION_LLM_HORIZON_WEIGHT",
            DEFAULT_MOTION_LLM_HORIZON_WEIGHT,
        ),
        static_knowledge_base_enabled=_env_bool(
            "STATIC_KNOWLEDGE_BASE_ENABLED",
            DEFAULT_STATIC_KNOWLEDGE_BASE_ENABLED,
        ),
        handwritten_object_priors_enabled=_env_bool(
            "HANDWRITTEN_OBJECT_PRIORS_ENABLED",
            DEFAULT_HANDWRITTEN_OBJECT_PRIORS_ENABLED,
        ),
        handwritten_location_priors_enabled=_env_bool(
            "HANDWRITTEN_LOCATION_PRIORS_ENABLED",
            DEFAULT_HANDWRITTEN_LOCATION_PRIORS_ENABLED,
        ),
        handwritten_room_priors_enabled=_env_bool(
            "HANDWRITTEN_ROOM_PRIORS_ENABLED",
            DEFAULT_HANDWRITTEN_ROOM_PRIORS_ENABLED,
        ),
        static_object_prompts_enabled=_env_bool(
            "STATIC_OBJECT_PROMPTS_ENABLED",
            DEFAULT_STATIC_OBJECT_PROMPTS_ENABLED,
        ),
        allow_handcrafted_search_rules=_env_bool(
            "ALLOW_HANDCRAFTED_SEARCH_RULES",
            DEFAULT_ALLOW_HANDCRAFTED_SEARCH_RULES,
        ),
        llm_commonsense_prior_enabled=_env_bool(
            "LLM_COMMONSENSE_PRIOR_ENABLED",
            DEFAULT_LLM_COMMONSENSE_PRIOR_ENABLED,
        ),
        llm_prior_generation_mode=_env_value(
            "LLM_PRIOR_GENERATION_MODE",
            DEFAULT_LLM_PRIOR_GENERATION_MODE,
        ),
        llm_prior_require_reason=_env_bool(
            "LLM_PRIOR_REQUIRE_REASON",
            DEFAULT_LLM_PRIOR_REQUIRE_REASON,
        ),
        llm_prior_allow_search_hints=_env_bool(
            "LLM_PRIOR_ALLOW_SEARCH_HINTS",
            DEFAULT_LLM_PRIOR_ALLOW_SEARCH_HINTS,
        ),
        llm_prior_allow_detector_prompts=_env_bool(
            "LLM_PRIOR_ALLOW_DETECTOR_PROMPTS",
            DEFAULT_LLM_PRIOR_ALLOW_DETECTOR_PROMPTS,
        ),
        llm_prior_can_confirm_target=_env_bool(
            "LLM_PRIOR_CAN_CONFIRM_TARGET",
            DEFAULT_LLM_PRIOR_CAN_CONFIRM_TARGET,
        ),
        llm_prior_max_hypotheses=_env_int(
            "LLM_PRIOR_MAX_HYPOTHESES",
            DEFAULT_LLM_PRIOR_MAX_HYPOTHESES,
        ),
        llm_prior_max_detector_prompts=_env_int(
            "LLM_PRIOR_MAX_DETECTOR_PROMPTS",
            DEFAULT_LLM_PRIOR_MAX_DETECTOR_PROMPTS,
        ),
        llm_prior_output_language=_env_value(
            "LLM_PRIOR_OUTPUT_LANGUAGE",
            DEFAULT_LLM_PRIOR_OUTPUT_LANGUAGE,
        ),
        evidence_gating_enabled=_env_bool(
            "EVIDENCE_GATING_ENABLED",
            DEFAULT_EVIDENCE_GATING_ENABLED,
        ),
        video_target_confirmation_by_context_only=_env_bool(
            "VIDEO_TARGET_CONFIRMATION_BY_CONTEXT_ONLY",
            DEFAULT_VIDEO_TARGET_CONFIRMATION_BY_CONTEXT_ONLY,
        ),
        target_confirmation_require_visual_evidence=_env_bool(
            "TARGET_CONFIRMATION_REQUIRE_VISUAL_EVIDENCE",
            DEFAULT_TARGET_CONFIRMATION_REQUIRE_VISUAL_EVIDENCE,
        ),
        target_confirmation_require_bbox=_env_bool(
            "TARGET_CONFIRMATION_REQUIRE_BBOX",
            DEFAULT_TARGET_CONFIRMATION_REQUIRE_BBOX,
        ),
        target_confirmation_require_crop_verify=_env_bool(
            "TARGET_CONFIRMATION_REQUIRE_CROP_VERIFY",
            DEFAULT_TARGET_CONFIRMATION_REQUIRE_CROP_VERIFY,
        ),
        target_confirmation_require_mask=_env_bool(
            "TARGET_CONFIRMATION_REQUIRE_MASK",
            DEFAULT_TARGET_CONFIRMATION_REQUIRE_MASK,
        ),
        target_confirmation_min_score=_env_float(
            "TARGET_CONFIRMATION_MIN_SCORE",
            DEFAULT_TARGET_CONFIRMATION_MIN_SCORE,
        ),
        target_inferred_is_not_found=_env_bool(
            "TARGET_INFERRED_IS_NOT_FOUND",
            DEFAULT_TARGET_INFERRED_IS_NOT_FOUND,
        ),
        video_search_ranker_enabled=_env_bool(
            "VIDEO_SEARCH_RANKER_ENABLED", DEFAULT_VIDEO_SEARCH_RANKER_ENABLED
        ),
        video_search_ranker_high_score_threshold=_env_float(
            "VIDEO_SEARCH_RANKER_HIGH_SCORE_THRESHOLD",
            DEFAULT_VIDEO_SEARCH_RANKER_HIGH_SCORE_THRESHOLD,
        ),
        video_search_ranker_context_only_can_confirm=_env_bool(
            "VIDEO_SEARCH_RANKER_CONTEXT_ONLY_CAN_CONFIRM",
            DEFAULT_VIDEO_SEARCH_RANKER_CONTEXT_ONLY_CAN_CONFIRM,
        ),
        observation_memory_enabled=_env_bool(
            "OBSERVATION_MEMORY_ENABLED",
            DEFAULT_OBSERVATION_MEMORY_ENABLED,
        ),
        observation_memory_store_path=_env_value(
            "OBSERVATION_MEMORY_STORE_PATH",
            DEFAULT_OBSERVATION_MEMORY_STORE_PATH,
        ),
        observation_memory_write_visual_only=_env_bool(
            "OBSERVATION_MEMORY_WRITE_VISUAL_ONLY",
            DEFAULT_OBSERVATION_MEMORY_WRITE_VISUAL_ONLY,
        ),
        observation_memory_allow_llm_summary=_env_bool(
            "OBSERVATION_MEMORY_ALLOW_LLM_SUMMARY",
            DEFAULT_OBSERVATION_MEMORY_ALLOW_LLM_SUMMARY,
        ),
        observation_memory_llm_summary_as_hypothesis=_env_bool(
            "OBSERVATION_MEMORY_LLM_SUMMARY_AS_HYPOTHESIS",
            DEFAULT_OBSERVATION_MEMORY_LLM_SUMMARY_AS_HYPOTHESIS,
        ),
        observation_memory_retrieval_top_k=_env_int(
            "OBSERVATION_MEMORY_RETRIEVAL_TOP_K",
            DEFAULT_OBSERVATION_MEMORY_RETRIEVAL_TOP_K,
        ),
        observation_memory_require_provenance=_env_bool(
            "OBSERVATION_MEMORY_REQUIRE_PROVENANCE",
            DEFAULT_OBSERVATION_MEMORY_REQUIRE_PROVENANCE,
        ),
        prior_usage_audit_enabled=_env_bool(
            "PRIOR_USAGE_AUDIT_ENABLED",
            DEFAULT_PRIOR_USAGE_AUDIT_ENABLED,
        ),
        prior_usage_report_path=_env_value(
            "PRIOR_USAGE_REPORT_PATH",
            DEFAULT_PRIOR_USAGE_REPORT_PATH,
        ),
        grounding_prompt_llm_expansion_enabled=_env_bool(
            "GROUNDING_PROMPT_LLM_EXPANSION_ENABLED",
            DEFAULT_GROUNDING_PROMPT_LLM_EXPANSION_ENABLED,
        ),
        grounding_prompt_require_non_empty=_env_bool(
            "GROUNDING_PROMPT_REQUIRE_NON_EMPTY",
            DEFAULT_GROUNDING_PROMPT_REQUIRE_NON_EMPTY,
        ),
        grounding_prompt_fail_fast_on_empty=_env_bool(
            "GROUNDING_PROMPT_FAIL_FAST_ON_EMPTY",
            DEFAULT_GROUNDING_PROMPT_FAIL_FAST_ON_EMPTY,
        ),
        grounding_prompt_retry_on_empty=_env_bool(
            "GROUNDING_PROMPT_RETRY_ON_EMPTY",
            DEFAULT_GROUNDING_PROMPT_RETRY_ON_EMPTY,
        ),
        grounding_prompt_max_retries=_env_int(
            "GROUNDING_PROMPT_MAX_RETRIES",
            DEFAULT_GROUNDING_PROMPT_MAX_RETRIES,
        ),
        grounding_prompt_max_terms=_env_int(
            "GROUNDING_PROMPT_MAX_TERMS",
            DEFAULT_GROUNDING_PROMPT_MAX_TERMS,
        ),
        grounding_prompt_min_terms=_env_int(
            "GROUNDING_PROMPT_MIN_TERMS",
            DEFAULT_GROUNDING_PROMPT_MIN_TERMS,
        ),
        grounding_prompt_debug_output=_env_value(
            "GROUNDING_PROMPT_DEBUG_OUTPUT",
            DEFAULT_GROUNDING_PROMPT_DEBUG_OUTPUT,
        ),
        grounding_prompt_retry_debug_output=_env_value(
            "GROUNDING_PROMPT_RETRY_DEBUG_OUTPUT",
            DEFAULT_GROUNDING_PROMPT_RETRY_DEBUG_OUTPUT,
        ),
        live_search_semantic_reasoning_enabled=_env_bool(
            "LIVE_SEARCH_SEMANTIC_REASONING_ENABLED",
            DEFAULT_LIVE_SEARCH_SEMANTIC_REASONING_ENABLED,
        ),
        live_search_reasoner_backend=_env_value(
            "LIVE_SEARCH_REASONER_BACKEND", DEFAULT_LIVE_SEARCH_REASONER_BACKEND
        ),
        live_search_reasoner_mode=_env_value(
            "LIVE_SEARCH_REASONER_MODE", DEFAULT_LIVE_SEARCH_REASONER_MODE
        ),
        live_search_reasoner_min_confidence=_env_float(
            "LIVE_SEARCH_REASONER_MIN_CONFIDENCE",
            DEFAULT_LIVE_SEARCH_REASONER_MIN_CONFIDENCE,
        ),
        live_search_reasoner_allow_forward=_env_bool(
            "LIVE_SEARCH_REASONER_ALLOW_FORWARD",
            DEFAULT_LIVE_SEARCH_REASONER_ALLOW_FORWARD,
        ),
        live_search_reasoner_max_turn_deg=_env_float(
            "LIVE_SEARCH_REASONER_MAX_TURN_DEG",
            DEFAULT_LIVE_SEARCH_REASONER_MAX_TURN_DEG,
        ),
        live_search_reasoner_min_replan_seconds=_env_float(
            "LIVE_SEARCH_REASONER_MIN_REPLAN_SECONDS",
            DEFAULT_LIVE_SEARCH_REASONER_MIN_REPLAN_SECONDS,
        ),
        live_search_reasoner_scene_ttl_seconds=_env_float(
            "LIVE_SEARCH_REASONER_SCENE_TTL_SECONDS",
            DEFAULT_LIVE_SEARCH_REASONER_SCENE_TTL_SECONDS,
        ),
        vlm_runtime_transport=_env_value("VLM_RUNTIME_TRANSPORT", DEFAULT_VLM_RUNTIME_TRANSPORT),
        vlm_runtime_fallback_to_subprocess=_env_bool(
            "VLM_RUNTIME_FALLBACK_TO_SUBPROCESS",
            DEFAULT_VLM_RUNTIME_FALLBACK_TO_SUBPROCESS,
        ),
        vlm_runtime_allow_api_concurrency=_env_bool(
            "VLM_RUNTIME_ALLOW_API_CONCURRENCY",
            DEFAULT_VLM_RUNTIME_ALLOW_API_CONCURRENCY,
        ),
        vlm_runtime_quick_max_tokens=_env_int(
            "VLM_RUNTIME_QUICK_MAX_TOKENS", DEFAULT_VLM_RUNTIME_QUICK_MAX_TOKENS
        ),
        vlm_runtime_verify_max_tokens=_env_int(
            "VLM_RUNTIME_VERIFY_MAX_TOKENS", DEFAULT_VLM_RUNTIME_VERIFY_MAX_TOKENS
        ),
        vlm_runtime_semantic_max_tokens=_env_int(
            "VLM_RUNTIME_SEMANTIC_MAX_TOKENS", DEFAULT_VLM_RUNTIME_SEMANTIC_MAX_TOKENS
        ),
        vlm_runtime_quick_timeout_seconds=_env_float(
            "VLM_RUNTIME_QUICK_TIMEOUT_SECONDS",
            DEFAULT_VLM_RUNTIME_QUICK_TIMEOUT_SECONDS,
        ),
        vlm_runtime_verify_timeout_seconds=_env_float(
            "VLM_RUNTIME_VERIFY_TIMEOUT_SECONDS",
            DEFAULT_VLM_RUNTIME_VERIFY_TIMEOUT_SECONDS,
        ),
        vlm_runtime_semantic_timeout_seconds=_env_float(
            "VLM_RUNTIME_SEMANTIC_TIMEOUT_SECONDS",
            DEFAULT_VLM_RUNTIME_SEMANTIC_TIMEOUT_SECONDS,
        ),
        vlm_runtime_semantic_background_enabled=_env_bool(
            "VLM_RUNTIME_SEMANTIC_BACKGROUND_ENABLED",
            DEFAULT_VLM_RUNTIME_SEMANTIC_BACKGROUND_ENABLED,
        ),
        vlm_runtime_semantic_initial_warmup_blocking=_env_bool(
            "VLM_RUNTIME_SEMANTIC_INITIAL_WARMUP_BLOCKING",
            DEFAULT_VLM_RUNTIME_SEMANTIC_INITIAL_WARMUP_BLOCKING,
        ),
        vlm_runtime_semantic_max_inflight=_env_int(
            "VLM_RUNTIME_SEMANTIC_MAX_INFLIGHT", DEFAULT_VLM_RUNTIME_SEMANTIC_MAX_INFLIGHT
        ),
        vlm_runtime_semantic_ttl_seconds=_env_float(
            "VLM_RUNTIME_SEMANTIC_TTL_SECONDS", DEFAULT_VLM_RUNTIME_SEMANTIC_TTL_SECONDS
        ),
        vlm_runtime_semantic_translation_refresh_m=_env_float(
            "VLM_RUNTIME_SEMANTIC_TRANSLATION_REFRESH_M",
            DEFAULT_VLM_RUNTIME_SEMANTIC_TRANSLATION_REFRESH_M,
        ),
        vlm_runtime_semantic_heading_sector_deg=_env_float(
            "VLM_RUNTIME_SEMANTIC_HEADING_SECTOR_DEG",
            DEFAULT_VLM_RUNTIME_SEMANTIC_HEADING_SECTOR_DEG,
        ),
        vlm_runtime_semantic_visual_change_enabled=_env_bool(
            "VLM_RUNTIME_SEMANTIC_VISUAL_CHANGE_ENABLED",
            DEFAULT_VLM_RUNTIME_SEMANTIC_VISUAL_CHANGE_ENABLED,
        ),
        vlm_runtime_planner_semantic_soft_stale_seconds=_env_float(
            "VLM_RUNTIME_PLANNER_SEMANTIC_SOFT_STALE_SECONDS",
            DEFAULT_VLM_RUNTIME_PLANNER_SEMANTIC_SOFT_STALE_SECONDS,
        ),
        vlm_runtime_planner_semantic_hard_stale_seconds=_env_float(
            "VLM_RUNTIME_PLANNER_SEMANTIC_HARD_STALE_SECONDS",
            DEFAULT_VLM_RUNTIME_PLANNER_SEMANTIC_HARD_STALE_SECONDS,
        ),
        vlm_runtime_verify_same_frame_max_calls=_env_int(
            "VLM_RUNTIME_VERIFY_SAME_FRAME_MAX_CALLS",
            DEFAULT_VLM_RUNTIME_VERIFY_SAME_FRAME_MAX_CALLS,
        ),
        live_search_graph_match_partial_threshold=_env_float(
            "LIVE_SEARCH_GRAPH_MATCH_PARTIAL_THRESHOLD",
            DEFAULT_LIVE_SEARCH_GRAPH_MATCH_PARTIAL_THRESHOLD,
        ),
        live_search_graph_match_strong_threshold=_env_float(
            "LIVE_SEARCH_GRAPH_MATCH_STRONG_THRESHOLD",
            DEFAULT_LIVE_SEARCH_GRAPH_MATCH_STRONG_THRESHOLD,
        ),
        live_search_negative_memory_enabled=_env_bool(
            "LIVE_SEARCH_NEGATIVE_MEMORY_ENABLED",
            DEFAULT_LIVE_SEARCH_NEGATIVE_MEMORY_ENABLED,
        ),
        live_search_negative_memory_ttl_seconds=_env_float(
            "LIVE_SEARCH_NEGATIVE_MEMORY_TTL_SECONDS",
            DEFAULT_LIVE_SEARCH_NEGATIVE_MEMORY_TTL_SECONDS,
        ),
        live_search_reasoner_use_psg=_env_bool(
            "LIVE_SEARCH_REASONER_USE_PSG", DEFAULT_LIVE_SEARCH_REASONER_USE_PSG
        ),
        live_search_reasoner_use_observation_memory=_env_bool(
            "LIVE_SEARCH_REASONER_USE_OBSERVATION_MEMORY",
            DEFAULT_LIVE_SEARCH_REASONER_USE_OBSERVATION_MEMORY,
        ),
        live_search_reasoner_use_llm_situated_prior=_env_bool(
            "LIVE_SEARCH_REASONER_USE_LLM_SITUATED_PRIOR",
            DEFAULT_LIVE_SEARCH_REASONER_USE_LLM_SITUATED_PRIOR,
        ),
        live_search_reasoner_debug_output_dir=_env_value(
            "LIVE_SEARCH_REASONER_DEBUG_OUTPUT_DIR",
            DEFAULT_LIVE_SEARCH_REASONER_DEBUG_OUTPUT_DIR,
        ),
        pandarxt16_enabled=_env_bool(
            "PANDARXT16_ENABLED", DEFAULT_PANDARXT16_ENABLED
        ),
        pandarxt16_diagnostic_preprocess_enabled=_env_bool(
            "PANDARXT16_DIAGNOSTIC_PREPROCESS_ENABLED",
            DEFAULT_PANDARXT16_DIAGNOSTIC_PREPROCESS_ENABLED,
        ),
        pandarxt16_zero_return_max_m=_env_float(
            "PANDARXT16_ZERO_RETURN_MAX_M", DEFAULT_PANDARXT16_ZERO_RETURN_MAX_M
        ),
        pandarxt16_clock_require_validated_for_metric=_env_bool(
            "PANDARXT16_CLOCK_REQUIRE_VALIDATED_FOR_METRIC",
            DEFAULT_PANDARXT16_CLOCK_REQUIRE_VALIDATED_FOR_METRIC,
        ),
        dual_lidar_safety_enabled=_env_bool(
            "DUAL_LIDAR_SAFETY_ENABLED", DEFAULT_DUAL_LIDAR_SAFETY_ENABLED
        ),
        dual_lidar_require_validated_extrinsics=_env_bool(
            "DUAL_LIDAR_REQUIRE_VALIDATED_EXTRINSICS",
            DEFAULT_DUAL_LIDAR_REQUIRE_VALIDATED_EXTRINSICS,
        ),
        dual_lidar_unknown_is_clear=_env_bool(
            "DUAL_LIDAR_UNKNOWN_IS_CLEAR", DEFAULT_DUAL_LIDAR_UNKNOWN_IS_CLEAR
        ),
        dual_lidar_max_evidence_age_seconds=_env_float(
            "DUAL_LIDAR_MAX_EVIDENCE_AGE_SECONDS",
            DEFAULT_DUAL_LIDAR_MAX_EVIDENCE_AGE_SECONDS,
        ),
        go2w_current_length_m=_env_float(
            "GO2W_CURRENT_LENGTH_M", DEFAULT_GO2W_CURRENT_LENGTH_M
        ),
        go2w_current_width_m=_env_float(
            "GO2W_CURRENT_WIDTH_M", DEFAULT_GO2W_CURRENT_WIDTH_M
        ),
        go2w_current_height_m=_env_float(
            "GO2W_CURRENT_HEIGHT_M", DEFAULT_GO2W_CURRENT_HEIGHT_M
        ),
        go2w_current_highest_point=_env_value(
            "GO2W_CURRENT_HIGHEST_POINT", DEFAULT_GO2W_CURRENT_HIGHEST_POINT
        ),
        go2w_current_geometry_config=_env_value(
            "GO2W_CURRENT_GEOMETRY_CONFIG", DEFAULT_GO2W_CURRENT_GEOMETRY_CONFIG
        ),
        go2w_current_state_config=_env_value(
            "GO2W_CURRENT_STATE_CONFIG", DEFAULT_GO2W_CURRENT_STATE_CONFIG
        ),
        go2w_pandar_preprocess_config=_env_value(
            "GO2W_PANDAR_PREPROCESS_CONFIG", DEFAULT_GO2W_PANDAR_PREPROCESS_CONFIG
        ),
    )
