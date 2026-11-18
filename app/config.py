"""Application configuration loaded from environment variables."""

from __future__ import annotations

import os
import sys
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
DEFAULT_YOLOE_PYTHON = str(
    Path(__file__).resolve().parents[1] / ".yoloe_env" / "bin" / "python"
)
DEFAULT_YOLOE_MODEL = str(
    Path(__file__).resolve().parents[1] / "models" / "yoloe-26s-seg-pf.pt"
)
DEFAULT_YOLOE_TEXT_ENCODER = str(
    Path(__file__).resolve().parents[1] / "mobileclip2_b.ts"
)
DEFAULT_YOLOE_DEVICE = "cuda:0"
DEFAULT_YOLOE_PRECISION = "fp16"
DEFAULT_YOLOE_CONFIDENCE = 0.25
DEFAULT_YOLOE_IOU_THRESHOLD = 0.50
DEFAULT_YOLOE_IMAGE_SIZE = 640
DEFAULT_YOLOE_MAX_DETECTIONS = 100
DEFAULT_YOLOE_TIMEOUT_SECONDS = 30.0
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
# 轻量化改造计划 §20：只有过了 GroundingDINO 基础门槛的 bbox 才跑 SAM2。
# 注意它**高于** DEFAULT_GROUNDING_DINO_BOX_THRESHOLD(0.10)：0.10~0.35 的框仍是
# 物体（保留 2D 语义），只是不做 mask 精修 —— worker 侧对 mask=None 已有分支
# （refined_box 用原框、mask_rle/mask_area_ratio 留空），不需要额外处理。
DEFAULT_SAM2_MIN_BOX_SCORE = 0.35
# 感知提速计划（2026-10-13）阶段一的三个开关。**默认全开**是验收之后的结论，
# 不是先验假设：等价性（固定输入上对象 JSON 逐字节相同）与性能都实测过，
# 见 `reports/perception_optimization_phase_1.md`。关了就是旧路径，两条都能跑。
#
# 1) 同帧 DINO 图像特征复用：一个词的 backbone 编码给同帧所有词共用。
#    关掉 ⇒ 每个词都重算 backbone（旧行为，实测 19 词多花约 0.894 s）。
DEFAULT_ENABLE_DINO_FRAME_FEATURE_CACHE = True
# 2) mask 游程编码实现："vector"（NumPy）或 "python"（旧的逐像素循环）。
DEFAULT_MASK_RLE_ENCODER = "vector"
# 3) 分项计时。默认关：它会往每个阶段插 CUDA 同步，只适合测量，
#    开着会让生产路径的耗时结构本身发生变化。
DEFAULT_DETECTION_PROFILE_ENABLED = False
DEFAULT_DETECTION_DEVICE = "auto"
DEFAULT_RAM_PLUS_ROOT = "/home/mxt/robotscene/external/recognize-anything"
DEFAULT_RAM_PLUS_CHECKPOINT = "/home/mxt/robotscene/external/ram_plus_ckpt/ram_plus_swin_large_14m.pth"
DEFAULT_ENABLE_RAM_PLUS = True
DEFAULT_RAM_SCENE_TAG_INTERVAL_SECONDS = 1.8
DEFAULT_RAM_SCENE_MAX_TERMS = 12
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
# Live verify responses measured at 31–35 s; leave room for provider variance
# so a healthy response is not discarded and retried as a duplicate request.
DEFAULT_VLM_RUNTIME_VERIFY_TIMEOUT_SECONDS = 45.0
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
# §20：RAM++/GroundingDINO 连续空手多少次、且距上次整帧 target sweep 多久，
# 才允许再问一次 VLM「整张图里到底有没有目标」。只有这两个参数。
DEFAULT_VLM_TARGET_SWEEP_INTERVAL_SECONDS = 10.0
DEFAULT_VLM_TARGET_SWEEP_EMPTY_CYCLES = 4
# 本地（机器狗 Orin NX）复核 provider：只作用于 verify 环节，显式可选、默认关闭。
# 单独一套 base_url/model，避免改动 SILICONFLOW_BASE_URL 而波及 quick/semantic
# 共用客户端。GO2W_LOCAL_VLM_VERIFY=1 时才启用，否则复核照旧走云端。
DEFAULT_LOCAL_VLM_VERIFY_ENABLED = False
DEFAULT_LOCAL_VLM_BASE_URL = "http://127.0.0.1:18081/v1"
DEFAULT_LOCAL_VLM_MODEL = "lfm-local"
DEFAULT_LOCAL_VLM_API_KEY = "sk-local"
# 本地 3B 在 Orin NX 上明显慢于云端，超时给足；超时按可恢复错误处理。
DEFAULT_LOCAL_VLM_TIMEOUT_SECONDS = 180.0
# "full+crop" 与云端两图输入对齐；"crop" 只发目标裁剪图，用于消融对比。
DEFAULT_LOCAL_VLM_VERIFY_IMAGES = "full+crop"
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

# §15 配置项统一化：把散落在调用点的阈值收进 settings。
# ⚠ 默认值一律取**代码当前真正生效的值**，不是计划书 §15 那张例表里的值 ——
# 后者的表头写的是"建议 / 例如"，且其中 4 项与现网实测值不符（逐项注明在下）。
# 这样接线的行为变化严格为零：不设环境变量时，狗怎么跑还是怎么跑。
DEFAULT_GO2W_LIO_ROTATION_WHEEL_MAX_TRANSLATION_M = 0.08
DEFAULT_GO2W_LIO_ROTATION_PSLAM_MAX_TRANSLATION_M = 0.20
DEFAULT_GO2W_LIO_MAX_POSE_SPEED_MPS = 2.0
DEFAULT_GO2W_LIO_RECOVERY_VALID_SAMPLES = 4
DEFAULT_GO2W_PLACE_MERGE_DISTANCE_M = 0.35
DEFAULT_GO2W_PLACE_RELOCATION_MIN_M = 0.50
# 持久三维对象图的入口置信度门。检测端工作在 box_threshold=0.12 这种偏召回的
# 阈值上，"低分候选"是目标搜索的原料，不是垃圾 —— 所以门开在**入图**这一步，
# 而不是去抬检测阈值（那会把目标召回一起砍掉）。0.35 与 SAM2_MIN_BOX_SCORE 对齐。
# 出处：docs/GO2W_WebUI三维物体框中文统一与低置信过滤_整改实施计划_20260917.md §5.1
DEFAULT_SEMANTIC_MAP_MIN_OBJECT_CONFIDENCE = 0.35
DEFAULT_GO2W_PERCEPTION_SETTLE_SECONDS = 0.8
DEFAULT_GO2W_TARGET_POSSIBLE_VERIFY_ATTEMPTS = 2
# 计划书写 0.3，autonomous_explorer.py 实际 sleep 0.2 —— 取现值。
DEFAULT_GO2W_TARGET_POSSIBLE_REOBSERVE_DELAY_SECONDS = 0.2
# 计划书写 0.12 / 0.35 / 0.80，geometric_relation_extractor.py:78-81 实际
# 是 0.10 / 0.25 / 0.90 —— 取现值。这三个量比的是 bbox_2d 的中心差，而
# bbox_2d 是**归一化到 0~1 的图像坐标**（app/prompts.py:47,103），所以
# NORM 这个名字是对的，单位不是米。
DEFAULT_GO2W_RELATION_LEFT_RIGHT_MIN_NORM = 0.10
DEFAULT_GO2W_RELATION_DEPTH_ORDER_MIN_M = 0.25
DEFAULT_GO2W_RELATION_NEAR_MAX_M = 0.90
# 阶段二 §5.1.1：网页主入口的 TargetProfile 接线。此前 `node._target_profile`
# 只在 run_autonomous_loop.py 的 `_run_state_machine_search`（CLI/legacy 专用）
# 里赋值，网页走 run_go2w → _run_go2w_explorer → AutonomousExplorer.run()，
# 永远到不了那里 ⇒ 检测器拿到 None ⇒ 丢掉了 detector_terms()/上下文词/关系锚点词。
# 默认 True 只是"把该接的线接上"，不改变词表内容（use_llm 仍是 False）。
DEFAULT_GO2W_WEB_TARGET_PROFILE_WIRING = True
# 网页侧解析画像时是否调用 LLM。CLI 路径用 True，网页路径此前硬编码 False。
# 升为 True 会多一次**启动期**（不是每帧）云端调用，并引入上下文/锚点词 ⇒
# 必须与其它改动分开对照后才允许翻默认，故默认保持 False。
DEFAULT_GO2W_WEB_TARGET_PROFILE_USE_LLM = False
# 阶段二 §5.1.2：把一次检测的产物按职责拆成 scene_objects / target_candidates /
# relation_anchors，并用三态代替 `target_present = bool(detections)`。
# 旧行为把 RAM 场景词当作目标。2026-10-15 真实只读会话复现办公椅抢占
# 水杯复核位后启用既有职责划分；类别/属性候选仍须走真实视觉复核，不能确认目标。
# 保留显式 false 供历史对照，不能作为自主搜索就绪证据。
DEFAULT_GO2W_RESULT_ROLE_SEPARATION = True
# 阶段二 §5.1.3：复核（Verify）挑哪个框。旧行为是在**全量** detections 上取
# 最高分 —— 而 grounded_sam 那条路的 detections 混着 RAM 场景词，于是一把
# 0.95 的椅子会把 0.32 的目标桶挤掉复核位。取值：
#   target_candidates（默认）—— 只在目标候选里挑（需配合上一条开关：
#                               没有权威候选视图时自动退回旧行为）
#   all_detections           —— 旧行为，用于对照与回退
# 只有恰为 "all_detections" 才是退回旧行为；其它取值（含拼错的）走候选视图，
# 因为"复核目标"才是这个函数本来的语义。
DEFAULT_GO2W_VERIFY_CANDIDATE_SCOPE = "target_candidates"
# 阶段二 §5.1.4：检测分频。开关默认 false ⇒ 一轮一整套 prompt，与今天逐字一致；
# 打开后按类 TTL 决定这一轮跑哪些词槽（见 app/perception/detection_scheduler.py）。
#
# 目标槽 TTL 默认 0（每轮都刷）：§3.2 明令不得新增漏检关键目标，把目标槽也
# 节流是拿搜索能力换速度。可节流的是锚点与场景词 —— 站着没动、朝向没变、
# 地点没变时，它们的结果本来就不会变。
#
# **本项不依赖 `go2w_result_role_separation`**（第三次补充复审·必修）：按槽组装
# 会给物体标 `detection_fresh_slots`（"这条结果对哪些槽还算最新"），组装层为了
# 保住 `scene` 会把旧的全量条目一并并进来，它里面那个**过期的目标框**必须被挡在
# 选框池之外。这道过滤落在 `detections` 这个列表**被造出来的地方**，对
# 职责分离开/关 × 复核范围两种取值**四种组合都生效** —— 早先它只长在
# `classify_roles` 里，而入口只在职责分离开着时才调那个函数，于是**单独翻本开关**
# 时旧高分框会重新赢下锁框与复核位，报出来却像"目标确认为真"。
# 所以：**可以只翻这一个开关**；分频关着时那枚标记谁都不写 ⇒ 过滤一条都不丢，
# 行为与加这个功能之前逐字一致。判据见 tests/test_detection_scheduler.py 第 6c 节。
DEFAULT_GO2W_DETECTION_CADENCE_ENABLED = True
DEFAULT_GO2W_TARGET_REFRESH_INTERVAL_S = 3.0
DEFAULT_GO2W_ANCHOR_REFRESH_INTERVAL_S = 3.0
DEFAULT_GO2W_SCENE_REFRESH_INTERVAL_S = 3.0
DEFAULT_GO2W_DETECTION_TRANSLATION_REFRESH_M = 0.8
DEFAULT_GO2W_DETECTION_ROTATION_REFRESH_DEG = 30.0
DEFAULT_GO2W_DETECTION_MIN_INTERVAL_S = 1.0
# 场景词的**硬顶**：无论 TTL 配多大，等满这么久必须刷一次，并把实际等待
# 时长写进事件（§5.2 要求可见）。这是"绝不无限期不刷"的兜底，不是调优旋钮。
DEFAULT_GO2W_SCENE_MAX_STALE_S = 20.0
# 阶段二 §5.1.6：多词分组前向**实验**开关。
#   per_term（默认）—— 每个词一次 predict()，与今天逐字一致
#   grouped        —— 多词拼一条 caption、一次 predict()，再用 token 跨度
#                     （`remove_combined=True` + `get_phrases_from_posmap`）
#                     把框关联回词
# 默认保持 per_term：分组是**待评估的候选**，不是新默认。要翻默认必须先在
# 固定评估集上过 §3.2 门槛（不得新增漏检/假找到/错误关系）。
# 取值字面量与 `app/detectors/grounded_sam_worker.py: DEFAULT_PROMPT_MODE`
# 必须一致 —— worker 跑在另一个解释器里、import 不到 app，这是本文件既有约定。
DEFAULT_GO2W_DINO_PROMPT_MODE = "per_term"
# 阶段二 §5.1.5：有界异步复核。
#
# 旧行为是**同步**复核：搜索线程按在 `node._verify_target` 上等 VLM，超时
# 45 s；而 daemon 是单槽串行的、**取消不了**，被放弃的慢请求继续占着通道，
# 后到的复核排在它后面一起被闸门判掉 —— 结果就是"复核很慢、而且没结论"。
# 打开后改成"提交 + 轮询"：搜索线程不再等，超时由**客户端**在 poll 侧判，
# 产出 UNKNOWN（**不是** ABSENT）。默认 false ⇒ 逐字回到同步路径。
DEFAULT_GO2W_ASYNC_VERIFY_ENABLED = False
DEFAULT_GO2W_VERIFY_MAX_INFLIGHT = 1
DEFAULT_GO2W_VERIFY_MAX_PENDING = 1
# 客户端等待上限。**注意它比旧的内联超时短得多**：这里判超时只是让搜索线程
# 继续走（出一条 UNKNOWN），worker 线程该挂还是挂着，结果晚到会被序号丢弃。
# 所以这条不是"VLM 多久算失败"，而是"我们愿意为一条复核等多久"。
DEFAULT_GO2W_VERIFY_CLIENT_TIMEOUT_S = 12.0


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
    yoloe_python: str = DEFAULT_YOLOE_PYTHON
    yoloe_model: str = DEFAULT_YOLOE_MODEL
    yoloe_text_encoder: str = DEFAULT_YOLOE_TEXT_ENCODER
    yoloe_device: str = DEFAULT_YOLOE_DEVICE
    yoloe_precision: str = DEFAULT_YOLOE_PRECISION
    yoloe_confidence: float = DEFAULT_YOLOE_CONFIDENCE
    yoloe_iou_threshold: float = DEFAULT_YOLOE_IOU_THRESHOLD
    yoloe_image_size: int = DEFAULT_YOLOE_IMAGE_SIZE
    yoloe_max_detections: int = DEFAULT_YOLOE_MAX_DETECTIONS
    yoloe_timeout_seconds: float = DEFAULT_YOLOE_TIMEOUT_SECONDS
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
    sam2_min_box_score: float = DEFAULT_SAM2_MIN_BOX_SCORE
    enable_dino_frame_feature_cache: bool = DEFAULT_ENABLE_DINO_FRAME_FEATURE_CACHE
    mask_rle_encoder: str = DEFAULT_MASK_RLE_ENCODER
    detection_profile_enabled: bool = DEFAULT_DETECTION_PROFILE_ENABLED
    detection_device: str = DEFAULT_DETECTION_DEVICE
    ram_plus_root: str = DEFAULT_RAM_PLUS_ROOT
    ram_plus_checkpoint: str = DEFAULT_RAM_PLUS_CHECKPOINT
    enable_ram_plus: bool = DEFAULT_ENABLE_RAM_PLUS
    ram_scene_tag_interval_seconds: float = DEFAULT_RAM_SCENE_TAG_INTERVAL_SECONDS
    ram_scene_max_terms: int = DEFAULT_RAM_SCENE_MAX_TERMS
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
    vlm_target_sweep_interval_seconds: float = DEFAULT_VLM_TARGET_SWEEP_INTERVAL_SECONDS
    vlm_target_sweep_empty_cycles: int = DEFAULT_VLM_TARGET_SWEEP_EMPTY_CYCLES
    local_vlm_verify_enabled: bool = DEFAULT_LOCAL_VLM_VERIFY_ENABLED
    local_vlm_base_url: str = DEFAULT_LOCAL_VLM_BASE_URL
    local_vlm_model: str = DEFAULT_LOCAL_VLM_MODEL
    local_vlm_api_key: str = DEFAULT_LOCAL_VLM_API_KEY
    local_vlm_timeout_seconds: float = DEFAULT_LOCAL_VLM_TIMEOUT_SECONDS
    local_vlm_verify_images: str = DEFAULT_LOCAL_VLM_VERIFY_IMAGES
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

    # §15 的 12 个阈值旋钮。逐个都有默认值与范围校验（见 _env_*_ranged）。
    go2w_lio_rotation_wheel_max_translation_m: float = (
        DEFAULT_GO2W_LIO_ROTATION_WHEEL_MAX_TRANSLATION_M
    )
    go2w_lio_rotation_pslam_max_translation_m: float = (
        DEFAULT_GO2W_LIO_ROTATION_PSLAM_MAX_TRANSLATION_M
    )
    go2w_lio_max_pose_speed_mps: float = DEFAULT_GO2W_LIO_MAX_POSE_SPEED_MPS
    go2w_lio_recovery_valid_samples: int = DEFAULT_GO2W_LIO_RECOVERY_VALID_SAMPLES
    go2w_place_merge_distance_m: float = DEFAULT_GO2W_PLACE_MERGE_DISTANCE_M
    go2w_place_relocation_min_m: float = DEFAULT_GO2W_PLACE_RELOCATION_MIN_M
    semantic_map_min_object_confidence: float = (
        DEFAULT_SEMANTIC_MAP_MIN_OBJECT_CONFIDENCE
    )
    go2w_perception_settle_seconds: float = DEFAULT_GO2W_PERCEPTION_SETTLE_SECONDS
    go2w_target_possible_verify_attempts: int = (
        DEFAULT_GO2W_TARGET_POSSIBLE_VERIFY_ATTEMPTS
    )
    go2w_target_possible_reobserve_delay_seconds: float = (
        DEFAULT_GO2W_TARGET_POSSIBLE_REOBSERVE_DELAY_SECONDS
    )
    go2w_relation_left_right_min_norm: float = DEFAULT_GO2W_RELATION_LEFT_RIGHT_MIN_NORM
    go2w_relation_depth_order_min_m: float = DEFAULT_GO2W_RELATION_DEPTH_ORDER_MIN_M
    go2w_relation_near_max_m: float = DEFAULT_GO2W_RELATION_NEAR_MAX_M
    go2w_web_target_profile_wiring: bool = DEFAULT_GO2W_WEB_TARGET_PROFILE_WIRING
    go2w_web_target_profile_use_llm: bool = DEFAULT_GO2W_WEB_TARGET_PROFILE_USE_LLM
    go2w_result_role_separation: bool = DEFAULT_GO2W_RESULT_ROLE_SEPARATION
    go2w_verify_candidate_scope: str = DEFAULT_GO2W_VERIFY_CANDIDATE_SCOPE
    go2w_detection_cadence_enabled: bool = DEFAULT_GO2W_DETECTION_CADENCE_ENABLED
    go2w_target_refresh_interval_s: float = DEFAULT_GO2W_TARGET_REFRESH_INTERVAL_S
    go2w_anchor_refresh_interval_s: float = DEFAULT_GO2W_ANCHOR_REFRESH_INTERVAL_S
    go2w_scene_refresh_interval_s: float = DEFAULT_GO2W_SCENE_REFRESH_INTERVAL_S
    go2w_scene_max_stale_s: float = DEFAULT_GO2W_SCENE_MAX_STALE_S
    go2w_detection_translation_refresh_m: float = DEFAULT_GO2W_DETECTION_TRANSLATION_REFRESH_M
    go2w_detection_rotation_refresh_deg: float = DEFAULT_GO2W_DETECTION_ROTATION_REFRESH_DEG
    go2w_detection_min_interval_s: float = DEFAULT_GO2W_DETECTION_MIN_INTERVAL_S
    go2w_async_verify_enabled: bool = DEFAULT_GO2W_ASYNC_VERIFY_ENABLED
    go2w_verify_max_inflight: int = DEFAULT_GO2W_VERIFY_MAX_INFLIGHT
    go2w_verify_max_pending: int = DEFAULT_GO2W_VERIFY_MAX_PENDING
    go2w_verify_client_timeout_s: float = DEFAULT_GO2W_VERIFY_CLIENT_TIMEOUT_S
    go2w_dino_prompt_mode: str = DEFAULT_GO2W_DINO_PROMPT_MODE

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


def _env_float_ranged(name: str, default: float, low: float, high: float) -> float:
    """§15 的范围校验：越界回落默认值，并把回落这件事打到 stderr。

    刻意不夹到最近的边界值：那会悄悄改掉操作员设的数，人只会看到一个
    不等于自己输入的有效值 —— 这正是"默认值 ≠ 生效值"那类假绿的来源。
    回落 + 出声，至少让启动日志里能看出来。
    """
    value = _env_float(name, default)
    if not (low <= value <= high):
        print(
            f"[config] {name}={value} 超出允许范围 [{low}, {high}]，"
            f"已回落到默认值 {default}",
            file=sys.stderr,
        )
        return default
    return value


def _env_int_ranged(name: str, default: int, low: int, high: int) -> int:
    value = _env_int(name, default)
    if not (low <= value <= high):
        print(
            f"[config] {name}={value} 超出允许范围 [{low}, {high}]，"
            f"已回落到默认值 {default}",
            file=sys.stderr,
        )
        return default
    return value


def _env_choice(name: str, default: str, allowed: tuple[str, ...]) -> str:
    """枚举型开关：未知取值回落默认值，并**出声**。

    与 `_env_*_ranged` 同一条理由 —— 静默回落会让"我配的是 X"和"实际跑的是
    默认值"在日志里长得一模一样，而那正是一次对照实验最容易骗到自己的地方。
    """
    value = (_env_value(name, default) or default).strip().lower()
    if value not in allowed:
        print(
            f"[config] {name}={value!r} 不是合法取值 {list(allowed)}，"
            f"已回落到默认值 {default!r}",
            file=sys.stderr,
        )
        return default
    return value


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
        yoloe_python=_env_value("YOLOE_PYTHON", DEFAULT_YOLOE_PYTHON),
        yoloe_model=_env_value("YOLOE_MODEL", DEFAULT_YOLOE_MODEL),
        yoloe_text_encoder=_env_value(
            "YOLOE_TEXT_ENCODER", DEFAULT_YOLOE_TEXT_ENCODER
        ),
        yoloe_device=_env_value("YOLOE_DEVICE", DEFAULT_YOLOE_DEVICE),
        yoloe_precision=_env_value("YOLOE_PRECISION", DEFAULT_YOLOE_PRECISION),
        yoloe_confidence=_env_float("YOLOE_CONFIDENCE", DEFAULT_YOLOE_CONFIDENCE),
        yoloe_iou_threshold=_env_float(
            "YOLOE_IOU_THRESHOLD", DEFAULT_YOLOE_IOU_THRESHOLD
        ),
        yoloe_image_size=_env_int("YOLOE_IMAGE_SIZE", DEFAULT_YOLOE_IMAGE_SIZE),
        yoloe_max_detections=_env_int(
            "YOLOE_MAX_DETECTIONS", DEFAULT_YOLOE_MAX_DETECTIONS
        ),
        yoloe_timeout_seconds=_env_float(
            "YOLOE_TIMEOUT_SECONDS", DEFAULT_YOLOE_TIMEOUT_SECONDS
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
        sam2_min_box_score=_env_float(
            "SAM2_MIN_BOX_SCORE", DEFAULT_SAM2_MIN_BOX_SCORE
        ),
        max_detected_objects=_env_int("MAX_DETECTED_OBJECTS", DEFAULT_MAX_DETECTED_OBJECTS),
        enable_dino_frame_feature_cache=_env_bool(
            "ENABLE_DINO_FRAME_FEATURE_CACHE",
            DEFAULT_ENABLE_DINO_FRAME_FEATURE_CACHE,
        ),
        mask_rle_encoder=_env_value("MASK_RLE_ENCODER", DEFAULT_MASK_RLE_ENCODER),
        detection_profile_enabled=_env_bool(
            "DETECTION_PROFILE_ENABLED", DEFAULT_DETECTION_PROFILE_ENABLED
        ),
        detection_device=_env_value("DETECTION_DEVICE", DEFAULT_DETECTION_DEVICE),
        ram_plus_root=_env_value("RAM_PLUS_ROOT", DEFAULT_RAM_PLUS_ROOT),
        ram_plus_checkpoint=_env_value(
            "RAM_PLUS_CHECKPOINT", DEFAULT_RAM_PLUS_CHECKPOINT
        ),
        enable_ram_plus=_env_bool("ENABLE_RAM_PLUS", DEFAULT_ENABLE_RAM_PLUS),
        ram_scene_tag_interval_seconds=_env_float(
            "RAM_SCENE_TAG_INTERVAL_SECONDS", DEFAULT_RAM_SCENE_TAG_INTERVAL_SECONDS
        ),
        ram_scene_max_terms=_env_int("RAM_SCENE_MAX_TERMS", DEFAULT_RAM_SCENE_MAX_TERMS),
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
        vlm_target_sweep_interval_seconds=_env_float(
            "VLM_TARGET_SWEEP_INTERVAL_SECONDS",
            DEFAULT_VLM_TARGET_SWEEP_INTERVAL_SECONDS,
        ),
        vlm_target_sweep_empty_cycles=_env_int(
            "VLM_TARGET_SWEEP_EMPTY_CYCLES",
            DEFAULT_VLM_TARGET_SWEEP_EMPTY_CYCLES,
        ),
        local_vlm_verify_enabled=_env_bool(
            "GO2W_LOCAL_VLM_VERIFY",
            DEFAULT_LOCAL_VLM_VERIFY_ENABLED,
        ),
        local_vlm_base_url=_env_value(
            "GO2W_LOCAL_VLM_BASE_URL", DEFAULT_LOCAL_VLM_BASE_URL
        ),
        local_vlm_model=_env_value(
            "GO2W_LOCAL_VLM_MODEL", DEFAULT_LOCAL_VLM_MODEL
        ),
        local_vlm_api_key=_env_value(
            "GO2W_LOCAL_VLM_API_KEY", DEFAULT_LOCAL_VLM_API_KEY
        ),
        local_vlm_timeout_seconds=_env_float(
            "GO2W_LOCAL_VLM_TIMEOUT_SECONDS", DEFAULT_LOCAL_VLM_TIMEOUT_SECONDS
        ),
        local_vlm_verify_images=_env_value(
            "GO2W_LOCAL_VLM_VERIFY_IMAGES", DEFAULT_LOCAL_VLM_VERIFY_IMAGES
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
        # §15 的 12 个阈值旋钮。范围取"物理上讲得通"的宽区间，不当调参门限用：
        # 它的职责是拦住手滑写错的量级（比如把 0.35 写成 35），不是替操作员定策略。
        go2w_lio_rotation_wheel_max_translation_m=_env_float_ranged(
            "GO2W_LIO_ROTATION_WHEEL_MAX_TRANSLATION_M",
            DEFAULT_GO2W_LIO_ROTATION_WHEEL_MAX_TRANSLATION_M,
            0.0,
            1.0,
        ),
        go2w_lio_rotation_pslam_max_translation_m=_env_float_ranged(
            "GO2W_LIO_ROTATION_PSLAM_MAX_TRANSLATION_M",
            DEFAULT_GO2W_LIO_ROTATION_PSLAM_MAX_TRANSLATION_M,
            0.0,
            2.0,
        ),
        go2w_lio_max_pose_speed_mps=_env_float_ranged(
            "GO2W_LIO_MAX_POSE_SPEED_MPS", DEFAULT_GO2W_LIO_MAX_POSE_SPEED_MPS, 0.1, 10.0
        ),
        go2w_lio_recovery_valid_samples=_env_int_ranged(
            "GO2W_LIO_RECOVERY_VALID_SAMPLES",
            DEFAULT_GO2W_LIO_RECOVERY_VALID_SAMPLES,
            1,
            100,
        ),
        go2w_place_merge_distance_m=_env_float_ranged(
            "GO2W_PLACE_MERGE_DISTANCE_M",
            DEFAULT_GO2W_PLACE_MERGE_DISTANCE_M,
            0.01,
            5.0,
        ),
        go2w_place_relocation_min_m=_env_float_ranged(
            "GO2W_PLACE_RELOCATION_MIN_M",
            DEFAULT_GO2W_PLACE_RELOCATION_MIN_M,
            0.0,
            5.0,
        ),
        semantic_map_min_object_confidence=_env_float_ranged(
            "SEMANTIC_MAP_MIN_OBJECT_CONFIDENCE",
            DEFAULT_SEMANTIC_MAP_MIN_OBJECT_CONFIDENCE,
            0.0,
            1.0,
        ),
        go2w_perception_settle_seconds=_env_float_ranged(
            "GO2W_PERCEPTION_SETTLE_SECONDS",
            DEFAULT_GO2W_PERCEPTION_SETTLE_SECONDS,
            0.0,
            30.0,
        ),
        go2w_target_possible_verify_attempts=_env_int_ranged(
            "GO2W_TARGET_POSSIBLE_VERIFY_ATTEMPTS",
            DEFAULT_GO2W_TARGET_POSSIBLE_VERIFY_ATTEMPTS,
            1,
            20,
        ),
        go2w_target_possible_reobserve_delay_seconds=_env_float_ranged(
            "GO2W_TARGET_POSSIBLE_REOBSERVE_DELAY_SECONDS",
            DEFAULT_GO2W_TARGET_POSSIBLE_REOBSERVE_DELAY_SECONDS,
            0.0,
            10.0,
        ),
        go2w_relation_left_right_min_norm=_env_float_ranged(
            "GO2W_RELATION_LEFT_RIGHT_MIN_NORM",
            DEFAULT_GO2W_RELATION_LEFT_RIGHT_MIN_NORM,
            0.0,
            2.0,
        ),
        go2w_relation_depth_order_min_m=_env_float_ranged(
            "GO2W_RELATION_DEPTH_ORDER_MIN_M",
            DEFAULT_GO2W_RELATION_DEPTH_ORDER_MIN_M,
            0.0,
            5.0,
        ),
        go2w_relation_near_max_m=_env_float_ranged(
            "GO2W_RELATION_NEAR_MAX_M",
            DEFAULT_GO2W_RELATION_NEAR_MAX_M,
            0.0,
            10.0,
        ),
        go2w_web_target_profile_wiring=_env_bool(
            "GO2W_WEB_TARGET_PROFILE_WIRING",
            DEFAULT_GO2W_WEB_TARGET_PROFILE_WIRING,
        ),
        go2w_web_target_profile_use_llm=_env_bool(
            "GO2W_WEB_TARGET_PROFILE_USE_LLM",
            DEFAULT_GO2W_WEB_TARGET_PROFILE_USE_LLM,
        ),
        go2w_result_role_separation=_env_bool(
            "GO2W_RESULT_ROLE_SEPARATION",
            DEFAULT_GO2W_RESULT_ROLE_SEPARATION,
        ),
        go2w_verify_candidate_scope=(
            _env_value("GO2W_VERIFY_CANDIDATE_SCOPE", DEFAULT_GO2W_VERIFY_CANDIDATE_SCOPE)
            or DEFAULT_GO2W_VERIFY_CANDIDATE_SCOPE
        ).strip().lower(),
        go2w_detection_cadence_enabled=_env_bool(
            "GO2W_DETECTION_CADENCE_ENABLED",
            DEFAULT_GO2W_DETECTION_CADENCE_ENABLED,
        ),
        go2w_target_refresh_interval_s=_env_float_ranged(
            "GO2W_TARGET_REFRESH_INTERVAL_S",
            DEFAULT_GO2W_TARGET_REFRESH_INTERVAL_S,
            0.0,
            600.0,
        ),
        go2w_anchor_refresh_interval_s=_env_float_ranged(
            "GO2W_ANCHOR_REFRESH_INTERVAL_S",
            DEFAULT_GO2W_ANCHOR_REFRESH_INTERVAL_S,
            0.0,
            600.0,
        ),
        go2w_scene_refresh_interval_s=_env_float_ranged(
            "GO2W_SCENE_REFRESH_INTERVAL_S",
            DEFAULT_GO2W_SCENE_REFRESH_INTERVAL_S,
            0.0,
            600.0,
        ),
        go2w_scene_max_stale_s=_env_float_ranged(
            "GO2W_SCENE_MAX_STALE_S",
            DEFAULT_GO2W_SCENE_MAX_STALE_S,
            0.0,
            3600.0,
        ),
        go2w_detection_translation_refresh_m=_env_float_ranged(
            "GO2W_DETECTION_TRANSLATION_REFRESH_M",
            DEFAULT_GO2W_DETECTION_TRANSLATION_REFRESH_M,
            0.0,
            100.0,
        ),
        go2w_detection_rotation_refresh_deg=_env_float_ranged(
            "GO2W_DETECTION_ROTATION_REFRESH_DEG",
            DEFAULT_GO2W_DETECTION_ROTATION_REFRESH_DEG,
            0.0,
            3600.0,
        ),
        go2w_detection_min_interval_s=_env_float_ranged(
            "GO2W_DETECTION_MIN_INTERVAL_S",
            DEFAULT_GO2W_DETECTION_MIN_INTERVAL_S,
            0.0,
            60.0,
        ),
        go2w_async_verify_enabled=_env_bool(
            "GO2W_ASYNC_VERIFY_ENABLED",
            DEFAULT_GO2W_ASYNC_VERIFY_ENABLED,
        ),
        go2w_verify_max_inflight=_env_int_ranged(
            "GO2W_VERIFY_MAX_INFLIGHT",
            DEFAULT_GO2W_VERIFY_MAX_INFLIGHT,
            1,
            8,
        ),
        go2w_verify_max_pending=_env_int_ranged(
            "GO2W_VERIFY_MAX_PENDING",
            DEFAULT_GO2W_VERIFY_MAX_PENDING,
            0,
            8,
        ),
        go2w_verify_client_timeout_s=_env_float_ranged(
            "GO2W_VERIFY_CLIENT_TIMEOUT_S",
            DEFAULT_GO2W_VERIFY_CLIENT_TIMEOUT_S,
            # 下界允许 0：那就是"提交完立刻返回"，等待改由后续周期收敛。
            # 但那要配合把重试节奏也改长，否则 `verify_attempts=1` 之下复核
            # 永远等不到答案 —— 默认给的是**有界等待**，不是 0。
            0.0,
            300.0,
        ),
        # 未知取值回落默认并出声：静默回落会让"我开的是 grouped"与"实际跑的
        # 是 per_term"长得一模一样（`_resolve_prompt_mode` 那边还会把生效值/
        # 请求值两个字段都回填进 diagnostics）。
        go2w_dino_prompt_mode=_env_choice(
            "GO2W_DINO_PROMPT_MODE",
            DEFAULT_GO2W_DINO_PROMPT_MODE,
            ("per_term", "grouped"),
        ),
    )
