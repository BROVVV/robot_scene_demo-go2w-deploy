"""Command-line entry point for the robot scene understanding demo."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path

from openai import OpenAIError
from pydantic import ValidationError

from app.config import Settings, SettingsError, get_settings
from app.detectors.grounded_sam_subprocess import (
    DetectorRuntimeError,
    GroundedSAMSubprocessDetector,
)
from app.llm_clients.siliconflow_client import SiliconFlowVisionClient
from app.memory.observation_memory_store import (
    ObservationMemoryStore,
    build_scene_memory_updates,
)
from app.reasoning.dynamic_detector_prompts import build_dynamic_detector_prompts
from app.reasoning.evidence_gate import gate_scene_target
from app.reasoning.llm_prior_generator import LLMPriorGenerator, LLMPriorInput
from app.reasoning.prior_usage_auditor import (
    build_prior_usage_report,
    write_prior_usage_report,
)
from app.schemas import SceneAnalysisResult
from app.services.knowledge_aware_analyzer import KnowledgeAwareAnalyzer
from app.services.knowledge_output_writer import write_knowledge_aware_outputs
from app.services.accuracy_pipeline import enhance_image_result
from app.services.detector_scene_builder import build_scene_from_detections
from app.services.output_writer import prepare_analysis_result, write_analysis_outputs
from app.services.route_planner import format_route_plan
from app.services.scene_analyzer import SceneAnalyzer
from app.services.target_matcher import format_target_decision
from app.task_understanding.task_pipeline import (
    navigation_target_text,
    parsed_task_to_dict,
    prepare_navigation_task_from_text,
    write_task_understanding_outputs,
)
from app.video.target_profile import TargetProfileResolver
from app.video.video_target_search_pipeline import run_video_target_search_pipeline
from app.navigation.nav2_cli import add_nav2_arguments, run_nav2_from_args


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="机器狗场景理解 Demo：输入单张图片和目标描述，输出场景理解结果。"
    )
    parser.add_argument("--image", help="场景图片路径")
    parser.add_argument("--video", help="第一人称视频路径；会进入视频目标搜索与视觉导航规划")
    parser.add_argument("--target", help="目标描述文本，例如：桌子上的手机")
    parser.add_argument(
        "--detector",
        choices=["llm", "grounded_sam"],
        default=None,
        help="物体检测后端。llm 使用视觉大模型；grounded_sam 使用 Grounding DINO + SAM2。",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="使用 examples/mock_scene_result.json，不调用真实 API",
    )
    parser.add_argument(
        "--enable-knowledge",
        action="store_true",
        help="兼容旧参数：启用 LLM prior、观察记忆、证据门控和增强输出。",
    )
    reasoning_group = parser.add_mutually_exclusive_group()
    reasoning_group.add_argument(
        "--enable-llm-reasoning",
        dest="enable_llm_reasoning",
        action="store_true",
        help="启用 LLM-first 情境化搜索推理。",
    )
    reasoning_group.add_argument(
        "--disable-llm-reasoning",
        dest="enable_llm_reasoning",
        action="store_false",
        help="关闭 LLM 情境推理并使用旧知识推理链路。",
    )
    parser.add_argument(
        "--enable-reasoning-memory",
        action="store_true",
        help="检索并写入情境搜索经验记忆。",
    )
    parser.add_argument(
        "--quadruped-mode",
        action="store_true",
        help="将建议强制约束为机械狗视角动作。",
    )
    parser.add_argument(
        "--hide-unexecutable-actions",
        action="store_true",
        help="输出中隐藏不可执行动作，仅保留需人工说明。",
    )
    parser.add_argument(
        "--motion-profile",
        choices=[
            "strict_safe",
            "platform_assisted_indoor",
            "platform_assisted_open_area",
            "platform_assisted_auto",
        ],
        help="运动视界策略档位。",
    )
    parser.add_argument(
        "--platform-obstacle-avoidance",
        action="store_true",
        help="假设机械狗底层已有基础避障能力，启用平台辅助较长移动段。",
    )
    parser.add_argument(
        "--max-open-step",
        type=float,
        help="平台避障开放区域最大单段移动距离。",
    )
    parser.add_argument(
        "--max-indoor-step",
        type=float,
        help="平台避障室内最大单段移动距离。",
    )
    parser.add_argument(
        "--disable-dynamic-motion-horizon",
        action="store_true",
        help="关闭动态运动视界，恢复严格安全单步裁剪。",
    )
    target_profile_group = parser.add_mutually_exclusive_group()
    target_profile_group.add_argument(
        "--enable-target-profile", dest="enable_target_profile", action="store_true"
    )
    target_profile_group.add_argument(
        "--disable-target-profile", dest="enable_target_profile", action="store_false"
    )
    crop_group = parser.add_mutually_exclusive_group()
    crop_group.add_argument(
        "--enable-crop-verify", dest="enable_crop_verify", action="store_true"
    )
    crop_group.add_argument(
        "--disable-crop-verify", dest="enable_crop_verify", action="store_false"
    )
    parser.add_argument("--high-recall", action="store_true", help="启用低阈值高召回候选检测")
    parser.add_argument("--box-threshold", type=float)
    parser.add_argument("--text-threshold", type=float)
    parser.add_argument("--max-candidates", type=int)
    llm_prior_group = parser.add_mutually_exclusive_group()
    llm_prior_group.add_argument(
        "--enable-llm-prior", dest="enable_llm_prior", action="store_true"
    )
    llm_prior_group.add_argument(
        "--disable-llm-prior", dest="enable_llm_prior", action="store_false"
    )
    memory_group = parser.add_mutually_exclusive_group()
    memory_group.add_argument(
        "--enable-observation-memory",
        dest="enable_observation_memory",
        action="store_true",
    )
    memory_group.add_argument(
        "--disable-observation-memory",
        dest="enable_observation_memory",
        action="store_false",
    )
    gate_group = parser.add_mutually_exclusive_group()
    gate_group.add_argument(
        "--enable-evidence-gating",
        dest="enable_evidence_gating",
        action="store_true",
    )
    gate_group.add_argument(
        "--disable-evidence-gating",
        dest="enable_evidence_gating",
        action="store_false",
    )
    parser.add_argument(
        "--disable-handwritten-priors",
        action="store_true",
        help="禁用人工写死的物体/房间/位置先验和搜索规则。",
    )
    parser.add_argument(
        "--disable-static-kb",
        action="store_true",
        help="禁用 data/scene_kb 静态知识库检索与写入。",
    )
    parser.add_argument("--prior-audit", action="store_true", help="输出先验使用审计。")
    video_nav_group = parser.add_mutually_exclusive_group()
    video_nav_group.add_argument(
        "--enable-video-navigation",
        dest="enable_video_navigation",
        action="store_true",
        help="视频分析后自动生成 Video-to-Navigation 规划",
    )
    video_nav_group.add_argument(
        "--disable-video-navigation",
        dest="enable_video_navigation",
        action="store_false",
        help="关闭视频视觉导航规划",
    )
    parser.add_argument(
        "--video-navigation-mode",
        choices=["visual_preview", "metric_preview", "plan_only", "execute"],
        default=None,
    )
    parser.add_argument(
        "--video-pose-backend",
        choices=["auto", "mock", "relative", "metric"],
        default=None,
    )
    parser.add_argument("--depth-dir")
    parser.add_argument("--video-map-transform-json")
    parser.add_argument("--force-exploration", action="store_true")
    parser.set_defaults(
        enable_target_profile=None,
        enable_crop_verify=None,
        enable_llm_reasoning=None,
        enable_reasoning_memory=None,
        enable_llm_prior=None,
        enable_observation_memory=None,
        enable_evidence_gating=None,
        enable_video_navigation=None,
    )
    add_nav2_arguments(parser)
    return parser.parse_args(argv)


def run(
    image_path: str,
    target_text: str,
    detector_backend: str | None = None,
    enable_knowledge: bool = False,
    enable_target_profile: bool | None = None,
    enable_crop_verify: bool | None = None,
    high_recall: bool = False,
    box_threshold: float | None = None,
    text_threshold: float | None = None,
    max_candidates: int | None = None,
    enable_llm_reasoning: bool | None = None,
    enable_reasoning_memory: bool | None = None,
    quadruped_mode: bool = True,
    hide_unexecutable_actions: bool = False,
    motion_profile: str | None = None,
    platform_obstacle_avoidance: bool | None = None,
    max_open_step: float | None = None,
    max_indoor_step: float | None = None,
    disable_dynamic_motion_horizon: bool = False,
    enable_llm_prior: bool | None = None,
    enable_observation_memory: bool | None = None,
    enable_evidence_gating: bool | None = None,
    disable_handwritten_priors: bool = False,
    disable_static_kb: bool = False,
    prior_audit: bool = False,
) -> list[Path]:
    if enable_knowledge:
        _warn_deprecated_enable_knowledge()
    settings = _apply_prior_free_flags(
        get_settings(),
        enable_knowledge=enable_knowledge,
        enable_llm_prior=enable_llm_prior,
        enable_observation_memory=enable_observation_memory,
        enable_evidence_gating=enable_evidence_gating,
        disable_handwritten_priors=disable_handwritten_priors,
        disable_static_kb=disable_static_kb,
        prior_audit=prior_audit,
    )
    output_dir = Path(settings.output_dir)
    parsed_task, gate_result, nav_task = prepare_navigation_task_from_text(target_text)
    task_output_paths = write_task_understanding_outputs(
        output_dir,
        parsed_task,
        gate_result,
        nav_task,
    )
    print("自然语言任务理解：")
    print(json.dumps(parsed_task_to_dict(parsed_task), ensure_ascii=False, indent=2))
    print()
    print(gate_result.user_feedback)
    print()
    if not nav_task.executable:
        print("任务未进入视觉检测、规划或 ROS2 dry-run 管线。")
        print("已生成：")
        for path in task_output_paths.values():
            print(path)
        return list(task_output_paths.values())

    image = Path(image_path)
    if not image.is_file():
        raise FileNotFoundError(f"Image file not found: {image_path}")
    target_text = navigation_target_text(nav_task, fallback=target_text)
    profile_enabled = (
        settings.enable_target_profile
        if enable_target_profile is None
        else enable_target_profile
    )
    effective_box = box_threshold
    effective_text = text_threshold
    if high_recall or settings.enable_gdino_high_recall:
        effective_box = (
            effective_box
            if effective_box is not None
            else settings.grounding_dino_high_recall_box_threshold
        )
        effective_text = (
            effective_text
            if effective_text is not None
            else settings.grounding_dino_high_recall_text_threshold
        )
    settings = replace(
        settings,
        enable_target_profile=profile_enabled,
        grounding_dino_box_threshold=(
            effective_box
            if effective_box is not None
            else settings.grounding_dino_box_threshold
        ),
        grounding_dino_text_threshold=(
            effective_text
            if effective_text is not None
            else settings.grounding_dino_text_threshold
        ),
        crop_verify_max_candidates=(
            max_candidates
            if max_candidates is not None
            else settings.crop_verify_max_candidates
        ),
        motion_horizon_profile=motion_profile or settings.motion_horizon_profile,
        platform_obstacle_avoidance_assumed=(
            platform_obstacle_avoidance
            if platform_obstacle_avoidance is not None
            else settings.platform_obstacle_avoidance_assumed
        ),
        motion_platform_open_max_step_m=(
            max_open_step
            if max_open_step is not None
            else settings.motion_platform_open_max_step_m
        ),
        motion_platform_indoor_max_step_m=(
            max_indoor_step
            if max_indoor_step is not None
            else settings.motion_platform_indoor_max_step_m
        ),
        enable_dynamic_motion_horizon=(
            False
            if disable_dynamic_motion_horizon
            else settings.enable_dynamic_motion_horizon
        ),
    )
    backend = detector_backend or settings.detection_backend
    profile = TargetProfileResolver(settings=settings).resolve(
        target_text,
        use_llm=profile_enabled,
    )
    print(
        "[TargetProfile] "
        f"raw_target={target_text} terms={profile.detector_terms(settings.target_profile_max_terms)}"
    )
    print(
        "[Detector] "
        f"backend={backend} box_threshold={settings.grounding_dino_box_threshold:.3f} "
        f"text_threshold={settings.grounding_dino_text_threshold:.3f} "
        f"max_candidates={settings.crop_verify_max_candidates}"
    )
    print(
        "[MotionHorizon] "
        f"profile={settings.motion_horizon_profile} "
        f"platform_obstacle_avoidance_assumed={settings.platform_obstacle_avoidance_assumed} "
        f"dynamic={settings.enable_dynamic_motion_horizon}"
    )

    if backend == "grounded_sam":
        grounded_detector = GroundedSAMSubprocessDetector(
            settings,
            target_profile=profile if profile_enabled else None,
            parsed_task=parsed_task,
            navigation_task=nav_task,
        )
        analyzer = SceneAnalyzer(
            object_detector=grounded_detector,
            output_dir=output_dir,
        )
        visual_retry_callback = lambda terms: build_scene_from_detections(
            grounded_detector.detect_with_dynamic_terms(
                str(image),
                target_text,
                terms,
            ),
            target_text,
        )
    elif backend == "llm":
        vision_client = SiliconFlowVisionClient(settings=settings)
        analyzer = SceneAnalyzer(
            llm_client=vision_client,
            output_dir=output_dir,
            enable_low_object_retry=settings.enable_low_object_retry,
            min_objects_for_complex_scene=settings.min_objects_for_complex_scene,
        )
        visual_retry_callback = lambda terms: SceneAnalysisResult.model_validate(
            vision_client.analyze_scene(
                str(image),
                target_text,
                extra_instructions=(
                    "这是由情境推理产生的二次视觉复核。仅根据图像确认目标，"
                    "不得把上下文线索当作目标本体。额外开放词表提示："
                    + "、".join(terms)
                ),
            )
        )
    else:
        raise ValueError(f"Unsupported detector backend: {backend}")
    result = analyzer.analyze(
        str(image),
        target_text,
        extra_instructions=(
            profile.prompt_context()
            if backend == "llm" and profile_enabled
            else None
        ),
    )
    result, accuracy_paths = enhance_image_result(
        result,
        image,
        profile,
        settings,
        output_dir,
        enable_crop_verify=enable_crop_verify,
    )
    result, prior_free_paths = _run_prior_free_runtime(
        result,
        target_text,
        settings,
        output_dir,
        image_path=image,
    )
    print(
        "[ScoreFusion] "
        f"confirmed={result.candidate_summary.get('num_confirmed', 0)} "
        f"rejected={result.candidate_summary.get('num_rejected', 0)} "
        f"verified={result.candidate_summary.get('num_verified', 0)}"
    )
    if enable_knowledge:
        knowledge_result = KnowledgeAwareAnalyzer(
            output_dir=output_dir,
            update_kb=True,
            settings=settings,
            enable_llm_reasoning=enable_llm_reasoning,
            enable_reasoning_memory=enable_reasoning_memory,
            quadruped_mode=quadruped_mode,
            allow_remote_reasoning=True,
            hide_unexecutable_actions=hide_unexecutable_actions,
            visual_retry_callback=visual_retry_callback,
        ).enrich_base_scene(result, target_text)
        output_paths = export_and_print_result(
            knowledge_result.base_scene,
            output_dir,
            image_path=image,
            settings=settings,
        )
        knowledge_paths = write_knowledge_aware_outputs(
            knowledge_result,
            output_dir,
            image_path=image,
        )
        _print_knowledge_outputs(knowledge_result, knowledge_paths)
        return (
            list(task_output_paths.values())
            + output_paths
            + list(accuracy_paths.values())
            + list(prior_free_paths.values())
            + list(knowledge_paths.values())
        )

    return (
        list(task_output_paths.values())
        + export_and_print_result(result, output_dir, image_path=image, settings=settings)
        + list(accuracy_paths.values())
        + list(prior_free_paths.values())
    )


def run_mock(
    mock_path: str | Path = "examples/mock_scene_result.json",
    enable_knowledge: bool = False,
    enable_llm_reasoning: bool | None = None,
    enable_reasoning_memory: bool | None = None,
    quadruped_mode: bool = True,
    hide_unexecutable_actions: bool = False,
    motion_profile: str | None = None,
    platform_obstacle_avoidance: bool | None = None,
    max_open_step: float | None = None,
    max_indoor_step: float | None = None,
    disable_dynamic_motion_horizon: bool = False,
    enable_llm_prior: bool | None = None,
    enable_observation_memory: bool | None = None,
    enable_evidence_gating: bool | None = None,
    disable_handwritten_priors: bool = False,
    disable_static_kb: bool = False,
    prior_audit: bool = False,
) -> list[Path]:
    path = Path(mock_path)
    if not path.is_file():
        raise FileNotFoundError(f"Mock result file not found: {path}")

    import json

    data = json.loads(path.read_text(encoding="utf-8"))
    result = SceneAnalysisResult.model_validate(data)
    if enable_knowledge:
        _warn_deprecated_enable_knowledge()
    settings = _apply_prior_free_flags(
        get_settings(),
        enable_knowledge=enable_knowledge,
        enable_llm_prior=enable_llm_prior,
        enable_observation_memory=enable_observation_memory,
        enable_evidence_gating=enable_evidence_gating,
        disable_handwritten_priors=disable_handwritten_priors,
        disable_static_kb=disable_static_kb,
        prior_audit=prior_audit,
    )
    settings = replace(
        settings,
        motion_horizon_profile=motion_profile or settings.motion_horizon_profile,
        platform_obstacle_avoidance_assumed=(
            platform_obstacle_avoidance
            if platform_obstacle_avoidance is not None
            else settings.platform_obstacle_avoidance_assumed
        ),
        motion_platform_open_max_step_m=(
            max_open_step
            if max_open_step is not None
            else settings.motion_platform_open_max_step_m
        ),
        motion_platform_indoor_max_step_m=(
            max_indoor_step
            if max_indoor_step is not None
            else settings.motion_platform_indoor_max_step_m
        ),
        enable_dynamic_motion_horizon=(
            False
            if disable_dynamic_motion_horizon
            else settings.enable_dynamic_motion_horizon
        ),
    )
    output_dir = Path(settings.output_dir)
    parsed_task, gate_result, nav_task = prepare_navigation_task_from_text(
        result.target_decision.target_text
    )
    task_output_paths = write_task_understanding_outputs(
        output_dir,
        parsed_task,
        gate_result,
        nav_task,
    )
    if not nav_task.executable:
        print(gate_result.user_feedback)
        return list(task_output_paths.values())
    target_text = navigation_target_text(nav_task, fallback=result.target_decision.target_text)
    result, prior_free_paths = _run_prior_free_runtime(
        result,
        target_text,
        settings,
        output_dir,
    )
    output_paths = export_and_print_result(result, output_dir, settings=settings)
    if not enable_knowledge:
        return list(task_output_paths.values()) + output_paths + list(prior_free_paths.values())

    knowledge_result = KnowledgeAwareAnalyzer(
        update_kb=False,
        enable_llm_reasoning=enable_llm_reasoning,
        enable_reasoning_memory=enable_reasoning_memory,
        quadruped_mode=quadruped_mode,
        hide_unexecutable_actions=hide_unexecutable_actions,
        settings=settings,
    ).enrich_base_scene(
        result,
        target_text,
    )
    knowledge_paths = write_knowledge_aware_outputs(
        knowledge_result,
        output_dir,
    )
    _print_knowledge_outputs(knowledge_result, knowledge_paths)
    return (
        list(task_output_paths.values())
        + output_paths
        + list(prior_free_paths.values())
        + list(knowledge_paths.values())
    )


def export_and_print_result(
    result: SceneAnalysisResult,
    output_dir: Path,
    image_path: str | Path | None = None,
    settings=None,
) -> list[Path]:
    result = prepare_analysis_result(result)
    output_paths = write_analysis_outputs(
        result,
        output_dir,
        image_path=image_path,
        settings=settings,
    )

    print(f"场景摘要：{result.scene_summary_zh}")
    print()
    print(format_target_decision(result))
    print()
    parsed_task, gate_result, nav_task = prepare_navigation_task_from_text(
        result.target_decision.target_text
    )
    print("自然语言任务解析与能力门控：")
    print(json.dumps(parsed_task_to_dict(parsed_task), ensure_ascii=False, indent=2))
    print(gate_result.user_feedback)
    print(f"进入导航管线：{nav_task.executable}")
    print()
    print(format_route_plan(result))
    print()
    print("已生成：")

    ordered_paths = [
        output_paths["scene_result"],
        output_paths["object_table"],
        output_paths["relation_table"],
        output_paths["topology_png"],
        output_paths["topology_graphml"],
        output_paths["ros2_motion_plan"],
        output_paths["motion_horizon_decision"],
    ]
    if "annotated_image" in output_paths:
        ordered_paths.append(output_paths["annotated_image"])
    for path in ordered_paths:
        print(path)

    return ordered_paths


def _apply_prior_free_flags(
    settings: Settings,
    *,
    enable_knowledge: bool,
    enable_llm_prior: bool | None,
    enable_observation_memory: bool | None,
    enable_evidence_gating: bool | None,
    disable_handwritten_priors: bool,
    disable_static_kb: bool,
    prior_audit: bool,
) -> Settings:
    if enable_knowledge:
        enable_llm_prior = True if enable_llm_prior is None else enable_llm_prior
        enable_observation_memory = (
            True if enable_observation_memory is None else enable_observation_memory
        )
        enable_evidence_gating = (
            True if enable_evidence_gating is None else enable_evidence_gating
        )
        disable_handwritten_priors = True
        disable_static_kb = True
    return replace(
        settings,
        llm_commonsense_prior_enabled=(
            settings.llm_commonsense_prior_enabled
            if enable_llm_prior is None
            else enable_llm_prior
        ),
        observation_memory_enabled=(
            settings.observation_memory_enabled
            if enable_observation_memory is None
            else enable_observation_memory
        ),
        evidence_gating_enabled=(
            settings.evidence_gating_enabled
            if enable_evidence_gating is None
            else enable_evidence_gating
        ),
        handwritten_object_priors_enabled=(
            False
            if disable_handwritten_priors
            else settings.handwritten_object_priors_enabled
        ),
        handwritten_location_priors_enabled=(
            False
            if disable_handwritten_priors
            else settings.handwritten_location_priors_enabled
        ),
        handwritten_room_priors_enabled=(
            False if disable_handwritten_priors else settings.handwritten_room_priors_enabled
        ),
        allow_handcrafted_search_rules=(
            False if disable_handwritten_priors else settings.allow_handcrafted_search_rules
        ),
        static_knowledge_base_enabled=(
            False if disable_static_kb else settings.static_knowledge_base_enabled
        ),
        prior_usage_audit_enabled=prior_audit or settings.prior_usage_audit_enabled,
    )


def _run_prior_free_runtime(
    result: SceneAnalysisResult,
    target_text: str,
    settings: Settings,
    output_dir: Path,
    image_path: str | Path | None = None,
) -> tuple[SceneAnalysisResult, dict[str, Path]]:
    output_dir.mkdir(parents=True, exist_ok=True)
    paths: dict[str, Path] = {}
    memory_store = ObservationMemoryStore(settings=settings)
    memory_summaries = (
        memory_store.retrieve(target_text, settings.observation_memory_retrieval_top_k)
        if settings.observation_memory_enabled
        else []
    )
    llm_prior = LLMPriorGenerator(settings=settings).generate(
        LLMPriorInput(
            target=target_text,
            scene_summary=result.scene_summary_zh,
            observed_objects=_observed_objects_payload(result),
            observed_relations=[
                item.model_dump(mode="json") for item in result.relations
            ],
            observation_memory_summaries=memory_summaries,
            robot_capabilities={
                "platform": "quadruped",
                "can_manipulate": settings.quadruped_can_manipulate,
                "can_open_container": settings.quadruped_can_open_container,
                "can_look_down": settings.quadruped_can_look_down,
            },
            language=settings.llm_prior_output_language,
        )
    )
    paths["llm_generated_priors"] = _write_json(
        llm_prior,
        output_dir / "llm_generated_priors.json",
    )
    dynamic_prompts = build_dynamic_detector_prompts(
        target_text,
        llm_prior,
        result.scene_summary_zh,
        max_prompts=settings.llm_prior_max_detector_prompts,
    )
    paths["dynamic_detector_prompts"] = _write_json(
        dynamic_prompts,
        output_dir / "dynamic_detector_prompts.json",
    )
    result, gate_report = gate_scene_target(result, settings)
    paths["evidence_gating_report"] = _write_json(
        gate_report,
        output_dir / "evidence_gating_report.json",
    )
    memory_updates = build_scene_memory_updates(result, gate_report, image_path)
    written_memory: list[dict] = []
    memory_error = None
    if settings.observation_memory_enabled and memory_updates:
        try:
            written_memory = memory_store.append_many(memory_updates)
        except ValueError as exc:
            memory_error = str(exc)
    paths["observation_memory_updates"] = _write_json(
        {
            "enabled": settings.observation_memory_enabled,
            "memory_store_path": settings.observation_memory_store_path,
            "retrieved_count": len(memory_summaries),
            "update_count": len(memory_updates),
            "written_count": len(written_memory),
            "error": memory_error,
            "memory_updates": memory_updates,
        },
        output_dir / "observation_memory_updates.json",
    )
    audit = build_prior_usage_report(
        settings=settings,
        llm_prior=llm_prior,
        dynamic_prompts=dynamic_prompts,
        evidence_report=gate_report,
        observation_memory_used=bool(memory_summaries or written_memory),
        static_kb_used=settings.static_knowledge_base_enabled,
        handcrafted_priors_used=(
            settings.handwritten_object_priors_enabled
            or settings.handwritten_location_priors_enabled
            or settings.handwritten_room_priors_enabled
            or settings.allow_handcrafted_search_rules
        ),
    )
    paths["prior_usage_report"] = write_prior_usage_report(
        audit,
        settings.prior_usage_report_path,
    )
    return result, paths


def _observed_objects_payload(result: SceneAnalysisResult) -> list[dict]:
    return [
        {
            "object_id": obj.id,
            "label": obj.name,
            "label_zh": obj.name_zh,
            "bbox": [
                obj.bbox_2d.x1,
                obj.bbox_2d.y1,
                obj.bbox_2d.x2,
                obj.bbox_2d.y2,
            ],
            "source": "current_visual_observation",
            "confidence": obj.confidence,
        }
        for obj in result.objects
    ]


def _write_json(payload: object, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return path


def _warn_deprecated_enable_knowledge() -> None:
    print(
        "--enable-knowledge is deprecated. Use --enable-llm-prior "
        "--enable-observation-memory --enable-evidence-gating instead.",
        file=sys.stderr,
    )


def _print_knowledge_outputs(
    result,
    output_paths: dict[str, Path],
) -> None:
    print()
    print("知识增强推理：")
    print(result.reasoning_summary_zh)
    print()
    print("任务规划：")
    print(result.task_plan.summary_zh)
    print()
    print("知识增强输出：")
    for path in output_paths.values():
        print(path)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.video:
            if not args.target:
                raise ValueError("--video 模式必须提供 --target。")
            settings = get_settings()
            video_config = argparse.Namespace(
                output_dir=settings.output_dir,
                sample_fps=settings.video_sample_fps,
                max_frames=settings.video_max_frames,
                enable_knowledge=args.enable_knowledge,
                enable_video_memory=settings.video_enable_scene_memory,
                enable_video_psg=settings.video_enable_video_psg,
                enable_navigation_topology=settings.video_enable_navigation_topology,
                enable_scene_mapping=settings.video_enable_scene_mapping,
                use_scene_map_for_search=settings.video_use_scene_map_for_search,
                no_annotate=False,
                enable_llm_prior=args.enable_llm_prior,
                enable_observation_memory=args.enable_observation_memory,
                enable_evidence_gating=args.enable_evidence_gating,
                disable_handwritten_priors=args.disable_handwritten_priors,
                disable_static_kb=args.disable_static_kb,
                prior_audit=args.prior_audit,
                enable_video_navigation=args.enable_video_navigation,
                video_navigation_mode=args.video_navigation_mode,
                video_pose_backend=args.video_pose_backend,
                depth_dir=args.depth_dir,
                video_map_transform_json=args.video_map_transform_json,
                force_exploration=args.force_exploration,
            )
            result = run_video_target_search_pipeline(
                video_path=args.video,
                target=args.target,
                detector=args.detector or settings.detection_backend,
                config=video_config,
                enable_tracking=settings.video_enable_tracking,
                enable_crop_verify=(
                    settings.enable_crop_verify
                    if args.enable_crop_verify is None
                    else args.enable_crop_verify
                ),
                enable_evidence_gating=(
                    settings.evidence_gating_enabled
                    if args.enable_evidence_gating is None
                    else args.enable_evidence_gating
                ),
                enable_scene_mapping=settings.video_enable_scene_mapping,
                enable_navigation_topology=settings.video_enable_navigation_topology,
                use_scene_map_for_search=settings.video_use_scene_map_for_search,
            )
            nav_summary = result.get("video_navigation", {}).get("summary", {})
            print("视频分析完成。")
            print(f"目标状态：{result.get('target_status')}")
            if nav_summary:
                print(
                    "视频导航规划："
                    f"{nav_summary.get('navigation_strategy')} / "
                    f"{nav_summary.get('scale_status')} / "
                    f"executable={nav_summary.get('executable')}"
                )
            print("已生成：")
            for path in result.get("output_files", {}).values():
                print(Path(path))
            run_nav2_from_args(
                args,
                task_context={
                    "natural_language_task": args.target,
                    "target_status": result.get("target_status"),
                    "source_pipeline": "video",
                },
            )
            return 0
        if args.mock:
            run_mock(
                enable_knowledge=args.enable_knowledge,
                enable_llm_reasoning=args.enable_llm_reasoning,
                enable_reasoning_memory=args.enable_reasoning_memory,
                quadruped_mode=(
                    args.quadruped_mode or args.enable_llm_reasoning is not False
                ),
                hide_unexecutable_actions=args.hide_unexecutable_actions,
                motion_profile=args.motion_profile,
                platform_obstacle_avoidance=(
                    True if args.platform_obstacle_avoidance else None
                ),
                max_open_step=args.max_open_step,
                max_indoor_step=args.max_indoor_step,
                disable_dynamic_motion_horizon=args.disable_dynamic_motion_horizon,
                enable_llm_prior=args.enable_llm_prior,
                enable_observation_memory=args.enable_observation_memory,
                enable_evidence_gating=args.enable_evidence_gating,
                disable_handwritten_priors=args.disable_handwritten_priors,
                disable_static_kb=args.disable_static_kb,
                prior_audit=args.prior_audit,
            )
        else:
            if not args.image or not args.target:
                raise ValueError("非 mock 模式必须同时提供 --image 和 --target。")
            run(
                args.image,
                args.target,
                args.detector,
                enable_knowledge=args.enable_knowledge,
                enable_target_profile=args.enable_target_profile,
                enable_crop_verify=args.enable_crop_verify,
                high_recall=args.high_recall,
                box_threshold=args.box_threshold,
                text_threshold=args.text_threshold,
                max_candidates=args.max_candidates,
                enable_llm_reasoning=args.enable_llm_reasoning,
                enable_reasoning_memory=args.enable_reasoning_memory,
                quadruped_mode=(
                    args.quadruped_mode or args.enable_llm_reasoning is not False
                ),
                hide_unexecutable_actions=args.hide_unexecutable_actions,
                motion_profile=args.motion_profile,
                platform_obstacle_avoidance=(
                    True if args.platform_obstacle_avoidance else None
                ),
                max_open_step=args.max_open_step,
                max_indoor_step=args.max_indoor_step,
                disable_dynamic_motion_horizon=args.disable_dynamic_motion_horizon,
                enable_llm_prior=args.enable_llm_prior,
                enable_observation_memory=args.enable_observation_memory,
                enable_evidence_gating=args.enable_evidence_gating,
                disable_handwritten_priors=args.disable_handwritten_priors,
                disable_static_kb=args.disable_static_kb,
                prior_audit=args.prior_audit,
            )
        run_nav2_from_args(
            args,
            task_context={"natural_language_task": args.target or "mock", "source_pipeline": "single_image"},
        )
    except (
        SettingsError,
        FileNotFoundError,
        ImportError,
        DetectorRuntimeError,
        OpenAIError,
        ValueError,
        ValidationError,
    ) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
