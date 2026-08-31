"""Command-line entry point for prerecorded first-person video understanding."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from openai import OpenAIError
from pydantic import ValidationError

from app.config import DEFAULT_OUTPUT_DIR, SettingsError
from app.detectors.grounded_sam_subprocess import DetectorRuntimeError
from app.video.full_scene_mapper import VideoFullSceneMapper
from app.video.video_target_search_pipeline import run_video_target_search_pipeline
from app.video.video_reader import VideoReadError
from app.navigation.nav2_cli import add_nav2_arguments, run_nav2_from_args


def normalize_video_args(args: argparse.Namespace) -> argparse.Namespace:
    """Normalize legacy scene-map flags into the target-search-first contract."""

    target_text = args.target.strip()
    legacy_full_scene_requested = (
        args.enable_full_scene_map or args.mode == "full_scene_map"
    )
    if legacy_full_scene_requested and target_text and not args.scene_map_only:
        args.mode = "target_search"
        args.enable_scene_mapping = True
        args.enable_navigation_topology = True
    elif legacy_full_scene_requested:
        args.mode = "scene_map_only"

    if args.scene_map_only:
        args.mode = "scene_map_only"

    if target_text and args.mode == "scene_map_only":
        raise ValueError(
            "检测到 --target，但当前是 scene_map_only。"
            "如果要找目标，请使用 --mode target_search --enable-scene-mapping。"
        )

    if target_text:
        args.mode = "target_search"

    if args.use_scene_map_for_search:
        args.enable_navigation_topology = True
        args.enable_scene_mapping = True
    elif args.enable_navigation_topology:
        args.enable_scene_mapping = True

    if not args.enable_scene_mapping:
        args.enable_navigation_topology = False
        args.use_scene_map_for_search = False

    return args


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="输入机器狗第一视角视频，执行目标搜索，并可选启用场景建图辅助。"
    )
    parser.add_argument("--video", required=True, help="输入视频路径")
    parser.add_argument("--target", default="", help="要寻找的目标物；scene_map_only 模式可省略")
    parser.add_argument(
        "--mode",
        choices=["target_search", "scene_map_only", "full_scene_map"],
        default="target_search",
        help="视频运行模式。默认 target_search。scene_map_only 仅用于高级调试；full_scene_map 为旧命令兼容。",
    )
    parser.add_argument(
        "--enable-full-scene-map",
        action="store_true",
        help="兼容开关；带 --target 时转为目标搜索内建图辅助，否则等价于 --scene-map-only",
    )
    parser.add_argument(
        "--scene-map-only",
        action="store_true",
        help="只调试全场景建图，不执行目标搜索",
    )
    parser.add_argument(
        "--enable-scene-mapping",
        action="store_true",
        help="在目标搜索之后额外执行全场景建图辅助",
    )
    parser.add_argument(
        "--use-scene-map-for-search",
        action="store_true",
        help="使用场景拓扑给后续目标搜索区域排序",
    )
    parser.add_argument(
        "--target-required-for-search",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="target_search 模式下是否要求 --target",
    )
    parser.add_argument(
        "--detector",
        choices=["mock", "llm", "grounded_sam"],
        default="llm",
        help="逐关键帧使用的检测后端",
    )
    parser.add_argument("--sample-fps", type=float, default=None, help="每秒采样帧数，默认读取配置")
    parser.add_argument("--max-frames", type=int, default=None, help="最大分析帧数，默认读取配置")
    parser.add_argument("--enable-knowledge", action="store_true", help="启用上下文候选区域规则")
    parser.add_argument(
        "--enable-video-memory",
        action="store_true",
        help="启用逐帧场景记忆、负证据、长期记忆和视频 PSG",
    )
    video_psg_group = parser.add_mutually_exclusive_group()
    video_psg_group.add_argument(
        "--enable-video-psg", dest="enable_video_psg", action="store_true"
    )
    video_psg_group.add_argument(
        "--disable-video-psg", dest="enable_video_psg", action="store_false"
    )
    topology_group = parser.add_mutually_exclusive_group()
    topology_group.add_argument(
        "--build-navigation-topology",
        "--enable-navigation-topology",
        dest="enable_navigation_topology",
        action="store_true",
    )
    topology_group.add_argument(
        "--disable-navigation-topology",
        dest="enable_navigation_topology",
        action="store_false",
    )
    parser.add_argument("--psg-max-predicted-nodes", type=int)
    parser.add_argument("--psg-confidence-threshold", type=float)
    parser.add_argument("--topology-observed-only", action="store_true")
    frame_obs_group = parser.add_mutually_exclusive_group()
    frame_obs_group.add_argument(
        "--save-frame-observations",
        dest="save_frame_observations",
        action="store_true",
    )
    frame_obs_group.add_argument(
        "--no-save-frame-observations",
        dest="save_frame_observations",
        action="store_false",
    )
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR, help="输出目录")
    video_nav_group = parser.add_mutually_exclusive_group()
    video_nav_group.add_argument(
        "--enable-video-navigation",
        dest="enable_video_navigation",
        action="store_true",
        help="分析视频后自动生成 Video-to-Navigation 视觉规划",
    )
    video_nav_group.add_argument(
        "--disable-video-navigation",
        dest="enable_video_navigation",
        action="store_false",
        help="关闭自动 Video-to-Navigation 视觉规划",
    )
    parser.add_argument(
        "--video-navigation-mode",
        choices=["visual_preview", "metric_preview", "plan_only", "execute"],
        default=None,
        help="视频导航规划模式；普通 RGB 默认 visual_preview",
    )
    parser.add_argument(
        "--video-pose-backend",
        choices=["auto", "mock", "relative", "metric"],
        default=None,
        help="视频轨迹估计后端",
    )
    parser.add_argument("--depth-dir", help="RGB-D/深度帧目录；存在时允许 metric 轨迹接口")
    parser.add_argument(
        "--video-map-transform-json",
        help="包含 T_map_video_map 的 JSON；只有提供可靠尺度/外参时才用于 Nav2 handoff",
    )
    parser.add_argument(
        "--force-exploration",
        action="store_true",
        help="忽略目标线索，强制生成探索式导航规划",
    )
    parser.add_argument("--no-annotate", action="store_true", help="不生成标注关键帧")
    tracking_group = parser.add_mutually_exclusive_group()
    tracking_group.add_argument(
        "--enable-tracking", dest="enable_tracking", action="store_true"
    )
    tracking_group.add_argument(
        "--disable-tracking", dest="enable_tracking", action="store_false"
    )
    crop_group = parser.add_mutually_exclusive_group()
    crop_group.add_argument(
        "--enable-crop-verify", dest="enable_crop_verify", action="store_true"
    )
    crop_group.add_argument(
        "--disable-crop-verify", dest="enable_crop_verify", action="store_false"
    )
    parser.add_argument("--verify-every-n-frames", type=int)
    parser.add_argument("--track-iou-threshold", type=float)
    parser.add_argument("--target-confirm-min-frames", type=int)
    parser.add_argument("--target-confirm-score", type=float)
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
    parser.add_argument("--disable-handwritten-priors", action="store_true")
    parser.add_argument("--disable-static-kb", action="store_true")
    parser.add_argument("--prior-audit", action="store_true")
    parser.set_defaults(
        enable_tracking=None,
        enable_crop_verify=None,
        enable_llm_prior=None,
        enable_observation_memory=None,
        enable_evidence_gating=None,
        enable_video_psg=None,
        enable_navigation_topology=None,
        topology_observed_only=None,
        save_frame_observations=None,
        enable_video_navigation=None,
    )
    add_nav2_arguments(parser)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    try:
        args = normalize_video_args(parse_args(argv))
        target_text = args.target.strip()
        mode = args.mode
        if mode == "scene_map_only":
            result, paths = VideoFullSceneMapper(output_dir=args.output_dir).run(
                video_path=args.video,
                detector=args.detector,
                sample_fps=args.sample_fps,
                max_frames=args.max_frames,
                enable_video_memory=args.enable_video_memory,
                enable_video_psg=args.enable_video_psg,
                enable_navigation_topology=args.enable_navigation_topology,
                psg_max_predicted_nodes=args.psg_max_predicted_nodes,
                psg_confidence_threshold=args.psg_confidence_threshold,
                topology_observed_only=args.topology_observed_only,
                save_frame_observations=args.save_frame_observations,
                annotate=not args.no_annotate,
            )
            print(result.summary_zh)
            print("已生成：")
            for path in paths.values():
                print(Path(path))
            return 0

        if args.target_required_for_search and not args.target.strip():
            raise ValueError("target_search 模式必须提供 --target；scene_map_only 模式可省略。")
        if args.enable_knowledge:
            print(
                "--enable-knowledge is deprecated. Use --enable-llm-prior "
                "--enable-observation-memory --enable-evidence-gating instead.",
                file=sys.stderr,
            )
        result = run_video_target_search_pipeline(
            video_path=args.video,
            target=args.target,
            detector=args.detector,
            config=args,
            enable_tracking=(
                True if args.enable_tracking is None else args.enable_tracking
            ),
            enable_crop_verify=(
                True if args.enable_crop_verify is None else args.enable_crop_verify
            ),
            enable_evidence_gating=(
                True
                if args.enable_evidence_gating is None
                else args.enable_evidence_gating
            ),
            enable_scene_mapping=args.enable_scene_mapping,
            enable_navigation_topology=bool(args.enable_navigation_topology),
            use_scene_map_for_search=args.use_scene_map_for_search,
        )
    except (
        SettingsError,
        FileNotFoundError,
        ImportError,
        VideoReadError,
        DetectorRuntimeError,
        OpenAIError,
        ValidationError,
        RuntimeError,
        ValueError,
    ) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1

    if result["target_confirmed"]:
        best = result["best_evidence"]
        print(f"已找到目标：{args.target}")
        print(f"最佳证据：{best['timestamp_sec']:.2f}s，置信度 {best['confidence']:.3f}")
    else:
        print(f"未直接找到目标：{args.target}")
        print(f"原因：{result['reason']}")
        if result.get("environment_memories_written", 0) > 0:
            print(
                "已生成并写入环境记忆："
                f"{result['environment_memories_written']} 条；"
                f"负目标证据：{result.get('negative_evidence_count', 0)} 条；"
                f"PSG 假设：{len(result.get('psg_hypotheses', []))} 条。"
            )
    decision = result.get("navigation_decision", {})
    print(f"目标状态：{result.get('target_status')}")
    print(decision.get("reason") or result["navigation_interpretation"]["suggestion"])
    if result.get("video_navigation"):
        nav_summary = result["video_navigation"].get("summary", {})
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


if __name__ == "__main__":
    raise SystemExit(main())
