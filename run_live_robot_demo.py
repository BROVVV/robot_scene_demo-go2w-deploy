"""Consume atomic Go2-W frame bundles with the existing perception pipeline."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from app.live_robot.frame_bundle_reader import FrameBundleReader, FrameBundleUnavailable
from app.live_robot.live_search_pipeline import run_live_bundle_search


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Go2-W stationary live target search")
    parser.add_argument("--target", required=True)
    parser.add_argument("--detector", choices=["mock", "llm", "grounded_sam"], default="llm")
    parser.add_argument(
        "--search-mode",
        choices=["observe_only", "step_search", "nav2_plan_only", "nav2_execute"],
        default="observe_only",
    )
    parser.add_argument("--spool-root", default="runtime/go2w/spool")
    parser.add_argument("--output-root", default="outputs/live_sessions")
    parser.add_argument("--max-frames", type=int, default=5)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument("--poll-interval-sec", type=float, default=0.1)
    parser.add_argument("--enable-video-memory", action="store_true")
    parser.add_argument("--enable-video-psg", action="store_true")
    parser.add_argument("--enable-scene-mapping", action="store_true")
    parser.add_argument("--enable-navigation-topology", action="store_true")
    parser.add_argument("--disable-crop-verify", action="store_true")
    parser.add_argument(
        "--disable-llm-profile",
        action="store_true",
        help="Resolve the target profile with the deterministic fallback and do not call any external LLM.",
    )
    parser.add_argument("--no-annotate", action="store_true")
    parser.add_argument("--semantic-reasoning", action="store_true")
    parser.add_argument(
        "--search-reasoner", choices=["legacy", "semantic_navigation", "hybrid"],
        default="legacy",
    )
    parser.add_argument(
        "--search-reasoner-mode", choices=["shadow", "active"],
        default="shadow",
    )
    return parser.parse_args(argv)


def collect_bundles(reader, *, max_frames, timeout_sec, poll_interval):
    if max_frames < 1:
        raise ValueError("max_frames must be positive")
    if timeout_sec <= 0.0 or poll_interval <= 0.0:
        raise ValueError("timeout and poll interval must be positive")
    bundles = {}
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline and len(bundles) < max_frames:
        try:
            bundle = reader.read_latest()
            key = (bundle.payload["session_id"], bundle.frame_id)
            bundles[key] = bundle
            health = bundle.payload.get("sensor_health") or {}
            if not health.get("camera") or not health.get("lidar"):
                break
        except FrameBundleUnavailable:
            pass
        time.sleep(poll_interval)
    if not bundles:
        raise FrameBundleUnavailable("no complete bundle arrived before timeout")
    return sorted(bundles.values(), key=lambda item: item.frame_id)


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.search_mode != "observe_only":
        print(
            json.dumps(
                {
                    "status": "blocked",
                    "reason": "motion_and_nav2_modes_are_not_authorized_in_this_deployment",
                    "search_mode": args.search_mode,
                },
                ensure_ascii=False,
            )
        )
        return 3
    try:
        bundles = collect_bundles(
            FrameBundleReader(args.spool_root),
            max_frames=args.max_frames,
            timeout_sec=args.timeout_sec,
            poll_interval=args.poll_interval_sec,
        )
        session_id = str(bundles[-1].payload["session_id"])
        output = Path(args.output_root) / session_id
        result = run_live_bundle_search(
            bundles,
            target=args.target,
            detector=args.detector,
            output_dir=output,
            search_mode=args.search_mode,
            annotate=not args.no_annotate,
            enable_crop_verify=not args.disable_crop_verify,
            use_llm_profile=not args.disable_llm_profile,
            semantic_reasoning=args.semantic_reasoning,
            search_reasoner=args.search_reasoner,
            search_reasoner_mode=args.search_reasoner_mode,
        )
        result["requested_features"] = {
            "video_memory": args.enable_video_memory,
            "video_psg": args.enable_video_psg,
            "scene_mapping": args.enable_scene_mapping,
            "navigation_topology": args.enable_navigation_topology,
        }
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result.get("status") != "blocked_wait_for_sensors" else 3
    except (FrameBundleUnavailable, ValueError, RuntimeError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}, ensure_ascii=False))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
