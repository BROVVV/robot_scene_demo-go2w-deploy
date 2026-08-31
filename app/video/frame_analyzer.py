"""Reuse the existing single-image analyzers for sampled video frames."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw

from app.config import get_settings
from app.detectors.grounded_sam_subprocess import GroundedSAMSubprocessDetector
from app.llm_clients.siliconflow_client import SiliconFlowVisionClient
from app.schemas import SceneAnalysisResult
from app.services.image_annotator import export_annotated_image
from app.services.scene_analyzer import SceneAnalyzer
from app.video.models import FrameAnalysisResult, VideoFrame
from app.video.spatial_context import get_image_position
from app.video.target_profile import TargetProfile
from app.video.target_search import target_label_match_score


class FrameAnalyzer:
    """Analyze multiple frames without rebuilding the detector for each frame."""

    def __init__(
        self,
        detector: str,
        target: str,
        output_dir: str | Path,
        target_profile: TargetProfile | None = None,
        annotate: bool = True,
        mock_path: str | Path = "examples/mock_scene_result.json",
        scene_dir_name: str = "video_scene_results",
        annotated_dir_name: str = "video_frames_annotated",
    ) -> None:
        self.detector = detector
        self.target = target
        self.target_profile = target_profile
        self.output_dir = Path(output_dir)
        self.scene_dir = self.output_dir / scene_dir_name
        self.annotated_dir = self.output_dir / annotated_dir_name
        self.annotate = annotate
        self.mock_path = Path(mock_path)
        self.analyzer = self._build_analyzer()

    def analyze(self, frame: VideoFrame) -> FrameAnalysisResult:
        result = self._analyze_scene(frame)
        objects = [
            _object_payload(frame, obj.model_dump(mode="json"))
            for obj in result.objects
        ]
        relations = [relation.model_dump(mode="json") for relation in result.relations]

        self.scene_dir.mkdir(parents=True, exist_ok=True)
        raw_path = self.scene_dir / f"frame_{frame.frame_id:06d}_scene_result.json"
        raw_path.write_text(
            json.dumps(result.model_dump(mode="json"), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        annotated_path: Path | None = None
        if self.annotate:
            annotated_path = self.annotated_dir / f"frame_{frame.frame_id:06d}.jpg"
            export_annotated_image(result, frame.image_path, annotated_path)
            _add_timestamp(annotated_path, frame.timestamp_sec)

        return FrameAnalysisResult(
            frame_id=frame.frame_id,
            timestamp_sec=frame.timestamp_sec,
            image_path=str(frame.image_path),
            annotated_frame_path=str(annotated_path) if annotated_path else None,
            scene_summary=result.scene_summary_zh,
            objects=objects,
            relations=relations,
            raw_result_path=str(raw_path),
            metadata={
                "target_decision": result.target_decision.model_dump(mode="json"),
                "detector": self.detector,
                "target_profile": (
                    self.target_profile.to_dict() if self.target_profile else None
                ),
            },
        )

    def _build_analyzer(self) -> SceneAnalyzer | None:
        if self.detector == "mock":
            return None
        settings = get_settings()
        if self.detector == "grounded_sam":
            return SceneAnalyzer(
                object_detector=GroundedSAMSubprocessDetector(
                    settings,
                    target_profile=self.target_profile,
                ),
                output_dir=self.scene_dir,
            )
        if self.detector == "llm":
            return SceneAnalyzer(
                llm_client=SiliconFlowVisionClient(settings=settings),
                output_dir=self.scene_dir,
                enable_low_object_retry=False,
                min_objects_for_complex_scene=settings.min_objects_for_complex_scene,
            )
        raise ValueError(f"Unsupported video detector: {self.detector}")

    def _analyze_scene(self, frame: VideoFrame) -> SceneAnalysisResult:
        if self.detector == "mock":
            if not self.mock_path.is_file():
                raise FileNotFoundError(f"Mock result file not found: {self.mock_path}")
            data = json.loads(self.mock_path.read_text(encoding="utf-8"))
            matched_ids = [
                obj["id"]
                for obj in data.get("objects", [])
                if target_label_match_score(
                    self.target,
                    str(obj.get("name", "")),
                    str(obj.get("name_zh", "")),
                )
                > 0
            ]
            data["target_decision"] = {
                "target_text": self.target,
                "is_present": bool(matched_ids),
                "matched_object_ids": matched_ids,
                "match_reason_zh": (
                    "Mock 场景中存在匹配物体。"
                    if matched_ids
                    else "Mock 场景中没有与目标匹配的物体。"
                ),
                "confidence": 0.9 if matched_ids else 0.0,
            }
            if not matched_ids:
                data["route_plan"] = {
                    "route_type": "explore_likely_location",
                    "summary_zh": "当前帧未观察到目标，仅用于视频搜索流程测试。",
                    "steps": [
                        {
                            "step_id": 1,
                            "action": "stop",
                            "distance_m": None,
                            "turn_angle_deg": None,
                            "description_zh": "停止并继续检查后续关键帧",
                        }
                    ],
                    "safety_notes_zh": ["Mock 数据，仅用于流程测试"],
                }
            return SceneAnalysisResult.model_validate(data)
        assert self.analyzer is not None
        return self.analyzer.analyze(
            str(frame.image_path),
            self.target,
            extra_instructions=(
                self.target_profile.prompt_context()
                if self.detector == "llm" and self.target_profile
                else None
            ),
        )


def analyze_frame(
    frame: VideoFrame,
    target: str,
    detector: str,
    output_dir: str | Path,
    annotate: bool = True,
) -> FrameAnalysisResult:
    """Convenience wrapper for callers analyzing a single frame."""
    return FrameAnalyzer(detector, target, output_dir, annotate=annotate).analyze(frame)


def highlight_target_candidates(
    frame_result: FrameAnalysisResult,
    target: str,
) -> None:
    """Overlay an explicit target marker after cross-frame matching."""
    if not frame_result.annotated_frame_path:
        return
    path = Path(frame_result.annotated_frame_path)
    if not path.is_file():
        return
    candidates = [
        obj for obj in frame_result.objects if obj.get("is_target_candidate")
    ]
    if not candidates:
        return

    with Image.open(path) as source:
        image = source.convert("RGB")
    draw = ImageDraw.Draw(image)
    line_width = max(4, image.width // 180)
    for obj in candidates:
        x1, y1, x2, y2 = obj.get("bbox", [0.0, 0.0, 1.0, 1.0])
        pixels = (
            int(x1 * image.width),
            int(y1 * image.height),
            int(x2 * image.width),
            int(y2 * image.height),
        )
        draw.rectangle(pixels, outline="#00ff66", width=line_width)
        label = (
            f"TARGET {obj.get('label') or target} "
            f"{float(obj.get('confidence', 0.0)):.2f} @ {frame_result.timestamp_sec:.2f}s"
        )
        text_box = draw.textbbox((0, 0), label)
        text_width = text_box[2] - text_box[0]
        text_height = text_box[3] - text_box[1]
        label_y = max(0, pixels[1] - text_height - 10)
        draw.rectangle(
            (pixels[0], label_y, pixels[0] + text_width + 10, label_y + text_height + 8),
            fill="#008f3a",
        )
        draw.text((pixels[0] + 5, label_y + 4), label, fill="white")
    image.save(path)


def _object_payload(frame: VideoFrame, obj: dict[str, Any]) -> dict[str, Any]:
    bbox_data = obj.get("bbox_2d") or {}
    bbox = [
        float(bbox_data.get("x1", 0.0)),
        float(bbox_data.get("y1", 0.0)),
        float(bbox_data.get("x2", 1.0)),
        float(bbox_data.get("y2", 1.0)),
    ]
    mask_area_ratio = None
    for attribute in obj.get("attributes") or []:
        if str(attribute).startswith("mask_area_ratio="):
            try:
                mask_area_ratio = float(str(attribute).split("=", 1)[1])
            except ValueError:
                pass
    return {
        "object_id": f"frame_{frame.frame_id:06d}_{obj['id']}",
        "source_object_id": obj["id"],
        "track_id": None,
        "label": obj.get("name", "object"),
        "label_zh": obj.get("name_zh", "物体"),
        "category": obj.get("category", "unknown"),
        "color": obj.get("color"),
        "confidence": float(obj.get("confidence", 0.0)),
        "detector_score": obj.get("detector_score") or obj.get("confidence"),
        "text_score": obj.get("text_score"),
        "bbox": bbox,
        "mask_area_ratio": mask_area_ratio,
        "image_position": get_image_position(bbox),
        "is_target_candidate": False,
        "attributes": obj.get("attributes") or [],
        "final_score": obj.get("final_score"),
        "decision": obj.get("decision"),
        "crop_verify": obj.get("crop_verify"),
    }


def _add_timestamp(path: Path, timestamp_sec: float) -> None:
    with Image.open(path) as source:
        image = source.convert("RGB")
    draw = ImageDraw.Draw(image)
    label = f"t={timestamp_sec:.2f}s"
    padding = 8
    text_box = draw.textbbox((0, 0), label)
    width = text_box[2] - text_box[0]
    height = text_box[3] - text_box[1]
    draw.rectangle((8, 8, 8 + width + padding * 2, 8 + height + padding), fill="#111827")
    draw.text((8 + padding, 8 + padding // 2), label, fill="white")
    image.save(path)
