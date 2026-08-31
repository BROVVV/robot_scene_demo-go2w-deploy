"""Scene analysis orchestration service."""

from __future__ import annotations

import json
from pathlib import Path

from app.config import DEFAULT_OUTPUT_DIR
from app.detectors.base import BaseObjectDetector
from app.llm_clients.base import BaseVisionLLMClient
from app.schemas import SceneAnalysisResult
from app.services.detector_scene_builder import build_scene_from_detections


class SceneAnalyzer:
    """Run scene analysis through a vision LLM client and persist the result."""

    def __init__(
        self,
        llm_client: BaseVisionLLMClient | None = None,
        object_detector: BaseObjectDetector | None = None,
        output_dir: str | Path = DEFAULT_OUTPUT_DIR,
        enable_low_object_retry: bool = False,
        min_objects_for_complex_scene: int = 10,
    ) -> None:
        self.llm_client = llm_client
        self.object_detector = object_detector
        self.output_dir = Path(output_dir)
        self.enable_low_object_retry = enable_low_object_retry
        self.min_objects_for_complex_scene = min_objects_for_complex_scene

    def analyze(
        self,
        image_path: str,
        target_text: str,
        extra_instructions: str | None = None,
    ) -> SceneAnalysisResult:
        if self.object_detector is not None:
            detections = self.object_detector.detect(image_path, target_text)
            result = build_scene_from_detections(detections, target_text)
            self.save_result(result)
            return result

        if self.llm_client is None:
            raise ValueError("SceneAnalyzer requires either llm_client or object_detector.")

        raw_result = self.llm_client.analyze_scene(
            image_path,
            target_text,
            extra_instructions=extra_instructions,
        )
        result = SceneAnalysisResult.model_validate(raw_result)
        if (
            self.enable_low_object_retry
            and len(result.objects) < self.min_objects_for_complex_scene
        ):
            raw_result = self.llm_client.analyze_scene(
                image_path,
                target_text,
                extra_instructions=_build_low_object_count_retry_prompt(
                    result, self.min_objects_for_complex_scene
                ),
            )
            result = SceneAnalysisResult.model_validate(raw_result)
        self.save_result(result)
        return result

    def save_result(self, result: SceneAnalysisResult) -> Path:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        output_path = self.output_dir / "scene_result.json"
        output_path.write_text(
            json.dumps(result.model_dump(mode="json"), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return output_path


def _build_low_object_count_retry_prompt(
    previous_result: SceneAnalysisResult,
    min_objects: int,
) -> str:
    previous_objects = "、".join(
        f"{obj.id}:{obj.name_zh}" for obj in previous_result.objects
    )
    return (
        f"上一次你只识别出 {len(previous_result.objects)} 个物体"
        f"（{previous_objects or '无'}），低于最低要求 {min_objects} 个。"
        "这张图是复杂室内场景，请重新从左到右、从近到远逐区扫描，"
        "不要只保留目标相关物体。必须补充所有可见且可命名的独立物体，"
        "包括多把椅子、多件衣服、人物、桌子、柜子、抽屉、显示器、电脑主机、"
        "机器人/设备、箱子、篮子、鞋、瓶子/杯子、线缆、架子、门、地面上物体。"
        "局部可见或被遮挡但能判断类别的物体也要加入，并在 attributes 中写 partial/occluded。"
        "如果确实无法达到最低数量，必须在 scene_summary_zh 中明确说明为什么。"
    )
