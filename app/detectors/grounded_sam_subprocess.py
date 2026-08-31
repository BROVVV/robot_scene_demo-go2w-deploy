"""Grounding DINO + SAM2 detector via an external Python environment."""

from __future__ import annotations

from dataclasses import asdict
import json
import os
import re
import subprocess
import tempfile
from pathlib import Path
from typing import Any

from app.config import Settings
from app.detectors.base import BaseObjectDetector, DetectedObject
from app.detectors.vocabulary import (
    build_detection_prompts,
    category_for_label,
    color_for_label,
    label_zh,
)
from app.perception.grounding_prompt_planner import (
    GroundingPromptError,
    GroundingPromptPlanner,
)
from app.task_understanding.schemas import GroundingPromptPlan
from app.reasoning.target_profile import TargetProfile


class DetectorRuntimeError(RuntimeError):
    """Raised when the external detector cannot run or returns invalid output."""


class GroundedSAMSubprocessDetector(BaseObjectDetector):
    def __init__(
        self,
        settings: Settings,
        target_profile: TargetProfile | None = None,
        parsed_task: Any | None = None,
        navigation_task: Any | None = None,
        llm_client: Any | None = None,
    ) -> None:
        self.settings = settings
        self.target_profile = target_profile
        self.parsed_task = parsed_task
        self.navigation_task = navigation_task
        self.llm_client = llm_client
        self.grounding_prompt_plan: GroundingPromptPlan | None = None
        self.grounding_prompt_retry_plan: GroundingPromptPlan | None = None

    def detect(self, image_path: str, target_text: str) -> list[DetectedObject]:
        return self.detect_with_dynamic_terms(image_path, target_text, [])

    def detect_with_dynamic_terms(
        self,
        image_path: str,
        target_text: str,
        dynamic_terms: list[str],
    ) -> list[DetectedObject]:
        image = Path(image_path).resolve()
        if not image.is_file():
            raise FileNotFoundError(f"Image file not found: {image_path}")

        with tempfile.TemporaryDirectory() as tmp_dir:
            output_path = Path(tmp_dir) / "detections.json"
            env = os.environ.copy()
            if self.settings.grounded_sam_pythonpath:
                existing_pythonpath = env.get("PYTHONPATH")
                env["PYTHONPATH"] = (
                    self.settings.grounded_sam_pythonpath
                    if not existing_pythonpath
                    else f"{self.settings.grounded_sam_pythonpath}:{existing_pythonpath}"
                )

            prompt_text = " || ".join(self._build_prompts(target_text, dynamic_terms))
            if self.settings.grounding_prompt_require_non_empty and not prompt_text.strip():
                raise GroundingPromptError(
                    "GroundingDINO prompt is empty. Detection was not executed."
                )
            payload = self._run_worker(
                image=image,
                output_path=output_path,
                text_prompt=prompt_text,
                env=env,
                box_threshold=self.settings.grounding_dino_box_threshold,
                text_threshold=self.settings.grounding_dino_text_threshold,
            )
            if not payload.get("objects"):
                payload = self._maybe_retry_prompt_expansion(
                    image=image,
                    output_path=output_path,
                    target_text=target_text,
                    previous_prompt=prompt_text,
                    previous_payload=payload,
                    env=env,
                )
            if not payload.get("objects"):
                payload = self._maybe_retry_high_recall_thresholds(
                    image=image,
                    output_path=output_path,
                    text_prompt=(
                        self.grounding_prompt_retry_plan.grounding_prompt
                        if self.grounding_prompt_retry_plan is not None
                        else prompt_text
                    ),
                    previous_payload=payload,
                    env=env,
                )

        return [
            _to_detected_object(item, self.target_profile)
            for item in payload.get("objects", [])
        ]

    def _build_prompts(self, target_text: str, dynamic_terms: list[str]) -> list[str]:
        if (
            self.settings.grounding_prompt_llm_expansion_enabled
            and self.parsed_task is not None
        ):
            planner = GroundingPromptPlanner(
                llm_client=self.llm_client,
                settings=self.settings,
                max_terms=self.settings.grounding_prompt_max_terms,
                min_terms=self.settings.grounding_prompt_min_terms,
            )
            self.grounding_prompt_plan = planner.build(
                parsed_task=self.parsed_task,
                navigation_task=self.navigation_task,
                target_profile=self.target_profile,
            )
            self._write_prompt_plan(
                self.grounding_prompt_plan,
                self.settings.grounding_prompt_debug_output,
            )
            terms = _prompt_to_terms(self.grounding_prompt_plan.grounding_prompt)
            terms.extend(dynamic_terms)
            prompt = _terms_to_prompt(_dedupe_terms(terms))
            return [prompt] if prompt else []

        prompts = build_detection_prompts(
            target_text,
            dynamic_terms=(
                [
                    *(
                        self.target_profile.detector_terms()
                        if self.target_profile is not None
                        else []
                    ),
                    *dynamic_terms,
                ]
            ),
            context_terms=(
                self.target_profile.context_labels_en
                if self.target_profile is not None
                and self.settings.static_object_prompts_enabled
                else None
            ),
            include_base_terms=(
                self.settings.static_object_prompts_enabled
                and self.target_profile is None
                and not dynamic_terms
            ),
        )
        if not prompts:
            prompts = build_detection_prompts(
                target_text,
                include_base_terms=self.settings.static_object_prompts_enabled,
            )
        return prompts

    def _run_worker(
        self,
        *,
        image: Path,
        output_path: Path,
        text_prompt: str,
        env: dict[str, str],
        box_threshold: float,
        text_threshold: float,
    ) -> dict[str, Any]:
        command = [
            self.settings.grounded_sam_python,
            str(Path(__file__).with_name("grounded_sam_worker.py")),
            "--image",
            str(image),
            "--output",
            str(output_path),
            "--root",
            self.settings.grounded_sam_root,
            "--text-prompt",
            text_prompt,
            "--grounding-config",
            self.settings.grounding_dino_config,
            "--grounding-checkpoint",
            self.settings.grounding_dino_checkpoint,
            "--box-threshold",
            str(box_threshold),
            "--text-threshold",
            str(text_threshold),
            "--sam2-config",
            self.settings.sam2_config,
            "--sam2-checkpoint",
            self.settings.sam2_checkpoint,
            "--max-objects",
            str(self.settings.max_detected_objects),
            "--device",
            self.settings.detection_device,
        ]
        if not self.settings.enable_sam2:
            command.append("--disable-sam2")
        try:
            completed = subprocess.run(
                command,
                cwd=self.settings.grounded_sam_root,
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=self.settings.detector_timeout_seconds,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise DetectorRuntimeError(
                "Grounding DINO/SAM2 detector timed out after "
                f"{self.settings.detector_timeout_seconds:.1f}s. "
                "建议减少开放词表、降低输入分辨率或暂时关闭 SAM2。"
            ) from exc
        if completed.returncode != 0:
            raise DetectorRuntimeError(
                "Grounding DINO/SAM2 detector failed.\n"
                f"Command: {' '.join(command)}\n"
                f"stdout:\n{completed.stdout}\n"
                f"stderr:\n{completed.stderr}"
            )
        if not output_path.is_file():
            raise DetectorRuntimeError(
                "Grounding DINO/SAM2 detector finished but did not create output JSON."
            )
        return json.loads(output_path.read_text(encoding="utf-8"))

    def _maybe_retry_prompt_expansion(
        self,
        *,
        image: Path,
        output_path: Path,
        target_text: str,
        previous_prompt: str,
        previous_payload: dict[str, Any],
        env: dict[str, str],
    ) -> dict[str, Any]:
        if (
            self.grounding_prompt_plan is None
            or not self.settings.grounding_prompt_retry_on_empty
            or self.settings.grounding_prompt_max_retries <= 0
        ):
            return previous_payload
        planner = GroundingPromptPlanner(
            llm_client=self.llm_client,
            settings=self.settings,
            max_terms=self.settings.grounding_prompt_max_terms,
            min_terms=self.settings.grounding_prompt_min_terms,
        )
        summary = {
            "num_raw_candidates": len(previous_payload.get("objects", [])),
            "target": target_text,
        }
        self.grounding_prompt_retry_plan = planner.build_retry(
            parsed_task=self.parsed_task,
            previous_prompt=previous_prompt,
            detection_summary=summary,
            target_profile=self.target_profile,
            previous_plan=self.grounding_prompt_plan,
        )
        self._write_prompt_plan(
            self.grounding_prompt_retry_plan,
            self.settings.grounding_prompt_retry_debug_output,
        )
        return self._run_worker(
            image=image,
            output_path=output_path,
            text_prompt=self.grounding_prompt_retry_plan.grounding_prompt,
            env=env,
            box_threshold=self.settings.grounding_dino_box_threshold,
            text_threshold=self.settings.grounding_dino_text_threshold,
        )

    def _maybe_retry_high_recall_thresholds(
        self,
        *,
        image: Path,
        output_path: Path,
        text_prompt: str,
        previous_payload: dict[str, Any],
        env: dict[str, str],
    ) -> dict[str, Any]:
        if not (
            self.settings.enable_gdino_high_recall
            and (
                self.settings.grounding_dino_box_threshold
                > self.settings.grounding_dino_high_recall_box_threshold
                or self.settings.grounding_dino_text_threshold
                > self.settings.grounding_dino_high_recall_text_threshold
            )
        ):
            return previous_payload
        try:
            return self._run_worker(
                image=image,
                output_path=output_path,
                text_prompt=text_prompt,
                env=env,
                box_threshold=self.settings.grounding_dino_high_recall_box_threshold,
                text_threshold=self.settings.grounding_dino_high_recall_text_threshold,
            )
        except DetectorRuntimeError:
            return previous_payload

    @staticmethod
    def _write_prompt_plan(plan: GroundingPromptPlan, path_value: str) -> None:
        path = Path(path_value)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(asdict(plan), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )


def _to_detected_object(
    item: dict,
    target_profile: TargetProfile | None = None,
) -> DetectedObject:
    label = str(item.get("label") or "object").lower().strip()
    bbox = item.get("bbox_2d") or [0.0, 0.0, 1.0, 1.0]
    attributes = list(item.get("attributes") or [])
    attributes.append("detected_by_grounding_dino")
    if item.get("mask_area_ratio") is not None:
        attributes.append("segmented_by_sam2")
        attributes.append(f"mask_area_ratio={float(item['mask_area_ratio']):.4f}")

    is_direct_target = (
        target_profile is not None
        and _matches_any_term(label, target_profile.direct_terms())
    )
    return DetectedObject(
        label=label,
        label_zh=(
            target_profile.canonical_name_zh
            if is_direct_target and target_profile is not None
            else label_zh(label)
        ),
        category=(
            target_profile.target_type
            if is_direct_target and target_profile is not None
            else category_for_label(label)
        ),
        color=color_for_label(label),
        bbox_2d=(
            _clamp(float(bbox[0])),
            _clamp(float(bbox[1])),
            _clamp(float(bbox[2])),
            _clamp(float(bbox[3])),
        ),
        score=_clamp(float(item.get("score", 0.5))),
        text_score=(
            None
            if item.get("text_score") is None
            else _clamp(float(item["text_score"]))
        ),
        attributes=attributes,
        mask_area_ratio=(
            None
            if item.get("mask_area_ratio") is None
            else _clamp(float(item["mask_area_ratio"]))
        ),
        source="grounding_dino_sam2",
        source_prompt_term=str(item.get("source_prompt_term") or label),
    )


def _clamp(value: float) -> float:
    return max(0.0, min(1.0, value))


def _matches_any_term(label: str, terms: list[str]) -> bool:
    normalized_label = " ".join(
        re.sub(r"[^a-z0-9\u4e00-\u9fff]+", " ", label.lower()).split()
    )
    for term in terms:
        normalized_term = " ".join(
            re.sub(r"[^a-z0-9\u4e00-\u9fff]+", " ", term.lower()).split()
        )
        if normalized_term and normalized_term in normalized_label:
            return True
    return False


def _prompt_to_terms(prompt: str) -> list[str]:
    return [item.strip() for item in re.split(r"\s*\.\s*", prompt) if item.strip()]


def _terms_to_prompt(terms: list[str]) -> str:
    cleaned = _dedupe_terms(terms)
    return " . ".join(cleaned) + (" ." if cleaned else "")


def _dedupe_terms(terms: list[str]) -> list[str]:
    result: list[str] = []
    seen: set[str] = set()
    for term in terms:
        normalized = " ".join(
            re.sub(r"[^a-z0-9 -]+", " ", str(term).lower()).split()
        ).strip()
        if not normalized or normalized in seen:
            continue
        seen.add(normalized)
        result.append(normalized)
    return result
