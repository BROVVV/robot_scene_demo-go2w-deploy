"""Persistent YOLOE-Seg client and DetectedObject adapter."""

from __future__ import annotations

import json
import os
import select
import subprocess
import tempfile
import time
import uuid
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from app.config import Settings
from app.detectors.base import BaseObjectDetector, DetectedObject
from app.detectors.process_priority import detector_process_command
from app.detectors.grounded_sam_subprocess import DetectorRuntimeError, _matches_any_term
from app.detectors.vocabulary import (
    BASE_TERMS,
    TARGET_TERM_RULES,
    category_for_label,
    color_for_label,
    resolve_display_label,
)
from app.reasoning.target_profile import TargetProfile


def _normalize_terms(values: Iterable[Any]) -> list[str]:
    return sorted({" ".join(str(value).strip().lower().split()) for value in values if str(value).strip()})


def _target_rules(target_text: str) -> list[str]:
    terms: list[str] = []
    for key, values in TARGET_TERM_RULES.items():
        if key in target_text:
            terms.extend(values)
    return terms


def _prompt_terms_and_slots(
    *, target_text: str, dynamic_terms: Sequence[str], scene_terms: Sequence[str],
    target_profile: TargetProfile | None, slots: frozenset[str] | None,
    include_static_scene: bool,
) -> tuple[list[str], dict[str, list[str]]]:
    if slots is not None and not slots:
        return [], {}
    include_target = slots is None or "target" in slots
    include_anchor = slots is None or "anchor" in slots
    include_scene = slots is None or "scene" in slots
    by_slot: dict[str, list[str]] = {}
    if include_target:
        target_terms = list(dynamic_terms)
        target_terms.extend(_target_rules(target_text))
        if target_profile is not None:
            target_terms.extend(target_profile.detector_terms())
            target_terms.extend(target_profile.context_labels_en)
        by_slot["target"] = _normalize_terms(target_terms)
    if include_anchor and target_profile is not None:
        by_slot["anchor"] = _normalize_terms(target_profile.relation_anchor_labels_en)
    if include_scene:
        values = list(scene_terms)
        # YOLOE v1 intentionally does not call RAM++.  Autonomous search can
        # therefore have target/anchor terms while its dynamic scene list is
        # empty.  Keep the established static scene vocabulary in that case;
        # explicit rotating scene groups (teleoperation mapping) remain
        # untouched so their cadence and prompt budget do not change.
        if include_static_scene and not values:
            values.extend(BASE_TERMS)
        by_slot["scene"] = _normalize_terms(values)
    terms = _normalize_terms(term for values in by_slot.values() for term in values)
    return terms, {slot: values for slot, values in by_slot.items() if values}


def _to_detected_object(
    item: Mapping[str, Any], target_profile: TargetProfile | None,
    *, model_id: str,
) -> DetectedObject:
    label = " ".join(str(item.get("source_prompt_term") or item.get("label") or "object").lower().split())
    bbox = list(item.get("bbox_2d") or [0.0, 0.0, 1.0, 1.0])
    if len(bbox) != 4:
        raise DetectorRuntimeError(f"YOLOE returned invalid bbox: {bbox!r}")
    clamped = tuple(max(0.0, min(1.0, float(value))) for value in bbox)
    if clamped[2] <= clamped[0] or clamped[3] <= clamped[1]:
        raise DetectorRuntimeError(f"YOLOE returned zero-area bbox: {bbox!r}")
    mask_rle = item.get("mask_rle")
    mask_size = item.get("mask_size")
    normalized_mask: list[list[int]] | None = None
    normalized_size: tuple[int, int] | None = None
    if isinstance(mask_rle, list) and mask_rle and isinstance(mask_size, list) and len(mask_size) == 2:
        try:
            height, width = int(mask_size[0]), int(mask_size[1])
            runs = [[int(value) for value in run] for run in mask_rle]
            if (
                height > 0 and width > 0
                and all(
                    len(run) == 3
                    and 0 <= run[0] < height
                    and 0 <= run[1] < width
                    and run[2] > 0
                    and run[1] + run[2] <= width
                    for run in runs
                )
            ):
                normalized_mask = runs
                normalized_size = (height, width)
        except (TypeError, ValueError):
            pass
    valid_mask = normalized_mask is not None and normalized_size is not None
    is_direct_target = bool(
        target_profile is not None
        and _matches_any_term(label, target_profile.direct_terms())
    )
    attributes = ["detected_by_yoloe", f"detector_model={model_id}"]
    if valid_mask:
        attributes.extend(["segmented_by_yoloe", f"mask_area_ratio={float(item.get('mask_area_ratio') or 0.0):.4f}"])
    return DetectedObject(
        label=label,
        label_zh=resolve_display_label(label),
        category=(target_profile.target_type if is_direct_target and target_profile else category_for_label(label)),
        bbox_2d=clamped,
        score=max(0.0, min(1.0, float(item.get("score", 0.0)))),
        text_score=None,
        color=color_for_label(label),
        attributes=attributes,
        mask_area_ratio=(
            max(0.0, min(1.0, float(item["mask_area_ratio"])))
            if valid_mask and item.get("mask_area_ratio") is not None else None
        ),
        mask_rle=normalized_mask,
        mask_size=normalized_size,
        mask_source=("yoloe_seg" if valid_mask else None),
        source="yoloe_seg",
        source_prompt_term=label,
        detection_slots=tuple(str(slot) for slot in item.get("detection_slots") or ()),
    )


class YOLOESegSubprocessDetector(BaseObjectDetector):
    capabilities_backend = "yoloe_seg"

    def __init__(self, settings: Settings, target_profile: TargetProfile | None = None) -> None:
        self.settings = settings
        self.target_profile = target_profile
        self._worker: subprocess.Popen | None = None
        self._worker_stderr: Any = None
        self._request_seq = 0
        self._session_id = uuid.uuid4().hex[:8]
        self.last_diagnostics: dict[str, Any] = {}
        self.last_diagnostics_available = False
        self.detect_attempts: list[dict[str, Any]] = []

    def configure_target(self, target_profile: TargetProfile | None) -> None:
        # YOLOE text embeddings are rebuilt per request; the CUDA worker and
        # loaded segmentation model remain alive across target changes.
        self.target_profile = target_profile

    @property
    def prompt_free(self) -> bool:
        return Path(self.settings.yoloe_model).name.endswith("-seg-pf.pt")

    def detect(self, image_path: str, target_text: str) -> list[DetectedObject]:
        return self.detect_with_dynamic_terms(image_path, target_text, [])

    def detect_with_dynamic_terms(
        self, image_path: str, target_text: str, dynamic_terms: list[str],
        ram_scene_terms: list[str] | None = None, *, frame_id: str = "",
        slots: frozenset[str] | None = None,
    ) -> list[DetectedObject]:
        image = Path(image_path).expanduser().resolve()
        if not image.is_file():
            raise FileNotFoundError(f"Image file not found: {image_path}")
        self.detect_attempts = []
        if slots is not None and not slots:
            return []
        if self.prompt_free:
            # The checkpoint has a fused vocabulary.  Do not translate the
            # current scene, task, or rotating schedule into model prompts.
            terms, slot_terms = [], {}
        else:
            terms, slot_terms = _prompt_terms_and_slots(
                target_text=target_text,
                dynamic_terms=dynamic_terms,
                scene_terms=ram_scene_terms or [],
                target_profile=self.target_profile,
                slots=slots,
                include_static_scene=bool(self.settings.static_object_prompts_enabled),
            )
            if not terms:
                return []
        payload = self._request(
            operation="detect", image=str(image), terms=terms,
            slots=sorted(slot_terms), slot_terms=slot_terms, frame_id=str(frame_id),
        )
        objects: list[DetectedObject] = []
        for item in payload.get("objects") or []:
            try:
                objects.append(_to_detected_object(item, self.target_profile, model_id=Path(self.settings.yoloe_model).name))
            except DetectorRuntimeError:
                # A malformed/zero-area instance is not allowed to poison the
                # whole frame; the worker diagnostics still expose its count.
                continue
        return objects

    def discover_scene_terms(self, _image_path: str) -> list[str]:
        # Capability table explicitly says this backend does not run RAM++.
        return []

    def health(self) -> dict[str, Any]:
        return self._request(operation="health")

    def _command(self) -> list[str]:
        command = [
            self.settings.yoloe_python,
            str(Path(__file__).with_name("yoloe_seg_worker.py")),
            "--serve",
            "--model", self.settings.yoloe_model,
            "--device", self.settings.yoloe_device,
            "--precision", self.settings.yoloe_precision,
            "--confidence", str(self.settings.yoloe_confidence),
            "--iou-threshold", str(self.settings.yoloe_iou_threshold),
            "--image-size", str(self.settings.yoloe_image_size),
            "--max-detections", str(self.settings.yoloe_max_detections),
        ]
        if not self.prompt_free:
            command.extend(["--text-encoder", self.settings.yoloe_text_encoder])
        return command

    def _ensure_worker(self) -> None:
        if self._worker is not None and self._worker.poll() is None:
            return
        python = Path(self.settings.yoloe_python).expanduser()
        model = Path(self.settings.yoloe_model).expanduser()
        if not python.is_file() or not os.access(python, os.X_OK):
            raise DetectorRuntimeError(f"YOLOE_PYTHON is not executable: {python}")
        if not model.is_file():
            raise DetectorRuntimeError(f"YOLOE_MODEL not found: {model}")
        if not self.prompt_free:
            text_encoder = Path(self.settings.yoloe_text_encoder).expanduser()
            if not text_encoder.is_file():
                raise DetectorRuntimeError(f"YOLOE_TEXT_ENCODER not found: {text_encoder}")
        env = os.environ.copy()
        project_root = str(Path(__file__).resolve().parents[2])
        env["PYTHONPATH"] = os.pathsep.join(filter(None, [project_root, env.get("PYTHONPATH", "")]))
        self._worker_stderr = tempfile.NamedTemporaryFile(prefix="yoloe_seg_worker_", suffix=".log", delete=False)
        self._worker = subprocess.Popen(
            detector_process_command(self._command()), cwd=project_root, env=env, text=True,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self._worker_stderr,
        )

    def _request(self, operation: str, **values: Any) -> dict[str, Any]:
        self._ensure_worker()
        self._request_seq += 1
        request_id = f"{self._session_id}-{self._request_seq}"
        request = {
            "operation": operation,
            "request_id": request_id,
            "confidence": self.settings.yoloe_confidence,
            "iou_threshold": self.settings.yoloe_iou_threshold,
            "image_size": self.settings.yoloe_image_size,
            "max_detections": self.settings.yoloe_max_detections,
            **values,
        }
        assert self._worker is not None and self._worker.stdin is not None and self._worker.stdout is not None
        try:
            self._worker.stdin.write(json.dumps(request, ensure_ascii=False) + "\n")
            self._worker.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            raise DetectorRuntimeError(self._worker_died_message()) from exc
        deadline = time.monotonic() + float(self.settings.yoloe_timeout_seconds)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self._worker.stdout], [], [], max(0.0, remaining))[0]:
                self.close()
                raise DetectorRuntimeError(
                    f"YOLOE worker timed out after {self.settings.yoloe_timeout_seconds:.1f}s"
                )
            line = self._worker.stdout.readline()
            if line == "":
                raise DetectorRuntimeError(self._worker_died_message())
            try:
                payload = json.loads(line)
            except json.JSONDecodeError as exc:
                self.close()
                raise DetectorRuntimeError(
                    f"YOLOE worker returned non-JSON output: {line[:300]!r}"
                ) from exc
            if payload.get("request_id") not in {None, request_id}:
                self.close()
                raise DetectorRuntimeError(
                    f"YOLOE worker request_id mismatch: expected {request_id}, got {payload.get('request_id')}"
                )
            if payload.get("error"):
                raise DetectorRuntimeError("YOLOE worker error: " + str(payload["error"]))
            diagnostics = payload.get("diagnostics")
            if isinstance(diagnostics, dict):
                self.last_diagnostics = diagnostics
                self.last_diagnostics_available = True
            else:
                self.last_diagnostics_available = False
            self.detect_attempts.append({
                "request_id": request_id,
                "operation": operation,
                "frame_id": str(values.get("frame_id") or ""),
                "source_image": str(values.get("image") or ""),
                "diagnostics": diagnostics if isinstance(diagnostics, dict) else None,
            })
            return payload

    def _worker_died_message(self) -> str:
        tail = ""
        if self._worker_stderr is not None:
            try:
                self._worker_stderr.flush()
                self._worker_stderr.seek(0)
                tail = self._worker_stderr.read()[-3000:]
            except Exception:
                pass
        if isinstance(tail, bytes):
            tail = tail.decode("utf-8", errors="replace")
        return "YOLOE worker exited unexpectedly. stderr tail:\n" + str(tail)

    def close(self) -> None:
        worker = self._worker
        self._worker = None
        if worker is not None:
            try:
                if worker.poll() is None and worker.stdin is not None:
                    worker.stdin.write(json.dumps({"operation": "shutdown", "request_id": "shutdown"}) + "\n")
                    worker.stdin.flush()
                    worker.wait(timeout=5)
            except Exception:
                try:
                    worker.terminate()
                    worker.wait(timeout=2)
                except Exception:
                    try:
                        worker.kill()
                        worker.wait(timeout=2)
                    except Exception:
                        pass
            finally:
                # Reap a forced exit and release the protocol descriptors.
                # Retaining a dead Popen or its pipes across repeated sessions
                # otherwise leaves zombies/file descriptors in the Web owner.
                for stream in (worker.stdin, worker.stdout, worker.stderr):
                    if stream is not None:
                        try:
                            stream.close()
                        except Exception:
                            pass
        if self._worker_stderr is not None:
            try:
                self._worker_stderr.close()
            except Exception:
                pass
            self._worker_stderr = None
