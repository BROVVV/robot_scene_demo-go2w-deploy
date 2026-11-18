#!/usr/bin/env python3
"""Persistent Ultralytics YOLOE-Seg JSON-lines worker.

Only this process imports Ultralytics.  The ROS/Web processes therefore keep
their existing dependency environments and communicate through a small,
request-id checked protocol.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
from contextlib import redirect_stdout
from pathlib import Path
from typing import Any, Iterable


_shutdown_tracing = False


def _shutdown_event(stage: str) -> None:
    if not _shutdown_tracing:
        return
    try:
        print(json.dumps({"event": "yoloe_worker_shutdown", "stage": stage,
                          "pid": os.getpid(), "monotonic": time.monotonic(),
                          "wall_time": time.time()}), file=sys.stderr, flush=True)
    except Exception:
        pass  # Diagnostic output must not prevent a shutdown response.


def _start_shutdown_trace() -> None:
    """Opt-in evidence before the existing five-second owner stop deadline."""
    if os.environ.get("GO2W_YOLOE_EXIT_TRACE") != "1":
        return
    global _shutdown_tracing
    _shutdown_tracing = True
    _shutdown_event("request_received")
    try:
        import atexit
        import faulthandler
        atexit.register(_shutdown_event, "python_atexit")
        # The timer survives main() returning and can expose a wait in object
        # destruction or interpreter/thread shutdown. No exit hook is skipped.
        faulthandler.dump_traceback_later(3., repeat=False, file=sys.stderr)
    except Exception:
        _shutdown_event("trace_unavailable")


def normalize_terms(values: Iterable[Any]) -> list[str]:
    """Return a stable, unique vocabulary used by both cache and class IDs."""
    normalized = {
        " ".join(str(value).strip().lower().split())
        for value in values
        if str(value).strip()
    }
    return sorted(normalized)


def prompt_signature(terms: Iterable[Any], slots: Iterable[Any], model_id: str) -> str:
    payload = {
        "model": str(model_id),
        "slots": sorted({str(slot) for slot in slots}),
        "terms": normalize_terms(terms),
    }
    return hashlib.sha256(
        json.dumps(payload, ensure_ascii=False, sort_keys=True).encode("utf-8")
    ).hexdigest()


def encode_row_runs(mask: Any) -> list[list[int]]:
    """Encode a 2-D binary mask as project-native [row, x, length] runs."""
    import numpy as np

    data = np.asarray(mask, dtype=bool)
    if data.ndim != 2:
        return []
    occupied_rows = np.flatnonzero(data.any(axis=1))
    if not occupied_rows.size:
        return []
    # Skip only provably zero margins of the mask itself, never its detection
    # box. Keep holes, disconnected components and original image coordinates.
    y0, y1 = occupied_rows[0], occupied_rows[-1] + 1
    data = data[y0:y1]
    occupied_columns = np.flatnonzero(data.any(axis=0))
    x0, x1 = occupied_columns[0], occupied_columns[-1] + 1
    data = data[:, x0:x1]
    # Zero guards keep adjacent rows separate. NumPy nonzero returns row-major
    # order, exactly matching the previous per-row [start, end] traversal.
    padded = np.zeros((data.shape[0], data.shape[1] + 2), dtype=bool)
    padded[:, 1:-1] = data
    rows, edges = np.nonzero(padded[:, 1:] != padded[:, :-1])
    starts, ends = edges[::2], edges[1::2]
    return np.column_stack((rows[::2] + y0, starts + x0, ends - starts)).tolist()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _as_numpy(value: Any) -> Any:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return value


def _prediction_precision(precision: str) -> int:
    """Translate the project precision setting to current Ultralytics API."""
    if precision == "fp16":
        return 16
    if precision == "fp32":
        return 32
    raise ValueError(f"unsupported YOLOE precision: {precision}")


class YOLOERuntime:
    def __init__(self, args: argparse.Namespace) -> None:
        self.model_path = Path(args.model).expanduser().resolve()
        if not self.model_path.is_file():
            raise FileNotFoundError(f"YOLOE model not found: {self.model_path}")
        self.prompt_free = self.model_path.name.endswith("-seg-pf.pt")
        if not (self.prompt_free or self.model_path.name.endswith("-seg.pt")):
            raise RuntimeError("YOLOE_MODEL must be *-seg.pt or *-seg-pf.pt")
        self.text_encoder_path: Path | None = None
        if not self.prompt_free:
            self.text_encoder_path = Path(args.text_encoder).expanduser().resolve()
            if not self.text_encoder_path.is_file():
                raise FileNotFoundError(
                    f"YOLOE text encoder not found: {self.text_encoder_path}; "
                    "runtime downloads are disabled by deployment policy"
                )
            expected_encoder = "mobileclip2_b.ts" if self.model_path.name.startswith("yoloe-26") else "mobileclip_blt.ts"
            if self.text_encoder_path.name != expected_encoder:
                raise RuntimeError(
                    f"YOLOE text encoder mismatch: {self.model_path.name} requires {expected_encoder}"
                )
            if self.text_encoder_path.parent != Path.cwd().resolve():
                raise RuntimeError(
                    "YOLOE_TEXT_ENCODER must be in the worker cwd because Ultralytics "
                    f"loads it by basename; expected {Path.cwd() / expected_encoder}"
                )

        # stdout is the JSONL protocol channel.  Ultralytics and its
        # dependencies may print progress or deprecation messages even with
        # verbose=False, so all third-party output belongs on stderr.
        with redirect_stdout(sys.stderr):
            import torch
            import ultralytics
            if not self.prompt_free:
                import clip  # noqa: F401 - fail now instead of auto-installing in a live session
            from ultralytics import YOLOE

            self.torch = torch
            self.ultralytics_version = str(ultralytics.__version__)
            self.device = str(args.device)
            if not self.device.startswith("cuda"):
                raise RuntimeError(
                    f"YOLOE requires an explicit CUDA device; got {self.device!r}"
                )
            if not torch.cuda.is_available():
                raise RuntimeError("YOLOE CUDA preflight failed: torch.cuda.is_available() is false")
            try:
                torch.empty(1, device=self.device)
            except Exception as exc:
                raise RuntimeError(f"YOLOE cannot initialize {self.device}: {exc}") from exc

        self.precision = str(args.precision).lower()
        if self.precision not in {"fp16", "fp32"}:
            raise RuntimeError("YOLOE_PRECISION must be fp16 or fp32")
        self.image_size = int(args.image_size)
        self.confidence = float(args.confidence)
        self.iou_threshold = float(args.iou_threshold)
        self.max_detections = int(args.max_detections)
        self.model_sha256 = _sha256(self.model_path)
        self.text_encoder_sha256 = (
            _sha256(self.text_encoder_path) if self.text_encoder_path is not None else None
        )
        with redirect_stdout(sys.stderr):
            self.model = YOLOE(str(self.model_path))
            self.model.to(self.device)
        names = self.model.names
        self.model_terms = list(names.values()) if isinstance(names, dict) else list(names)
        if self.prompt_free and not hasattr(self.model.model.model[-1], "lrpc"):
            raise RuntimeError("prompt-free YOLOE checkpoint has no fused LRPC head")
        self.current_prompt_hash = ""
        self.current_terms: list[str] = list(self.model_terms) if self.prompt_free else []

    def _set_terms(self, terms: list[str], slots: list[str]) -> tuple[str, bool]:
        if getattr(self, "prompt_free", False):
            raise RuntimeError("prompt-free YOLOE does not accept per-frame terms")
        signature = prompt_signature(terms, slots, self.model_path.name)
        if signature == self.current_prompt_hash:
            return signature, True
        # Supplying embeddings explicitly avoids wrapper-version ambiguity and
        # makes the text encoder download visible during deployment warm-up.
        with redirect_stdout(sys.stderr):
            embeddings = self.model.get_text_pe(terms)
            self.model.set_classes(terms, embeddings)
        self.current_terms = list(terms)
        self.current_prompt_hash = signature
        return signature, False

    def _predict(self, source: Any, **kwargs: Any) -> Any:
        """Run Ultralytics without allowing library output onto JSONL stdout."""
        kwargs["quantize"] = _prediction_precision(self.precision)
        with redirect_stdout(sys.stderr):
            return self.model.predict(source, **kwargs)

    def health(self) -> dict[str, Any]:
        free_bytes, total_bytes = self.torch.cuda.mem_get_info(self.device)
        return {
            "ok": True,
            "backend": "yoloe_seg",
            "model_id": self.model_path.name,
            "model_path": str(self.model_path),
            "model_sha256": self.model_sha256,
            "prompt_mode": "prompt_free" if self.prompt_free else "text_prompt",
            "class_count": len(self.model_terms),
            "text_encoder_path": (
                str(self.text_encoder_path) if self.text_encoder_path is not None else None
            ),
            "text_encoder_sha256": self.text_encoder_sha256,
            "library_version": self.ultralytics_version,
            "torch_version": str(self.torch.__version__),
            "torch_cpu_threads": int(self.torch.get_num_threads()),
            "torch_cpu_interop_threads": int(self.torch.get_num_interop_threads()),
            "device": self.device,
            "device_name": self.torch.cuda.get_device_name(self.device),
            "precision": self.precision,
            "cuda_free_bytes": int(free_bytes),
            "cuda_total_bytes": int(total_bytes),
            "fallback_used": False,
        }

    def warmup(self, terms: list[str]) -> dict[str, Any]:
        import numpy as np

        if not self.prompt_free:
            terms = normalize_terms(terms or ["person", "chair"])
            self._set_terms(terms, ["scene"])
        started = time.perf_counter()
        self._predict(
            np.zeros((64, 64, 3), dtype=np.uint8),
            imgsz=max(64, min(self.image_size, 320)),
            conf=self.confidence,
            iou=self.iou_threshold,
            max_det=min(self.max_detections, 10),
            device=self.device,
            retina_masks=True,
            verbose=False,
        )
        return {**self.health(), "warmup_latency_ms": round((time.perf_counter() - started) * 1000, 3)}

    def detect(self, request: dict[str, Any]) -> dict[str, Any]:
        total_started = time.perf_counter()
        import numpy as np
        from PIL import Image

        image_path = Path(str(request.get("image") or "")).expanduser().resolve()
        if not image_path.is_file():
            raise FileNotFoundError(f"image not found: {image_path}")
        requested_terms = normalize_terms(request.get("terms") or [])
        if self.prompt_free:
            # The fused checkpoint already carries its 4k+ class vocabulary.
            # Request terms are intentionally ignored: no scene-dependent
            # set_classes(), prompt rotation, or text encoder is involved.
            terms = self.model_terms
            signature = prompt_signature([], ["prompt_free"], self.model_path.name)
            cache_hit = True
        else:
            terms = requested_terms
            if not terms:
                return {"objects": [], "diagnostics": self._diagnostics(request, "", True, 0.0, 0, 0, 0, total_started=total_started)}
            slots = sorted({str(slot) for slot in (request.get("slots") or [])})
            signature, cache_hit = self._set_terms(terms, slots)
        with Image.open(image_path) as image:
            width, height = image.size

        started = time.perf_counter()
        results = self._predict(
            str(image_path),
            imgsz=int(request.get("image_size") or self.image_size),
            conf=float(request.get("confidence") or self.confidence),
            iou=float(request.get("iou_threshold") or self.iou_threshold),
            max_det=int(request.get("max_detections") or self.max_detections),
            device=self.device,
            retina_masks=True,
            verbose=False,
        )
        prediction_finished = time.perf_counter()
        latency_ms = (prediction_finished - started) * 1000.0
        result = results[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return {"objects": [], "diagnostics": self._diagnostics(request, signature, cache_hit, latency_ms, 0, 0, 0, total_started=total_started, prediction_finished=prediction_finished)}

        xyxy = np.asarray(_as_numpy(boxes.xyxy), dtype=float)
        scores = np.asarray(_as_numpy(boxes.conf), dtype=float)
        class_ids = np.asarray(_as_numpy(boxes.cls), dtype=int)
        masks_data = None
        if getattr(result, "masks", None) is not None:
            masks_data = np.asarray(_as_numpy(result.masks.data))

        objects: list[dict[str, Any]] = []
        masks_valid = 0
        masks_invalid = 0
        slot_terms = request.get("slot_terms") or {}
        for index, (box, score, class_id) in enumerate(zip(xyxy, scores, class_ids)):
            if class_id < 0 or class_id >= len(terms):
                continue
            x1, y1, x2, y2 = [float(value) for value in box]
            x1, x2 = sorted((max(0.0, min(float(width), x1)), max(0.0, min(float(width), x2))))
            y1, y2 = sorted((max(0.0, min(float(height), y1)), max(0.0, min(float(height), y2))))
            if x2 <= x1 or y2 <= y1:
                continue
            term = terms[int(class_id)]
            item: dict[str, Any] = {
                "label": term,
                "source_prompt_term": term,
                "score": float(max(0.0, min(1.0, score))),
                "text_score": None,
                "bbox_2d": [x1 / width, y1 / height, x2 / width, y2 / height],
                "detection_slots": (
                    ["scene"] if self.prompt_free else [
                        slot for slot, values in slot_terms.items()
                        if term in normalize_terms(values or [])
                    ]
                ),
            }
            mask = masks_data[index] if masks_data is not None and index < len(masks_data) else None
            if mask is not None:
                binary = np.asarray(mask) > 0.5
                if binary.shape != (height, width):
                    # Retina masks should already match the source.  A mismatch
                    # is treated as bbox-only evidence, never silently resized.
                    masks_invalid += 1
                elif binary.any():
                    item["mask_rle"] = encode_row_runs(binary)
                    item["mask_size"] = [height, width]
                    item["mask_area_ratio"] = float(binary.mean())
                    masks_valid += 1
                else:
                    masks_invalid += 1
            else:
                masks_invalid += 1
            objects.append(item)

        return {
            "objects": objects,
            "diagnostics": self._diagnostics(
                request, signature, cache_hit, latency_ms,
                len(objects), masks_valid, masks_invalid,
                total_started=total_started, prediction_finished=prediction_finished,
            ),
        }

    def _diagnostics(
        self, request: dict[str, Any], signature: str, cache_hit: bool,
        latency_ms: float, detection_count: int, masks_valid: int, masks_invalid: int,
        *, total_started: float | None = None, prediction_finished: float | None = None,
    ) -> dict[str, Any]:
        health = self.health()
        finished = time.perf_counter()
        return {
            **health,
            "request_id": str(request.get("request_id") or ""),
            "frame_id": str(request.get("frame_id") or ""),
            "confidence": float(request.get("confidence") or self.confidence),
            "iou_threshold": float(request.get("iou_threshold") or self.iou_threshold),
            "image_size": int(request.get("image_size") or self.image_size),
            "max_detections": int(request.get("max_detections") or self.max_detections),
            "prompt_hash": signature,
            "prompt_term_count": 0 if self.prompt_free else len(request.get("terms") or []),
            "prompt_cache_hit": bool(cache_hit),
            "detection_slots": sorted({str(slot) for slot in request.get("slots") or []}),
            "inference_latency_ms": round(float(latency_ms), 3),
            "postprocess_latency_ms": round(max(0.0, finished - prediction_finished) * 1000.0, 3) if prediction_finished is not None else 0.0,
            "queue_wait_ms": 0.0,
            "total_latency_ms": round((finished - total_started) * 1000.0, 3) if total_started is not None else round(float(latency_ms), 3),
            "detection_count": int(detection_count),
            "masks_valid": int(masks_valid),
            "masks_invalid": int(masks_invalid),
            "fallback_used": False,
        }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serve", action="store_true")
    parser.add_argument("--health", action="store_true")
    parser.add_argument("--warmup", action="store_true")
    parser.add_argument("--model", required=True)
    parser.add_argument("--text-encoder", default="")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--precision", default="fp16")
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--iou-threshold", type=float, default=0.50)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--max-detections", type=int, default=100)
    return parser


def _emit(payload: dict[str, Any]) -> None:
    print(json.dumps(payload, ensure_ascii=False, allow_nan=False), flush=True)


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        runtime = YOLOERuntime(args)
        if args.health or args.warmup:
            _emit(runtime.warmup(["person", "chair"]) if args.warmup else runtime.health())
            return 0
        if not args.serve:
            raise RuntimeError("one of --serve/--health/--warmup is required")
        for line in sys.stdin:
            try:
                request = json.loads(line)
                operation = str(request.get("operation") or "detect")
                if operation == "health":
                    response = runtime.health()
                elif operation == "detect":
                    response = runtime.detect(request)
                elif operation == "shutdown":
                    _start_shutdown_trace()
                    _emit({"ok": True, "request_id": request.get("request_id")})
                    _shutdown_event("ack_flushed")
                    return 0
                else:
                    raise ValueError(f"unsupported operation: {operation}")
                response["request_id"] = request.get("request_id")
                _emit(response)
            except Exception as exc:  # protocol errors stay machine-readable
                _emit({"request_id": locals().get("request", {}).get("request_id"), "error": f"{type(exc).__name__}: {exc}"})
        return 0
    except Exception as exc:
        _emit({"error": f"{type(exc).__name__}: {exc}"})
        return 2


def _entrypoint() -> int:
    code = main()
    _shutdown_event("main_returned")
    return code


if __name__ == "__main__":
    raise SystemExit(_entrypoint())

