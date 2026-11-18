"""Short-lived, image-verified pipe surface tracking for slime review ROIs."""
from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable

import numpy as np


class PipeContextTracker:
    def __init__(self, *, max_age_s: float = 8.0, min_quality: float = 0.65) -> None:
        self.max_age_s = max(0.0, float(max_age_s))
        self.min_quality = max(0.0, min(1.0, float(min_quality)))
        self._gray: np.ndarray | None = None
        self._masks: list[np.ndarray] = []
        self._timestamp = 0.0

    def reset(self) -> None:
        self._gray = None
        self._masks = []
        self._timestamp = 0.0

    def contexts(
        self,
        image_path: str,
        detections: Iterable[Any],
        *,
        timestamp: float,
        pipe_terms: set[str],
    ) -> list[dict[str, Any]]:
        """Return only reliably tracked contexts; refresh cache on real masks."""
        try:
            import cv2
        except ImportError:
            return []
        image = cv2.imread(str(Path(image_path)), cv2.IMREAD_GRAYSCALE)
        if image is None:
            return []
        measured = [
            mask for item in detections
            if _terms(item).intersection(pipe_terms)
            for mask in [_mask(item, image.shape)]
            if mask is not None and int(mask.sum()) >= 16
        ]
        if measured:
            self._gray, self._masks, self._timestamp = image, measured, float(timestamp)
            return []
        age = float(timestamp) - self._timestamp
        if self._gray is None or not self._masks or age < 0.0 or age > self.max_age_s:
            self._gray = image
            return []
        if self._gray.shape != image.shape:
            self.reset()
            self._gray = image
            return []

        contexts: list[dict[str, Any]] = []
        tracked_masks: list[np.ndarray] = []
        for old_mask in self._masks:
            features = cv2.goodFeaturesToTrack(
                self._gray,
                maxCorners=80,
                qualityLevel=0.01,
                minDistance=4,
                mask=(old_mask.astype(np.uint8) * 255),
            )
            if features is None or len(features) < 6:
                continue
            moved, status, _ = cv2.calcOpticalFlowPyrLK(self._gray, image, features, None)
            if moved is None or status is None:
                continue
            valid = status.reshape(-1).astype(bool)
            source = features.reshape(-1, 2)[valid]
            target = moved.reshape(-1, 2)[valid]
            if len(source) < 6:
                continue
            matrix, inliers = cv2.estimateAffinePartial2D(
                source, target, method=cv2.RANSAC, ransacReprojThreshold=2.5
            )
            if matrix is None or inliers is None:
                continue
            quality = float(np.mean(inliers.astype(bool))) * min(1.0, len(source) / 20.0)
            # Reject implausibly large view jumps; tracking is not geometric
            # reprojection and must not survive a turn it cannot explain.
            translation = float(np.hypot(matrix[0, 2], matrix[1, 2]))
            if quality < self.min_quality or translation > 0.25 * max(image.shape):
                continue
            warped = cv2.warpAffine(
                old_mask.astype(np.uint8), matrix, (image.shape[1], image.shape[0]),
                flags=cv2.INTER_NEAREST, borderValue=0,
            ).astype(bool)
            if int(warped.sum()) < 16:
                continue
            tracked_masks.append(warped)
            contexts.append({"surface": warped, "tracking_quality": round(quality, 4), "age_s": age})
        self._gray = image
        if tracked_masks:
            self._masks = tracked_masks
        else:
            self._masks = []
        return contexts


def _terms(item: Any) -> set[str]:
    get = item.get if isinstance(item, dict) else lambda key, default=None: getattr(item, key, default)
    return {
        str(value).strip().casefold()
        for value in (get("label"), get("label_zh"), get("source_prompt_term"))
        if value
    }


def _mask(item: Any, shape: tuple[int, int]) -> np.ndarray | None:
    get = item.get if isinstance(item, dict) else lambda key, default=None: getattr(item, key, default)
    runs, size = get("mask_rle"), get("mask_size")
    if not runs or not size or tuple(int(v) for v in size) != tuple(shape):
        return None
    out = np.zeros(shape, dtype=bool)
    try:
        for row, start, length in runs:
            row, start, length = int(row), int(start), int(length)
            if 0 <= row < shape[0] and length > 0:
                out[row, max(0, start):min(shape[1], start + length)] = True
    except (TypeError, ValueError):
        return None
    return out
