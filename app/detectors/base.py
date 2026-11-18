"""Shared detector interfaces."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class DetectedObject:
    label: str
    label_zh: str
    category: str
    bbox_2d: tuple[float, float, float, float]
    score: float
    text_score: float | None = None
    color: str | None = None
    attributes: list[str] = field(default_factory=list)
    mask_area_ratio: float | None = None
    mask_rle: list[list[int]] | None = None
    mask_size: tuple[int, int] | None = None
    # Mask provenance must stay independent from the depth method.  In
    # particular an HSV cue is not a SAM2 mask merely because both use RLE.
    mask_source: str | None = None
    # Optional visible host-surface mask used by the hazard localizer when the
    # anomaly's own pixels have no valid depth.
    context_mask_rle: list[list[int]] | None = None
    context_mask_size: tuple[int, int] | None = None
    host_detection_index: int | None = None
    context_quality: str | None = None
    hazard_state: str | None = None
    source: str = "detector"
    source_prompt_term: str | None = None
    # `source_prompt_term` 是**哪个槽问出来的**（检测分频，2026-10-14 第四次补充
    # 复审必修）。一次 `detect` 会让目标槽、锚点槽、场景槽共用一个 prompt，回来
    # 的框只说得出自己由哪个词生成，说不出那个词属于哪个槽 —— 于是组装层只能
    # 按**整条检测**记"这条结果还对哪些槽算最新"。上下文词（画像的
    # `context_labels_en` 是并进目标槽 prompt 的）因此能借场景槽的新鲜度留下
    # 来。归属只有检测器算得出来（词表就在它手里），所以在这里随框一起带上。
    # 空元组 = "不知道"，下游按旧行为处理。
    detection_slots: tuple[str, ...] = ()


class BaseObjectDetector:
    def configure_target(self, target_profile: object | None) -> None:
        """Update later target/anchor prompts without rebuilding model state."""
        self.target_profile = target_profile

    def detect(self, image_path: str, target_text: str) -> list[DetectedObject]:
        raise NotImplementedError

    def detect_with_dynamic_terms(
        self,
        image_path: str,
        target_text: str,
        dynamic_terms: list[str],
        scene_terms: list[str] | None = None,
        *,
        frame_id: str = "",
        slots: frozenset[str] | None = None,
    ) -> list[DetectedObject]:
        raise NotImplementedError

    def close(self) -> None:
        """Release optional subprocess/GPU resources."""
