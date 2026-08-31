"""Navigation mode helpers for visual preview and Nav2 handoff."""

from __future__ import annotations

from enum import Enum


class VideoNavigationMode(str, Enum):
    DISABLED = "disabled"
    OFFLINE_PREVIEW = "offline_preview"
    VISUAL_PREVIEW = "visual_preview"
    METRIC_PREVIEW = "metric_preview"
    PLAN_ONLY = "plan_only"
    EXECUTE = "execute"

    @property
    def uses_nav2(self) -> bool:
        return self in {self.PLAN_ONLY, self.EXECUTE}

    @property
    def is_visual(self) -> bool:
        return self in {self.VISUAL_PREVIEW, self.METRIC_PREVIEW}


def normalize_video_navigation_mode(value: str | None) -> VideoNavigationMode:
    if not value:
        return VideoNavigationMode.VISUAL_PREVIEW
    selected = VideoNavigationMode(value)
    if selected == VideoNavigationMode.OFFLINE_PREVIEW:
        return VideoNavigationMode.VISUAL_PREVIEW
    return selected
