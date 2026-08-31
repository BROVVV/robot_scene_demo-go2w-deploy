"""Base interface for vision LLM clients."""

from __future__ import annotations

from abc import ABC, abstractmethod


class BaseVisionLLMClient(ABC):
    """Common interface for scene analysis vision LLM providers."""

    @abstractmethod
    def analyze_scene(
        self,
        image_path: str,
        target_text: str,
        extra_instructions: str | None = None,
    ) -> dict:
        """Analyze one scene image and target description."""
