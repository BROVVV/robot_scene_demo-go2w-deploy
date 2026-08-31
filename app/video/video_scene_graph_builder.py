"""Compatibility entry point for the observed video scene graph builder."""

from __future__ import annotations

from app.video.observed_scene_graph_builder import ObservedSceneGraphBuilder


VideoSceneGraphBuilder = ObservedSceneGraphBuilder

__all__ = ["ObservedSceneGraphBuilder", "VideoSceneGraphBuilder"]
