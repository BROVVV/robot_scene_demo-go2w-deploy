"""Compatibility entry point for navigation topology building."""

from __future__ import annotations

from app.video.video_navigation_topology_builder import VideoNavigationTopologyBuilder


NavigationTopologyBuilder = VideoNavigationTopologyBuilder

__all__ = ["NavigationTopologyBuilder", "VideoNavigationTopologyBuilder"]
