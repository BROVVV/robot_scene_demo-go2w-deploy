"""Compatibility entry point for building the video PSG prediction layer."""

from __future__ import annotations

from app.video.video_psg_predictor import VideoPSGPredictor


PSGLayerBuilder = VideoPSGPredictor

__all__ = ["PSGLayerBuilder", "VideoPSGPredictor"]
