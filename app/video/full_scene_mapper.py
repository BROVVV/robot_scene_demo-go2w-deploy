"""End-to-end video full-scene semantic mapping pipeline."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from app.config import get_settings
from app.video.frame_scene_parser import FrameSceneParser
from app.video.models import VideoFrame, VideoMetadata
from app.video.object_tracker import VideoObjectTracker
from app.video.observed_scene_graph_builder import ObservedSceneGraphBuilder
from app.video.place_segmenter import PlaceSegmenter
from app.video.psg_graph_merger import PSGGraphMerger
from app.video.navigation_graph_exporter import (
    write_navigation_topology_debug,
    write_navigation_topology_graphml,
    write_navigation_topology_json,
)
from app.video.navigation_topology_schema import PlaceSegment
from app.video.schemas import FrameObservation, ObjectTrack, PSGLayer, SceneGraph
from app.video.video_full_scene_report import write_video_full_scene_report
from app.video.video_graph_io import (
    write_json,
    write_scene_graph_graphml,
    write_scene_graph_json,
)
from app.video.video_graph_visualizer import render_topology_png
from app.video.video_navigation_topology_builder import VideoNavigationTopologyBuilder
from app.video.video_psg_predictor import VideoPSGPredictor
from app.video.video_reader import read_and_sample_video


ProgressCallback = Callable[[int, int, str], None]


@dataclass
class VideoFullSceneMapResult:
    summary_zh: str
    video_meta: dict[str, Any]
    frame_observations: list[FrameObservation]
    object_tracks: list[ObjectTrack]
    place_segments: list[PlaceSegment]
    observed_graph: SceneGraph
    psg_layer: PSGLayer
    hybrid_graph: SceneGraph
    navigation_topology: dict[str, Any] | None
    navigation_map: dict[str, Any] | None
    merge_report: dict[str, object]

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": "full_scene_map",
            "summary_zh": self.summary_zh,
            "video_meta": self.video_meta,
            "frame_count": len(self.frame_observations),
            "object_count": len(self.object_tracks),
            "place_segments": [item.to_dict() for item in self.place_segments],
            "observed_graph": self.observed_graph.to_dict(),
            "psg_layer": self.psg_layer.to_dict(),
            "hybrid_graph": self.hybrid_graph.to_dict(),
            "navigation_topology": self.navigation_topology,
            "navigation_map": self.navigation_map,
            "merge_report": self.merge_report,
            "next_best_views": [item.to_dict() for item in self.psg_layer.next_best_views],
        }


class VideoFullSceneMapper:
    """Run full-scene mapping over sampled video frames."""

    def __init__(self, output_dir: str | Path = "outputs") -> None:
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def run(
        self,
        video_path: str | Path,
        detector: str = "llm",
        sample_fps: float | None = None,
        max_frames: int | None = None,
        enable_video_memory: bool = False,
        enable_video_psg: bool | None = None,
        enable_navigation_topology: bool | None = None,
        psg_max_predicted_nodes: int | None = None,
        psg_confidence_threshold: float | None = None,
        topology_observed_only: bool | None = None,
        save_frame_observations: bool | None = None,
        annotate: bool = True,
        progress_callback: ProgressCallback | None = None,
    ) -> tuple[VideoFullSceneMapResult, dict[str, Path]]:
        del enable_video_memory
        settings = get_settings()
        sample_fps = settings.video_sample_fps if sample_fps is None else sample_fps
        max_frames = settings.video_max_frames if max_frames is None else max_frames
        enable_video_psg = settings.video_enable_video_psg if enable_video_psg is None else enable_video_psg
        enable_navigation_topology = (
            settings.video_enable_navigation_topology
            if enable_navigation_topology is None
            else enable_navigation_topology
        )
        psg_max_predicted_nodes = (
            settings.video_psg_max_predicted_nodes
            if psg_max_predicted_nodes is None
            else psg_max_predicted_nodes
        )
        psg_confidence_threshold = (
            settings.video_psg_confidence_threshold
            if psg_confidence_threshold is None
            else psg_confidence_threshold
        )
        topology_observed_only = (
            settings.video_topology_observed_only
            if topology_observed_only is None
            else topology_observed_only
        )
        save_frame_observations = (
            settings.video_save_frame_observations
            if save_frame_observations is None
            else save_frame_observations
        )

        metadata, frames = self._read_frames(video_path, detector, sample_fps, max_frames)
        parser = FrameSceneParser(
            detector=detector,
            output_dir=self.output_dir,
            annotate=annotate and detector != "mock",
        )
        observations: list[FrameObservation] = []
        errors: list[dict[str, Any]] = []
        total = len(frames)
        for index, frame in enumerate(frames, start=1):
            if progress_callback:
                progress_callback(index - 1, total, f"正在全场景解析 {frame.timestamp_sec:.2f}s")
            try:
                observations.append(parser.parse(frame))
            except Exception as exc:
                errors.append(
                    {
                        "frame_id": frame.frame_id,
                        "timestamp_sec": frame.timestamp_sec,
                        "frame_path": str(frame.image_path),
                        "error": str(exc),
                    }
                )
            if progress_callback:
                progress_callback(index, total, f"已完成 {index}/{total} 帧")
        if not observations:
            details = errors[0]["error"] if errors else "unknown error"
            raise RuntimeError(f"All sampled frames failed full-scene analysis. First error: {details}")

        object_tracks = VideoObjectTracker().build_tracks(observations)
        place_segments = PlaceSegmenter().segment(observations, object_tracks)
        observed_graph = ObservedSceneGraphBuilder().build(observations, object_tracks, place_segments)
        psg_layer = (
            VideoPSGPredictor(
                max_predicted_nodes=psg_max_predicted_nodes,
                confidence_threshold=psg_confidence_threshold,
            ).predict(observed_graph)
            if enable_video_psg
            else PSGLayer()
        )
        hybrid_graph, merge_report = PSGGraphMerger(psg_confidence_threshold).merge(
            observed_graph,
            psg_layer,
        )
        topology = None
        navigation_map = None
        if enable_navigation_topology:
            topology_builder = VideoNavigationTopologyBuilder(observed_only=topology_observed_only)
            topology = topology_builder.build(hybrid_graph, psg_layer.next_best_views)
            navigation_map = topology_builder.build_navigation_map(topology)

        summary = _summary(observations, object_tracks, psg_layer)
        result = VideoFullSceneMapResult(
            summary_zh=summary,
            video_meta={**metadata.to_dict(), "processing_errors": errors},
            frame_observations=observations,
            object_tracks=object_tracks,
            place_segments=place_segments,
            observed_graph=observed_graph,
            psg_layer=psg_layer,
            hybrid_graph=hybrid_graph,
            navigation_topology=topology,
            navigation_map=navigation_map,
            merge_report=merge_report,
        )
        paths = self._write_outputs(
            result=result,
            video_path=str(video_path),
            detector=detector,
            sample_fps=sample_fps,
            save_frame_observations=save_frame_observations,
        )
        return result, paths

    def _read_frames(
        self,
        video_path: str | Path,
        detector: str,
        sample_fps: float,
        max_frames: int,
    ) -> tuple[VideoMetadata, list[VideoFrame]]:
        path = Path(video_path)
        if detector == "mock" and not path.exists():
            frame = VideoFrame(
                frame_id=0,
                timestamp_sec=0.0,
                image_path=path,
                width=1,
                height=1,
            )
            metadata = VideoMetadata(
                video_path=str(path),
                fps=sample_fps,
                duration_sec=0.0,
                frame_count=1,
                width=1,
                height=1,
                sampled_keyframes=1,
            )
            return metadata, [frame]
        return read_and_sample_video(
            path,
            sample_fps=sample_fps,
            max_frames=max_frames,
            output_dir=self.output_dir / "video_frames",
        )

    def _write_outputs(
        self,
        *,
        result: VideoFullSceneMapResult,
        video_path: str,
        detector: str,
        sample_fps: float,
        save_frame_observations: bool,
    ) -> dict[str, Path]:
        paths: dict[str, Path] = {}
        if save_frame_observations:
            paths["video_frame_observations"] = write_json(
                {
                    "frames": [item.to_dict() for item in result.frame_observations],
                    "processing_errors": result.video_meta.get("processing_errors", []),
                },
                self.output_dir / "video_frame_observations.json",
            )
        paths["video_all_objects"] = write_json(
            [item.to_dict() for item in result.object_tracks],
            self.output_dir / "video_all_objects.json",
        )
        paths["video_place_segments"] = write_json(
            [item.to_dict() for item in result.place_segments],
            self.output_dir / "video_place_segments.json",
        )
        paths["video_object_tracks"] = write_json(
            [item.to_dict() for item in result.object_tracks],
            self.output_dir / "video_object_tracks.json",
        )
        paths["video_observed_scene_graph_json"] = write_scene_graph_json(
            result.observed_graph,
            self.output_dir / "video_observed_scene_graph.json",
        )
        paths["video_observed_scene_graph_graphml"] = write_scene_graph_graphml(
            result.observed_graph,
            self.output_dir / "video_observed_scene_graph.graphml",
        )
        paths["video_psg_layer"] = write_json(
            result.psg_layer.to_dict(),
            self.output_dir / "video_psg_layer.json",
        )
        paths["video_predictive_scene_graph_json"] = write_json(
            {
                "nodes": [item.to_dict() for item in result.psg_layer.predicted_nodes],
                "edges": [item.to_dict() for item in result.psg_layer.predicted_edges],
                "hypotheses": [item.to_dict() for item in result.psg_layer.hypotheses],
            },
            self.output_dir / "video_predictive_scene_graph.json",
        )
        paths["video_predictive_scene_graph"] = write_scene_graph_graphml(
            SceneGraph(result.psg_layer.predicted_nodes, result.psg_layer.predicted_edges),
            self.output_dir / "video_predictive_scene_graph.graphml",
        )
        paths["video_hybrid_scene_graph_json"] = write_scene_graph_json(
            result.hybrid_graph,
            self.output_dir / "video_hybrid_scene_graph.json",
            merge_report=result.merge_report,
        )
        paths["video_hybrid_scene_graph_graphml"] = write_scene_graph_graphml(
            result.hybrid_graph,
            self.output_dir / "video_hybrid_scene_graph.graphml",
        )
        if result.navigation_topology is not None:
            paths["video_navigation_topology"] = write_navigation_topology_json(
                result.navigation_topology,
                self.output_dir / "video_navigation_topology.json",
            )
            paths["video_navigation_topology_graphml"] = write_navigation_topology_graphml(
                result.navigation_topology,
                self.output_dir / "video_navigation_topology.graphml",
            )
            rendered = render_topology_png(
                result.navigation_topology,
                self.output_dir / "video_navigation_topology.png",
            )
            if rendered:
                paths["video_navigation_topology_png"] = rendered
            paths["video_navigation_topology_debug"] = write_navigation_topology_debug(
                result.navigation_topology,
                self.output_dir / "video_navigation_topology_debug.md",
            )
        if result.navigation_map is not None:
            paths["video_navigation_map"] = write_json(
                result.navigation_map,
                self.output_dir / "video_navigation_map.json",
            )
        paths["video_full_scene_map"] = write_json(
            result.to_dict(),
            self.output_dir / "video_full_scene_map.json",
        )
        paths["video_full_scene_report"] = write_video_full_scene_report(
            video_path=video_path,
            detector=detector,
            sample_fps=sample_fps,
            frame_observations=result.frame_observations,
            object_tracks=result.object_tracks,
            observed_graph=result.observed_graph,
            psg_layer=result.psg_layer,
            topology=result.navigation_topology,
            output_path=self.output_dir / "video_full_scene_report.md",
        )
        return paths


def _summary(
    observations: list[FrameObservation],
    tracks: list[ObjectTrack],
    psg_layer: PSGLayer,
) -> str:
    scene_types = sorted({item.scene_type for item in observations if item.scene_type})
    landmarks = [track.label_zh for track in tracks if track.navigation_role in {"landmark", "passage"}]
    obstacles = [track.label_zh for track in tracks if track.navigation_role == "obstacle"]
    return (
        f"已解析 {len(observations)} 帧，观察到 {len(tracks)} 个跨帧物体/结构；"
        f"场景类型：{', '.join(scene_types) or 'unknown'}；"
        f"导航地标：{', '.join(landmarks[:8]) or '暂无'}；"
        f"障碍物候选：{', '.join(obstacles[:8]) or '暂无'}；"
        f"PSG 探索候选：{len(psg_layer.predicted_nodes)} 个。"
    )
