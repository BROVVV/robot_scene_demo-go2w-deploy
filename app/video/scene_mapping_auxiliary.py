"""Auxiliary scene mapping for video target search."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from app.config import get_settings
from app.video.frame_scene_parser import observation_from_frame_analysis
from app.video.models import FrameAnalysisResult
from app.video.navigation_graph_exporter import (
    write_navigation_topology_debug,
    write_navigation_topology_graphml,
    write_navigation_topology_json,
)
from app.video.observed_scene_graph_builder import ObservedSceneGraphBuilder
from app.video.place_segmenter import PlaceSegmenter
from app.video.psg_graph_merger import PSGGraphMerger
from app.video.schemas import FrameObservation, PSGLayer, SceneGraph
from app.video.video_graph_io import (
    write_json,
    write_scene_graph_graphml,
    write_scene_graph_json,
)
from app.video.video_graph_visualizer import render_topology_png
from app.video.video_navigation_topology_builder import VideoNavigationTopologyBuilder
from app.video.video_psg_predictor import VideoPSGPredictor
from app.video.object_tracker import VideoObjectTracker


def run_scene_mapping_auxiliary(
    video_path: str,
    frame_results: list[dict[str, Any]] | list[FrameAnalysisResult],
    object_tracks: list[dict[str, Any]],
    target_profile: dict[str, Any] | Any | None,
    config: Any,
) -> dict[str, Any]:
    """Build a place-centric scene map after target search.

    This function treats scene mapping as an auxiliary layer. It derives scene
    observations from already analyzed target-search frames and does not make
    target confirmation decisions.
    """

    del object_tracks
    settings = get_settings()
    output_dir = Path(getattr(config, "output_dir", None) or settings.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    enable_video_psg = bool(
        _configured(config, "enable_video_psg", settings.video_enable_video_psg)
    )
    enable_navigation_topology = bool(
        _configured(
            config,
            "enable_navigation_topology",
            settings.video_enable_navigation_topology,
        )
    )
    psg_confidence_threshold = float(
        _configured(
            config,
            "psg_confidence_threshold",
            settings.video_psg_confidence_threshold,
        )
    )
    psg_max_predicted_nodes = int(
        _configured(
            config,
            "psg_max_predicted_nodes",
            settings.video_psg_max_predicted_nodes,
        )
    )
    topology_observed_only = bool(
        _configured(
            config,
            "topology_observed_only",
            settings.video_topology_observed_only,
        )
    )

    observations = [_observation_from_result(item) for item in frame_results]
    scene_tracks = VideoObjectTracker().build_tracks(observations)
    place_segments = PlaceSegmenter().segment(observations, scene_tracks)
    observed_graph = ObservedSceneGraphBuilder().build(
        observations,
        scene_tracks,
        place_segments,
    )
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
    navigation_topology = None
    navigation_map = None
    if enable_navigation_topology:
        topology_builder = VideoNavigationTopologyBuilder(
            observed_only=topology_observed_only,
        )
        navigation_topology = topology_builder.build(
            hybrid_graph,
            psg_layer.next_best_views,
        )
        navigation_topology.setdefault("metadata", {}).update(
            {
                "version": "navigation_topology_v1",
                "main_task": "target_search",
                "target": _target_name(target_profile),
                "used_for_search": False,
                "coordinate_mode": "topological_only",
                "has_metric_pose": False,
                "source_video": video_path,
            }
        )
        navigation_map = topology_builder.build_navigation_map(navigation_topology)

    paths = _write_scene_mapping_outputs(
        output_dir=output_dir,
        observations=observations,
        object_tracks=scene_tracks,
        place_segments=place_segments,
        observed_graph=observed_graph,
        psg_layer=psg_layer,
        hybrid_graph=hybrid_graph,
        navigation_topology=navigation_topology,
        navigation_map=navigation_map,
        merge_report=merge_report,
    )
    return {
        "video_path": video_path,
        "frame_observations": [item.to_dict() for item in observations],
        "object_tracks": [item.to_dict() for item in scene_tracks],
        "place_segments": [item.to_dict() for item in place_segments],
        "observed_graph": observed_graph.to_dict(),
        "psg_layer": psg_layer.to_dict(),
        "hybrid_graph": hybrid_graph.to_dict(),
        "navigation_topology": navigation_topology,
        "navigation_map": navigation_map,
        "merge_report": merge_report,
        "output_files": {key: str(path) for key, path in paths.items()},
    }


def _write_scene_mapping_outputs(
    *,
    output_dir: Path,
    observations: list[FrameObservation],
    object_tracks: list[Any],
    place_segments: list[Any],
    observed_graph: SceneGraph,
    psg_layer: PSGLayer,
    hybrid_graph: SceneGraph,
    navigation_topology: dict[str, Any] | None,
    navigation_map: dict[str, Any] | None,
    merge_report: dict[str, Any],
) -> dict[str, Path]:
    paths: dict[str, Path] = {}
    paths["video_frame_observations"] = write_json(
        {"frames": [item.to_dict() for item in observations]},
        output_dir / "video_frame_observations.json",
    )
    paths["video_place_segments"] = write_json(
        [item.to_dict() for item in place_segments],
        output_dir / "video_place_segments.json",
    )
    paths["video_all_objects"] = write_json(
        [item.to_dict() for item in object_tracks],
        output_dir / "video_all_objects.json",
    )
    paths["video_observed_scene_graph_json"] = write_scene_graph_json(
        observed_graph,
        output_dir / "video_observed_scene_graph.json",
    )
    paths["video_observed_scene_graph_graphml"] = write_scene_graph_graphml(
        observed_graph,
        output_dir / "video_observed_scene_graph.graphml",
    )
    paths["video_psg_layer"] = write_json(
        psg_layer.to_dict(),
        output_dir / "video_psg_layer.json",
    )
    paths["video_hybrid_scene_graph_json"] = write_scene_graph_json(
        hybrid_graph,
        output_dir / "video_hybrid_scene_graph.json",
        merge_report=merge_report,
    )
    paths["video_hybrid_scene_graph_graphml"] = write_scene_graph_graphml(
        hybrid_graph,
        output_dir / "video_hybrid_scene_graph.graphml",
    )
    if navigation_topology is not None:
        paths["video_navigation_topology"] = write_navigation_topology_json(
            navigation_topology,
            output_dir / "video_navigation_topology.json",
        )
        paths["video_navigation_topology_graphml"] = write_navigation_topology_graphml(
            navigation_topology,
            output_dir / "video_navigation_topology.graphml",
        )
        rendered = render_topology_png(
            navigation_topology,
            output_dir / "video_navigation_topology.png",
        )
        if rendered:
            paths["video_navigation_topology_png"] = rendered
        paths["video_navigation_topology_debug"] = write_navigation_topology_debug(
            navigation_topology,
            output_dir / "video_navigation_topology_debug.md",
        )
    if navigation_map is not None:
        paths["video_navigation_map"] = write_json(
            navigation_map,
            output_dir / "video_navigation_map.json",
        )
    return paths


def _observation_from_result(item: dict[str, Any] | FrameAnalysisResult) -> FrameObservation:
    if isinstance(item, FrameAnalysisResult):
        return observation_from_frame_analysis(item)
    return observation_from_frame_analysis(FrameAnalysisResult(**item))


def _target_name(target_profile: Any | None) -> str | None:
    if target_profile is None:
        return None
    if hasattr(target_profile, "to_dict"):
        payload = target_profile.to_dict()
    else:
        payload = dict(target_profile)
    return payload.get("canonical_name_zh") or payload.get("raw_target")


def _configured(config: Any, name: str, default: Any) -> Any:
    value = getattr(config, name, default)
    return default if value is None else value
