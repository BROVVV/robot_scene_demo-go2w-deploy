from types import SimpleNamespace
from unittest.mock import patch

from app.video.models import FrameAnalysisResult
from app.video.video_target_search_pipeline import run_video_target_search_pipeline


def _frame() -> FrameAnalysisResult:
    return FrameAnalysisResult(
        frame_id=0,
        timestamp_sec=0.0,
        image_path="mock.jpg",
        annotated_frame_path=None,
        scene_summary="客厅里有沙发、电视柜和墙面。",
        objects=[
            {
                "object_id": "sofa",
                "label": "sofa",
                "label_zh": "沙发",
                "confidence": 0.9,
                "bbox": [0.1, 0.4, 0.5, 0.9],
            },
            {
                "object_id": "tv_stand",
                "label": "TV stand",
                "label_zh": "电视柜",
                "confidence": 0.88,
                "bbox": [0.45, 0.45, 0.9, 0.8],
            },
            {
                "object_id": "wall",
                "label": "living room wall",
                "label_zh": "客厅墙面",
                "confidence": 0.84,
                "bbox": [0.0, 0.0, 1.0, 0.45],
            },
            {
                "object_id": "floor",
                "label": "floor",
                "label_zh": "地面",
                "confidence": 0.8,
                "bbox": [0.0, 0.75, 1.0, 1.0],
            },
        ],
        relations=[],
    )


def _search_result(frame: FrameAnalysisResult) -> dict:
    return {
        "task": {"target": "电视", "video_path": "mock.mp4", "detector": "mock"},
        "video_meta": {"sampled_keyframes": 1, "video_path": "mock.mp4"},
        "target_found": False,
        "best_evidence": None,
        "timeline": [],
        "candidate_regions": [],
        "navigation_interpretation": {"suggestion": "继续搜索。", "reason": "无位姿。"},
        "target_profile": {
            "raw_target": "电视",
            "canonical_name_zh": "电视",
            "zh_terms": ["电视", "电视机"],
            "en_terms": ["television", "TV", "tv screen", "monitor", "display"],
            "context_terms": ["sofa", "TV stand", "living room wall"],
            "likely_regions_zh": ["living_room", "客厅"],
        },
        "_runtime_artifacts": {
            "frame_results": [frame],
            "object_tracks": [],
            "target_profile": {
                "raw_target": "电视",
                "canonical_name_zh": "电视",
                "zh_terms": ["电视", "电视机"],
                "en_terms": ["television", "TV", "tv screen", "monitor", "display"],
                "context_terms": ["sofa", "TV stand", "living room wall"],
                "likely_regions_zh": ["living_room", "客厅"],
            },
        },
    }


def test_target_search_runs_before_scene_mapping_and_outputs_both(tmp_path) -> None:
    frame = _frame()
    config = SimpleNamespace(
        output_dir=str(tmp_path),
        sample_fps=1.0,
        max_frames=1,
        enable_knowledge=False,
        enable_video_memory=False,
        no_annotate=True,
        enable_llm_prior=False,
        enable_observation_memory=False,
        verify_every_n_frames=None,
        track_iou_threshold=None,
        target_confirm_min_frames=None,
        target_confirm_score=None,
        disable_handwritten_priors=True,
        disable_static_kb=True,
        prior_audit=False,
        enable_video_psg=False,
        psg_max_predicted_nodes=None,
        psg_confidence_threshold=None,
        topology_observed_only=True,
    )
    with patch(
        "app.video.video_target_search_pipeline.run_video_search",
        return_value=(_search_result(frame), {}),
    ) as run_search:
        result = run_video_target_search_pipeline(
            video_path="mock.mp4",
            target="电视",
            detector="mock",
            config=config,
            enable_tracking=True,
            enable_crop_verify=False,
            enable_evidence_gating=True,
            enable_scene_mapping=True,
            enable_navigation_topology=True,
            use_scene_map_for_search=True,
        )

    run_search.assert_called_once()
    assert result["target"] == "电视"
    assert result["scene_mapping_enabled"] is True
    assert result["navigation_topology_enabled"] is True
    assert result["target_confirmed"] is False
    assert result["target_status"] == "target_unconfirmed_but_likely_area_found"
    assert (tmp_path / "video_target_search.json").is_file()
    assert (tmp_path / "video_target_timeline.json").is_file()
    assert (tmp_path / "video_navigation_trace.json").is_file()
    assert (tmp_path / "video_navigation_topology.json").is_file()
    assert (tmp_path / "video_topology_search_ranking.json").is_file()
