import pytest

from run_video_demo import normalize_video_args, parse_args


def test_legacy_full_scene_with_target_becomes_target_search_auxiliary() -> None:
    args = normalize_video_args(
        parse_args(
            [
                "--video",
                "input.mp4",
                "--target",
                "电视",
                "--mode",
                "full_scene_map",
            ]
        )
    )

    assert args.mode == "target_search"
    assert args.enable_scene_mapping is True
    assert args.enable_navigation_topology is True


def test_scene_map_only_rejects_target() -> None:
    args = parse_args(
        [
            "--video",
            "input.mp4",
            "--target",
            "电视",
            "--scene-map-only",
        ]
    )

    with pytest.raises(ValueError, match="scene_map_only"):
        normalize_video_args(args)


def test_topology_search_sorting_enables_scene_mapping() -> None:
    args = normalize_video_args(
        parse_args(
            [
                "--video",
                "input.mp4",
                "--target",
                "电视",
                "--use-scene-map-for-search",
            ]
        )
    )

    assert args.mode == "target_search"
    assert args.enable_scene_mapping is True
    assert args.enable_navigation_topology is True
    assert args.use_scene_map_for_search is True
