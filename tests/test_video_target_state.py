from app.video.video_target_state import (
    TARGET_CANDIDATE,
    TARGET_NOT_SEEN,
    TARGET_UNCONFIRMED_BUT_LIKELY_AREA_FOUND,
    TARGET_VISUAL_CONFIRMED,
    determine_target_state,
)


def test_visual_evidence_confirms_target() -> None:
    result = {"target_found": True, "best_evidence": {"bbox": [0.1, 0.1, 0.5, 0.5]}}

    assert determine_target_state(result) == TARGET_VISUAL_CONFIRMED


def test_candidate_without_gate_stays_candidate() -> None:
    result = {
        "target_found": False,
        "timeline": [{"type": "direct_detection", "frame_id": 3}],
    }

    assert determine_target_state(result) == TARGET_CANDIDATE


def test_ranked_likely_area_does_not_confirm_target() -> None:
    result = {"target_found": False, "timeline": []}
    ranked = [
        {
            "place_id": "place_003",
            "target_search_score": 0.82,
            "can_confirm_target": False,
        }
    ]

    assert (
        determine_target_state(result, ranked_places=ranked)
        == TARGET_UNCONFIRMED_BUT_LIKELY_AREA_FOUND
    )


def test_no_candidate_or_ranked_area_is_not_seen() -> None:
    assert determine_target_state({"target_found": False}) == TARGET_NOT_SEEN
