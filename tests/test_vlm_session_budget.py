import pytest

from app.live_robot.vlm_budget import bounded_vlm_timeout


def test_vlm_timeout_is_capped_by_remaining_search_budget():
    assert bounded_vlm_timeout(65.0, 110.0, now_monotonic=100.0) == 10.0


def test_vlm_timeout_preserves_shorter_provider_limit_without_session_cap():
    assert bounded_vlm_timeout(12.0, 200.0, now_monotonic=100.0) == 12.0
    assert bounded_vlm_timeout(12.0, None, now_monotonic=100.0) == 12.0


def test_vlm_request_is_rejected_after_session_deadline():
    with pytest.raises(TimeoutError, match="session budget expired"):
        bounded_vlm_timeout(65.0, 100.0, now_monotonic=100.0)
