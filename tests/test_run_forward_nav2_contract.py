from pathlib import Path


SCRIPT = Path("scripts/go2w/run_forward_nav2_test.py")


def test_bounded_trial_requires_continuous_readiness_before_goal():
    source = SCRIPT.read_text()
    assert "def wait_for_stable_backend(timeout_s,stable_s=.6):" in source
    assert "if now-stable_since>=stable_s:return pose" in source
    assert source.count("before=wait_for_stable_backend(") == 2
    assert "if blockers:raise RuntimeError('motion preflight incomplete:" in source
    assert "handle=backend.execute_goal(goal)" in source


def test_stable_wait_remains_fail_closed():
    source = SCRIPT.read_text()
    body = source.split("def wait_for_stable_backend", 1)[1].split(
        "backend=create_nav2_search_backend", 1
    )[0]
    assert "last_health=backend.health()" in body
    assert "last_health.ready and pose is not None" in body
    assert "stable_since=None" in body
    assert "raise AssertionError(last_health)" in body


def test_collision_diagnostic_uses_the_ledger_observed_travel_cap():
    source = SCRIPT.read_text()
    assert "getattr(budget,'collision_diagnostic',False)" in source
    assert "args.distance<=budget.MAX_TRANSLATION_M-budget.STOP_RESERVE_M" in source
