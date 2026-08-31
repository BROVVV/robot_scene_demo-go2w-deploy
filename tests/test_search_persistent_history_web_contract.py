from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_search_page_has_natural_language_entry_history_and_error_detail() -> None:
    html = (ROOT / "app/manual_web_demo/templates/index.html").read_text(encoding="utf-8")
    script = (ROOT / "app/manual_web_demo/static/search_ui.js").read_text(encoding="utf-8")

    assert 'id="search-target"' in html
    assert 'id="search-history"' in html
    assert 'id="search-error-detail"' in html
    assert 'id="btn-history-current"' in html
    assert 'task_text: target' in script
    # The normal start path must not override server-side real/dry-run policy.
    start_payload = script.split('api("/api/search/start", {', 1)[1].split("}).then", 1)[0]
    assert "enable_autonomous_motion" not in start_payload
    assert "dry_run_motion" not in start_payload


def test_history_view_keeps_live_state_separate() -> None:
    script = (ROOT / "app/manual_web_demo/static/search_ui.js").read_text(encoding="utf-8")
    assert "viewingHistoryId" in script
    assert "currentLiveState" in script
    assert 'fetch("/api/search/history/"' in script
