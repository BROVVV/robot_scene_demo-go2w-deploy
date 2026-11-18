from __future__ import annotations

from pathlib import Path

import pytest

from app.manual_web_demo.runtime_admin import RuntimeAdminService


class _Process:
    pid = 4321


def _service(tmp_path: Path) -> RuntimeAdminService:
    supervisor = tmp_path / "scripts" / "go2w" / "reset_webui_runtime.sh"
    supervisor.parent.mkdir(parents=True)
    supervisor.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    return RuntimeAdminService(
        project_root=tmp_path,
        mode="teleop_mapping",
        detector="grounded_sam",
    )


def test_reset_requires_exact_confirmation(tmp_path: Path) -> None:
    service = _service(tmp_path)

    with pytest.raises(ValueError, match="confirmation_mismatch"):
        service.request_reset(action="map", confirmation="yes")

    assert not service.lock_path.exists()


def test_reset_launches_enum_only_detached_supervisor(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    service = _service(tmp_path)
    captured = {}

    def fake_popen(argv, **kwargs):
        captured["argv"] = argv
        captured["kwargs"] = kwargs
        return _Process()

    monkeypatch.setattr("app.manual_web_demo.runtime_admin.subprocess.Popen", fake_popen)
    result = service.request_reset(
        action="objects", confirmation="CLEAR_CURRENT_OBJECTS"
    )

    assert result["accepted"] is True
    assert result["supervisor_pid"] == 4321
    assert service.lock_path.exists()
    assert captured["argv"] == [
        "/usr/bin/env",
        "bash",
        str(service.supervisor),
        "--action",
        "objects",
        "--mode",
        "teleop_mapping",
        "--detector",
        "grounded_sam",
        "--lock-file",
        str(service.lock_path),
    ]
    assert captured["kwargs"]["start_new_session"] is True
    assert service.status()["busy"] is True


def test_reset_rejects_concurrent_request(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    service = _service(tmp_path)
    monkeypatch.setattr(
        "app.manual_web_demo.runtime_admin.subprocess.Popen",
        lambda *args, **kwargs: _Process(),
    )
    service.request_reset(action="map", confirmation="CLEAR_CURRENT_MAP")

    with pytest.raises(RuntimeError, match="reset_already_running"):
        service.request_reset(action="map", confirmation="CLEAR_CURRENT_MAP")
