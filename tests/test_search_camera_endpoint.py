from pathlib import Path

from app.manual_web_demo import search_models


def config(monkeypatch, tmp_path):
    path = tmp_path / 'configs/go2w/autonomous_search_web.yaml'
    path.parent.mkdir(parents=True)
    path.write_text('search:\n  backend: nav2\n  enable_autonomous_motion: false\n'
                    '  rgbd_base_url: http://192.168.123.18:8080\n')
    monkeypatch.setattr(search_models, '_PROJECT_ROOT', tmp_path)


def test_shared_camera_endpoint_reaches_production_search_request(monkeypatch, tmp_path):
    config(monkeypatch, tmp_path)
    monkeypatch.setenv('D435_BASE_URL', 'http://127.0.0.1:8080/')
    request = search_models.SearchStartRequest.from_dict({
        'task_text': '寻找椅子', 'rgbd_base_url': 'http://untrusted.invalid:9999'})
    assert request.rgbd_base_url == 'http://127.0.0.1:8080'


def test_workstation_without_camera_override_keeps_configured_endpoint(monkeypatch, tmp_path):
    config(monkeypatch, tmp_path)
    monkeypatch.delenv('D435_BASE_URL', raising=False)
    assert search_models.load_search_defaults()['rgbd_base_url'] == 'http://192.168.123.18:8080'


def test_empty_camera_override_does_not_erase_valid_configuration(monkeypatch, tmp_path):
    config(monkeypatch, tmp_path)
    monkeypatch.setenv('D435_BASE_URL', '  ')
    assert search_models.load_search_defaults()['rgbd_base_url'] == 'http://192.168.123.18:8080'
