"""Camera ownership, producer loss and reboot cannot hide ROS fallback."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from app.manual_web_demo.camera_source_health import fresh_http_camera

ROOT = Path(__file__).resolve().parents[1]
def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

worker_module = load('camera_owner_worker', 'scripts/go2w/manual_web_demo_ros_worker.py')
capture = load('camera_owner_capture', 'scripts/go2w/robot_camera_http_capture.py')
JPEG = b'\xff\xd8\xff\xc0\x00\x11\x08\x00\x03\x00\x04' + bytes(10) + b'\xff\xd9'


def setup_status(tmp_path, monkeypatch):
    monkeypatch.setattr(capture, '_BOOT_ID', 'boot-new')
    monkeypatch.setattr(capture.time, 'monotonic', lambda: 10.)
    latest = tmp_path / 'latest.jpg'
    capture._write_frame(JPEG, latest, tmp_path / 'camera_status.json')
    return latest, tmp_path / 'camera_http_status.json'


def test_http_writer_keeps_original_jpeg_and_independent_boot_heartbeat(tmp_path, monkeypatch):
    latest, path = setup_status(tmp_path, monkeypatch)
    assert latest.read_bytes() == JPEG
    status = fresh_http_camera(path, latest_path=latest, boot_id='boot-new', now=10.5, max_age=1.)
    assert status['width'] == 4 and status['height'] == 3
    assert status['source'] == 'robot_http_mjpeg'
    # The fallback writer may update the legacy shared status. This cannot
    # renew the HTTP producer's independent heartbeat.
    (tmp_path / 'camera_status.json').write_text(json.dumps({'available': True, 'received_at': 12.}))
    assert fresh_http_camera(path, latest_path=latest, boot_id='boot-new', now=12., max_age=1.) is None


@pytest.mark.parametrize('patch', [
    {'owner_boot_id': 'boot-old'}, {'available': False}, {'source': 'ros'},
    {'received_at': float('nan')}, {'received_at': float('inf')},
    {'received_at': True}, {'received_at': 11.}, {'received_at': 8.9},
    {'width': None}, {'height': 0}, {'width': True}, {'latest_path': '/other/latest.jpg'},
])
def test_invalid_or_stale_producer_never_suppresses_ros(tmp_path, monkeypatch, patch):
    latest, path = setup_status(tmp_path, monkeypatch)
    value = json.loads(path.read_text()); value.update(patch); path.write_text(json.dumps(value))
    assert fresh_http_camera(path, latest_path=latest, boot_id='boot-new', now=10., max_age=1.) is None


@pytest.mark.parametrize('value', ['not-json', '[]', 'null'])
def test_unreadable_status_uses_ros_fallback(tmp_path, value):
    path = tmp_path / 'status'; path.write_text(value)
    assert fresh_http_camera(path, latest_path=tmp_path / 'latest', boot_id='boot-new', now=10., max_age=1.) is None


class Node:
    def __init__(self): self.live = []; self.created = 0; self.destroyed = 0
    def create_subscription(self, *args):
        handle = object(); self.live.append(handle); self.created += 1; return handle
    def destroy_subscription(self, handle):
        self.live.remove(handle); self.destroyed += 1


def worker(tmp_path, owner):
    value = worker_module.ManualWebDemoWorker.__new__(worker_module.ManualWebDemoWorker)
    value.settings = SimpleNamespace(camera_owner=owner, camera_stale_seconds=1.)
    value.node = Node(); value._camera_subscription = None
    value._camera_message_type = object; value._camera_qos = object()
    value._http_camera_status_path = tmp_path / 'camera_http_status.json'
    value._latest_path = tmp_path / 'latest.jpg'; value._camera_status_path = tmp_path / 'camera_status.json'
    value._camera_boot_id = 'boot-new'; value._camera_available = False
    value._using_http_camera = False; value._last_http_received = None
    return value


def test_producer_loss_and_recovery_transfer_ownership_without_duplicate_ros(tmp_path, monkeypatch):
    setup_status(tmp_path, monkeypatch)
    messages = []; monkeypatch.setattr(worker_module, 'emit', messages.append)
    value = worker(tmp_path, 'http_preferred')
    value._camera_source_tick(); assert not value.node.live and value._using_http_camera
    monkeypatch.setattr(worker_module.time, 'monotonic', lambda: 12.)
    value._camera_source_tick(); value._camera_source_tick()
    assert len(value.node.live) == value.node.created == 1
    assert not value._using_http_camera
    assert not value._camera_available
    value._on_camera(SimpleNamespace(data=JPEG, format='jpeg'))
    assert value._camera_available
    assert json.loads(value._http_camera_status_path.read_text())['received_at'] == 10.
    capture._write_frame(JPEG + b'HTTP', value._latest_path, value._camera_status_path)
    value._camera_source_tick()
    assert not value.node.live and value.node.destroyed == 1 and value._using_http_camera
    # A ROS callback already queued before unsubscribe cannot overwrite HTTP.
    value._on_camera(SimpleNamespace(data=b'queued old frame', format='jpeg'))
    assert value._latest_path.read_bytes() == JPEG + b'HTTP'


def test_desktop_default_keeps_ros_even_when_http_heartbeat_exists(tmp_path, monkeypatch):
    setup_status(tmp_path, monkeypatch)
    value = worker(tmp_path, 'ros'); value._camera_source_tick()
    assert len(value.node.live) == 1 and not value._using_http_camera
