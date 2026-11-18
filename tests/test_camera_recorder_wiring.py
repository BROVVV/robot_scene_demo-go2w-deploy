from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_camera_recorder_controls_and_script_are_wired() -> None:
    html = (ROOT / "app/manual_web_demo/templates/index.html").read_text(
        encoding="utf-8"
    )
    assert 'id="btn-camera-record"' in html
    assert 'id="btn-camera-record-stop"' in html
    assert 'id="camera-record-status"' in html
    assert 'id="camera-record-downloads"' in html
    assert '/static/camera_recorder.js?' in html


def test_camera_recorder_composites_camera_and_detection_overlay() -> None:
    source = (ROOT / "app/manual_web_demo/static/camera_recorder.js").read_text(
        encoding="utf-8"
    )
    assert 'document.getElementById("search-camera")' in source
    assert 'document.getElementById("search-overlay")' in source
    assert 'document.getElementById("search-map")' in source
    assert 'document.getElementById("slam-map-3d")' in source
    assert "drawContainedImage(context, camera" in source
    assert "context.drawImage(overlay" in source
    assert "captureStream(20)" in source
    assert "new MediaRecorder" in source
    assert 'makeChannel("camera_boxes"' in source
    assert 'makeChannel("topology"' in source
    assert 'makeChannel("2d_map_objects"' in source
    assert 'link.download = "go2w_" + channel.key' in source


def test_camera_recorder_starts_and_stops_available_channels_together() -> None:
    source = (ROOT / "app/manual_web_demo/static/camera_recorder.js").read_text(
        encoding="utf-8"
    )
    assert "channels.forEach(function (channel) { channel.recorder.start(1000); });" in source
    assert "active.forEach(function (channel) { channel.recorder.stop(); });" in source
    assert "pendingStops = channels.length" in source
    assert '"● " + channels.length + " 路同步录像中 "' in source
    assert "if (mapCanvas) result.push(" in source
    assert "!topologySvg || !mapCanvas" not in source


def test_topology_svg_and_2d_map_are_rendered_into_separate_recording_canvases() -> None:
    source = (ROOT / "app/manual_web_demo/static/camera_recorder.js").read_text(
        encoding="utf-8"
    )
    assert "new XMLSerializer().serializeToString(clone)" in source
    assert "drawTopologyFrame" in source
    assert "drawMapFrame" in source
    assert "context,\n        mapCanvas," in source
