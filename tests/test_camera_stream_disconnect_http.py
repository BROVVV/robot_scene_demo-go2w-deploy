"""Closing real camera HTTP streams must let Web shutdown reach its worker."""
from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request

import pytest

pytest.importorskip('fastapi')
pytest.importorskip('uvicorn')
ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('partial', [False, True])
def test_closed_camera_connections_do_not_prevent_graceful_web_stop(tmp_path, partial):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    environment = dict(os.environ, CAMERA_STREAM_TEST_OUT=str(tmp_path),
        CAMERA_STREAM_TEST_PORT=str(port), GO2W_SEARCH_REPLAY_ROOT=str(tmp_path/'replay'),
        GO2W_SEARCH_REPLAY_ENABLED='0', MANUAL_DEMO_RUNTIME_DIR=str(tmp_path/'runtime'),
        MANUAL_DEMO_LOGS_DIR=str(tmp_path/'logs'), SILICONFLOW_API_KEY='', OPENAI_API_KEY='')
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    base = 'http://127.0.0.1:' + str(port)
    log = (tmp_path/'web.log').open('w')
    process = subprocess.Popen([sys.executable, str(ROOT/'tests/fixtures/camera_stream_http_app.py')],
        cwd=tmp_path, env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        ready = False
        deadline = time.monotonic()+8.
        while time.monotonic() < deadline:
            if process.poll() is not None:
                break
            try:
                with opener.open(base+'/api/status', timeout=.5) as response:
                    ready = json.loads(response.read())['control']['display_only']
                if ready:
                    break
            except (OSError, urllib.error.URLError):
                time.sleep(.02)
        assert ready, (tmp_path/'web.log').read_text()
        for _ in range(3):
            with opener.open(base+'/api/camera.mjpeg', timeout=2) as response:
                length = None
                for _ in range(12):
                    line = response.readline()
                    if line.lower().startswith(b'content-length:'):
                        length = int(line.split(b':', 1)[1])
                    if line in (b'\r\n', b'\n'):
                        break
                assert length
                data = response.read(2 if partial else length)
                assert data.startswith(b'\xff\xd8')
        process.send_signal(signal.SIGTERM)
        try:
            code = process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            pytest.fail('closed video requests kept the server running before worker shutdown')
        assert code in (0, -signal.SIGTERM)
        assert (tmp_path/'worker_stopped').read_text() == 'stopped'
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)
        log.close()
