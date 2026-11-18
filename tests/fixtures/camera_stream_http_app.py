"""Real HTTP fixture for camera disconnect/shutdown; no ROS worker or model."""
from dataclasses import replace
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
out = Path(os.environ['CAMERA_STREAM_TEST_OUT'])
os.environ['GO2W_SEARCH_REPLAY_ROOT'] = str(out/'replay')
os.environ['GO2W_SEARCH_REPLAY_ENABLED'] = '0'
from app.manual_web_demo.config import ManualDemoSettings

override = os.environ.get('GO2W_TEST_WEB_SOURCE')
if override:
    spec = importlib.util.spec_from_file_location('app.manual_web_demo.web_server', override)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
else:
    from app.manual_web_demo import web_server as module
import uvicorn


class Worker:
    def set_on_message(self, callback):
        self.callback = callback
    def start(self):
        self.callback('camera_status', {'available': True, 'received_at': time.monotonic(),
            'width': 4, 'height': 3, 'format': 'jpeg'})
    def stop(self):
        (out/'worker_stopped').write_text('stopped')
    def alive(self):
        return True
    def status(self):
        return {'state': 'ready', 'motion_available': False, 'state_fresh': False}
    def request_status(self):
        pass
    def forbidden(self, *args, **kwargs):
        raise AssertionError('read-only fixture issued a control command')
    request_stop = request_estop = request_pulse = forbidden


config = replace(ManualDemoSettings(), runtime_dir=str(out/'runtime'),
    logs_dir=str(out/'logs'), analysis_frames_dir=str(out/'analysis'),
    display_only=True, web_3d_map_enabled=False, web_2d_map_enabled=True)
config.runtime_dir_path.mkdir(parents=True)
config.latest_frame_path.write_bytes(module._solid_jpeg())
runtime = module.DemoRuntime(config, worker=Worker(), enable_replay=False,
    search_session_dir=str(out/'sessions'), analyzer_fn=lambda _: {})
app = module.create_app(config=config, runtime=runtime)
uvicorn.run(app, host='127.0.0.1', port=int(os.environ['CAMERA_STREAM_TEST_PORT']), log_level='warning')
