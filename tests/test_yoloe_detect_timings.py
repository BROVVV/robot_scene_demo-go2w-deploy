"""Detector timings must account for work after the model returns."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image

from app.detectors import yoloe_seg_worker as worker


@pytest.mark.parametrize('empty', [False, True])
def test_total_includes_postprocessing_and_health(tmp_path, monkeypatch, empty):
    image = tmp_path / 'frame.png'
    Image.new('RGB', (10, 8)).save(image)
    clock = [100.0]
    monkeypatch.setattr(worker.time, 'perf_counter', lambda: clock[0])
    runtime = object.__new__(worker.YOLOERuntime)
    runtime.prompt_free = True
    runtime.model_terms = ['chair']
    runtime.model_path = Path('test-seg-pf.pt')
    runtime.image_size = 640
    runtime.confidence = 0.25
    runtime.iou_threshold = 0.5
    runtime.max_detections = 100
    runtime.device = 'cuda:0'

    def predict(*args, **kwargs):
        clock[0] += 2
        if empty:
            return [SimpleNamespace(boxes=None)]
        masks = np.zeros((1, 8, 10))
        masks[0, 2:6, 3:7] = 1
        return [SimpleNamespace(
            boxes=[0], masks=SimpleNamespace(data=masks)
        )]

    class Boxes:
        xyxy = np.array([[0, 0, 10, 8]])
        conf = np.array([0.8])
        cls = np.array([0])

        def __len__(self):
            return 1

    def wrapped_predict(*args, **kwargs):
        result = predict(*args, **kwargs)
        if not empty:
            result[0].boxes = Boxes()
        return result

    original_encode = worker.encode_row_runs

    def encode(mask):
        clock[0] += 3
        return original_encode(mask)

    def health():
        clock[0] += 5
        return {'ok': True}

    runtime._predict = wrapped_predict
    runtime.health = health
    monkeypatch.setattr(worker, 'encode_row_runs', encode)
    result = runtime.detect({'image': str(image), 'request_id': 'timing'})
    diagnostics = result['diagnostics']
    assert diagnostics['inference_latency_ms'] == 2000
    assert diagnostics['postprocess_latency_ms'] == (5000 if empty else 8000)
    assert diagnostics['total_latency_ms'] == (7000 if empty else 10000)
    assert diagnostics['masks_invalid'] == 0
    if not empty:
        assert result['objects'][0]['mask_rle'] == [[y, 3, 4] for y in range(2, 6)]
        assert result['objects'][0]['mask_size'] == [8, 10]
        assert result['objects'][0]['mask_area_ratio'] == 0.2
