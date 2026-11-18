from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from app.config import Settings
from app.detectors.grounded_sam_subprocess import (
    DetectorRuntimeError,
    GroundedSAMSubprocessDetector,
)
from app.video.target_profile import TargetProfile


class _FakeStdin:
    def __init__(self) -> None:
        self.requests: list[dict] = []
        self.closed = False

    def write(self, text: str) -> None:
        self.requests.append(json.loads(text))

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True


class _FakeWorker:
    """Fake the persistent JSONL worker: stdin records requests, readline serves
    the queued response lines in order, poll reports alive."""

    def __init__(self, response_lines: list[str]) -> None:
        self._lines = list(response_lines)
        self.stdin = _FakeStdin()

    @property
    def stdout(self) -> "_FakeWorker":
        return self

    def readline(self) -> str:
        if not self._lines:
            return ""  # EOF == worker died
        return self._lines.pop(0)

    def poll(self) -> None:
        return None

    def terminate(self) -> None:
        pass

    def wait(self, timeout=None) -> int:
        return 0

    def kill(self) -> None:
        pass


def _response(objects: list[dict]) -> str:
    return json.dumps({"objects": objects}) + "\n"


class GroundedSAMRuntimeTest(unittest.TestCase):
    def test_persistent_worker_and_mask_parsing(self) -> None:
        detector = GroundedSAMSubprocessDetector(
            Settings(
                siliconflow_api_key="",
                grounded_sam_root="/tmp",
                grounded_sam_python="python",
                enable_sam2=True,
            ),
            target_profile=TargetProfile(
                raw_query="找灭火器",
                canonical_name_zh="灭火器",
                primary_labels_en=["fire extinguisher"],
                context_labels_en=["wall"],
            ),
        )
        worker = _FakeWorker(
            [
                _response(
                    [
                        {
                            "label": "red fire extinguisher",
                            "score": 0.91,
                            "text_score": 0.9,
                            "bbox_2d": [0.1, 0.2, 0.2, 0.6],
                            "mask_area_ratio": 0.12,
                            "mask_size": [480, 640],
                            "mask_rle": [[100, 200, 40], [101, 205, 38]],
                            "source_prompt_term": "fire extinguisher",
                        }
                    ]
                ),
                _response(
                    [
                        {
                            "label": "red fire extinguisher",
                            "score": 0.88,
                            "bbox_2d": [0.15, 0.25, 0.25, 0.65],
                            "mask_size": [480, 640],
                            "mask_rle": [[100, 200, 40]],
                        }
                    ]
                ),
            ]
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            image = Path(tmpdir) / "image.jpg"
            image.write_bytes(b"mock")
            with patch(
                "app.detectors.grounded_sam_subprocess.subprocess.Popen",
                return_value=worker,
            ) as popen:
                objects = detector.detect(str(image), "找灭火器")
                again = detector.detect(str(image), "找灭火器")
            self.assertEqual(popen.call_count, 1)  # 常驻：session 内只 spawn 一次
        self.assertEqual(objects[0].label_zh, "灭火器")
        self.assertEqual(objects[0].mask_rle, [[100, 200, 40], [101, 205, 38]])
        self.assertEqual(objects[0].mask_size, (480, 640))
        self.assertIn("segmented_by_sam2", objects[0].attributes)
        self.assertEqual(len(again), 1)

    def test_dynamic_retry_uses_focused_prompt_and_parses_result(self) -> None:
        detector = GroundedSAMSubprocessDetector(
            Settings(
                siliconflow_api_key="",
                grounded_sam_root="/tmp",
                grounded_sam_python="python",
                enable_sam2=False,
            ),
            target_profile=TargetProfile(
                raw_query="找灭火器",
                canonical_name_zh="灭火器",
                primary_labels_en=["fire extinguisher"],
                context_labels_en=["wall"],
            ),
        )
        worker = _FakeWorker(
            [
                _response(
                    [
                        {
                            "label": "red fire extinguisher",
                            "score": 0.91,
                            "bbox_2d": [0.1, 0.2, 0.2, 0.6],
                        }
                    ]
                )
            ]
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            image = Path(tmpdir) / "image.jpg"
            image.write_bytes(b"mock")
            with patch(
                "app.detectors.grounded_sam_subprocess.subprocess.Popen",
                return_value=worker,
            ):
                objects = detector.detect_with_dynamic_terms(
                    str(image),
                    "找灭火器",
                    ["portable extinguisher"],
                )

        prompt = worker.stdin.requests[0]["text_prompt"]
        self.assertIn("fire extinguisher", prompt)
        self.assertIn("portable extinguisher", prompt)
        self.assertNotIn("office chair", prompt)
        self.assertEqual(objects[0].label_zh, "灭火器")

    def test_request_carries_sam2_min_box_score_from_settings(self) -> None:
        """轻量化计划 §20：门槛要从 settings 走到 worker 的**请求体**里。

        行为级断言 —— 用的是真 ``Settings()`` 默认值，不是手写的常量，所以它还
        顺带证明了「默认值确实落在了请求那一行上」。worker 侧是
        ``request.get("sam2_min_box_score", args.sam2_min_box_score)``，请求优先；
        请求里没这一项时 worker 会用自己的 argparse 默认值，settings 改了不生效。
        """
        detector = GroundedSAMSubprocessDetector(
            Settings(
                siliconflow_api_key="",
                grounded_sam_root="/tmp",
                grounded_sam_python="python",
                enable_sam2=True,
            ),
            target_profile=TargetProfile(
                raw_query="找灭火器",
                canonical_name_zh="灭火器",
                primary_labels_en=["fire extinguisher"],
            ),
        )
        worker = _FakeWorker([_response([])])
        with tempfile.TemporaryDirectory() as tmpdir:
            image = Path(tmpdir) / "image.jpg"
            image.write_bytes(b"mock")
            with patch(
                "app.detectors.grounded_sam_subprocess.subprocess.Popen",
                return_value=worker,
            ):
                detector.detect(str(image), "找灭火器")
        self.assertEqual(worker.stdin.requests[0]["sam2_min_box_score"], 0.35)

    def test_worker_death_is_wrapped_as_detector_runtime_error(self) -> None:
        detector = GroundedSAMSubprocessDetector(
            Settings(
                siliconflow_api_key="",
                grounded_sam_root="/tmp",
                grounded_sam_python="python",
                enable_sam2=False,
            ),
            target_profile=TargetProfile(
                raw_query="找灭火器",
                canonical_name_zh="灭火器",
                primary_labels_en=["fire extinguisher"],
            ),
        )
        worker = _FakeWorker([])  # readline returns "" immediately == died
        with tempfile.TemporaryDirectory() as tmpdir:
            image = Path(tmpdir) / "image.jpg"
            image.write_bytes(b"not-an-image-but-worker-is-mocked")
            with patch(
                "app.detectors.grounded_sam_subprocess.subprocess.Popen",
                return_value=worker,
            ):
                with self.assertRaisesRegex(
                    DetectorRuntimeError,
                    "exited unexpectedly",
                ):
                    detector.detect(str(image), "找灭火器")


if __name__ == "__main__":
    unittest.main()
