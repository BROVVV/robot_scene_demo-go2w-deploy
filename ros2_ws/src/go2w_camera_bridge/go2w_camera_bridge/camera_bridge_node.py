"""Publish Go2-W built-in RGB as standard ROS 2 messages.

The image header uses host ROS time captured at complete message/RPC receipt.
It is deliberately never described as an exposure or hardware capture time.
"""

from __future__ import annotations

import os
import select
import struct
import subprocess
import threading
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from unitree_go.msg import Go2FrontVideoData

from .camera_bridge_core import (
    FrameDecoder,
    image_content_metrics,
    load_calibration,
    select_topic_payload,
)


class CameraBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2w_camera_bridge")
        project_root = Path(
            os.environ.get(
                "GO2W_PROJECT_ROOT", str(Path(__file__).resolve().parents[4])
            )
        )
        control_root = Path(
            os.environ.get("GO2W_CONTROL_ROOT", str(project_root / "unitree_go2w_control"))
        )
        # The robot's VideoHub RPC is read-only and has proven reliable.  The
        # custom DDS/H.264 topic remains available only when explicitly chosen.
        self.declare_parameter("source", "rpc")
        self.declare_parameter("input_topic", "/frontvideostream")
        self.declare_parameter("frame_id", "front_camera_optical_frame")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("interface", os.environ.get("GO2W_INTERFACE", "enp6s0"))
        self.declare_parameter(
            "sdk_python_path",
            os.environ.get(
                "GO2W_SDK_PYTHON_PATH",
                str(control_root / "vendor" / "unitree_sdk2_python"),
            ),
        )
        self.declare_parameter(
            "rpc_worker_python",
            os.environ.get(
                "GO2W_CONDA_PYTHON",
                os.environ.get(
                    "GO2W_CONTROL_PYTHON", str(control_root / ".venv" / "bin" / "python")
                ),
            ),
        )
        self.declare_parameter(
            "rpc_worker_script",
            os.environ.get(
                "GO2W_RPC_WORKER_SCRIPT",
                str(project_root / "scripts" / "go2w" / "videohub_rpc_worker.py"),
            ),
        )
        self.declare_parameter("rpc_timeout_sec", 3.0)
        self.declare_parameter("topic_fallback_timeout_sec", 2.0)
        self.declare_parameter("rpc_max_fps", 30.0)

        self._source_mode = str(self.get_parameter("source").value).lower()
        if self._source_mode not in {"auto", "topic", "rpc"}:
            raise ValueError("source must be auto, topic, or rpc")
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._bridge = CvBridge()
        self._decoder = FrameDecoder()
        self._sequence = 0
        self._last_topic_mono = 0.0
        self._rpc_started = False
        self._rpc_stop = threading.Event()
        self._rpc_process = None
        self._rpc_restart_delay = 1.0
        self._calibration = self._load_calibration()

        sensor_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._raw_pub = self.create_publisher(
            Image, "/camera/front/image_raw", sensor_qos
        )
        self._compressed_pub = self.create_publisher(
            CompressedImage, "/camera/front/image_raw/compressed", sensor_qos
        )
        self._info_pub = self.create_publisher(
            CameraInfo, "/camera/front/camera_info", sensor_qos
        )
        self._diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/camera/front/status", 10
        )

        if self._source_mode in {"auto", "topic"}:
            self._topic_sub = self.create_subscription(
                Go2FrontVideoData,
                str(self.get_parameter("input_topic").value),
                self._on_topic,
                sensor_qos,
            )
        if self._source_mode == "rpc":
            self._start_rpc()
        elif self._source_mode == "auto":
            delay = float(self.get_parameter("topic_fallback_timeout_sec").value)
            self._fallback_timer = self.create_timer(delay, self._check_fallback)

    def destroy_node(self) -> bool:
        self._rpc_stop.set()
        process = self._rpc_process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
        return super().destroy_node()

    def _load_calibration(self):
        value = str(self.get_parameter("calibration_file").value).strip()
        if not value:
            return None
        try:
            return load_calibration(value)
        except Exception as exc:  # diagnostic instead of guessed intrinsics
            self.get_logger().error("Camera calibration rejected: %s", exc)
            return None

    def _on_topic(self, message: Go2FrontVideoData) -> None:
        receive_stamp = self.get_clock().now()
        receive_wall_ns = time.time_ns()
        try:
            payload, field = select_topic_payload(message)
            published = self._publish_frame(
                payload,
                receive_stamp,
                receive_wall_ns,
                source=f"topic:{field}",
                source_time_frame=int(message.time_frame),
            )
            if published:
                self._last_topic_mono = time.monotonic()
                self._rpc_stop.set()  # prefer a proven-decodable topic
                process = self._rpc_process
                if process is not None and process.poll() is None:
                    process.terminate()
        except Exception as exc:
            self._publish_diagnostic(
                DiagnosticStatus.ERROR,
                "topic frame rejected",
                {"error": str(exc), "capture_time_trusted": "false"},
            )

    def _check_fallback(self) -> None:
        timeout = float(self.get_parameter("topic_fallback_timeout_sec").value)
        if self._last_topic_mono == 0.0 or time.monotonic() - self._last_topic_mono > timeout:
            self._start_rpc()

    def _start_rpc(self) -> None:
        if self._rpc_started:
            return
        self._rpc_started = True
        self._rpc_stop.clear()
        threading.Thread(target=self._rpc_loop, daemon=True).start()

    def _rpc_loop(self) -> None:
        """Run the RPC worker with per-frame fault tolerance and reconnect.

        A single corrupt JPEG must not kill the stream (that previously
        stopped camera publishing permanently). If the worker stalls or the
        pipe times out, the worker is restarted with bounded backoff.
        """
        while rclpy.ok() and not self._rpc_stop.is_set():
            process = None
            try:
                command = [
                    str(self.get_parameter("rpc_worker_python").value),
                    str(self.get_parameter("rpc_worker_script").value),
                    "--interface",
                    str(self.get_parameter("interface").value),
                    "--timeout",
                    str(self.get_parameter("rpc_timeout_sec").value),
                    "--max-fps",
                    str(self.get_parameter("rpc_max_fps").value),
                ]
                process = subprocess.Popen(
                    command,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )
                self._rpc_process = process
            except Exception as exc:
                self._publish_diagnostic(
                    DiagnosticStatus.ERROR,
                    "videohub RPC initialization failed",
                    {"error": str(exc), "capture_time_trusted": "false"},
                )
                return
            try:
                self._rpc_stream_loop(process)
            except Exception as exc:
                self._publish_diagnostic(
                    DiagnosticStatus.ERROR,
                    "videohub RPC stream interrupted; reconnecting",
                    {"error": str(exc), "capture_time_trusted": "false"},
                )
            finally:
                if process.poll() is None:
                    process.terminate()
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                self._rpc_process = None
            if self._rpc_stop.is_set() or not rclpy.ok():
                break
            self.get_logger().warn(
                f"restarting videohub RPC worker in "
                f"{self._rpc_restart_delay:.1f}s"
            )
            time.sleep(self._rpc_restart_delay)
            self._rpc_restart_delay = min(
                self._rpc_restart_delay * 2.0, 10.0
            )
        self._rpc_started = False

    def _rpc_stream_loop(self, process: subprocess.Popen) -> None:
        while rclpy.ok() and not self._rpc_stop.is_set():
            if process.poll() is not None:
                error = process.stderr.read(4096).decode(
                    "utf-8", errors="replace"
                )
                raise RuntimeError(
                    f"RPC worker exited with {process.returncode}: "
                    f"{error.strip()}"
                )
            header = self._read_exact(process.stdout, 20)
            if header is None:
                raise RuntimeError("RPC worker stream read timed out or ended")
            rpc_start_ns, rpc_end_ns, size = struct.unpack("<QQI", header)
            if size <= 0 or size > 20_000_000:
                raise RuntimeError(f"invalid RPC JPEG size: {size}")
            data = self._read_exact(process.stdout, size)
            if data is None:
                raise RuntimeError("RPC worker ended during JPEG payload")
            receive_stamp = self.get_clock().now()
            try:
                self._publish_frame(
                    data,
                    receive_stamp,
                    rpc_end_ns,
                    source="videohub_rpc_subprocess",
                    rpc_start_ns=rpc_start_ns,
                    rpc_end_ns=rpc_end_ns,
                )
            except Exception as exc:
                # A corrupt frame must not kill the whole RPC loop.
                self._publish_diagnostic(
                    DiagnosticStatus.ERROR,
                    "RPC frame skipped after decode failure",
                    {"error": str(exc), "capture_time_trusted": "false"},
                    stamp=receive_stamp,
                )
                continue
            self._rpc_restart_delay = 1.0

    @staticmethod
    def _read_exact(stream, size: int, timeout_sec: float = 3.0):
        chunks = []
        remaining = size
        deadline = time.monotonic() + timeout_sec
        while remaining > 0:
            remaining_timeout = max(0.0, deadline - time.monotonic())
            if remaining_timeout <= 0.0:
                return None
            ready, _, _ = select.select(
                [stream], [], [], remaining_timeout
            )
            if not ready:
                return None
            chunk = os.read(stream.fileno(), remaining)
            if not chunk:
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _publish_frame(
        self,
        payload: bytes,
        receive_stamp,
        receive_wall_ns: int,
        *,
        source: str,
        source_time_frame: int | None = None,
        rpc_start_ns: int | None = None,
        rpc_end_ns: int | None = None,
    ) -> bool:
        decode_start_ns = time.time_ns()
        image, codec = self._decoder.decode(payload)
        if image is None:
            self._publish_diagnostic(
                DiagnosticStatus.WARN,
                "waiting for decodable H.264 frame",
                {
                    "source": source,
                    "capture_time_trusted": "false",
                    "input_codec": codec,
                },
                stamp=receive_stamp,
            )
            return False
        decode_end_ns = time.time_ns()
        height, width = image.shape[:2]
        content = image_content_metrics(image)
        if not content["passed"]:
            self._publish_diagnostic(
                DiagnosticStatus.ERROR,
                "decoded frame rejected by transport corruption check",
                {
                    "source": source,
                    "input_codec": codec,
                    "solid_green_fraction": content["solid_green_fraction"],
                    "channel_stddev_max": content["channel_stddev_max"],
                    "capture_time_trusted": "false",
                },
                stamp=receive_stamp,
            )
            return False
        self._sequence += 1

        if codec == "jpeg":
            compressed_payload = payload
        else:
            encoded_ok, encoded = cv2.imencode(
                ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90]
            )
            if not encoded_ok:
                raise RuntimeError("failed to encode H.264 frame as JPEG")
            compressed_payload = encoded.tobytes()

        raw = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        raw.header.stamp = receive_stamp.to_msg()
        raw.header.frame_id = self._frame_id
        compressed = CompressedImage()
        compressed.header = raw.header
        compressed.format = "jpeg"
        compressed.data = compressed_payload
        info = self._camera_info(width, height)
        info.header = raw.header

        self._raw_pub.publish(raw)
        self._compressed_pub.publish(compressed)
        self._info_pub.publish(info)
        publish_ns = time.time_ns()
        calibrated = bool(self._calibration and self._calibration.calibrated)
        values = {
            "source": source,
            "input_codec": codec,
            "frame_sequence": str(self._sequence),
            "source_time_frame_untrusted": str(source_time_frame or ""),
            "receive_wall_time_ns": str(receive_wall_ns),
            "rpc_start_wall_time_ns": str(rpc_start_ns or ""),
            "rpc_end_wall_time_ns": str(rpc_end_ns or ""),
            "decode_start_wall_time_ns": str(decode_start_ns),
            "decode_end_wall_time_ns": str(decode_end_ns),
            "publish_wall_time_ns": str(publish_ns),
            "capture_time_trusted": "false",
            "timestamp_semantics": "host ROS time at complete frame receipt",
            "width": str(width),
            "height": str(height),
            "input_payload_bytes": str(len(payload)),
            "jpeg_payload_bytes": str(len(compressed_payload)),
            "camera_calibrated": str(calibrated).lower(),
            "content_check_passed": "true",
            "solid_green_fraction": str(content["solid_green_fraction"]),
            "channel_stddev_max": str(content["channel_stddev_max"]),
        }
        level = DiagnosticStatus.OK if calibrated else DiagnosticStatus.WARN
        message = "frame published" if calibrated else "frame published; CameraInfo uncalibrated"
        self._publish_diagnostic(level, message, values, stamp=receive_stamp)
        return True

    def _camera_info(self, width: int, height: int) -> CameraInfo:
        info = CameraInfo()
        info.width = width
        info.height = height
        calibration = self._calibration
        if calibration and calibration.calibrated:
            if (width, height) != (calibration.width, calibration.height):
                self.get_logger().error(
                    "Calibration resolution %dx%d does not match frame %dx%d",
                    calibration.width,
                    calibration.height,
                    width,
                    height,
                )
                return info
            info.distortion_model = calibration.distortion_model
            info.d = list(calibration.d)
            info.k = list(calibration.k)
            info.r = list(calibration.r)
            info.p = list(calibration.p)
        return info

    def _publish_diagnostic(self, level, message, values, stamp=None) -> None:
        array = DiagnosticArray()
        array.header.stamp = (stamp or self.get_clock().now()).to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = "go2w_camera_bridge/front_rgb"
        status.hardware_id = "go2w_builtin_front_rgb"
        status.message = message
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in values.items()]
        array.status = [status]
        self._diagnostic_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
