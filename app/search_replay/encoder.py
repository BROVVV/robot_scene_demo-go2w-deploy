"""FFmpeg 编码与成品校验。

成品判定不是"文件非零"，而是：ffprobe 能解析出 h264/yuv420p、尺寸/帧率/
时长与渲染计划一致，且首/中/尾三帧能真实解码。校验通过后才把
``*.partial.mp4`` 原子改名为正式文件。
"""

from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

from app.search_replay.models import ReplayConfig, json_safe
from app.search_replay.renderer import RENDERER_VERSION, RenderMetadata, ReplayRenderer

ENCODER_VERSION = "search_replay_encoder_v1"


class EncodeError(RuntimeError):
    pass


@dataclass
class EncoderResult:
    ok: bool
    path: Path | None = None
    partial_path: Path | None = None
    width: int = 0
    height: int = 0
    fps: float = 0.0
    duration: float = 0.0
    frames: int = 0
    size_bytes: int = 0
    codec: str = ""
    pix_fmt: str = ""
    error: str = ""
    probe: dict[str, Any] = field(default_factory=dict)
    sample_checks: list[dict[str, Any]] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "path": str(self.path) if self.path else None,
            "width": self.width,
            "height": self.height,
            "fps": round(self.fps, 3),
            "duration": round(self.duration, 3),
            "frames": self.frames,
            "size_bytes": self.size_bytes,
            "codec": self.codec,
            "pix_fmt": self.pix_fmt,
            "error": self.error,
            "sample_checks": self.sample_checks,
        }


def _log(path: Path | None, message: str) -> None:
    if path is None:
        return
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as handle:
            handle.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {message}\n")
    except OSError:
        pass


def ffmpeg_path() -> str:
    return shutil.which("ffmpeg") or "/usr/bin/ffmpeg"


def ffprobe_path() -> str:
    return shutil.which("ffprobe") or "/usr/bin/ffprobe"


def ffmpeg_available() -> bool:
    return os.access(ffmpeg_path(), os.X_OK) and os.access(ffprobe_path(), os.X_OK)


def probe_video(path: str | Path) -> dict[str, Any]:
    """Run ffprobe and return a compact, JSON-safe summary."""
    command = [
        ffprobe_path(), "-v", "error", "-print_format", "json",
        "-show_streams", "-show_format", str(path),
    ]
    try:
        result = subprocess.run(command, capture_output=True, text=True,
                                timeout=60, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"ok": False, "error": f"ffprobe 启动失败: {exc}"}
    if result.returncode != 0:
        return {"ok": False, "error": (result.stderr or "").strip()[:800]}
    try:
        payload = json.loads(result.stdout or "{}")
    except json.JSONDecodeError:
        return {"ok": False, "error": "ffprobe 输出不是 JSON"}
    streams = payload.get("streams") or []
    video = next(
        (stream for stream in streams
         if stream.get("codec_type") == "video"),
        None,
    )
    if video is None:
        return {"ok": False, "error": "没有视频流"}
    duration = _float(video.get("duration")) or _float(
        (payload.get("format") or {}).get("duration")
    )
    fps = _ratio(video.get("avg_frame_rate")) or _ratio(video.get("r_frame_rate"))
    return {
        "ok": True,
        "codec": str(video.get("codec_name") or ""),
        "pix_fmt": str(video.get("pix_fmt") or ""),
        "width": int(video.get("width") or 0),
        "height": int(video.get("height") or 0),
        "fps": round(fps, 3),
        "duration": round(duration, 3),
        "frames": int(_float(video.get("nb_frames")) or 0),
        "bit_rate": int(_float((payload.get("format") or {}).get("bit_rate")) or 0),
        "format_name": str((payload.get("format") or {}).get("format_name") or ""),
    }


def verify_video(
    path: str | Path,
    *,
    width: int | None = None,
    height: int | None = None,
    fps: float | None = None,
    duration: float | None = None,
) -> tuple[bool, str, dict[str, Any]]:
    """Verify an already published artifact can actually be played.

    A non-empty file is not proof: a crashed copy/partial rename leaves a
    parseable header with a truncated stream.  Probe the container, compare
    the plan, then decode the stream end to end (``-f null -``) so missing
    packets surface as a non-zero exit instead of a ``READY`` badge.
    """
    probe = probe_video(path)
    if not probe.get("ok"):
        return False, str(probe.get("error") or "ffprobe 无法解析"), probe
    if str(probe.get("codec") or "") != "h264":
        return False, f"编码不是 h264（实际 {probe.get('codec')}）", probe
    if str(probe.get("pix_fmt") or "") != "yuv420p":
        return False, f"像素格式不是 yuv420p（实际 {probe.get('pix_fmt')}）", probe
    if width and int(probe.get("width") or 0) != int(width):
        return False, f"宽度 {probe.get('width')} 与记录 {width} 不符", probe
    if height and int(probe.get("height") or 0) != int(height):
        return False, f"高度 {probe.get('height')} 与记录 {height} 不符", probe
    if fps and abs(float(probe.get("fps") or 0) - float(fps)) > 0.5:
        return False, f"帧率 {probe.get('fps')} 与记录 {fps} 不符", probe
    actual = float(probe.get("duration") or 0)
    if duration:
        tolerance = max(1.5, float(duration) * 0.05)
        if actual <= 0 or abs(actual - float(duration)) > tolerance:
            return False, (f"时长 {actual:.3f}s 与记录 {float(duration):.3f}s "
                           "不符，文件可能不完整"), probe
    command = [ffmpeg_path(), "-v", "error", "-xerror", "-i", str(path),
               "-f", "null", "-"]
    try:
        result = subprocess.run(command, capture_output=True, text=True,
                                timeout=180, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, f"解码校验失败: {exc}", probe
    if result.returncode != 0:
        detail = (result.stderr or "").strip()[:300] or "解码退出码非零"
        return False, f"视频流不完整: {detail}", probe
    return True, "", probe


def _float(value: Any) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return 0.0
    return result if math.isfinite(result) else 0.0


def _ratio(value: Any) -> float:
    if not isinstance(value, str) or "/" not in value:
        return _float(value)
    numerator, _, denominator = value.partition("/")
    bottom = _float(denominator)
    return _float(numerator) / bottom if bottom else 0.0


class VideoEncoder:
    """One encoder at a time is enforced by the manager; this class is stateless."""

    def __init__(
        self,
        config: ReplayConfig,
        *,
        render_log: Path | None = None,
    ) -> None:
        self.config = config
        self.render_log = render_log

    def encode(
        self,
        renderer: ReplayRenderer,
        output_path: str | Path,
        *,
        progress: Callable[[float], None] | None = None,
    ) -> EncoderResult:
        if not ffmpeg_available():
            return EncoderResult(ok=False, error="未找到可执行的 ffmpeg/ffprobe")

        output = Path(output_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        partial = output.with_name(output.stem + ".partial.mp4")
        for stale in (partial,):
            try:
                stale.unlink()
            except FileNotFoundError:
                pass
            except OSError as exc:
                return EncoderResult(ok=False, error=f"无法清理旧临时文件: {exc}")

        metadata = renderer.prepare()
        command = self._command(metadata, partial)
        _log(self.render_log, f"encode start speed={metadata.speed} "
                             f"frames={metadata.frames} out={output.name}")
        try:
            process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
        except OSError as exc:
            return EncoderResult(ok=False, error=f"ffmpeg 启动失败: {exc}")

        broken = False
        frames_written = 0
        error_text = ""
        try:
            assert process.stdin is not None
            renderer.render(
                lambda frame: self._write(process.stdin, frame, metadata),
                progress=progress,
            )
            frames_written = metadata.frames
        except (BrokenPipeError, OSError) as exc:
            broken = True
            error_text = f"编码管道中断: {exc}"
        except EncodeError as exc:
            broken = True
            error_text = f"编码失败: {exc}"
        except Exception as exc:  # noqa: BLE001 - 渲染异常也必须收尾并报告
            broken = True
            error_text = f"渲染异常: {type(exc).__name__}: {exc}"
        finally:
            # communicate() 自己会 flush 并关闭 stdin；先手工 close 再
            # communicate 会在 Python 3.13 上抛 "flush of closed file"。
            try:
                _, stderr = process.communicate(timeout=120)
            except subprocess.TimeoutExpired:
                process.kill()
                _, stderr = process.communicate()
                broken = True
                error_text = error_text or "ffmpeg 超时"
            stderr_text = (stderr or b"").decode("utf-8", "replace").strip()

        if process.returncode != 0 or broken:
            _log(self.render_log, f"encode failed rc={process.returncode} "
                                  f"{stderr_text[:400]}")
            try:
                partial.unlink()
            except OSError:
                pass
            return EncoderResult(
                ok=False,
                error=error_text or (stderr_text[:800] or "ffmpeg 非零退出"),
            )
        if not partial.is_file() or partial.stat().st_size <= 0:
            return EncoderResult(ok=False, error="ffmpeg 未产出文件")

        probe = probe_video(partial)
        problem = self._validate_probe(probe, metadata)
        if problem:
            _log(self.render_log, f"encode validation failed: {problem}")
            return EncoderResult(
                ok=False, partial_path=partial,
                error=f"成品校验失败: {problem}", probe=probe,
                width=int(probe.get("width") or 0),
                height=int(probe.get("height") or 0),
                fps=float(probe.get("fps") or 0),
                duration=float(probe.get("duration") or 0),
            )

        checks = self._sample_frames(partial, metadata, probe)
        bad = [item for item in checks if not item.get("ok")]
        if bad:
            _log(self.render_log, f"encode frame samples bad: {bad}")
            return EncoderResult(
                ok=False, partial_path=partial,
                error="抽帧校验失败: " + "; ".join(
                    str(item.get("error") or "unknown") for item in bad
                ),
                probe=probe, sample_checks=checks,
            )

        try:
            os.replace(partial, output)
        except OSError as exc:
            return EncoderResult(ok=False, error=f"原子发布失败: {exc}")
        _log(self.render_log, f"encode ok {output.name} "
                              f"{probe.get('duration')}s {probe.get('width')}x"
                              f"{probe.get('height')} {output.stat().st_size}B")
        return EncoderResult(
            ok=True,
            path=output,
            width=int(probe.get("width") or metadata.width),
            height=int(probe.get("height") or metadata.height),
            fps=float(probe.get("fps") or metadata.fps),
            duration=float(probe.get("duration") or metadata.duration),
            frames=int(probe.get("frames") or frames_written),
            size_bytes=output.stat().st_size,
            codec=str(probe.get("codec") or ""),
            pix_fmt=str(probe.get("pix_fmt") or ""),
            probe=probe,
            sample_checks=checks,
        )

    # ------------------------------------------------------------------ #
    def _command(self, metadata: RenderMetadata, partial: Path) -> list[str]:
        return [
            ffmpeg_path(),
            "-hide_banner", "-nostdin", "-loglevel", "error", "-y",
            "-f", "rawvideo",
            "-pix_fmt", "rgb24",
            "-s", f"{metadata.width}x{metadata.height}",
            "-r", str(metadata.fps),
            "-i", "-",
            "-an",
            "-c:v", "libx264",
            "-preset", str(self.config.encoder_preset),
            "-crf", str(int(self.config.encoder_crf)),
            "-pix_fmt", "yuv420p",
            "-threads", str(int(self.config.encoder_threads)),
            "-movflags", "+faststart",
            "-metadata:s:v:0", "title=go2w search replay",
            str(partial),
        ]

    @staticmethod
    def _write(handle, frame, metadata: RenderMetadata) -> None:
        import numpy as np

        array = np.asarray(frame)
        if array.shape != (metadata.height, metadata.width, 3):
            raise EncodeError(
                f"帧尺寸异常: {array.shape} != "
                f"({metadata.height},{metadata.width},3)"
            )
        if array.dtype != np.uint8:
            array = array.astype(np.uint8, copy=False)
        handle.write(array.tobytes())

    def _validate_probe(self, probe: dict[str, Any],
                        metadata: RenderMetadata) -> str:
        if not probe.get("ok"):
            return str(probe.get("error") or "ffprobe 无法解析")
        codec = str(probe.get("codec") or "")
        if codec != "h264":
            return f"编码不是 h264（实际 {codec}）"
        if str(probe.get("pix_fmt") or "") != "yuv420p":
            return f"像素格式不是 yuv420p（实际 {probe.get('pix_fmt')}）"
        if int(probe.get("width") or 0) != metadata.width \
                or int(probe.get("height") or 0) != metadata.height:
            return (f"分辨率 {probe.get('width')}x{probe.get('height')} "
                    f"与计划 {metadata.width}x{metadata.height} 不符")
        fps = float(probe.get("fps") or 0)
        if abs(fps - metadata.fps) > 0.5:
            return f"帧率 {fps} 与计划 {metadata.fps} 不符"
        duration = float(probe.get("duration") or 0)
        expected = metadata.duration
        tolerance = max(1.5, expected * 0.02)
        if duration <= 0 or abs(duration - expected) > tolerance:
            return f"时长 {duration:.3f}s 与计划 {expected:.3f}s 不符"
        frames = int(probe.get("frames") or 0)
        if frames and abs(frames - metadata.frames) > max(
            3, int(metadata.fps * 0.5)
        ):
            return f"帧数 {frames} 与计划 {metadata.frames} 不符"
        return ""

    def _sample_frames(
        self, path: Path, metadata: RenderMetadata, probe: dict[str, Any],
    ) -> list[dict[str, Any]]:
        duration = float(probe.get("duration") or metadata.duration)
        # ``-ss`` seeks to the first frame at/after the offset.  Container
        # duration can be one frame longer than the last frame's PTS, so
        # ``duration - 0.1`` lands past the final frame for low frame rates
        # ("无输出").  Stay at least 1.5 frame intervals inside.
        last_offset = max(0.0, duration - 1.5 / max(1.0, float(metadata.fps)))
        targets = [("first", 0.05), ("middle", duration / 2.0),
                   ("last", last_offset)]
        checks: list[dict[str, Any]] = []
        with tempfile.TemporaryDirectory(prefix="replay_check_") as directory:
            for name, offset in targets:
                output = Path(directory) / f"{name}.png"
                command = [
                    ffmpeg_path(), "-v", "error", "-y",
                    "-ss", f"{max(0.0, offset):.3f}",
                    "-i", str(path),
                    "-frames:v", "1", str(output),
                ]
                try:
                    result = subprocess.run(command, capture_output=True,
                                            text=True, timeout=60, check=False)
                except (OSError, subprocess.TimeoutExpired) as exc:
                    checks.append({"name": name, "ok": False,
                                   "error": f"抽帧失败: {exc}"})
                    continue
                if result.returncode != 0 or not output.is_file():
                    checks.append({
                        "name": name, "ok": False,
                        "error": (result.stderr or "").strip()[:300] or "无输出",
                    })
                    continue
                try:
                    from PIL import Image, ImageStat

                    with Image.open(output) as image:
                        width, height = image.size
                        stat = ImageStat.Stat(image.convert("L"))
                        stddev = float(stat.stddev[0]) if stat.stddev else 0.0
                except (OSError, ValueError) as exc:
                    checks.append({"name": name, "ok": False,
                                   "error": f"帧不可解码: {exc}"})
                    continue
                ok = (width == metadata.width and height == metadata.height
                      and stddev > 1.0)
                checks.append({
                    "name": name, "ok": ok, "width": width, "height": height,
                    "luma_stddev": round(stddev, 3),
                    "error": "" if ok else "尺寸不符或整帧近乎纯色",
                })
        return checks


def fingerprint(recording_id: str, *, speed: int, config: ReplayConfig,
                width: int, height: int, fps: int) -> str:
    """Stable id for (recording, render config); repeated jobs reuse output."""
    import hashlib

    payload = json_safe({
        "encoder": ENCODER_VERSION,
        "renderer": RENDERER_VERSION,
        "recording_id": recording_id,
        "speed": speed,
        "width": width,
        "height": height,
        "fps": fps,
        "crf": config.encoder_crf,
        "preset": config.encoder_preset,
        "key_speed": config.key_speed,
        "key_pad_seconds": config.key_pad_seconds,
        "end_hold_seconds": config.end_hold_seconds,
        "max_jump_m": config.max_pose_jump_m,
        "max_jump_window_s": config.max_pose_jump_window_s,
        "max_odom_speed_mps": config.max_odom_speed_mps,
    })
    encoded = json.dumps(payload, sort_keys=True, ensure_ascii=False,
                         separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()[:24]
