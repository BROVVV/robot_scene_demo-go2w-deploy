"""RGB-D source contracts and atomic frame data structures.

This module is the workstation-side boundary between the D435 HTTP service (or
a future ROS bridge) and the perception/spatial stack.  It intentionally has no
ROS dependency and no RealSense SDK dependency so the same interface can be
implemented by HTTP, ROS2 bridge, replay files or a mock.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Protocol


@dataclass
class RGBDFrame:
    """One atomically-synchronized color+depth frame.

    ``color_ref`` and ``depth_ref`` are local file paths or HTTP URLs.  When a
    source returns URLs, callers should materialize them with
    :meth:`RGBDSource.materialize` before handing them to OpenCV-based
    perception modules.
    """

    frame_id: str
    timestamp: float
    color_ref: str
    depth_ref: str

    width: int
    height: int

    fx: float
    fy: float
    cx: float
    cy: float

    depth_unit_m: float = 0.001
    depth_aligned_to_color: bool = True

    device_timestamp_ms: float | None = None
    host_timestamp: float | None = None
    # 计划书 §11.1：host_timestamp | receive_time
    timestamp_quality: str = "host_timestamp"

    health: dict[str, Any] = field(default_factory=dict)
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "RGBDFrame":
        value = dict(value or {})
        return cls(
            frame_id=str(value.get("frame_id") or ""),
            timestamp=float(value.get("timestamp", 0.0)),
            color_ref=str(value.get("color_ref") or ""),
            depth_ref=str(value.get("depth_ref") or ""),
            width=int(value.get("width", 0)),
            height=int(value.get("height", 0)),
            fx=float(value.get("fx", 0.0)),
            fy=float(value.get("fy", 0.0)),
            cx=float(value.get("cx", 0.0)),
            cy=float(value.get("cy", 0.0)),
            depth_unit_m=float(value.get("depth_unit_m", 0.001)),
            depth_aligned_to_color=bool(value.get("depth_aligned_to_color", True)),
            device_timestamp_ms=value.get("device_timestamp_ms"),
            host_timestamp=value.get("host_timestamp"),
            timestamp_quality=str(value.get("timestamp_quality") or "host_timestamp"),
            health=dict(value.get("health") or {}),
            provenance=dict(value.get("provenance") or {}),
        )


@dataclass
class RGBDFrameBundle:
    """FrameBundle V2: an atomic RGB-D frame plus perception/spatial metadata.

    This is the successor of the old RGB-only FrameBundle.  Old replay bundles
    remain compatible by leaving ``depth_ref``/``rgbd_frame_id``/``intrinsics``
    empty and setting ``spatial_quality="RGB_ONLY"``.
    """

    bundle_id: str
    timestamp: float
    rgbd_frame: RGBDFrame | None = None
    image_ref: str | None = None
    depth_ref: str | None = None
    rgbd_frame_id: str | None = None
    intrinsics: dict[str, Any] | None = None
    depth_scale: float | None = None
    depth_aligned_to_rgb: bool = True
    spatial_quality: str = "RGB_ONLY"
    camera_xyz: list[float] | None = None
    map_xyz: list[float] | None = None
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "bundle_id": self.bundle_id,
            "timestamp": self.timestamp,
            "rgbd_frame": self.rgbd_frame.to_dict() if self.rgbd_frame else None,
            "image_ref": self.image_ref,
            "depth_ref": self.depth_ref,
            "rgbd_frame_id": self.rgbd_frame_id,
            "intrinsics": self.intrinsics,
            "depth_scale": self.depth_scale,
            "depth_aligned_to_rgb": self.depth_aligned_to_rgb,
            "spatial_quality": self.spatial_quality,
            "camera_xyz": self.camera_xyz,
            "map_xyz": self.map_xyz,
            "provenance": self.provenance,
        }


class RGBDSource(Protocol):
    """Common RGB-D source interface used by the spatial exploration stack."""

    def get_latest(self, timeout_seconds: float = 0.0) -> RGBDFrame:
        """Return the latest atomic RGB-D frame.

        Raises a source-specific exception (e.g. :class:`RGBDFrameUnavailable`)
        when no valid frame can be produced within the timeout.
        """
        ...

    def materialize(self, frame: RGBDFrame) -> RGBDFrame:
        """Return a frame whose ``color_ref``/``depth_ref`` are local files.

        Implementations that already return local paths may return the frame
        unchanged.
        """
        ...

    def health(self) -> dict[str, Any]:
        """Return a JSON-safe health snapshot of the source."""
        ...


class RGBDFrameUnavailable(RuntimeError):
    """No atomic RGB-D frame is currently available."""
