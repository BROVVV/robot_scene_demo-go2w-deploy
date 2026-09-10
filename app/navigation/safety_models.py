"""Local safety context consumed by the planner (plan sections 4, 5, 26).

This module never touches raw point clouds.  It consumes the already fused
per-bearing dual-LiDAR evidence produced by
``go2w_lidar_preprocessor.lidar_evidence`` and turns it into the structured
snapshot the viewpoint planner, stuck detector, recovery manager and the LLM
context builder all read.

The seven evidence states are kept 1:1 with ``EvidenceState`` (uppercase here
because these are planner-facing constants).  ``UNKNOWN`` is never treated as
free space.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any, Iterable

STATE_CLEAR = "CLEAR"
STATE_OCCUPIED = "OCCUPIED"
STATE_UNKNOWN = "UNKNOWN"
STATE_SELF_OCCLUDED = "SELF_OCCLUDED"
STATE_SENSOR_BLIND = "SENSOR_BLIND"
STATE_STALE = "STALE"
STATE_UNVALIDATED_GEOMETRY = "UNVALIDATED_GEOMETRY"

SAFETY_STATES = (
    STATE_CLEAR,
    STATE_OCCUPIED,
    STATE_UNKNOWN,
    STATE_SELF_OCCLUDED,
    STATE_SENSOR_BLIND,
    STATE_STALE,
    STATE_UNVALIDATED_GEOMETRY,
)

# Only these two states carry trustworthy range geometry.  Everything else is
# fail-closed: no clearance value may be inferred from it.
OBSERVABLE_STATES = (STATE_CLEAR, STATE_OCCUPIED)

SECTOR_COUNT = 24
SECTOR_WIDTH_DEG = 360.0 / SECTOR_COUNT

FRONT_HALF_ANGLE_DEG = 45.0
SWEPT_HALF_ANGLE_DEG = 20.0


def normalize_state(value: Any) -> str:
    """Accept ``EvidenceState``, its lowercase value, or a planner constant."""
    text = str(getattr(value, "value", value) or "").upper()
    return text if text in SAFETY_STATES else STATE_UNKNOWN


def wrap_deg(bearing_deg: float) -> float:
    return (float(bearing_deg) + 180.0) % 360.0 - 180.0


def sector_index(bearing_deg: float) -> int:
    return int(round(wrap_deg(bearing_deg) / SECTOR_WIDTH_DEG)) % SECTOR_COUNT


def sector_bearing(index: int) -> float:
    return wrap_deg((index % SECTOR_COUNT) * SECTOR_WIDTH_DEG)


@dataclass(frozen=True)
class SafetySector:
    bearing_deg: float
    clearance_m: float | None
    state: str = STATE_UNKNOWN
    observable: bool = False
    traversable: bool = False
    confidence: float = 0.0
    source: str = "dual_lidar"

    def to_dict(self) -> dict[str, Any]:
        return {
            "bearing_deg": round(self.bearing_deg, 2),
            "clearance_m": (
                None if self.clearance_m is None else round(self.clearance_m, 3)
            ),
            "state": self.state,
            "observable": self.observable,
            "traversable": self.traversable,
            "confidence": round(self.confidence, 3),
            "source": self.source,
        }


def make_safety_sector(
    *,
    bearing_deg: float,
    clearance_m: float | None,
    state: Any,
    hard_clearance_m: float = 0.40,
    confidence: float = 0.0,
    source: str = "dual_lidar",
) -> SafetySector:
    resolved = normalize_state(state)
    observable = resolved in OBSERVABLE_STATES and clearance_m is not None
    traversable = observable and float(clearance_m) >= hard_clearance_m
    return SafetySector(
        bearing_deg=wrap_deg(bearing_deg),
        clearance_m=None if not observable else float(clearance_m),
        state=resolved,
        observable=observable,
        traversable=traversable,
        confidence=float(confidence),
        source=str(source),
    )


@dataclass(frozen=True)
class LocalSafetySnapshot:
    timestamp: float = 0.0
    front_clearance_m: float | None = None
    rear_clearance_m: float | None = None
    left_clearance_m: float | None = None
    right_clearance_m: float | None = None
    sectors: tuple[SafetySector, ...] = ()
    forward_safe: bool = False
    backward_safe: bool = False
    safe_turn_left_deg: float = 0.0
    safe_turn_right_deg: float = 0.0
    forward_swept_clearance_m: float | None = None
    backward_swept_clearance_m: float | None = None
    turn_left_swept_clearance_m: float | None = None
    turn_right_swept_clearance_m: float | None = None
    nearest_free_sector_deg: float | None = None
    lidar_fresh: bool = False
    geometry_validated: bool = False
    pose_quality: str = "unavailable"
    front_state: str = STATE_UNKNOWN
    rear_state: str = STATE_UNKNOWN
    source_health: dict[str, Any] = field(default_factory=dict)

    def sector_at(self, bearing_deg: float) -> SafetySector:
        index = sector_index(bearing_deg)
        for sector in self.sectors:
            if sector_index(sector.bearing_deg) == index:
                return sector
        return SafetySector(bearing_deg=sector_bearing(index), clearance_m=None)

    def clearance_at(self, bearing_deg: float) -> float | None:
        return self.sector_at(bearing_deg).clearance_m

    def traversable_at(self, bearing_deg: float) -> bool:
        return self.sector_at(bearing_deg).traversable

    def min_clearance(self, center_deg: float, half_angle_deg: float) -> float | None:
        return _arc_clearance(self.sectors, center_deg, half_angle_deg)

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "front_clearance_m": _round_opt(self.front_clearance_m),
            "rear_clearance_m": _round_opt(self.rear_clearance_m),
            "left_clearance_m": _round_opt(self.left_clearance_m),
            "right_clearance_m": _round_opt(self.right_clearance_m),
            "sectors": [item.to_dict() for item in self.sectors],
            "forward_safe": self.forward_safe,
            "backward_safe": self.backward_safe,
            "safe_turn_left_deg": round(self.safe_turn_left_deg, 1),
            "safe_turn_right_deg": round(self.safe_turn_right_deg, 1),
            "forward_swept_clearance_m": _round_opt(self.forward_swept_clearance_m),
            "backward_swept_clearance_m": _round_opt(self.backward_swept_clearance_m),
            "turn_left_swept_clearance_m": _round_opt(self.turn_left_swept_clearance_m),
            "turn_right_swept_clearance_m": _round_opt(
                self.turn_right_swept_clearance_m
            ),
            "nearest_free_sector_deg": _round_opt(self.nearest_free_sector_deg, 1),
            "lidar_fresh": self.lidar_fresh,
            "geometry_validated": self.geometry_validated,
            "pose_quality": self.pose_quality,
            "front_state": self.front_state,
            "rear_state": self.rear_state,
            "source_health": dict(self.source_health),
        }

    def summary(self) -> dict[str, Any]:
        """Compact form for decision records / events (no 24-sector dump)."""
        return {
            "front_clearance_m": _round_opt(self.front_clearance_m),
            "rear_clearance_m": _round_opt(self.rear_clearance_m),
            "left_clearance_m": _round_opt(self.left_clearance_m),
            "right_clearance_m": _round_opt(self.right_clearance_m),
            "forward_safe": self.forward_safe,
            "backward_safe": self.backward_safe,
            "front_state": self.front_state,
            "rear_state": self.rear_state,
            "nearest_free_sector_deg": _round_opt(self.nearest_free_sector_deg, 1),
            "safe_turn_left_deg": round(self.safe_turn_left_deg, 1),
            "safe_turn_right_deg": round(self.safe_turn_right_deg, 1),
            "lidar_fresh": self.lidar_fresh,
            "geometry_validated": self.geometry_validated,
            "pose_quality": self.pose_quality,
        }


def build_local_safety_snapshot(
    *,
    sector_evidence: Iterable[Any],
    timestamp: float = 0.0,
    hard_clearance_m: float = 0.40,
    rotation_clearance_m: float = 0.40,
    lidar_fresh: bool = False,
    geometry_validated: bool = False,
    pose_quality: str = "unavailable",
    source_health: dict[str, Any] | None = None,
) -> LocalSafetySnapshot:
    """Fold fused per-bearing evidence into the 24-sector planner snapshot.

    Sectors with no evidence stay ``UNKNOWN``; a sector that receives several
    observations keeps the most conservative one.
    """
    observed: dict[int, SafetySector] = {}
    for item in sector_evidence:
        sector = _coerce_sector(item, hard_clearance_m)
        index = sector_index(sector.bearing_deg)
        sector = replace(sector, bearing_deg=sector_bearing(index))
        existing = observed.get(index)
        observed[index] = (
            sector if existing is None else _more_conservative(existing, sector)
        )

    # Sectors keep index order (bearing 0, 15, ... 165, -180, ... -15) so the
    # rotation span walk below can index them directly.  Unobserved sectors
    # stay UNKNOWN.
    sectors = tuple(
        observed[index]
        if index in observed
        else SafetySector(bearing_deg=sector_bearing(index), clearance_m=None)
        for index in range(SECTOR_COUNT)
    )
    front = _arc_sectors(sectors, 0.0, FRONT_HALF_ANGLE_DEG)
    rear = _arc_sectors(sectors, 180.0, FRONT_HALF_ANGLE_DEG)
    left = _arc_sectors(sectors, 90.0, FRONT_HALF_ANGLE_DEG)
    right = _arc_sectors(sectors, -90.0, FRONT_HALF_ANGLE_DEG)

    forward_swept = _arc_clearance(sectors, 0.0, SWEPT_HALF_ANGLE_DEG)
    backward_swept = _arc_clearance(sectors, 180.0, SWEPT_HALF_ANGLE_DEG)
    turn_left_swept = _arc_clearance(sectors, 90.0, SWEPT_HALF_ANGLE_DEG)
    turn_right_swept = _arc_clearance(sectors, -90.0, SWEPT_HALF_ANGLE_DEG)

    geometry_ok = bool(lidar_fresh and geometry_validated)
    return LocalSafetySnapshot(
        timestamp=float(timestamp),
        front_clearance_m=_min_clearance(front),
        rear_clearance_m=_min_clearance(rear),
        left_clearance_m=_min_clearance(left),
        right_clearance_m=_min_clearance(right),
        sectors=sectors,
        forward_safe=(
            geometry_ok
            and forward_swept is not None
            and forward_swept >= hard_clearance_m
        ),
        backward_safe=(
            geometry_ok
            and backward_swept is not None
            and backward_swept >= hard_clearance_m
        ),
        safe_turn_left_deg=_safe_turn_span(sectors, 1, rotation_clearance_m),
        safe_turn_right_deg=_safe_turn_span(sectors, -1, rotation_clearance_m),
        forward_swept_clearance_m=forward_swept,
        backward_swept_clearance_m=backward_swept,
        turn_left_swept_clearance_m=turn_left_swept,
        turn_right_swept_clearance_m=turn_right_swept,
        nearest_free_sector_deg=_nearest_free_sector(sectors),
        lidar_fresh=bool(lidar_fresh),
        geometry_validated=bool(geometry_validated),
        pose_quality=str(pose_quality),
        front_state=_arc_state(front),
        rear_state=_arc_state(rear),
        source_health=dict(source_health or {}),
    )


# The live bridge publishes one *minimum* range per fan
# (``front_m``/``left_m``/``right_m`` in ``robot_scene_live_bridge``), so the
# same value is replayed onto every bearing inside that fan.  That is
# conservative, and bearings nobody measured (the whole rear arc) simply stay
# UNKNOWN instead of being invented.
CLEARANCE_FRONT_HALF_ANGLE_DEG = 30.0
CLEARANCE_SIDE_HALF_ANGLE_DEG = 15.0
CLEARANCE_DIRECTIONS = (("front", 0.0), ("left", 90.0), ("right", -90.0))


def clearance_sector_evidence(
    clearance: dict[str, Any],
    *,
    hard_clearance_m: float = 0.40,
    no_return_clearance_m: float = 3.0,
    front_half_angle_deg: float = CLEARANCE_FRONT_HALF_ANGLE_DEG,
    side_half_angle_deg: float = CLEARANCE_SIDE_HALF_ANGLE_DEG,
) -> list[dict[str, Any]]:
    """Turn a frame-bundle ``clearance`` section into per-bearing evidence.

    ``front_status``/``left_status``/``right_status`` use the bridge vocabulary:
    ``measured`` carries a real range, ``no_return`` means the whole fan came
    back empty, and ``unknown`` means the reading may not be trusted at all.
    """
    evidence: list[dict[str, Any]] = []
    for name, center_deg in CLEARANCE_DIRECTIONS:
        status = str(clearance.get(f"{name}_status") or "unknown")
        value = clearance.get(f"{name}_m")
        if status == "no_return":
            distance: float | None = float(no_return_clearance_m)
        elif status == "measured" and value is not None:
            distance = float(value)
        else:
            continue
        state = STATE_OCCUPIED if distance < hard_clearance_m else STATE_CLEAR
        half = front_half_angle_deg if name == "front" else side_half_angle_deg
        steps = int(half // SECTOR_WIDTH_DEG)
        for step in range(-steps, steps + 1):
            evidence.append(
                {
                    "bearing_deg": center_deg + step * SECTOR_WIDTH_DEG,
                    "clearance_m": distance,
                    "state": state,
                    "confidence": 1.0 if status == "measured" else 0.6,
                    "source": f"bridge_clearance_{name}",
                }
            )
    return evidence


def local_safety_from_clearance(
    clearance: dict[str, Any],
    *,
    timestamp: float = 0.0,
    hard_clearance_m: float = 0.40,
    rotation_clearance_m: float = 0.40,
    no_return_clearance_m: float = 3.0,
    geometry_validated: bool = False,
    pose_quality: str = "unavailable",
    source_health: dict[str, Any] | None = None,
) -> LocalSafetySnapshot:
    """Snapshot built straight from a live frame bundle's clearance section."""
    return build_local_safety_snapshot(
        sector_evidence=clearance_sector_evidence(
            clearance,
            hard_clearance_m=hard_clearance_m,
            no_return_clearance_m=no_return_clearance_m,
        ),
        timestamp=timestamp,
        hard_clearance_m=hard_clearance_m,
        rotation_clearance_m=rotation_clearance_m,
        lidar_fresh=bool(clearance.get("lidar_fresh", False)),
        geometry_validated=bool(geometry_validated),
        pose_quality=pose_quality,
        source_health=source_health,
    )


@dataclass(frozen=True)
class PerceptionQuality:
    depth_valid_ratio: float = 0.0
    near_field_ratio: float = 0.0
    visual_feature_count: int = 0
    stable_object_count: int = 0
    estimated_information_gain: float = 0.0
    camera_blocked: bool = False
    too_close_to_surface: bool = False
    low_information: bool = False
    target_visible: bool = False
    target_depth_valid: bool = False
    reason: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "depth_valid_ratio": round(self.depth_valid_ratio, 3),
            "near_field_ratio": round(self.near_field_ratio, 3),
            "visual_feature_count": self.visual_feature_count,
            "stable_object_count": self.stable_object_count,
            "estimated_information_gain": round(self.estimated_information_gain, 3),
            "camera_blocked": self.camera_blocked,
            "too_close_to_surface": self.too_close_to_surface,
            "low_information": self.low_information,
            "target_visible": self.target_visible,
            "target_depth_valid": self.target_depth_valid,
            "reason": self.reason,
        }


def evaluate_perception_quality(
    *,
    depth_valid_ratio: float = 0.0,
    near_field_ratio: float = 0.0,
    visual_feature_count: int = 0,
    stable_object_count: int = 0,
    estimated_information_gain: float = 0.0,
    front_clearance_m: float | None = None,
    target_visible: bool = False,
    target_depth_valid: bool = False,
    camera_blocked_front_m: float = 0.45,
    near_field_ratio_threshold: float = 0.55,
    low_information_threshold: float = 0.20,
    near_field_threshold_m: float = 0.50,
) -> PerceptionQuality:
    """Plan section 5: "the camera sees a wall" is a first-class state.

    ``front_clearance_m is None`` (unknown geometry) counts as too close, so an
    unobservable front cannot be mistaken for a rich viewpoint.
    """
    too_close = front_clearance_m is None or front_clearance_m < near_field_threshold_m
    low_information = estimated_information_gain <= low_information_threshold
    blocked_front = (
        front_clearance_m is None or front_clearance_m < camera_blocked_front_m
    )
    camera_blocked = bool(
        blocked_front
        and near_field_ratio >= near_field_ratio_threshold
        and low_information
    )
    if camera_blocked:
        reason = (
            "CAMERA_BLOCKED_BY_NEAR_SURFACE: front "
            f"{_fmt(front_clearance_m)} < {camera_blocked_front_m:.2f}m, near-field "
            f"ratio {near_field_ratio:.2f}, information gain "
            f"{estimated_information_gain:.2f}"
        )
    elif too_close:
        reason = f"TOO_CLOSE_TO_SURFACE: front {_fmt(front_clearance_m)}"
    elif low_information:
        reason = f"LOW_INFORMATION: gain {estimated_information_gain:.2f}"
    else:
        reason = ""
    return PerceptionQuality(
        depth_valid_ratio=float(depth_valid_ratio),
        near_field_ratio=float(near_field_ratio),
        visual_feature_count=int(visual_feature_count),
        stable_object_count=int(stable_object_count),
        estimated_information_gain=float(estimated_information_gain),
        camera_blocked=camera_blocked,
        too_close_to_surface=bool(too_close),
        low_information=bool(low_information),
        target_visible=bool(target_visible),
        target_depth_valid=bool(target_depth_valid),
        reason=reason,
    )


# ---- helpers --------------------------------------------------------------


def _fmt(value: float | None) -> str:
    return "unknown" if value is None else f"{value:.2f}m"


def _round_opt(value: float | None, digits: int = 3) -> float | None:
    return None if value is None else round(float(value), digits)


def _coerce_sector(item: Any, hard_clearance_m: float) -> SafetySector:
    if isinstance(item, SafetySector):
        return item
    if isinstance(item, dict):
        bearing = item.get("bearing_deg")
        if bearing is None:
            bearing = item.get("bearing_rad")
            bearing = 0.0 if bearing is None else float(bearing) * 57.29577951308232
        return make_safety_sector(
            bearing_deg=float(bearing),
            clearance_m=item.get("clearance_m"),
            state=item.get("state"),
            hard_clearance_m=hard_clearance_m,
            confidence=float(item.get("confidence") or 0.0),
            source=str(item.get("source") or "dual_lidar"),
        )
    bearing = getattr(item, "bearing_deg", None)
    if bearing is None:
        radians = getattr(item, "bearing_rad", None)
        bearing = 0.0 if radians is None else float(radians) * 57.29577951308232
    return make_safety_sector(
        bearing_deg=float(bearing),
        clearance_m=getattr(item, "clearance_m", None),
        state=getattr(item, "state", None),
        hard_clearance_m=hard_clearance_m,
        confidence=float(getattr(item, "confidence", 0.0) or 0.0),
        source=str(getattr(item, "source", "dual_lidar") or "dual_lidar"),
    )


def _state_risk(state: str) -> int:
    if state == STATE_OCCUPIED:
        return 3
    if state == STATE_CLEAR:
        return 0
    return 2


def _more_conservative(current: SafetySector, incoming: SafetySector) -> SafetySector:
    current_risk = _state_risk(current.state)
    incoming_risk = _state_risk(incoming.state)
    if incoming_risk != current_risk:
        return incoming if incoming_risk > current_risk else current
    if incoming.clearance_m is None:
        return incoming
    if current.clearance_m is None:
        return current
    return incoming if incoming.clearance_m < current.clearance_m else current


def _arc_sectors(
    sectors: tuple[SafetySector, ...], center_deg: float, half_angle_deg: float
) -> list[SafetySector]:
    limit = half_angle_deg + 1e-6
    return [
        sector
        for sector in sectors
        if abs(wrap_deg(sector.bearing_deg - center_deg)) <= limit
    ]


def _min_clearance(sectors: list[SafetySector]) -> float | None:
    if not sectors:
        return None
    values = []
    for sector in sectors:
        if not sector.observable:
            return None
        values.append(float(sector.clearance_m))
    return min(values)


def _arc_clearance(
    sectors: tuple[SafetySector, ...], center_deg: float, half_angle_deg: float
) -> float | None:
    return _min_clearance(_arc_sectors(sectors, center_deg, half_angle_deg))


def _arc_state(sectors: list[SafetySector]) -> str:
    if not sectors:
        return STATE_UNKNOWN
    for sector in sectors:
        if sector.state == STATE_OCCUPIED:
            return STATE_OCCUPIED
    for sector in sectors:
        if sector.state != STATE_CLEAR:
            return sector.state
    return STATE_CLEAR


def _safe_turn_span(
    sectors: tuple[SafetySector, ...], direction: int, rotation_clearance_m: float
) -> float:
    span = 0.0
    for step in range(1, SECTOR_COUNT // 2 + 1):
        sector = sectors[(direction * step) % SECTOR_COUNT]
        if not sector.observable or float(sector.clearance_m) < rotation_clearance_m:
            break
        span = step * SECTOR_WIDTH_DEG
    return span


def _nearest_free_sector(sectors: tuple[SafetySector, ...]) -> float | None:
    free = [sector for sector in sectors if sector.traversable]
    if not free:
        return None
    return min(free, key=lambda item: abs(item.bearing_deg)).bearing_deg
