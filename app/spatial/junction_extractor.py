"""Lightweight junction observation candidates from an observed occupancy grid."""

from __future__ import annotations

import math

from app.spatial.models import FrontierCandidate, SpatialMapSnapshot, SpatialPose


def _cell(snapshot: SpatialMapSnapshot, x: float, y: float) -> tuple[int, int] | None:
    column = math.floor((x - snapshot.origin[0]) / snapshot.resolution_m)
    row = math.floor((y - snapshot.origin[1]) / snapshot.resolution_m)
    if 0 <= column < snapshot.width and 0 <= row < snapshot.height:
        return int(column), int(row)
    return None


def _known_free(
    snapshot: SpatialMapSnapshot,
    free: set[tuple[int, int]],
    x: float,
    y: float,
    *,
    clearance_m: float,
) -> bool:
    center = _cell(snapshot, x, y)
    if center is None or center not in free:
        return False
    radius = max(0, int(math.ceil(clearance_m / snapshot.resolution_m)))
    for row in range(center[1] - radius, center[1] + radius + 1):
        for column in range(center[0] - radius, center[0] + radius + 1):
            if (column - center[0]) ** 2 + (row - center[1]) ** 2 > radius ** 2:
                continue
            if (column, row) not in free:
                return False
    return True


def extract_junction_candidates(
    snapshot: SpatialMapSnapshot | None,
    robot_pose: SpatialPose | None,
    *,
    observation_distance_m: float = 1.8,
    clearance_m: float = 0.35,
    direction_step_deg: float = 15.0,
    min_direction_separation_deg: float = 45.0,
    max_candidates: int = 4,
) -> list[FrontierCandidate]:
    """Return free-space branch viewpoints when at least two directions exist.

    Rays and endpoints must stay in currently known free cells.  This is only
    candidate extraction; the route planner and Nav2 remain authoritative for
    reachability and motion safety.
    """
    if snapshot is None or robot_pose is None or snapshot.revision <= 0:
        return []
    map_frame = str(snapshot.provenance.get("frame_id") or robot_pose.frame_id)
    if map_frame != robot_pose.frame_id:
        return []
    free = set(snapshot.free)
    if not free:
        return []
    steps = max(8, int(round(360.0 / max(5.0, direction_step_deg))))
    open_directions: list[int] = []
    sample_step = max(snapshot.resolution_m, 0.05)
    samples = max(1, int(observation_distance_m / sample_step))
    for index in range(steps):
        angle = 2.0 * math.pi * index / steps
        clear = True
        for sample in range(1, samples + 1):
            distance = observation_distance_m * sample / samples
            cell = _cell(
                snapshot,
                robot_pose.x + distance * math.cos(angle),
                robot_pose.y + distance * math.sin(angle),
            )
            if cell is None or cell not in free:
                clear = False
                break
        if clear:
            open_directions.append(index)
    if not open_directions:
        return []

    open_set = set(open_directions)
    groups: list[list[int]] = []
    pending = set(open_directions)
    while pending:
        start = min(pending)
        group = [start]
        pending.remove(start)
        cursor = (start + 1) % steps
        while cursor in pending:
            group.append(cursor)
            pending.remove(cursor)
            cursor = (cursor + 1) % steps
        groups.append(group)
    if len(groups) > 1 and 0 in open_set and (steps - 1) in open_set:
        first = next(group for group in groups if 0 in group)
        last = next(group for group in groups if steps - 1 in group)
        if first is not last:
            merged = last + first
            groups = [group for group in groups if group not in (first, last)]
            groups.append(merged)

    centers: list[float] = []
    for group in groups:
        sine = sum(math.sin(2.0 * math.pi * item / steps) for item in group)
        cosine = sum(math.cos(2.0 * math.pi * item / steps) for item in group)
        centers.append(math.degrees(math.atan2(sine, cosine)))
    separated: list[tuple[list[int], float]] = []
    for group, center in sorted(zip(groups, centers), key=lambda item: -len(item[0])):
        if all(
            abs((center - accepted + 180.0) % 360.0 - 180.0)
            >= min_direction_separation_deg
            for _, accepted in separated
        ):
            separated.append((group, center))
    if len(separated) < 2:
        return []
    if len(separated) == 2:
        separation = abs(
            (separated[0][1] - separated[1][1] + 180.0) % 360.0 - 180.0
        )
        # Two nearly opposite openings are a corridor, not a branch.
        if separation >= 150.0:
            return []

    candidates: list[FrontierCandidate] = []
    for group, bearing in separated[: max(2, int(max_candidates))]:
        angle = math.radians(bearing)
        position = (
            robot_pose.x + observation_distance_m * math.cos(angle),
            robot_pose.y + observation_distance_m * math.sin(angle),
        )
        if not _known_free(
            snapshot, free, *position, clearance_m=clearance_m
        ):
            continue
        qx = round(position[0] / 0.25)
        qy = round(position[1] / 0.25)
        width_score = min(1.0, len(group) / max(1.0, steps / 4.0))
        candidates.append(
            FrontierCandidate(
                frontier_id=f"junction_{qx}_{qy}",
                position=position,
                frame=map_frame,
                bearing_deg=round(bearing, 2),
                distance_m=round(observation_distance_m, 3),
                size_score=round(width_score, 4),
                spatial_information_gain=round(0.6 + 0.4 * width_score, 4),
                reachable=True,
                provenance={
                    "source": "junction",
                    "open_direction_count": len(separated),
                    "direction_width_deg": round(len(group) * 360.0 / steps, 2),
                    "known_free_endpoint": True,
                    "clearance_m": clearance_m,
                    "reachability_checked": False,
                },
            )
        )
    return candidates if len(candidates) >= 2 else []
