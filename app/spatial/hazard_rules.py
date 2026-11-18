"""配置驱动的实验危险匹配与同一轨迹事件去重。"""
from __future__ import annotations

import math
from typing import Any

from app.spatial.object_mobility import labels_of


class HazardMonitor:
    def __init__(self, config: dict[str, Any] | None = None) -> None:
        self.config = config or {}
        self.events: dict[tuple[str, str], dict[str, Any]] = {}
        self._next_event_index = 1

    def reset(self) -> None:
        self.events.clear()
        self._next_event_index = 1

    def begin_frame(self, *, frame_id: str, timestamp: float, evaluated: bool = True) -> None:
        """Register one unique source frame, including valid non-hits.

        Detection branches from the same image share ``frame_id`` and therefore
        can never masquerade as multiple temporal confirmations.
        """
        if not evaluated:
            return
        for event in self.events.values():
            evidence = event.setdefault("recent_evidence", [])
            if not any(row.get("frame_id") == frame_id for row in evidence):
                evidence.append({"frame_id": frame_id, "timestamp": float(timestamp), "support": False})
            self._trim_evidence(event, timestamp)
            self._refresh_status(event, timestamp)

    def _new_event_id(self) -> str:
        """Return a session-unique ID even after duplicate events are merged.

        ``len(events) + 1`` reused identifiers whenever persistent-object
        binding collapsed two short-track events.  Reuse made evidence URLs
        ambiguous and could overwrite an earlier hazard image.  A monotonic
        counter is independent of the current dictionary size.
        """
        used = {
            str(event.get("hazard_event_id") or "")
            for event in self.events.values()
        }
        while True:
            event_id = f"hazard_{self._next_event_index:04d}"
            self._next_event_index += 1
            if event_id not in used:
                return event_id

    def bind_persistent(
        self, *, track_id: str, persistent_object_id: str
    ) -> dict[str, Any] | None:
        """Attach a track event to a persistent object and merge duplicates.

        Viewpoint changes can break a short term track before the persistent
        semantic map associates the new observation back to its existing
        object.  Hazard matching happens earlier by design, so a temporary new
        event may already exist.  Once the durable identity is known, collapse
        all events from the same rule and object into the earliest event.
        """
        current_items = [
            (key, event) for key, event in self.events.items()
            if event.get("track_id") == track_id
        ]
        survivor: dict[str, Any] | None = None
        for current_key, current in current_items:
            current["persistent_object_id"] = persistent_object_id
            matches = [
                (key, event) for key, event in self.events.items()
                if event.get("hazard_rule_id") == current.get("hazard_rule_id")
                and event.get("persistent_object_id") == persistent_object_id
                and _anchors_compatible(
                    event.get("host_local_anchor"), current.get("host_local_anchor"),
                    float(self.config.get("host_anchor_distance", 0.18)),
                )
            ]
            survivor_key, survivor = min(
                matches,
                key=lambda pair: (
                    float(pair[1].get("first_seen") or 0.0),
                    str(pair[1].get("hazard_event_id") or ""),
                ),
            )
            for other_key, other in matches:
                if other is survivor:
                    continue
                _merge_event(survivor, other)
                self.events.pop(other_key, None)
            # Follow the currently visible track with the surviving event so
            # subsequent frames hit the fast exact-key path.
            desired_key = (str(survivor["hazard_rule_id"]), track_id)
            if survivor_key != desired_key:
                self.events.pop(survivor_key, None)
                self.events.pop(current_key, None)
                self.events[desired_key] = survivor
            survivor["track_id"] = track_id
            survivor["persistent_object_id"] = persistent_object_id
        return survivor

    def bind_host(
        self, *, track_id: str, host_persistent_object_id: str
    ) -> dict[str, Any] | None:
        """Bind an anomaly to its pipe without collapsing separate pipe spots."""
        current_items = [
            (key, event) for key, event in self.events.items()
            if event.get("track_id") == track_id
        ]
        survivor: dict[str, Any] | None = None
        for current_key, current in current_items:
            current["host_object_id"] = host_persistent_object_id
            matches = [
                (key, event) for key, event in self.events.items()
                if event.get("hazard_rule_id") == current.get("hazard_rule_id")
                and event.get("host_object_id") == host_persistent_object_id
                and _anchors_compatible(
                    event.get("host_local_anchor"), current.get("host_local_anchor"),
                    float(self.config.get("host_anchor_distance", 0.18)),
                )
            ]
            _, survivor = min(
                matches,
                key=lambda pair: (
                    float(pair[1].get("first_seen") or 0.0),
                    str(pair[1].get("hazard_event_id") or ""),
                ),
            )
            for other_key, other in matches:
                if other is survivor:
                    continue
                _merge_event(survivor, other)
                self.events.pop(other_key, None)
            survivor["host_object_id"] = host_persistent_object_id
            self.events[current_key] = survivor
        return survivor

    def prompt_terms(self) -> list[str]:
        if not self.config.get("enabled", False):
            return []
        # Matching aliases and detector prompts are related, but they are not
        # interchangeable.  GroundingDINO is fed English prompts in this
        # project; pushing Chinese display aliases into it produced several
        # high confidence boxes on floor reflections during the first robot
        # run.  A rule may therefore keep multilingual aliases in ``labels``
        # while declaring the phrases that are safe for the detector in
        # ``prompt_labels``.  Existing configs without the new key retain the
        # old behaviour.
        return list(dict.fromkeys(
            str(label)
            for rule in self.config.get("rules", [])
            if rule.get("enabled", True)
            for label in rule.get("prompt_labels", rule.get("labels", []))
        ))

    def observe(self, item: Any, *, track_id: str, frame_id: str, timestamp: float,
                has_position: bool, location_mode: str | None = None,
                host_object_id: str | None = None,
                host_local_anchor: Any = None) -> tuple[dict[str, Any] | None, bool]:
        if not self.config.get("enabled", False):
            return None, False
        names = labels_of(item)
        for rule in self.config.get("rules", []):
            if not rule.get("enabled", True) or not names.intersection(
                    str(label).strip().casefold() for label in rule.get("labels", [])):
                continue
            color = rule.get("color")
            observed_color = item.get("color") if isinstance(item, dict) else getattr(item, "color", None)
            if color:
                evidence = item.get("color_evidence") if isinstance(item, dict) else getattr(item, "color_evidence", None)
                if evidence != "visual" or str(color).casefold() != str(observed_color or "").casefold():
                    continue
            rule_id = str(rule["id"])
            key = (rule_id, track_id)
            event = self.events.get(key)
            bbox = _value(item, "bbox_2d") or _value(item, "bbox")
            map_xyz = _value(item, "map_xyz")
            if event is None:
                rebound_key, event = self._rebind_candidate(
                    rule_id=rule_id,
                    frame_id=frame_id,
                    timestamp=timestamp,
                    bbox=bbox,
                    map_xyz=map_xyz,
                    host_object_id=host_object_id,
                    host_local_anchor=host_local_anchor,
                )
                if event is not None and rebound_key is not None:
                    self.events.pop(rebound_key, None)
                    self.events[key] = event
                    event["track_id"] = track_id
            is_new = event is None
            if event is None:
                event = {
                    "hazard_event_id": self._new_event_id(),
                    "hazard_rule_id": rule_id,
                    "hazard_label": str(rule.get("display_name") or rule_id),
                    "track_id": track_id,
                    "first_seen": timestamp,
                    "observation_count": 0,
                    "last_frame_id": None,
                    "last_sound_at": 0.0,
                    "recent_evidence": [],
                }
                self.events[key] = event
            if frame_id != event["last_frame_id"]:
                event["observation_count"] += 1
                event["last_frame_id"] = frame_id
            evidence = event.setdefault("recent_evidence", [])
            supports_confirmation = str(
                _value(item, "hazard_state") or "confirmed_demo_candidate"
            ) == "confirmed_demo_candidate"
            row = next((entry for entry in evidence if entry.get("frame_id") == frame_id), None)
            if row is None:
                row = {
                    "frame_id": frame_id,
                    "timestamp": float(timestamp),
                    "support": supports_confirmation,
                }
                evidence.append(row)
            else:
                row["support"] = bool(row.get("support")) or supports_confirmation
                row["timestamp"] = float(timestamp)
            event["last_seen"] = timestamp
            if bbox is not None:
                event["last_bbox_2d"] = [float(value) for value in bbox]
            if map_xyz is not None:
                event["last_map_xyz"] = [float(value) for value in map_xyz]
            if host_object_id:
                event["host_object_id"] = str(host_object_id)
            if host_local_anchor is not None:
                event["host_local_anchor"] = [float(value) for value in host_local_anchor]
            mode = str(location_mode or ("direct" if has_position else "unavailable"))
            event["location_mode"] = mode
            event["position_status"] = {
                "direct": "精确定位",
                "pipe_surface_estimate": "管道表面近似位置",
                "host_only": "仅关联宿主管道",
                "unavailable": "位置待确认",
            }.get(mode, "已定位" if has_position else "位置待确认")
            self._trim_evidence(event, timestamp)
            self._refresh_status(event, timestamp)
            return event, is_new
        return None, False

    def _rebind_candidate(
        self,
        *,
        rule_id: str,
        frame_id: str,
        timestamp: float,
        bbox: Any,
        map_xyz: Any,
        host_object_id: str | None,
        host_local_anchor: Any,
    ) -> tuple[tuple[str, str] | None, dict[str, Any] | None]:
        """Keep one event when SLAM correction resets the short term tracker.

        A map correction deliberately clears motion tracks so the coordinate
        jump cannot be mistaken for object motion.  The hazard identity must
        outlive that reset.  Rebind only to a recent event from the same rule
        with strong image overlap or a close 3-D position.  An event already
        used in this frame is excluded, so two boxes visible together remain
        two events.
        """
        window = float(self.config.get("track_rebind_window_s", 60.0))
        best: tuple[float, tuple[str, str], dict[str, Any]] | None = None
        for old_key, candidate in list(self.events.items()):
            if candidate.get("hazard_rule_id") != rule_id:
                continue
            if candidate.get("last_frame_id") == frame_id:
                continue
            age = timestamp - float(candidate.get("last_seen") or 0.0)
            if age < 0.0 or age > window:
                continue
            distance = _xyz_distance(map_xyz, candidate.get("last_map_xyz"))
            same_host = bool(
                host_object_id and candidate.get("host_object_id") == host_object_id
                and _anchors_compatible(
                    host_local_anchor, candidate.get("host_local_anchor"),
                    float(self.config.get("host_anchor_distance", 0.18)),
                )
            )
            map_limit = float(self.config.get("map_rebind_distance_m", 0.4))
            if not same_host and (distance is None or distance > map_limit):
                continue
            score = (1.0 if same_host else 0.0) + (0.5 if distance is not None else 0.0)
            if best is None or score > best[0]:
                best = (score, old_key, candidate)
        return (best[1], best[2]) if best is not None else (None, None)

    def _trim_evidence(self, event: dict[str, Any], now: float) -> None:
        limit = max(1, int(self.config.get("evidence_window_frames", 3)))
        window_s = max(0.1, float(self.config.get("evidence_window_s", 12.0)))
        rows = [
            row for row in event.get("recent_evidence", [])
            if 0.0 <= float(now) - float(row.get("timestamp") or 0.0) <= window_s
        ]
        event["recent_evidence"] = rows[-limit:]

    def _refresh_status(self, event: dict[str, Any], now: float) -> None:
        rows = event.get("recent_evidence", [])
        support = [row for row in rows if row.get("support")]
        event["window_observation_count"] = len(support)
        required = max(1, int(self.config.get("confirm_observations", 2)))
        within_span = (
            len(support) >= required
            and float(support[-1]["timestamp"]) - float(support[0]["timestamp"])
            <= float(self.config.get("evidence_window_s", 12.0))
        )
        if within_span:
            event["status"] = "confirmed"
            event["confirmed_at"] = float(now)
        elif event.get("confirmed_at") and float(now) - float(event.get("last_seen") or 0.0) > float(
            self.config.get("stale_review_after_s", 12.0)
        ):
            event["status"] = "review_pending"
        else:
            event["status"] = "suspected"


def _value(item: Any, name: str) -> Any:
    return item.get(name) if isinstance(item, dict) else getattr(item, name, None)


def _bbox_iou(left: Any, right: Any) -> float:
    try:
        lx1, ly1, lx2, ly2 = (float(value) for value in left)
        rx1, ry1, rx2, ry2 = (float(value) for value in right)
    except (TypeError, ValueError):
        return 0.0
    width = max(0.0, min(lx2, rx2) - max(lx1, rx1))
    height = max(0.0, min(ly2, ry2) - max(ly1, ry1))
    intersection = width * height
    union = max(0.0, lx2 - lx1) * max(0.0, ly2 - ly1)
    union += max(0.0, rx2 - rx1) * max(0.0, ry2 - ry1) - intersection
    return intersection / union if union > 0.0 else 0.0


def _xyz_distance(left: Any, right: Any) -> float | None:
    try:
        values = [(float(a), float(b)) for a, b in zip(left, right)]
    except (TypeError, ValueError):
        return None
    if len(values) < 2 or not all(math.isfinite(a) and math.isfinite(b) for a, b in values):
        return None
    return math.sqrt(sum((a - b) ** 2 for a, b in values[:3]))


def _anchors_compatible(left: Any, right: Any, limit: float) -> bool:
    if left is None or right is None:
        # Unknown anchors may be merged only while identity is still weak; a
        # known and unknown location must not swallow another visible spot.
        return left is None and right is None
    try:
        values = [(float(a), float(b)) for a, b in zip(left, right)]
    except (TypeError, ValueError):
        return False
    if len(values) < 2:
        return False
    return math.hypot(values[0][0] - values[0][1], values[1][0] - values[1][1]) <= limit


def _merge_event(target: dict[str, Any], source: dict[str, Any]) -> None:
    if float(source.get("last_seen") or 0.0) >= float(target.get("last_seen") or 0.0):
        for name in (
            "last_seen", "last_frame_id", "last_bbox_2d", "last_map_xyz",
            "position_status", "track_id",
        ):
            if name in source:
                target[name] = source[name]
    target["first_seen"] = min(
        float(target.get("first_seen") or 0.0),
        float(source.get("first_seen") or 0.0),
    )
    # Both events can contain overlapping frame evidence.  Taking the maximum
    # preserves confirmation without double-counting the same observations.
    target["observation_count"] = max(
        int(target.get("observation_count") or 0),
        int(source.get("observation_count") or 0),
    )
    target["last_announced_count"] = max(
        int(target.get("last_announced_count") or 0),
        int(source.get("last_announced_count") or 0),
    )
    target["last_sound_at"] = max(
        float(target.get("last_sound_at") or 0.0),
        float(source.get("last_sound_at") or 0.0),
    )
    if source.get("status") == "confirmed":
        target["status"] = "confirmed"
    evidence_by_frame = {
        str(row.get("frame_id")): dict(row)
        for row in [*target.get("recent_evidence", []), *source.get("recent_evidence", [])]
        if row.get("frame_id")
    }
    target["recent_evidence"] = sorted(
        evidence_by_frame.values(), key=lambda row: float(row.get("timestamp") or 0.0)
    )
    if not target.get("evidence_path") and source.get("evidence_path"):
        target["evidence_path"] = source["evidence_path"]
