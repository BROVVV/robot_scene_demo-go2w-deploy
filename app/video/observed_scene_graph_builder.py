"""Build the observed navigation scene graph from video observations."""

from __future__ import annotations

from collections import defaultdict

from app.video.navigation_topology_schema import PlaceSegment
from app.video.place_segmenter import PlaceSegmenter, assign_tracks_to_places
from app.video.schemas import (
    FrameObservation,
    ObjectTrack,
    SceneGraph,
    SceneGraphEdge,
    SceneGraphNode,
)


class ObservedSceneGraphBuilder:
    """Convert visually observed video state into a factual place graph."""

    def build(
        self,
        frame_observations: list[FrameObservation],
        object_tracks: list[ObjectTrack],
        place_segments: list[PlaceSegment] | None = None,
    ) -> SceneGraph:
        places = place_segments or PlaceSegmenter().segment(frame_observations, object_tracks)
        assignments = assign_tracks_to_places(places, object_tracks)
        primary_place = {
            track_id: place_ids[0]
            for track_id, place_ids in assignments.items()
            if place_ids
        }
        nodes: list[SceneGraphNode] = []
        edges: list[SceneGraphEdge] = []

        for place in places:
            nodes.append(_place_node(place))

        for left, right in zip(places, places[1:]):
            edges.append(
                SceneGraphEdge(
                    edge_id=f"edge_temporal_{len(edges) + 1:04d}",
                    source_node_id=left.place_id,
                    target_node_id=right.place_id,
                    relation="temporal_next",
                    source="observed",
                    confidence=0.9,
                    evidence_level="observed_confirmed",
                    reason="adjacent place segments in robot observation order",
                )
            )

        for track in object_tracks:
            node = _track_node(track, assignments.get(track.track_id, []), primary_place.get(track.track_id))
            nodes.append(node)
            for place_id in assignments.get(track.track_id, []):
                edges.append(
                    SceneGraphEdge(
                        edge_id=f"edge_contains_{len(edges) + 1:04d}",
                        source_node_id=place_id,
                        target_node_id=track.track_id,
                        relation="contains",
                        source="observed",
                        confidence=track.confidence,
                        evidence_level="observed_confirmed",
                        reason=f"{track.label} observed in {place_id}",
                    )
                )

        _add_place_free_space_nodes(nodes, edges, places, frame_observations)
        _add_passage_connections(edges, places, object_tracks, assignments)
        _add_frame_relations(edges, frame_observations, object_tracks)
        return SceneGraph(nodes=nodes, edges=_dedupe_edges(edges))


def _place_node(place: PlaceSegment) -> SceneGraphNode:
    label = place.scene_type or "unknown"
    return SceneGraphNode(
        node_id=place.place_id,
        node_type="place",
        label=label,
        label_zh=_scene_label_zh(label),
        category="place",
        source="observed",
        confidence=place.confidence,
        evidence_level="observed_confirmed",
        based_on=[f"frame_{frame_id:06d}" for frame_id in place.evidence_frames],
        can_confirm_target=True,
        attributes={
            "start_time": place.start_time,
            "end_time": place.end_time,
            "start_frame": place.start_frame,
            "end_frame": place.end_frame,
            "dominant_labels": place.dominant_labels,
            "passage_candidates": place.passage_candidates,
            "free_space_hint": place.free_space_hint,
            "split_reason": place.split_reason,
            "object_track_ids": place.object_track_ids,
            "navigation_role": "place",
        },
    )


def _track_node(
    track: ObjectTrack,
    place_ids: list[str],
    primary_place_id: str | None,
) -> SceneGraphNode:
    node_type = _node_type_for_track(track)
    return SceneGraphNode(
        node_id=track.track_id,
        node_type=node_type,
        label=track.label,
        label_zh=track.label_zh,
        category=track.category,
        source="observed",
        confidence=track.confidence,
        evidence_level="observed_confirmed",
        based_on=[f"frame_{frame_id:06d}" for frame_id in track.seen_frame_ids],
        can_confirm_target=True,
        attributes={
            "navigation_role": node_type if node_type != "object" else track.navigation_role,
            "stable_position_2d": track.stable_position_2d,
            "first_seen_sec": track.first_seen_sec,
            "last_seen_sec": track.last_seen_sec,
            "best_frame_id": track.best_frame_id,
            "best_frame_path": track.best_frame_path,
            "representative_bbox": track.representative_bbox,
            "attributes": track.attributes,
            "place_ids": place_ids,
            "primary_place_id": primary_place_id,
        },
    )


def _add_place_free_space_nodes(
    nodes: list[SceneGraphNode],
    edges: list[SceneGraphEdge],
    places: list[PlaceSegment],
    frames: list[FrameObservation],
) -> None:
    free_space_by_place = _free_space_by_place(places, frames)
    for place in places:
        items = free_space_by_place.get(place.place_id, [])
        if not items and place.free_space_hint:
            items = [
                {
                    "position_2d": place.free_space_hint,
                    "description_zh": f"{place.free_space_hint} 可通行区域",
                    "confidence": 0.6,
                    "evidence_frames": place.evidence_frames,
                }
            ]
        for index, item in enumerate(items[:2], start=1):
            node_id = f"free_space_{place.place_id}_{index:03d}"
            nodes.append(
                SceneGraphNode(
                    node_id=node_id,
                    node_type="free_space",
                    label="free_space",
                    label_zh="可通行区域",
                    category="free_space",
                    source="observed",
                    confidence=float(item["confidence"]),
                    evidence_level="observed_candidate",
                    based_on=[f"frame_{frame_id:06d}" for frame_id in item["evidence_frames"]],
                    can_confirm_target=False,
                    attributes={
                        "position_2d": item["position_2d"],
                        "description_zh": item["description_zh"],
                        "place_id": place.place_id,
                        "navigation_role": "free_space",
                    },
                )
            )
            edges.append(
                SceneGraphEdge(
                    edge_id=f"edge_passable_{len(edges) + 1:04d}",
                    source_node_id=place.place_id,
                    target_node_id=node_id,
                    relation="passable_in",
                    source="observed",
                    confidence=float(item["confidence"]),
                    evidence_level="observed_candidate",
                    reason="free-space candidate belongs to place",
                )
            )


def _add_passage_connections(
    edges: list[SceneGraphEdge],
    places: list[PlaceSegment],
    tracks: list[ObjectTrack],
    assignments: dict[str, list[str]],
) -> None:
    passage_ids = {track.track_id for track in tracks if _node_type_for_track(track) == "passage"}
    for index, place in enumerate(places[:-1]):
        next_place = places[index + 1]
        candidates = [
            track_id
            for track_id in place.object_track_ids
            if track_id in passage_ids
        ]
        spanning = [
            track_id
            for track_id in candidates
            if next_place.place_id in assignments.get(track_id, [])
        ]
        anchor_id = (spanning or candidates or [None])[0]
        if not anchor_id:
            continue
        edges.append(
            SceneGraphEdge(
                edge_id=f"edge_connected_{len(edges) + 1:04d}",
                source_node_id=place.place_id,
                target_node_id=next_place.place_id,
                relation="connected_to",
                source="observed",
                confidence=0.72,
                evidence_level="observed_candidate",
                reason=f"adjacent places linked through {anchor_id}",
                attributes={"via": anchor_id},
            )
        )
        edges.append(
            SceneGraphEdge(
                edge_id=f"edge_through_{len(edges) + 1:04d}",
                source_node_id=anchor_id,
                target_node_id=next_place.place_id,
                relation="through",
                source="observed",
                confidence=0.7,
                evidence_level="observed_candidate",
                reason=f"{anchor_id} may lead into {next_place.place_id}",
            )
        )


def _add_frame_relations(
    edges: list[SceneGraphEdge],
    observations: list[FrameObservation],
    tracks: list[ObjectTrack],
) -> None:
    track_by_frame_object = _track_lookup(observations, tracks)
    allowed = {
        "near", "left_of", "right_of", "in_front_of", "behind",
        "on", "under", "above", "below", "in", "inside", "contains",
        "attached_to", "blocks", "adjacent_to",
    }
    for frame in observations:
        for relation in frame.relations:
            source_id = track_by_frame_object.get(relation.subject_id or "")
            target_id = track_by_frame_object.get(relation.object_id or "")
            if not source_id or not target_id or source_id == target_id:
                continue
            rel = relation.relation if relation.relation in allowed else "near"
            edges.append(
                SceneGraphEdge(
                    edge_id=f"edge_rel_{len(edges) + 1:04d}",
                    source_node_id=source_id,
                    target_node_id=target_id,
                    relation=rel,
                    source="observed",
                    confidence=relation.confidence,
                    evidence_level="observed_confirmed",
                    reason=relation.description_zh,
                    attributes={"frame_id": frame.frame_id},
                )
            )


def _free_space_by_place(
    places: list[PlaceSegment],
    frames: list[FrameObservation],
) -> dict[str, list[dict[str, object]]]:
    by_frame = {frame.frame_id: frame for frame in frames}
    grouped: dict[str, dict[str, dict[str, object]]] = defaultdict(dict)
    for place in places:
        for frame_id in place.evidence_frames:
            frame = by_frame.get(frame_id)
            if not frame:
                continue
            for item in frame.free_space:
                key = item.position_2d or "unknown"
                entry = grouped[place.place_id].setdefault(
                    key,
                    {
                        "position_2d": key,
                        "description_zh": item.description_zh,
                        "confidence_values": [],
                        "evidence_frames": [],
                    },
                )
                entry["confidence_values"].append(item.confidence)  # type: ignore[index]
                entry["evidence_frames"].append(frame.frame_id)  # type: ignore[index]
    result: dict[str, list[dict[str, object]]] = {}
    for place_id, entries in grouped.items():
        result[place_id] = []
        for entry in entries.values():
            values = entry.pop("confidence_values")
            entry["confidence"] = round(sum(values) / len(values), 4)  # type: ignore[arg-type]
            result[place_id].append(entry)
    return result


def _node_type_for_track(track: ObjectTrack) -> str:
    role = track.navigation_role
    text = f"{track.label} {track.label_zh}".lower()
    if role == "free_space" or track.category == "free_space":
        return "free_space"
    if role == "passage" or any(term in text for term in ["door", "doorway", "门", "通道", "exit", "stairs", "elevator"]):
        return "passage"
    if role == "obstacle" or track.category in {"obstacle", "person"}:
        return "obstacle"
    if role in {"landmark", "room_anchor"}:
        return "landmark"
    return "object"


def _track_lookup(
    observations: list[FrameObservation],
    tracks: list[ObjectTrack],
) -> dict[str, str]:
    lookup: dict[str, str] = {}
    labels_by_frame: defaultdict[tuple[int, str], list[str]] = defaultdict(list)
    for track in tracks:
        for frame_id in track.seen_frame_ids:
            labels_by_frame[(frame_id, track.label)].append(track.track_id)
    for frame in observations:
        for obj in frame.objects:
            candidates = labels_by_frame.get((frame.frame_id, obj.label), [])
            if candidates:
                lookup[obj.frame_object_id] = candidates[0]
    return lookup


def _scene_label_zh(scene_type: str) -> str:
    return {
        "corridor": "走廊段",
        "room": "房间区域",
        "office": "办公室区域",
        "living_room": "客厅区域",
        "outdoor": "室外区域",
        "unknown": "未知地点",
    }.get(scene_type, scene_type)


def _dedupe_edges(edges: list[SceneGraphEdge]) -> list[SceneGraphEdge]:
    seen: set[tuple[str, str, str, str]] = set()
    deduped: list[SceneGraphEdge] = []
    for edge in edges:
        key = (edge.source_node_id, edge.target_node_id, edge.relation, edge.source)
        if key in seen:
            continue
        seen.add(key)
        edge.edge_id = f"edge_{len(deduped) + 1:04d}"
        deduped.append(edge)
    return deduped
