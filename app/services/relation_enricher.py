"""Deterministically add lightweight spatial relations so topology stays connected."""

from __future__ import annotations

import math
import networkx as nx

from app.schemas import (
    SceneAnalysisResult,
    SceneObject,
    SceneRelation,
    TopologyEdge,
    TopologyGraph,
    TopologyNode,
)


AUTO_RELATION_MARKER = "根据图像位置自动补全"


def enrich_scene_relations(
    result: SceneAnalysisResult,
    nearest_neighbors: int = 2,
) -> SceneAnalysisResult:
    """Add inferred spatial relations without calling the model again.

    The model should focus on semantic relations. This pass adds a sparse set of
    local spatial relations so every object participates in the topology graph.
    """

    objects = result.objects
    if len(objects) <= 1:
        return _with_topology(result, list(result.relations))

    relations = _dedupe_valid_relations(result.relations, {obj.id for obj in objects})
    existing_pairs = {_pair_key(relation.source_id, relation.target_id) for relation in relations}

    for source in objects:
        neighbors = _nearest_objects(source, objects)
        added_for_source = 0
        for target in neighbors:
            if added_for_source >= nearest_neighbors:
                break
            key = _pair_key(source.id, target.id)
            if key in existing_pairs:
                added_for_source += 1
                continue
            relations.append(_infer_relation(source, target))
            existing_pairs.add(key)
            added_for_source += 1

    relations = _connect_components(objects, relations, existing_pairs)
    return _with_topology(result, relations)


def _dedupe_valid_relations(
    relations: list[SceneRelation],
    object_ids: set[str],
) -> list[SceneRelation]:
    deduped: list[SceneRelation] = []
    seen: set[tuple[str, str, str]] = set()
    for relation in relations:
        if relation.source_id not in object_ids or relation.target_id not in object_ids:
            continue
        key = (relation.source_id, relation.target_id, relation.relation_type)
        if key in seen:
            continue
        seen.add(key)
        deduped.append(relation)
    return deduped


def _nearest_objects(
    source: SceneObject,
    objects: list[SceneObject],
) -> list[SceneObject]:
    return sorted(
        (obj for obj in objects if obj.id != source.id),
        key=lambda obj: _spatial_distance(source, obj),
    )


def _connect_components(
    objects: list[SceneObject],
    relations: list[SceneRelation],
    existing_pairs: set[frozenset[str]],
) -> list[SceneRelation]:
    graph = nx.Graph()
    graph.add_nodes_from(obj.id for obj in objects)
    graph.add_edges_from((relation.source_id, relation.target_id) for relation in relations)

    components = [set(component) for component in nx.connected_components(graph)]
    if len(components) <= 1:
        return relations

    object_by_id = {obj.id: obj for obj in objects}
    while len(components) > 1:
        best_pair: tuple[SceneObject, SceneObject] | None = None
        best_distance = math.inf
        best_indexes = (0, 1)

        for left_index, left_component in enumerate(components):
            for right_index in range(left_index + 1, len(components)):
                right_component = components[right_index]
                for left_id in left_component:
                    for right_id in right_component:
                        left_obj = object_by_id[left_id]
                        right_obj = object_by_id[right_id]
                        distance = _spatial_distance(left_obj, right_obj)
                        if distance < best_distance:
                            best_distance = distance
                            best_pair = (left_obj, right_obj)
                            best_indexes = (left_index, right_index)

        if best_pair is None:
            break

        source, target = best_pair
        key = _pair_key(source.id, target.id)
        if key not in existing_pairs:
            relations.append(_infer_relation(source, target))
            existing_pairs.add(key)

        left_index, right_index = best_indexes
        components[left_index] = components[left_index] | components[right_index]
        del components[right_index]

    return relations


def _infer_relation(source: SceneObject, target: SceneObject) -> SceneRelation:
    relation_type = _infer_relation_type(source, target)
    distance = _estimate_relation_distance(source, target)
    return SceneRelation(
        source_id=source.id,
        target_id=target.id,
        relation_type=relation_type,
        description_zh=f"{_describe_relation(source, target, relation_type)}（{AUTO_RELATION_MARKER}）",
        estimated_distance_m=distance,
        confidence=0.55,
    )


def _infer_relation_type(
    source: SceneObject,
    target: SceneObject,
) -> str:
    if not (_has_usable_bbox(source) and _has_usable_bbox(target)):
        return _infer_position_relation_type(source, target)

    source_center = _bbox_center(source)
    target_center = _bbox_center(target)
    dx = source_center[0] - target_center[0]
    dy = source_center[1] - target_center[1]

    if _bbox_iou(source, target) > 0.08:
        return "occluding"
    if _center_distance(source, target) < 0.18:
        return "near"
    if abs(dx) >= abs(dy):
        return "left_of" if dx < 0 else "right_of"
    return "behind" if dy < 0 else "in_front_of"


def _describe_relation(
    source: SceneObject,
    target: SceneObject,
    relation_type: str,
) -> str:
    labels = {
        "left_of": f"{source.name_zh}在{target.name_zh}左侧",
        "right_of": f"{source.name_zh}在{target.name_zh}右侧",
        "in_front_of": f"{source.name_zh}在{target.name_zh}前方",
        "behind": f"{source.name_zh}在{target.name_zh}后方",
        "near": f"{source.name_zh}靠近{target.name_zh}",
        "occluding": f"{source.name_zh}与{target.name_zh}区域重叠或互相遮挡",
    }
    return labels.get(relation_type, f"{source.name_zh}与{target.name_zh}存在空间关系")


def _estimate_relation_distance(
    source: SceneObject,
    target: SceneObject,
) -> float | None:
    source_distance = source.position.estimated_distance_m
    target_distance = target.position.estimated_distance_m
    if source_distance is None or target_distance is None:
        return None
    return round(abs(source_distance - target_distance), 2)


def _with_topology(
    result: SceneAnalysisResult,
    relations: list[SceneRelation],
) -> SceneAnalysisResult:
    topology = TopologyGraph(
        nodes=[
            TopologyNode(id=f"node_{index:03d}", label=obj.name_zh, object_id=obj.id)
            for index, obj in enumerate(result.objects, start=1)
        ],
        edges=[
            TopologyEdge(
                source_id=relation.source_id,
                target_id=relation.target_id,
                relation_type=relation.relation_type,
                label=relation.description_zh,
                relation_id=f"rel_{index:03d}",
            )
            for index, relation in enumerate(relations, start=1)
        ],
    )
    return result.model_copy(update={"relations": relations, "topology": topology})


def _bbox_center(obj: SceneObject) -> tuple[float, float]:
    bbox = obj.bbox_2d
    return ((bbox.x1 + bbox.x2) / 2, (bbox.y1 + bbox.y2) / 2)


def _center_distance(left: SceneObject, right: SceneObject) -> float:
    left_center = _bbox_center(left)
    right_center = _bbox_center(right)
    return math.dist(left_center, right_center)


def _spatial_distance(left: SceneObject, right: SceneObject) -> float:
    if _has_usable_bbox(left) and _has_usable_bbox(right):
        return _center_distance(left, right)

    left_coord = _position_coord(left)
    right_coord = _position_coord(right)
    return math.dist(left_coord, right_coord)


def _position_coord(obj: SceneObject) -> tuple[float, float, float]:
    horizontal = {"left": 0.0, "center": 0.5, "right": 1.0}.get(
        obj.position.horizontal,
        0.5,
    )
    vertical = {"front": 0.0, "middle": 0.5, "back": 1.0}.get(
        obj.position.vertical,
        0.5,
    )
    distance = obj.position.estimated_distance_m
    normalized_distance = 0.5 if distance is None else max(0.0, min(1.0, distance / 5.0))
    return horizontal, vertical, normalized_distance


def _infer_position_relation_type(
    source: SceneObject,
    target: SceneObject,
) -> str:
    source_coord = _position_coord(source)
    target_coord = _position_coord(target)
    dx = source_coord[0] - target_coord[0]
    dy = source_coord[1] - target_coord[1]
    dz = source_coord[2] - target_coord[2]

    if math.dist(source_coord, target_coord) < 0.25:
        return "near"
    if abs(dx) >= abs(dy) and abs(dx) >= abs(dz):
        return "left_of" if dx < 0 else "right_of"
    if abs(dz) >= abs(dy):
        return "in_front_of" if dz < 0 else "behind"
    return "in_front_of" if dy < 0 else "behind"


def _has_usable_bbox(obj: SceneObject) -> bool:
    bbox = obj.bbox_2d
    width = bbox.x2 - bbox.x1
    height = bbox.y2 - bbox.y1
    if width <= 0 or height <= 0:
        return False
    if width >= 0.98 and height >= 0.98:
        return False
    return True


def _bbox_iou(left: SceneObject, right: SceneObject) -> float:
    left_box = left.bbox_2d
    right_box = right.bbox_2d
    x1 = max(left_box.x1, right_box.x1)
    y1 = max(left_box.y1, right_box.y1)
    x2 = min(left_box.x2, right_box.x2)
    y2 = min(left_box.y2, right_box.y2)
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    if intersection == 0:
        return 0.0

    left_area = (left_box.x2 - left_box.x1) * (left_box.y2 - left_box.y1)
    right_area = (right_box.x2 - right_box.x1) * (right_box.y2 - right_box.y1)
    union = left_area + right_area - intersection
    if union <= 0:
        return 0.0
    return intersection / union


def _pair_key(left_id: str, right_id: str) -> frozenset[str]:
    return frozenset((left_id, right_id))
