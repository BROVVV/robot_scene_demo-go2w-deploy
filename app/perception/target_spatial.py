"""Measure the target view without borrowing scene geometry or granting motion."""
from __future__ import annotations

from collections import Counter, defaultdict
import math

from app.perception.depth_object_localizer import DepthObjectLocalizer, resolve_depth_frame
from app.perception.entity_roles import assign_entity_ids


def localize_target_candidates(candidates, *, source_frame_id="", current_frame=None,
                               cache=None, localizer=None):
    """Each entity uses its own acquisition; mixed views never share a depth frame.

    This produces measurements only. Target presence, verification, persistence,
    mobility checks and all motion authorization remain separate.
    """
    output = assign_entity_ids(candidates, source_frame_id=source_frame_id)
    grouped = defaultdict(list)
    frames = {}
    for index, item in enumerate(output):
        # Cached dictionaries may contain old spatial enrichment. Clear it
        # even when the new acquisition cannot be recovered.
        item.update(depth_m=None, camera_xyz=None, bearing_deg=None, elevation_deg=None,
                    map_xyz=None, spatial_quality="RGB_ONLY", spatial_provenance={})
        item.pop('persistent_object_id', None)
        item.pop('target_persistence', None)
        source = str(item.get("entity_source_frame_id") or source_frame_id or "")
        frame = resolve_depth_frame(source, current_frame, cache) if source else None
        if frame is None:
            item["spatial_provenance"] = {"reason": "target_capture_unavailable", "frame_id": source}
            continue
        values = (frame.timestamp, frame.fx, frame.fy, frame.cx, frame.cy, frame.depth_unit_m)
        try:
            valid = (frame.depth_aligned_to_color and all(math.isfinite(float(v)) for v in values)
                     and frame.timestamp > 0 and frame.fx > 0 and frame.fy > 0
                     and frame.depth_unit_m > 0 and frame.width > 0 and frame.height > 0)
        except (TypeError, ValueError, OverflowError):
            valid = False
        if not valid:
            item["spatial_provenance"] = {"reason": "invalid_target_rgbd_geometry", "frame_id": source}
            continue
        grouped[source].append(index)
        frames[source] = frame
    localizer = localizer or DepthObjectLocalizer()
    for source, indices in grouped.items():
        frame = frames[source]
        counts = Counter(str(output[i]["entity_id"]) for i in indices)
        usable = [i for i in indices if counts[str(output[i]["entity_id"])] == 1]
        for i in indices:
            if i not in usable:
                output[i]["spatial_provenance"] = {"reason": "duplicate_target_identity", "frame_id": source}
        inputs = [dict(output[i], id=output[i]["entity_id"],
                       frame_object_id=output[i]["entity_id"]) for i in usable]
        measured = {str(value.object_id): value for value in localizer.localize(inputs, frame)}
        for i in usable:
            item = output[i]
            value = measured.get(str(item["entity_id"]))
            if value is None:
                continue
            item.update(depth_m=value.depth_m,
                        camera_xyz=list(value.camera_xyz) if value.camera_xyz is not None else None,
                        bearing_deg=value.bearing_deg, elevation_deg=value.elevation_deg,
                        spatial_quality=value.spatial_quality,
                        spatial_provenance={**value.provenance, "image_timestamp": float(frame.timestamp),
                            "timestamp_quality": frame.timestamp_quality, "color_ref": frame.color_ref,
                            "depth_ref": frame.depth_ref, "depth_unit_m": frame.depth_unit_m,
                            "intrinsics": {k: getattr(frame, k) for k in ("fx", "fy", "cx", "cy")},
                            "depth_aligned_to_color": True, "motion_authorized": False})
    return output


def project_target_candidates(candidates, provider, *, pose_accepted):
    """Use acquisition-time TF only; never fall back to current/nominal pose."""
    output = []
    for original in candidates:
        item = dict(original)
        item["spatial_provenance"] = dict(original.get("spatial_provenance") or {})
        for key in ("map_frame", "image_time_transform", "map_projection_error"):
            item["spatial_provenance"].pop(key, None)
        item["map_xyz"] = None
        xyz = item.get("camera_xyz")
        item["spatial_quality"] = "CAMERA_LOCAL" if xyz is not None else "RGB_ONLY"
        provenance = item["spatial_provenance"]
        output.append(item)
        source = str(item.get("entity_source_frame_id") or "")
        stamp = provenance.get("image_timestamp")
        if (not pose_accepted or provider is None or not source
                or source != provenance.get("frame_id") or not isinstance(xyz, (tuple, list))
                or len(xyz) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in xyz)
                or not isinstance(stamp, (int, float)) or not math.isfinite(stamp)):
            continue
        try:
            if provider.quality() not in {"METRIC_LIDAR", "METRIC_RGBD"}:
                continue
            point = provider.camera_point_to_map(tuple(xyz), float(stamp))
            tf = provider.transform_provenance()
            if (point is None or len(point) != 3 or not all(math.isfinite(float(v)) for v in point)
                    or tf.get("target_frame") != "map" or tf.get("transform_source") != "tf2"
                    or tf.get("tf_available") is not True
                    or tf.get("transform_quality") not in {"METRIC_LIDAR", "METRIC_RGBD"}):
                continue
            item["map_xyz"] = [float(v) for v in point]
            item["spatial_quality"] = "METRIC_RGBD"
            provenance.update(map_frame="map", image_time_transform=dict(tf))
            anchor = None
            factory = getattr(provider, 'anchor_map_point', None)
            if callable(factory):
                try:
                    anchor = factory(tuple(item['map_xyz']), float(stamp))
                except Exception:
                    anchor = None
            provenance['anchor_status'] = 'anchored' if anchor else 'unavailable'
            if anchor:
                for key in ('anchor_id','point_in_anchor','anchor_graph_revision',
                            'map_id','map_epoch','position_covariance','transform_valid'):
                    if key in anchor:provenance[key]=anchor[key]
        except Exception as error:
            provenance["map_projection_error"] = type(error).__name__
    return output


def bind_target_measurements(targets, measured, *, source_frame_id=''):
    """Enrich the shared role dictionaries by exact capture/entity/box identity.

    The pure measurement functions return copies; the runner explicitly writes
    them into its shared detection/target/anchor views, never into scene items.
    """
    from app.perception.entity_roles import entity_id_for
    def key(item):
        bbox = item.get('bbox_2d') or item.get('bbox')
        if not isinstance(bbox,(list,tuple)) or len(bbox)!=4:return None
        try:
            bbox = tuple(float(v) for v in bbox)
            if not all(math.isfinite(v) for v in bbox):return None
        except (ValueError,TypeError):return None
        return (str(item.get('entity_id') or entity_id_for(item)),
                str(item.get('entity_source_frame_id') or source_frame_id or ''),bbox)
    by_key = defaultdict(list)
    for item in measured:
        if isinstance(item,dict) and key(item) is not None:by_key[key(item)].append(item)
    for item in targets:
        if not isinstance(item,dict):continue
        matches=by_key.get(key(item),[])
        if len(matches)==1:
            for name in ('persistent_object_id','target_persistence'):item.pop(name,None)
            item.update(matches[0])
        else:
            item.update(depth_m=None,camera_xyz=None,bearing_deg=None,map_xyz=None,
                        spatial_quality='RGB_ONLY',spatial_provenance={'reason':'ambiguous_target_binding'})
            item.pop('persistent_object_id',None);item.pop('target_persistence',None)
    return targets


def target_identity(candidate):
    if not isinstance(candidate, dict):return None
    entity, source = candidate.get('entity_id'), candidate.get('entity_source_frame_id')
    bbox = candidate.get('bbox_2d') or candidate.get('bbox')
    if (not entity or not source or not isinstance(bbox,(tuple,list)) or len(bbox)!=4
            or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in bbox)):
        return None
    return {'entity_id':str(entity),'source_frame_id':str(source),'bbox':list(bbox)}


def selected_current_target(observation):
    """Select from the target view only and require the actual observed capture."""
    from app.perception.entity_roles import select_candidate
    best,_ = select_candidate(observation.target_candidates, [], scope='target_candidates',
                              candidates_authoritative=True)
    identity = target_identity(best)
    capture = (observation.provenance or {}).get('current_capture_id')
    if identity is None or not capture or identity['source_frame_id'] != capture:return None
    return best


def validated_target_binding(candidate, object_map):
    """An association is usable only with this capture's metric measurement."""
    if candidate is None:return None
    p = candidate.get('spatial_provenance') or {}
    identity = target_identity(candidate)
    object_id = candidate.get('persistent_object_id')
    entry = object_map.objects.get(object_id)
    if (identity is None or not object_id or entry is None
            or candidate.get('target_persistence') != 'associated'
            or candidate.get('spatial_quality') != 'METRIC_RGBD'
            or p.get('map_frame') != 'map' or p.get('frame_id') != identity['source_frame_id']
            or not getattr(entry,'topology_eligible',False)
            or getattr(entry,'mobility_class',None) not in {'fixed','semi_static'}
            or getattr(entry,'mobility_state',None) != 'stationary'
            or getattr(entry,'status',None) in {'STALE','REJECTED'}
            or getattr(entry,'lifecycle_state','active') != 'active'
            or identity['source_frame_id'] not in entry.source_observation_ids):
        return None
    tf = p.get('image_time_transform') or {}
    for key in ('map_epoch','map_id'):
        value = str(tf.get(key) or '')
        if value and value != str((entry.provenance or {}).get(key) or ''):return None
    point = candidate.get('map_xyz')
    if (not isinstance(point,(tuple,list)) or len(point)!=3
            or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in point)):
        return None
    return object_id
