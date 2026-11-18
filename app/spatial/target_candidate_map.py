"""Persist measured target views through the existing mobility and ingest gates."""
from __future__ import annotations

from collections import defaultdict
from dataclasses import replace
import math

from app.perception.depth_object_localizer import ObjectSpatialObservation, object_labels, object_category
from app.spatial.dynamic_labels import drop_dynamic_objects
from app.spatial.object_mobility import ObjectMobilityTracker
from app.spatial.semantic_object_gate import filter_persistent_observations
from app.spatial.semantic_object_map import SemanticMapUpdateResult


def target_source_id(entity_id):
    return 'target:' + str(entity_id)


def merge_updates(scene, target):
    """Keep source identities and index ranges distinct; neither map is re-ingested."""
    scene = scene or SemanticMapUpdateResult()
    offset = max((a.observation_index for a in scene.associations), default=-1) + 1
    return SemanticMapUpdateResult(
        created_ids=list(dict.fromkeys(scene.created_ids + target.created_ids)),
        updated_ids=list(dict.fromkeys(scene.updated_ids + target.updated_ids)),
        associations=list(scene.associations) + [replace(a, observation_index=a.observation_index+offset)
                                                for a in target.associations],
        rejected_pairs=list(scene.rejected_pairs) + [
            {**r, **({'observation_index':r['observation_index']+offset} if 'observation_index' in r else {})}
            for r in target.rejected_pairs],
    )


def _measurement(item):
    p = dict(item.get('spatial_provenance') or {})
    tf = dict(p.get('image_time_transform') or {})
    source = str(item.get('entity_source_frame_id') or '')
    xyz, camera, stamp = item.get('map_xyz'), item.get('camera_xyz'), p.get('image_timestamp')
    def vector(v):
        return isinstance(v,(list,tuple)) and len(v)==3 and all(isinstance(x,(float,int)) and math.isfinite(x) for x in v)
    if (not item.get('entity_id') or not source or p.get('frame_id') != source
            or item.get('spatial_quality') != 'METRIC_RGBD' or p.get('map_frame') != 'map'
            or not vector(xyz) or not vector(camera)
            or not isinstance(stamp,(float,int)) or not math.isfinite(stamp) or stamp <= 0
            or tf.get('transform_source') != 'tf2' or tf.get('tf_available') is not True
            or tf.get('target_frame') != 'map'
            or tf.get('transform_quality') not in {'METRIC_LIDAR','METRIC_RGBD'}
            or not isinstance(item.get('depth_m'),(float,int))
            or not math.isfinite(item['depth_m']) or item['depth_m'] <= 0 or camera[2] <= 0):
        return None
    label, zh = object_labels(item)
    return ObjectSpatialObservation(
        object_id=target_source_id(item['entity_id']), label=label, label_zh=zh,
        category=object_category(item), bbox=item.get('bbox_2d') or item.get('bbox'),
        depth_m=item.get('depth_m'), camera_xyz=tuple(camera), map_xyz=tuple(xyz),
        bearing_deg=item.get('bearing_deg'), spatial_quality='METRIC_RGBD',
        confidence=item.get('confidence',item.get('score',0.)),
        provenance={**tf, **p, 'source_frame':source, 'frame_id':source,
                    'source_object_id':target_source_id(item['entity_id']),
                    'entity_id':str(item['entity_id']), 'timestamp':float(stamp),
                    'image_stamp':float(stamp), 'source_sensor':'rgbd',
                    'depth_method':p.get('depth_source','unknown')},
    )


class TargetCandidateMapper:
    def __init__(self, mobility_config=None):
        # Separate view stream, same algorithm/config/thresholds. A late full
        # scene must not move the target track's time backwards.
        self.tracker = ObjectMobilityTracker(mobility_config)

    def reset(self):
        self.tracker.reset()

    def ingest(self, candidates, object_map, *, place_id, min_confidence):
        output = [dict(item) for item in candidates]
        groups = defaultdict(list)
        diagnostics = []
        for index, item in enumerate(output):
            item.pop('persistent_object_id',None)
            item['target_persistence'] = 'unproven_geometry'
            obs = _measurement(item)
            if obs is not None:
                p = obs.provenance
                groups[(p['frame_id'],p['timestamp'],str(p.get('map_epoch') or ''))].append((index,obs))
        # Conflicting timestamps/epochs for one capture are not two observations.
        identities = defaultdict(set)
        for source, stamp, epoch in groups:identities[source].add((stamp,epoch))
        combined = SemanticMapUpdateResult()
        for (source,stamp,epoch), rows in sorted(groups.items(),key=lambda g:g[0][1]):
            if len(identities[source]) != 1:
                continue
            observations = [obs for _,obs in rows]
            ids = [obs.object_id for obs in observations]
            if len(set(ids)) != len(ids):
                continue
            tracks = self.tracker.update(observations,frame_id=source,timestamp=stamp,map_epoch=epoch)
            track_by_source = {obs.object_id:track for obs,track in zip(observations,tracks)}
            for (index,obs),track in zip(rows,tracks):
                for name,value in track.fields().items():setattr(obs,name,value)
                output[index]['target_persistence'] = track.mobility_state
                if track.mobility_class=='dynamic' and track.persistent_id in object_map.objects:
                    entry=object_map.objects[track.persistent_id]
                    entry.topology_eligible=False;entry.mobility_class='dynamic'
            kept = [obs for obs in drop_dynamic_objects(observations) if getattr(obs,'topology_eligible',False)]
            kept,dropped = filter_persistent_observations(kept,min_confidence=min_confidence)
            result = object_map.update_with_associations(kept,place_id=place_id,now=stamp,frame_id=source)
            indices = {obs.object_id:index for index,obs in rows}
            for association in result.associations:
                track=track_by_source[association.source_object_id]
                if track.persistent_id and track.persistent_id != association.persistent_object_id:
                    old=object_map.objects.get(track.persistent_id)
                    if old is not None:old.topology_eligible=False
                track.persistent_id=association.persistent_object_id
                item=output[indices[association.source_object_id]]
                item['persistent_object_id']=association.persistent_object_id
                item['target_persistence']='associated'
            diagnostics.append({'frame_id':source,'timestamp':stamp,'map_epoch':epoch,
                                'measured':len(rows),'kept':len(kept),'dropped':dropped,
                                'associations':[a.to_dict() for a in result.associations]})
            combined=merge_updates(combined,result)
        return output,combined,diagnostics
