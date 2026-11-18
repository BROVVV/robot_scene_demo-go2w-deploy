from app.live_robot.semantic_observer import local_detection_scene_objects, _from_payload
from app.reasoning.target_profile import TargetProfile


def profile():
    return TargetProfile(raw_query='杯子',canonical_name_zh='杯子',primary_labels_en=['cup'],
                         relation_anchor_labels_en=['table'])


def obj(term, **extra):
    return {'label': term,'source_prompt_term':term,'score':.9,'bbox_2d':[.1,.2,.3,.4],**extra}


def test_local_context_is_available_before_full_semantic_but_not_a_target():
    objects=[obj('cup'),obj('chair'),obj('table')]
    scene=local_detection_scene_objects(objects,profile(),source_frame_id='42',current_frame_id='42')
    assert {i['label'] for i in scene}=={'chair','table'}
    observation=_from_payload({'scene_objects':scene,'frame_id':'42'},robot_pose=None,sector=0,now=100.)
    assert {i['label'] for i in observation.objects}=={'chair','table'}
    assert objects[0].get('roles') is None


def test_old_slot_detections_are_not_relabelled_as_current_scene():
    scene=local_detection_scene_objects([obj('chair',entity_source_frame_id='41')],profile(),source_frame_id='42',current_frame_id='42')
    assert scene==[]


def test_unknown_frame_and_empty_detection_do_not_invent_scene_facts():
    assert local_detection_scene_objects([obj('chair')],profile(),source_frame_id='',current_frame_id='42')==[]
    assert local_detection_scene_objects([],profile(),source_frame_id='42',current_frame_id='42')==[]


def test_old_assembled_frame_is_not_current_even_without_entity_metadata():
    assert local_detection_scene_objects([obj('chair')],profile(),source_frame_id='41',current_frame_id='42')==[]


def test_mixed_assembly_keeps_only_individually_dated_current_context():
    scene=local_detection_scene_objects([obj('chair',entity_source_frame_id='42'),
        obj('table',entity_source_frame_id='41')],profile(),source_frame_id='',current_frame_id='42')
    assert [i['label'] for i in scene]==['chair']
