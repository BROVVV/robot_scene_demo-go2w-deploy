"""A strong scene match must never steer towards a more confident scene object."""
from dataclasses import replace

import pytest

from app.navigation.long_term_goal_selector import LongTermGoalSelector
from app.reasoning.semantic_navigation.models import GraphMatchResult, GraphMatchState
from app.reasoning.semantic_navigation.spatial_reasoner import SpatialSearchReasoner
from app.reasoning.target_profile import TargetProfile
from app.spatial.semantic_object_map import SemanticObjectMap, SemanticObjectEntry


def scene():
    objects = SemanticObjectMap()
    objects.objects = {
        'chair': SemanticObjectEntry('chair', 'office chair', confidence=.95,
            bearing_deg=-80, source_observation_ids=['42']),
        'cup': SemanticObjectEntry('cup', 'blue cup', confidence=.60,
            bearing_deg=25, source_observation_ids=['42']),
        'old_cup': SemanticObjectEntry('old_cup', 'blue cup', confidence=.99,
            bearing_deg=120, source_observation_ids=['41']),
    }
    return objects


def propose(objects, **overrides):
    kwargs = dict(graph_match=GraphMatchResult(GraphMatchState.STRONG, .9),
        frontiers=[], semantic_map=objects, observation_id='42',
        target_profile=TargetProfile(raw_query='蓝色水杯', canonical_name_zh='水杯',
            primary_labels_en=['cup'], colors=['blue']))
    kwargs.update(overrides)
    return SpatialSearchReasoner().propose(**kwargs)


def test_current_lower_score_target_wins_over_chair_and_historical_target():
    result = propose(scene())
    assert result.intent.target_object_id == 'cup'
    assert result.intent.preferred_bearing_deg == -25
    # Category membership selects a viewpoint, never confirms colour or target.
    assert result.intent.intent_type == 'APPROACH_TARGET'


@pytest.mark.parametrize('change', ['no_target', 'no_frame', 'no_profile', 'stale', 'rejected', 'bad_bearing'])
def test_no_current_usable_target_does_not_fall_back_to_scene(change):
    objects = scene()
    kwargs = {}
    if change == 'no_target':
        del objects.objects['cup']
    elif change == 'no_frame':
        kwargs['observation_id'] = None
    elif change == 'no_profile':
        kwargs['target_profile'] = None
    elif change in {'stale', 'rejected'}:
        objects.objects['cup'] = replace(objects.objects['cup'], status=change.upper())
    else:
        objects.objects['cup'] = replace(objects.objects['cup'], bearing_deg=float('nan'))
    assert propose(objects, **kwargs) is None


@pytest.mark.parametrize('match_state', ['STRONG', 'VERIFY'])
def test_selector_needs_explicit_persistent_target_ids(match_state):
    selector = LongTermGoalSelector()
    assert selector.select(match_state=match_state, frontiers=[], semantic_map=scene()) is None
    selected = selector.select(match_state=match_state, frontiers=[], semantic_map=scene(),
        target_object_ids=['cup'])
    assert selected.intent.target_object_id == 'cup'


def test_empty_authoritative_candidates_have_zero_target_score():
    from app.live_robot.semantic_observer import _from_payload, semantic_observation_to_live
    semantic = _from_payload({'scene_objects': [], 'frame_id': '42'},
        robot_pose=None, sector=0, now=100.)
    kwargs = dict(bundle_id='bundle_42', detections=[{'label': 'chair', 'score': .95}],
        target_present=False, pose=None)
    assert semantic_observation_to_live(semantic, target_candidates=[], **kwargs).target_match['score'] == 0
    assert semantic_observation_to_live(semantic, **kwargs).target_match['score'] == .95


def test_default_enables_target_roles_and_candidate_verification():
    from app.config import Settings
    settings = Settings(siliconflow_api_key='unused')
    assert settings.go2w_result_role_separation is True
    assert settings.go2w_verify_candidate_scope == 'target_candidates'


def test_optical_right_bearing_is_subtracted_from_map_yaw():
    result = propose(scene(), current_yaw_deg=80.)
    assert result.intent.preferred_bearing_deg == 55.


def test_metric_target_uses_current_map_position_instead_of_old_optical_bearing():
    objects = scene()
    objects.objects['cup'] = replace(objects.objects['cup'], map_xyz=(2., 3., 0.),
        provenance={'map_frame': 'map'})
    result = propose(objects, current_yaw_deg=80., current_position=(1., 2.))
    assert result.intent.preferred_bearing_deg == 45.
    assert result.intent.provenance['bearing_source'] == 'current_map_position_to_target'


@pytest.mark.parametrize('match_state',['STRONG','VERIFY'])
def test_unbound_graph_match_explores_observed_frontier_without_using_chair(match_state):
    from app.spatial.models import FrontierCandidate
    objects=scene();del objects.objects['cup']
    selected=LongTermGoalSelector().select(match_state=match_state,semantic_map=objects,
        target_object_ids=['missing_id'],frontiers=[FrontierCandidate('forward',position=(1.,0.),
            bearing_deg=0.,distance_m=1.,spatial_information_gain=.8)])
    assert selected.intent.intent_type=='EXPLORE_FRONTIER'
    assert selected.intent.target_object_id is None
    assert selected.intent.preferred_position==(1.,0.)
    assert selected.intent.provenance['unbound_target_match_state']==match_state
