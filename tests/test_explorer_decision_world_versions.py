from app.navigation.decision_models import DecisionRecord, NextMotionCommand


def test_execution_update_preserves_world_versions_and_records_outcome():
    command = NextMotionCommand('c', 'p', 'd', 'STOP')
    record = DecisionRecord.create(
        session_id='s', task_id='t', cycle=1, raw_task_text='target',
        canonical_target='target', current_place_id='P1', current_pose=None,
        target_match_level='zero_match', selected_long_term_goal={'goal_id': 'g'},
        candidate_ranking=[], navigation_plan={}, next_motion_command=command,
        reason_zh='test',
    )
    versioned = DecisionRecord(**{
        **record.to_dict(), 'snapshot_id': 'snap', 'map_id': 'map',
        'map_epoch': 'epoch', 'graph_revision': 2, 'grid_revision': 3,
        'semantic_revision': 4, 'sensor_profile_id': 'sha256:profile',
        'source_observation_ids': ['obs'],
        'candidates': [{'id': 'f1'}], 'selected_candidate_id': 'f1',
        'downstream_selected_id': 'g',
    })
    updated = versioned.with_execution(
        status='FAILED', message='no path', arrival_verified=False
    )
    assert updated.snapshot_id == 'snap'
    assert updated.map_epoch == 'epoch'
    assert updated.sensor_profile_id == 'sha256:profile'
    assert updated.source_observation_ids == ['obs']
    assert updated.execution_result == {'status': 'FAILED', 'message': 'no path'}
    assert updated.arrival_verified is False
