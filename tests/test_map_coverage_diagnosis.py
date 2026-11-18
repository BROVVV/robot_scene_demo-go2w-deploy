"""Independent diagnostic must not confuse isotropic radius with lateral width."""
import importlib.util
from pathlib import Path

import pytest

from app.navigation.corridor_analyzer import CorridorGrid, measure_path_clearance, classify_path_clearance

spec = importlib.util.spec_from_file_location('coverage_diagnosis', Path(__file__).resolve().parents[1] / 'scripts/go2w/diagnose_map_coverage.py')
diagnosis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnosis)


def grid():
    return {'width': 20, 'height': 60, 'resolution': .05, 'origin': [-.5, -1.5], 'frame': 'map', 'data': [0]*1200}


def test_short_wide_room_passes_lateral_gate_despite_small_isotropic_radius():
    g = grid()
    # No position has .655m clearance in every direction: room is only 1m long.
    # Its 3m lateral width nevertheless meets the actual production criterion.
    production = CorridorGrid(.05, -.5, -1.5, 20, 60, g['data'])
    assert classify_path_clearance(measure_path_clearance([(-.1, 0), (.1, 0)], production)).passable
    report = diagnosis.analyze_grid(g)
    assert report['lateral_clearance_sampling']['verdict'] == 'some_samples_pass'
    assert report['lateral_clearance_sampling']['best']['gap_m'] > .4
    assert report['motion_authorized'] is False


def test_robot_footprint_uses_observed_pose_instead_of_map_origin():
    g = grid()
    g['data'][30*20+10] = 100  # Near (0, 0).
    report = diagnosis.analyze_grid(g, {'x': 0, 'y': .9, 'yaw': 0})
    assert not report['robot_footprint_on_map']['already_blocked']
    assert diagnosis.analyze_grid(g, {'x': 0, 'y': 0, 'yaw': 0})['robot_footprint_on_map']['already_blocked']


def test_missing_pose_and_empty_map_do_not_invent_robot_relative_results():
    g = grid()
    g['data'] = [-1]*1200
    report = diagnosis.analyze_grid(g)
    assert report['robot_footprint_on_map'] is None
    assert report['occupied_relative_to_robot'] is None
    assert report['lateral_clearance_sampling']['best'] is None
    assert report['lateral_clearance_sampling']['verdict'] == 'no_sample_passes'


@pytest.mark.parametrize('change', [{'data': [0]}, {'resolution': float('nan')},
                                  {'origin_orientation': [0, 0, .1, .995]}])
def test_invalid_or_rotated_grids_are_explicitly_refused(change):
    g = grid()
    g.update(change)
    with pytest.raises(ValueError):
        diagnosis.analyze_grid(g)
