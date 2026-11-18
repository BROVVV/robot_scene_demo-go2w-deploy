"""Receipt association preserves clock uncertainty and never accepts calibration."""
import ast
import copy
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT/'scripts/go2w/capture_rgbd_posture.py'
spec = importlib.util.spec_from_file_location('rgbd_posture_capture_test', SCRIPT)
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)
WALL = 1_793_000_000_000_000_000
MONO = 5_000_000_000


def frame(offset=0):
    return {'rgbd': {'frame_id':'7','timestamp_quality':'shared_host_capture',
                     'fx':912.,'fy':913.,'depth_unit_m':.001},
        'capture_wall_ns': WALL+offset,
        'fetch_clock': {'began_wall_ns':WALL,'began_monotonic_ns':MONO,
                        'ended_wall_ns':WALL+10_000_000,'ended_monotonic_ns':MONO+10_000_000,
                        'began_uncertainty_ns':0,'ended_uncertainty_ns':0}}


def state(offset=0):
    return {'received_wall_ns':WALL+offset,'received_monotonic_ns':MONO+offset,
            'clock_uncertainty_ns':0,'q':[0.]*12,'rpy':[0.]*3,'state_finite':True}


def test_nearest_receipt_on_both_sides_and_tie_retains_first_sample():
    frames = [frame(), frame(10_000_000)]
    frames[1]['fetch_clock']['ended_wall_ns'] += 10_000_000
    frames[1]['fetch_clock']['ended_monotonic_ns'] += 10_000_000
    result = capture.associate_posture(frames, [state(-5_000_000),state(5_000_000),state(15_000_000)])
    assert [row['nearest_state_index'] for row in result] == [0,1]
    assert all(row['posture_usable'] and 'not hardware synchronized' in row['reason'] for row in result)
    assert [row['state_receive_delta_ms'] for row in result] == [-5.,-5.]
    assert frames[0]['rgbd']['fx'] == 912. and 'association' not in frames[0]


def test_missing_or_distant_posture_is_not_invented():
    assert not capture.associate_posture([frame()],[])[0]['posture_usable']
    row = capture.associate_posture([frame()],[state(31_000_000)])[0]
    assert row['nearest_state_index'] == -1 and not row['posture_usable']


@pytest.mark.parametrize('kind', ['remote','clock_step','slow_clock_read','future','old','fractional_clock'])
def test_invalid_frame_clock_cannot_pair(kind):
    sample = frame()
    if kind == 'remote':sample['rgbd']['timestamp_quality'] = 'receive_time'
    elif kind == 'clock_step':sample['fetch_clock']['ended_wall_ns'] += 20_000_000
    elif kind == 'slow_clock_read':sample['fetch_clock']['began_uncertainty_ns'] = 2_000_000
    elif kind == 'future':sample['capture_wall_ns'] += 50_000_000
    elif kind == 'old':sample['capture_wall_ns'] -= 600_000_000
    elif kind == 'fractional_clock':sample['fetch_clock']['began_wall_ns'] = float(WALL)
    row = capture.associate_posture([sample],[state()])[0]
    assert not row['posture_usable'] and row['nearest_state_index'] == -1


def test_states_from_another_wall_clock_segment_are_excluded():
    wrong = state(); wrong['received_wall_ns'] += 30_000_000
    row = capture.associate_posture([frame()],[wrong])[0]
    assert row['nearest_state_index'] == -1 and not row['posture_usable']


def test_invalid_closest_posture_does_not_borrow_a_more_distant_valid_sample():
    invalid = state(); invalid['q'][0] = None; invalid['state_finite'] = False
    row = capture.associate_posture([frame()],[invalid,state(10_000_000)])[0]
    assert row['nearest_state_index'] == 0 and not row['posture_usable']
    assert row['reason'] == 'nearest state has invalid posture'


def test_uncertainty_is_part_of_the_receipt_bound():
    sample = frame(); sample['fetch_clock']['began_uncertainty_ns'] = 800_000
    posture = state(29_000_000); posture['clock_uncertainty_ns'] = 800_000
    row = capture.associate_posture([sample],[posture])[0]
    assert row['nearest_state_index'] == 0 and not row['posture_usable']
    assert row['receipt_alignment_bound_ms'] > 30.


def test_unordered_receipt_samples_are_rejected():
    with pytest.raises(ValueError,match='ordered'):
        capture.associate_posture([frame()],[state(10),state(0)])


def test_clock_read_is_bracketed_and_reports_descheduling(monkeypatch):
    values = iter([10,2_000_010])
    monkeypatch.setattr(capture.time,'monotonic_ns',lambda:next(values))
    monkeypatch.setattr(capture.time,'time_ns',lambda:WALL)
    assert capture.observe_clock() == (WALL,1_000_010,1_000_000)


def test_frame_refs_are_portable_and_original_metadata_is_preserved(tmp_path):
    data = {'frame_id':'1','color_ref':str(tmp_path/'rgbd/color_1.jpg'),
            'depth_ref':str(tmp_path/'rgbd/depth_1.png'),'fx':912.}
    before = copy.deepcopy(data)
    archived = capture.portable_frame(data,tmp_path)
    assert archived['color_ref'] == 'rgbd/color_1.jpg' and archived['depth_ref'] == 'rgbd/depth_1.png'
    assert archived['source_original_refs']['color_ref'] == data['color_ref']
    assert archived['fx'] == 912. and data == before
    with pytest.raises(ValueError):capture.portable_frame(data,tmp_path/'other')


@pytest.mark.parametrize('options', [['--frames','0'],['--frames','11'],['--interval','nan'],
                                  ['--duration','31'],['--frames','10','--interval','2','--duration','2']])
def test_bad_capture_window_rejected_before_ros_import(tmp_path, options):
    with pytest.raises(SystemExit) as result:
        capture.parse_args(['--output-dir',str(tmp_path/'unused')]+options)
    assert result.value.code == 2 and not (tmp_path/'unused').exists()


def test_collector_has_no_command_or_pointcloud_endpoints():
    tree = ast.parse(SCRIPT.read_text())
    calls = {node.func.attr for node in ast.walk(tree) if isinstance(node,ast.Call)
             and isinstance(node.func,ast.Attribute)}
    assert not calls.intersection({'create_publisher','create_client','create_service','create_action_client'})
    assert 'PointCloud2' not in SCRIPT.read_text()
    assert 'calibration_accepted=False' in SCRIPT.read_text()
