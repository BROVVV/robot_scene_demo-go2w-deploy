import importlib.util
from pathlib import Path

import numpy as np
import pytest


MODULE=Path(__file__).resolve().parents[1]/'scripts/go2w/capture_motion_geometry.py'
spec=importlib.util.spec_from_file_location('capture_motion_geometry',MODULE)
capture=importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def test_pairs_nearest_state_on_both_sides_and_marks_missing():
    actual=capture.pair_clouds_to_states([100_000_000,130_000_000,200_000_000],
        [80_000_000,120_000_000,170_000_000],tolerance_ns=20_000_000)
    assert actual.tolist()==[0,1,-1]
    assert capture.pair_clouds_to_states([1],[]).tolist()==[-1]


def test_association_rejects_unordered_states():
    with pytest.raises(ValueError,match='ordered'):
        capture.pair_clouds_to_states([10],[12,5])
    assert np.array_equal(capture.pair_clouds_to_states([],[]),np.array([],dtype=np.int64))


def test_invalid_joint_sample_can_be_preserved_without_invalid_json():
    assert capture.finite_or_none(float('nan')) is None
    assert capture.finite_or_none(float('inf')) is None
    assert capture.finite_or_none(.25)==.25
