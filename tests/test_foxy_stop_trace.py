"""Stopping is a time bound plus absence of later nonzero commands."""
import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('foxy_stop_trace',
    Path(__file__).resolve().parents[1] / 'compat/foxy_nav2/probes/stop_trace.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_sparse_stop_does_not_count_pre_stop_velocity_as_a_rebound():
    # Shape of the real failing native trace: one pending velocity, then only
    # four observed zeros. Fixed five-sample tails rejected this valid stop.
    rows = [(10.00, .07, 0), (10.05, 0, 0), (10.10, 0, 0),
            (10.15, 0, 0), (10.59, 0, 0)]
    result = module.evaluate_stop(rows, 10., deadline=.2)
    assert result['first_zero_s'] == pytest.approx(.05)
    assert result['zero_samples_after_first'] == 4


def test_rebound_is_rejected_even_when_last_five_commands_are_zero():
    rows = [(1.01, 0, 0), (1.02, .1, 0)] + [(1.03+i*.01, 0, 0) for i in range(5)]
    with pytest.raises(AssertionError, match='resumed'):
        module.evaluate_stop(rows, 1., deadline=.2)


@pytest.mark.parametrize('axis', range(6))
def test_every_twist_axis_is_checked_after_the_first_zero(axis):
    resumed = [1.02] + [0.]*6
    resumed[axis+1] = .01
    with pytest.raises(AssertionError, match='resumed'):
        module.evaluate_stop([(1.01, 0, 0, 0, 0, 0, 0), resumed], 1., deadline=.2)


def test_zero_before_trigger_does_not_satisfy_the_deadline():
    with pytest.raises(AssertionError, match='late stop'):
        module.evaluate_stop([(1., 0, 0), (2.01, .1, 0), (2.21, 0, 0)], 2., deadline=.2)


def test_positive_velocity_without_a_stop_is_rejected():
    with pytest.raises(AssertionError, match='no zero'):
        module.evaluate_stop([(1.01, .1, 0)], 1., deadline=.2)


def test_one_zero_without_a_later_observation_does_not_prove_a_stable_stop():
    with pytest.raises(AssertionError, match='insufficient observation'):
        module.evaluate_stop([(1.01, 0, 0)], 1., deadline=.2)


@pytest.mark.parametrize('rows', [[(1., float('nan'))], [(1., float('inf'))],
    [(1., 0), (.9, 0)], [(1., 0, 0), (1.1, 0)]])
def test_invalid_trace_cannot_prove_a_stop(rows):
    with pytest.raises(ValueError):
        module.evaluate_stop(rows, 1., deadline=.2)
