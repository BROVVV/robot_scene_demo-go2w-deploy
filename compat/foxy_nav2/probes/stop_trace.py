"""Evaluate a stop by time and every subsequent velocity, not sample count."""
import math


def evaluate_stop(rows, started, *, deadline, minimum_observation=.2):
    started, deadline = float(started), float(deadline)
    minimum_observation = float(minimum_observation)
    if (not math.isfinite(started) or not math.isfinite(deadline) or deadline <= 0
            or not math.isfinite(minimum_observation) or minimum_observation < 0):
        raise ValueError('invalid stop timing bounds')
    selected = []
    previous = None
    width = None
    for row in rows:
        values = tuple(float(v) for v in row)
        if len(values) < 2 or not all(math.isfinite(v) for v in values):
            raise ValueError('invalid velocity trace')
        if width is not None and len(values) != width:
            raise ValueError('inconsistent velocity trace width')
        width = len(values)
        if previous is not None and values[0] < previous:
            raise ValueError('unordered velocity trace')
        previous = values[0]
        if values[0] >= started:
            selected.append(values)
    first = next((i for i, row in enumerate(selected)
                  if all(abs(v) < 1e-8 for v in row[1:])), None)
    if first is None:
        raise AssertionError('no zero velocity after stop trigger')
    delay = selected[first][0] - started
    if delay > deadline:
        raise AssertionError(('late stop', delay, deadline))
    if any(any(abs(v) >= 1e-8 for v in row[1:]) for row in selected[first:]):
        raise AssertionError('nonzero velocity resumed after first stop')
    observed = selected[-1][0] - selected[first][0]
    if observed + 1e-12 < minimum_observation:
        raise AssertionError(('insufficient observation after stop', observed, minimum_observation))
    return dict(first_zero_s=delay, final_zero=True,
                trigger_window_samples=len(selected),
                zero_samples_after_first=len(selected) - first,
                observation_after_first_zero_s=observed)
