"""Validate the commissioning envelope without opening normal Foxy execution.

This permits bringing up the already isolated-tested control chain for one
supervised hardware check. The command bridge still enforces the ledger and
all live motion gates; this validator never arms or sends a goal.
"""
import json
import math
from pathlib import Path
import time


def validate_supervised_trial(path, *, boot_id, now=None):
    path = Path(path)
    if not path.is_absolute() or path.is_symlink():
        raise ValueError('supervised trial requires an absolute, regular ledger')
    data = json.loads(path.read_text())
    now = time.time() if now is None else now
    created = data.get('created_wall')
    if (type(created) not in (int, float) or not math.isfinite(created)
            or not 0 <= now-created <= 300):
        raise ValueError('supervised trial ledger is missing or expired')
    policies = {
        3: dict(version=3, purpose='single_observed_recheck',
                maximum_steps=1, maximum_translation_m=.20,
                stop_reserve_m=.05, maximum_speed_mps=.12,
                reviewed_robot_boot_id=boot_id, steps=[], active_token=None),
        # The collision-diagnostic ledger is the one separately bounded path
        # that may leave a known occupied rear-corner padding cell.  Admitting
        # it here does not arm motion: the bridge, live scan, localization,
        # one-step reservation and 0.20 m observed-travel stop remain active.
        7: dict(version=7, purpose='supervised_collision_diagnostic',
                forward_only=True, maximum_steps=1,
                maximum_translation_m=.25, stop_reserve_m=.05,
                maximum_speed_mps=.12, maximum_turn_rad=.15,
                maximum_lateral_m=.10, maximum_duration_s=15.,
                reviewed_robot_boot_id=boot_id, steps=[], active_token=None),
    }
    expected = policies.get(data.get('version'))
    if expected is None:
        raise ValueError('supervised trial policy mismatch: version')
    for key, value in expected.items():
        if type(data.get(key)) is not type(value) or data.get(key) != value:
            raise ValueError('supervised trial policy mismatch: '+key)
    if not isinstance(data.get('window_id'), str) or not data['window_id'].strip():
        raise ValueError('supervised trial needs a named window')
    return data
