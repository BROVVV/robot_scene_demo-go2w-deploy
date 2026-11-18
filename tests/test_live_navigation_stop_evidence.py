"""A failed native stop must keep its trigger and remain failed."""
import importlib.util
import json
from pathlib import Path
import sys

import pytest


@pytest.fixture
def probe(monkeypatch):
    root=Path(__file__).resolve().parents[1]/'compat/foxy_nav2/probes'
    monkeypatch.syspath_prepend(str(root))
    spec=importlib.util.spec_from_file_location('live_nav_evidence',root/'live_navigation.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_sensor_stop_then_rebound_remains_failed_with_exact_trigger(probe,tmp_path):
    out=tmp_path/'live.json';report={'cases':[]}
    rows=[(10.05,0,0),(10.08,.3,0),(10.12,0,0),(10.4,0,0)]
    with pytest.raises(AssertionError,match='resumed'):
        probe.record_cancel_case(report,out,cycle=4,active_started=8.,triggered=10.,
            action_status=5,commands=rows,statuses=[{'monotonic':9.,'reason':'scan_stale'}])
    case=json.loads(out.read_text())['cases'][0]
    assert case['triggered']==10. and case['cycle']==4 and case['action_status']==5
    assert case['passed'] is False and 'resumed' in case['error']
    assert case['freshness_blocks']==[{'monotonic':9.,'reason':'scan_stale'}]


def test_clean_stop_evidence_uses_unchanged_deadline(probe,tmp_path):
    report={'cases':[]}
    case=probe.record_cancel_case(report,tmp_path/'live.json',cycle=0,active_started=8.,
        triggered=10.,action_status=5,commands=[(10.1,0,0),(10.4,0,0)],statuses=[])
    assert case['passed'] and case['first_zero_s']==pytest.approx(.1)


def test_late_stop_is_saved_and_rejected(probe,tmp_path):
    path=tmp_path/'live.json';report={'cases':[]}
    with pytest.raises(AssertionError,match='late stop'):
        probe.record_cancel_case(report,path,cycle=0,active_started=8.,triggered=10.,
            action_status=5,commands=[(10.31,0,0),(10.61,0,0)],statuses=[])
    assert not json.loads(path.read_text())['cases'][0]['passed']
