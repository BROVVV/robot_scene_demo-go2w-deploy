"""Regression for transient Nav2 health loss during long vision calls."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from app.spatial import provider_readiness
from app.spatial.nav2_spatial_provider import Nav2SpatialProvider


@pytest.mark.parametrize('unavailable', ['CAMERA_LOCAL', 'NO_GLOBAL_SPATIAL_POSE'])
def test_wait_does_not_accept_unavailable_pose_as_ready(monkeypatch, unavailable):
    qualities = iter([unavailable, unavailable, 'METRIC_LIDAR'])
    spins = []
    sleeps = []
    provider = SimpleNamespace(spin_once=lambda: spins.append(True),
                               quality=lambda: next(qualities))
    monkeypatch.setattr(provider_readiness.time, 'sleep', sleeps.append)
    assert provider_readiness.wait_for_spatial_inputs(provider) is True
    assert len(spins) == 3 and sleeps == [.05] * 3


def test_native_nav2_quality_waits_for_real_health_recovery(monkeypatch):
    provider = Nav2SpatialProvider(enable_ros=False)
    provider._map = SimpleNamespace()
    provider.get_pose = lambda: SimpleNamespace(frame_id='map')
    calls = []

    def spin():
        calls.append(True)
        if len(calls) == 3:
            import json
            import time
            provider._on_valid(SimpleNamespace(data=json.dumps(
                {'valid': True, 'wall_time': time.time()})))

    provider.spin_once = spin
    monkeypatch.setattr(provider_readiness.time, 'sleep', lambda _: None)
    assert provider.quality() == 'NO_GLOBAL_SPATIAL_POSE'
    assert provider_readiness.wait_for_spatial_inputs(provider) is True
    assert len(calls) == 3 and provider.quality() == 'METRIC_LIDAR'


def test_permanent_loss_is_bounded_and_remains_unavailable(monkeypatch):
    calls = []
    sleeps = []
    provider = SimpleNamespace(spin_once=lambda: calls.append(True),
                               quality=lambda: 'NO_GLOBAL_SPATIAL_POSE')
    monkeypatch.setattr(provider_readiness.time, 'sleep', sleeps.append)
    assert provider_readiness.wait_for_spatial_inputs(provider) is False
    assert len(calls) == 40 and sum(sleeps) == pytest.approx(2.)
    assert provider.quality() == 'NO_GLOBAL_SPATIAL_POSE'


def test_relative_provider_keeps_existing_wait_behavior(monkeypatch):
    calls = []
    provider = SimpleNamespace(spin_once=lambda: calls.append(True),
                               quality=lambda: 'RELATIVE_RGBD')
    monkeypatch.setattr(provider_readiness.time, 'sleep', lambda _: None)
    assert provider_readiness.wait_for_spatial_inputs(provider) is True
    assert len(calls) == 1


def test_observation_uses_shared_wait_for_nav2_recovery(monkeypatch):
    # Importing the real observation runner would create ROS-specific imports.
    # Compile its exact production wait statement and run it with a recovering
    # provider, so this checks integration rather than a copied loop.
    source = Path(__file__).resolve().parents[1] / 'scripts/go2w/run_semantic_exploration.py'
    tree = ast.parse(source.read_text())
    imports = [node for node in ast.walk(tree)
               if isinstance(node, ast.ImportFrom)
               and node.module == 'app.spatial.provider_readiness']
    calls = [node for node in ast.walk(tree)
             if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)
             and isinstance(node.value.func, ast.Name)
             and node.value.func.id == 'wait_for_spatial_inputs']
    assert len(imports) == len(calls) == 1
    assert [arg.id for arg in calls[0].value.args] == ['camera_provider']
    qualities = iter(['NO_GLOBAL_SPATIAL_POSE', 'METRIC_LIDAR'])
    spins = []
    provider = SimpleNamespace(spin_once=lambda: spins.append(True),
                               quality=lambda: next(qualities))
    monkeypatch.setattr(provider_readiness.time, 'sleep', lambda _: None)
    module = ast.Module(body=[imports[0], calls[0]], type_ignores=[])
    exec(compile(module, str(source), 'exec'), {'camera_provider': provider})
    assert len(spins) == 2
