"""Nav2 startup may observe/plan beside an obstacle; actual paths stay gated."""
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from scripts.go2w.run_semantic_exploration import _runner_start_safety


@pytest.mark.parametrize('backend,expected', [('nav2', {'require_front_clearance': False}),
                                             ('go2w_experimental', {})])
def test_runner_uses_path_geometry_only_for_nav2(backend, expected):
    node=SimpleNamespace(_safety_ok=Mock(return_value=(False,'stale')))
    assert _runner_start_safety(SimpleNamespace(backend=backend),node)==(False,'stale')
    node._safety_ok.assert_called_once_with(**expected)
