import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts/go2w/capture_map_semantic_baseline.py"
SPEC = importlib.util.spec_from_file_location("capture_map_semantic_baseline", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_run_failure_is_reported_not_raised(tmp_path):
    result = MODULE._run(["definitely-not-a-real-command"], cwd=tmp_path)
    assert result["returncode"] is None
    assert result["stderr"]
