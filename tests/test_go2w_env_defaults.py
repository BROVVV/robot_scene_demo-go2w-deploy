"""Explicit operator overrides must survive every launcher .env load."""
import json
import os
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / 'scripts/go2w/load_env_defaults.sh'


@pytest.mark.parametrize('autoexport', [False, True])
def test_exported_override_empty_and_literal_values_survive(tmp_path, autoexport):
    defaults = tmp_path / '.env'
    defaults.write_text("OBSERVATION_MEMORY_STORE_PATH=protected.jsonl\n"
                        "EXPLICIT_EMPTY=default\nUNSET_DEFAULT='fallback with spaces'\n"
                        "QUOTED_OVERRIDE=default\n")
    env = os.environ.copy()
    env.update(OBSERVATION_MEMORY_STORE_PATH=str(tmp_path / 'isolated memory.jsonl'),
               EXPLICIT_EMPTY='', QUOTED_OVERRIDE='a\n\"b\" $(touch MUST_NOT_EXIST) `false` $HOME')
    env.pop('UNSET_DEFAULT', None)
    script = '''set -euo pipefail
source "$1"
if [[ "$4" == 1 ]]; then set -a; fi
go2w_load_env_defaults "$2"
[[ "$-" == *a* ]] && actual=1 || actual=0
[[ "$actual" == "$4" ]]
"$3" -c 'import os,json; print(json.dumps({k:os.environ[k] for k in ("OBSERVATION_MEMORY_STORE_PATH","EXPLICIT_EMPTY","UNSET_DEFAULT","QUOTED_OVERRIDE")}))'
'''
    import sys
    result = subprocess.run(['bash', '-c', script, '_', str(HELPER), str(defaults),
                             sys.executable, str(int(autoexport))], env=env, cwd=tmp_path,
                            capture_output=True, text=True, check=True)
    values = json.loads(result.stdout)
    for key in ('OBSERVATION_MEMORY_STORE_PATH', 'EXPLICIT_EMPTY', 'QUOTED_OVERRIDE'):
        assert values[key] == env[key]
    assert values['UNSET_DEFAULT'] == 'fallback with spaces'
    assert not (tmp_path / 'MUST_NOT_EXIST').exists()
    assert not result.stderr


def test_missing_defaults_is_harmless(tmp_path):
    subprocess.run(['bash', '-ec', 'source "$1"; go2w_load_env_defaults "$2"', '_',
                    str(HELPER), str(tmp_path / 'missing')], check=True)
