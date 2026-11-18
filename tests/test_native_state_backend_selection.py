"""Explicit native selection must preserve argv and fail closed on bad binaries."""
from pathlib import Path
import json,os,subprocess,sys
import pytest
ROOT=Path(__file__).resolve().parents[1]
CASES=[('network_low_state_capture.py','GO2W_NATIVE_LOW_STATE_CAPTURE'),
       ('local_low_state_ros_relay.py','GO2W_NATIVE_LOW_STATE_RELAY')]
@pytest.mark.parametrize('script,variable',CASES)
def test_native_backend_is_execed_with_literal_arguments(tmp_path,script,variable):
    binary=tmp_path/'native_stub'
    binary.write_text('#!'+sys.executable+'\nimport json,sys\nprint(json.dumps(sys.argv[1:]))\n')
    binary.chmod(0o700)
    args=['--input','name with spaces; literal $value','--max-age','0.3']
    p=subprocess.run([sys.executable,str(ROOT/'scripts/go2w'/script)]+args,
                     capture_output=True,text=True,env=dict(os.environ,**{variable:str(binary)}))
    assert p.returncode==0,p.stderr
    assert json.loads(p.stdout)==args
@pytest.mark.parametrize('script,variable',CASES)
def test_bad_native_selection_does_not_silently_fall_back(tmp_path,script,variable):
    p=subprocess.run([sys.executable,str(ROOT/'scripts/go2w'/script),'--help'],
                     capture_output=True,text=True,env=dict(os.environ,**{variable:str(tmp_path/'missing')}))
    assert p.returncode!=0 and 'FileNotFoundError' in p.stderr
