"""Optional exit diagnostics keep JSONL intact and expose a shutdown wait."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


ROOT=Path(__file__).resolve().parents[1]


def run_worker(*,trace,slow=False):
    script='''
import sys,threading,time
from app.detectors import yoloe_seg_worker as worker
class FakeRuntime:
    def __init__(self,args):
        if SLOW: threading.Thread(target=lambda:time.sleep(3.3)).start()
worker.YOLOERuntime=FakeRuntime
sys.argv=['worker','--serve','--model','unused-seg.pt']
raise SystemExit(worker._entrypoint())
'''.replace('SLOW',repr(slow))
    return subprocess.run([sys.executable,'-c',script],cwd=ROOT,
        env=dict(os.environ,GO2W_YOLOE_EXIT_TRACE='1' if trace else '0'),
        input=json.dumps({'operation':'shutdown','request_id':'close-1'})+'\n',
        text=True,capture_output=True,timeout=7)


@pytest.mark.parametrize('trace',[False,True])
def test_normal_shutdown_has_one_ack_and_optional_stderr_milestones(trace):
    result=run_worker(trace=trace)
    assert result.returncode==0,result.stderr
    assert [json.loads(x) for x in result.stdout.splitlines()]==[{'ok':True,'request_id':'close-1'}]
    if trace:
        events=[json.loads(x) for x in result.stderr.splitlines()]
        assert [e['stage'] for e in events]==['request_received','ack_flushed','main_returned','python_atexit']
        assert all(e['event']=='yoloe_worker_shutdown' for e in events)
        assert [e['monotonic'] for e in events]==sorted(e['monotonic'] for e in events)
    else:assert not result.stderr


def test_trace_exposes_interpreter_thread_wait_without_forcing_exit():
    result=run_worker(trace=True,slow=True)
    assert result.returncode==0,result.stderr
    assert json.loads(result.stdout)=={'ok':True,'request_id':'close-1'}
    assert 'Timeout (0:00:03)' in result.stderr
    assert 'threading.py' in result.stderr and '_shutdown' in result.stderr
    assert '"stage": "main_returned"' in result.stderr
    assert '"stage": "python_atexit"' in result.stderr
