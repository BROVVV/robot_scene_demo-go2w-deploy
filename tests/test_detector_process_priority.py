import pytest
from app.detectors.process_priority import detector_process_command


def test_desktop_default_preserves_command(monkeypatch):
    monkeypatch.delenv('GO2W_ML_NICE', raising=False)
    assert detector_process_command(['/python','worker.py','--device','cuda:0']) == ['/python','worker.py','--device','cuda:0']


def test_background_priority_is_an_argv_wrapper_without_shell_or_model_changes(monkeypatch):
    monkeypatch.setenv('GO2W_ML_NICE','10')
    monkeypatch.setattr('app.detectors.process_priority.shutil.which',lambda _: '/usr/bin/nice')
    command=['/path with spaces/python','worker.py','--image-size','640','--precision','fp16']
    assert detector_process_command(command)==['/usr/bin/nice','-n','10']+command
    assert command[0]=='/path with spaces/python'


@pytest.mark.parametrize('value',['-1','20','nan','10.0','1; true'])
def test_invalid_priorities_fail_before_a_worker_starts(monkeypatch,value):
    monkeypatch.setenv('GO2W_ML_NICE',value)
    with pytest.raises(ValueError,match='0 to 19'):detector_process_command(['/python'])


def test_missing_scheduler_never_silently_ignores_requested_profile(monkeypatch):
    monkeypatch.setenv('GO2W_ML_NICE','10')
    monkeypatch.setattr('app.detectors.process_priority.shutil.which',lambda _:None)
    with pytest.raises(RuntimeError,match='nice executable'):detector_process_command(['/python'])
