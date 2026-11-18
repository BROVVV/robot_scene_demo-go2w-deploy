"""计划书 §P4-6：对 saved map 的授权必须绑在**真实证据 + 正确地图**上。

原来 ``verify(..., require_authorized=True)`` 只查两件事：``navigation_authorized
is True`` 和 ``physical_validation_evidence`` 非空。于是

    {"navigation_authorized": true, "physical_validation_evidence": ["x"]}

就能过 —— 空壳证据。这里钉住的判据是三条：证据要自报 schema 且字段齐、证据里的
``map_sha256`` 要等于**这张图**的 sha256、``navigation_authorized`` 由证据**推导**
而不是由它自己怎么写决定。

不碰真机、不碰 ROS：整份 bundle 是这里造的。
"""
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import signal
import time
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'go2w_save_navigation_map', ROOT / 'scripts/go2w/save_navigation_map.py')
snm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(snm)


@pytest.mark.parametrize('distro,timeout', [('foxy', '10000'), ('humble', '10.0')])
def test_map_save_timeout_uses_native_distro_units(monkeypatch, distro, timeout):
    monkeypatch.setenv('ROS_DISTRO', distro)
    command = snm.map_saver_command('/tmp/map')
    assert 'save_map_timeout:='+timeout in command
    assert 'map_subscribe_transient_local:=true' in command
    assert 'free_thresh_default:=0.19' in command


@pytest.mark.parametrize('absolute',[True,False])
def test_new_map_image_reference_is_portable_before_hashing(tmp_path,absolute):
    import yaml
    image=tmp_path/'map.pgm';image.write_bytes(b'P5\n1 1\n255\n\xcd')
    path=tmp_path/'map.yaml'
    metadata={'image':str(image) if absolute else 'map.pgm','mode':'trinary',
              'resolution':.05,'origin':[-.78,-2.49,0.], 'free_thresh':.19,'occupied_thresh':.65}
    path.write_text(yaml.safe_dump(metadata))
    image_bytes=image.read_bytes()
    snm.normalize_saved_image_path(path)
    actual=yaml.safe_load(path.read_text())
    assert actual==dict(metadata,image='map.pgm')
    assert image.read_bytes()==image_bytes
    snm.validate_saved_thresholds(path)


def test_saved_image_outside_the_bundle_is_rejected_without_rewriting(tmp_path):
    import yaml
    other=tmp_path/'outside';other.mkdir();image=other/'map.pgm';image.write_bytes(b'x')
    bundle=tmp_path/'bundle';bundle.mkdir();path=bundle/'map.yaml'
    path.write_text(yaml.safe_dump({'image':str(image)}));before=path.read_bytes()
    with pytest.raises(ValueError,match='outside its bundle'):snm.normalize_saved_image_path(path)
    assert path.read_bytes()==before


@pytest.mark.parametrize('free,accepted', [(.19,True),(.25,False)])
def test_written_thresholds_must_not_reload_unknown_as_free(tmp_path, free, accepted):
    path=tmp_path/'map.yaml'
    path.write_text(f'mode: trinary\nfree_thresh: {free}\noccupied_thresh: 0.65\n')
    if accepted: snm.validate_saved_thresholds(path)
    else:
        with pytest.raises(ValueError,match='preserve unknown'):
            snm.validate_saved_thresholds(path)


@pytest.mark.parametrize('missing', [None,'map.data','map.posegraph'])
def test_empty_foxy_save_response_requires_both_new_artifacts(tmp_path, monkeypatch, missing):
    monkeypatch.setenv('ROS_DISTRO','foxy')
    response=SimpleNamespace(get_fields_and_field_types=lambda:{})
    for name in ('map.data','map.posegraph'):
        if name!=missing:(tmp_path/name).write_bytes(b'actual output')
    if missing:
        with pytest.raises(RuntimeError,match='both artifacts'):
            snm.validate_posegraph_response(response,tmp_path)
    else: snm.validate_posegraph_response(response,tmp_path)


def test_result_code_failure_is_not_hidden_by_existing_artifacts(tmp_path):
    for name in ('map.data','map.posegraph'):(tmp_path/name).write_bytes(b'output')
    with pytest.raises(RuntimeError,match='serialization failed'):
        snm.validate_posegraph_response(SimpleNamespace(result=1),tmp_path)


@pytest.mark.parametrize('interrupt', [False, True])
def test_map_save_timeout_cleans_wrapper_and_its_native_child(tmp_path, monkeypatch, interrupt):
    pidfile = tmp_path/'native-child.pid'
    child = ('import os,signal,time,pathlib; signal.signal(signal.SIGTERM,signal.SIG_IGN); '
             f'pathlib.Path({str(pidfile)!r}).write_text(str(os.getpid())); time.sleep(30)')
    wrapper = f'import subprocess,sys,time; subprocess.Popen([sys.executable,"-c",{child!r}]); time.sleep(30)'
    original_popen = subprocess.Popen

    def ready_popen(*args, **kwargs):
        # Never risk signalling this test runner's own process group.
        assert kwargs.get('start_new_session') is True
        process = original_popen(*args, **kwargs)
        deadline = time.monotonic()+5
        while not pidfile.exists() and time.monotonic()<deadline:
            time.sleep(.02)
        if interrupt:
            original_wait = process.wait
            pending = [True]
            def interrupted_wait(*args, **kwargs):
                if pending[0]:
                    pending[0] = False
                    raise KeyboardInterrupt()
                return original_wait(*args, **kwargs)
            process.wait = interrupted_wait
        return process

    monkeypatch.setattr(snm.subprocess, 'Popen', ready_popen)
    child_pid = None
    try:
        with (tmp_path/'map.log').open('w') as log:
            with pytest.raises(KeyboardInterrupt if interrupt else subprocess.TimeoutExpired):
                snm.run_owned_command([sys.executable, '-c', wrapper], log=log, timeout=.1)
        child_pid = int(pidfile.read_text())
        proc = Path('/proc')/str(child_pid)
        deadline = time.monotonic()+2
        while proc.exists() and time.monotonic()<deadline:
            if (proc/'stat').read_text().rsplit(')',1)[1].split()[0] == 'Z':
                break
            time.sleep(.02)
        assert not proc.exists() or (proc/'stat').read_text().rsplit(')',1)[1].split()[0] == 'Z'
    finally:
        if child_pid is None and pidfile.exists(): child_pid=int(pidfile.read_text())
        if child_pid is not None:
            try: os.kill(child_pid, signal.SIGKILL)
            except ProcessLookupError: pass


def bundle(root: Path, *, evidence=(), authorized=False, initial_pose=None,
           origin_x=0.0) -> Path:
    # origin_x 用来造"另一张图"。注意绑定的锚是 **map.yaml 的 sha256**，所以差异
    # 必须落在 yaml 里：光改 pgm 不会让 map.yaml 的哈希变，用例就成了同义反复。
    root.mkdir(parents=True, exist_ok=True)
    (root / 'map.pgm').write_bytes(b'P5\n2 2\n255\n' + bytes([0, 0, 100, 255]))
    (root / 'map.posegraph').write_bytes(b'{"posegraph": []}')
    (root / 'map.data').write_bytes(b'{"data": []}')
    (root / 'map.yaml').write_text(
        f'image: map.pgm\nresolution: 0.05\norigin: [{origin_x}, 0, 0]\n')
    manifest = {
        'schema_version': 'go2w_navigation_map_v1', 'frame': 'map',
        'navigation_authorized': authorized, 'physical_validation_evidence': list(evidence),
        'initial_pose': initial_pose,
        'sha256': {name: snm.digest(root / name) for name in
                   ('map.yaml', 'map.pgm', 'map.posegraph', 'map.data')},
    }
    (root / 'provenance.json').write_text(json.dumps(manifest))
    return root / 'map.yaml'


def evidence_for(map_yaml: Path, **overrides) -> dict:
    """按**已经落盘的** bundle 造一条证据（sha 取自它自己的 provenance）。"""
    manifest = json.loads((map_yaml.parent / 'provenance.json').read_text())
    entry = {'schema_version': snm.EVIDENCE_SCHEMA, 'kind': 'real_robot_short_distance',
             'map_bundle_sha256': snm.bundle_digest(manifest), 'recorded_at': 1792000000.0,
             'passed': True,
             'measurements': {'steps': 3, 'max_step_m': 0.2, 'returned_rmse_m': 0.04}}
    entry.update(overrides)
    return entry


def with_evidence(map_yaml: Path, *entries) -> Path:
    path = map_yaml.parent / 'provenance.json'
    manifest = json.loads(path.read_text())
    manifest['physical_validation_evidence'] = list(entries)
    manifest['navigation_authorized'] = True
    path.write_text(json.dumps(manifest))
    return map_yaml


def test_a_fresh_save_verifies_but_grants_nothing(tmp_path):
    map_yaml = bundle(tmp_path)
    assert snm.verify(map_yaml)['navigation_authorized'] is False
    with pytest.raises(ValueError, match='no completed physical validation'):
        snm.verify(map_yaml, require_authorized=True)


def test_stale_epoch_archive_requires_mapping_geometry_and_disabled_execution():
    report = {
        'audit_mode': 'mapping',
        'evidence': {'map': {'valid': True}},
        'mapping_frontend': {'configuration_valid': True},
        'sensor_geometry': {'valid': True, 'profile_sha256': 'a' * 64},
        'floor_visibility': {'valid': True},
        'physical_execution': {'verdict': 'disabled'},
        # The purpose of this contract is to allow archival before Nav2 starts.
        'healthy_for_plan': False,
        'plan_blocking_errors': ['lifecycle:planner_server'],
    }
    assert snm.stale_epoch_archive_blockers(
        report, sensor_profile_sha256='a' * 64) == []
    for key, mutate in (
        ('map', lambda d: d['evidence']['map'].update(valid=False)),
        ('mapping_frontend', lambda d: d['mapping_frontend'].update(configuration_valid=False)),
        ('sensor_geometry', lambda d: d['sensor_geometry'].update(profile_sha256='b' * 64)),
        ('floor_visibility', lambda d: d['floor_visibility'].update(valid=False)),
        ('physical_execution', lambda d: d['physical_execution'].update(verdict='unknown')),
    ):
        import copy
        bad = copy.deepcopy(report)
        mutate(bad)
        assert key in snm.stale_epoch_archive_blockers(
            bad, sensor_profile_sha256='a' * 64)


def test_stale_epoch_flag_cannot_be_used_for_verify(tmp_path):
    map_yaml = bundle(tmp_path)
    result = subprocess.run(
        [sys.executable, str(ROOT / 'scripts/go2w/save_navigation_map.py'),
         '--verify', str(map_yaml), '--stale-epoch-archive'],
        capture_output=True, text=True)
    assert result.returncode == 2
    assert 'requires --output-dir' in result.stderr


def test_geometry_change_requires_new_map_validation(tmp_path):
    map_yaml=bundle(tmp_path)
    profile=tmp_path/'sensor_profile.yaml';profile.write_text('geometry: recorded-example\n')
    sha=snm.digest(profile)
    manifest=json.loads((tmp_path/'provenance.json').read_text())
    manifest['navigation_geometry']={'sensor_profile_sha256':sha}
    (tmp_path/'provenance.json').write_text(json.dumps(manifest))
    assert snm.verify(map_yaml,sensor_profile_sha256=sha)
    with pytest.raises(ValueError,match='differs'):
        snm.verify(map_yaml,sensor_profile_sha256='0'*64)
    with_evidence(map_yaml,evidence_for(map_yaml))
    with pytest.raises(ValueError,match='without valid evidence'):
        snm.verify(map_yaml,require_authorized=True)
    with_evidence(map_yaml,evidence_for(map_yaml,sensor_profile_sha256=sha))
    assert snm.verify(map_yaml,True,sha)
    profile.write_text('geometry: changed\n')
    with pytest.raises(ValueError,match='sensor geometry'):
        snm.verify(map_yaml)


def test_hand_written_shell_evidence_is_refused(tmp_path):
    """旧写法（一条 true + 一个字符串）必须被拒 —— 这是 P4-6 点名的那条路。"""
    map_yaml = bundle(tmp_path, authorized=True, evidence=['x'])
    assert snm.authorization(json.loads((tmp_path / 'provenance.json').read_text())) == {
        'navigation_authorized': False, 'evidence': [], 'declared_true_without_evidence': True}
    with pytest.raises(ValueError, match='without valid evidence'):
        snm.verify(map_yaml, require_authorized=True)


def test_evidence_bound_to_another_map_is_refused(tmp_path):
    """证据的 sha256 不是这张图的：那是"另一张图的验收"被贴过来了。

    所以这里必须让另一张图**真的不一样**（tag），不然两张图字节相同、sha 相同，
    这条用例会变成永远通过的同义反复。
    """
    map_yaml = bundle(tmp_path)
    other = bundle(tmp_path / 'other', origin_x=5.0)
    assert snm.digest(other) != snm.digest(map_yaml)
    with_evidence(map_yaml, evidence_for(other))
    assert snm.bound_evidence(json.loads((tmp_path / 'provenance.json').read_text())) == []
    with pytest.raises(ValueError, match='without valid evidence'):
        snm.verify(map_yaml, require_authorized=True)


def test_evidence_bound_to_this_map_grants_and_is_reported(tmp_path):
    map_yaml = with_evidence(bundle(tmp_path), evidence_for(bundle(tmp_path)))
    manifest = snm.verify(map_yaml, require_authorized=True)
    assert snm.authorization(manifest)['navigation_authorized'] is True
    assert len(snm.authorization(manifest)['evidence']) == 1


@pytest.mark.parametrize('overrides', [
    {'schema_version': 'something_else'}, {'schema_version': None},
    {'map_bundle_sha256': None}, {'recorded_at': 'yesterday'},
    {'recorded_at': float('nan')}, {'recorded_at': True}, {'passed': False}, {'passed': None},
    {'measurements': {'result': 'failed'}}, {'measurements': {'error': float('inf')}},
    {'measurements': {'ok': True}}, {'kind': 'failed_attempt'},
    {'measurements': {}}, {'measurements': None}, {'kind': None},
])
def test_incomplete_evidence_never_grants(tmp_path, overrides):
    map_yaml = bundle(tmp_path)
    with_evidence(map_yaml, evidence_for(map_yaml, **overrides))
    with pytest.raises(ValueError, match='without valid evidence'):
        snm.verify(map_yaml, require_authorized=True)


def test_declared_true_with_valid_evidence_is_consistent(tmp_path):
    """推导出来的授权与写下来的值一致时不算矛盾（这样现场好读）。"""
    map_yaml = bundle(tmp_path)
    with_evidence(map_yaml, evidence_for(map_yaml))
    grants = snm.authorization(json.loads((tmp_path / 'provenance.json').read_text()))
    assert grants['declared_true_without_evidence'] is False


def test_hash_mismatch_is_still_fatal(tmp_path):
    """证据绑定不能把原有的 hash 校验挤掉。"""
    map_yaml = bundle(tmp_path)
    with_evidence(map_yaml, evidence_for(map_yaml))
    (tmp_path / 'map.pgm').write_bytes(b'tampered')
    with pytest.raises(ValueError, match='hash mismatch'):
        snm.verify(map_yaml, require_authorized=True)


def test_trinary_map_cannot_reload_standard_unknown_shade_as_free(tmp_path):
    map_yaml = bundle(tmp_path)
    map_yaml.write_text(
        'image: map.pgm\nmode: trinary\nresolution: 0.05\norigin: [0, 0, 0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    path = tmp_path / 'provenance.json'
    manifest = json.loads(path.read_text())
    manifest['sha256']['map.yaml'] = snm.digest(map_yaml)
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='unknown cells as free'):
        snm.verify(map_yaml)

    map_yaml.write_text(map_yaml.read_text().replace('free_thresh: 0.25', 'free_thresh: 0.19'))
    manifest['sha256']['map.yaml'] = snm.digest(map_yaml)
    path.write_text(json.dumps(manifest))
    assert snm.verify(map_yaml)


def test_changed_image_with_updated_manifest_cannot_reuse_old_evidence(tmp_path):
    map_yaml = bundle(tmp_path)
    with_evidence(map_yaml, evidence_for(map_yaml))
    (tmp_path / 'map.pgm').write_bytes(b'P5\n2 2\n255\n' + bytes([255]*4))
    path = tmp_path / 'provenance.json'
    manifest = json.loads(path.read_text())
    manifest['sha256']['map.pgm'] = snm.digest(tmp_path / 'map.pgm')
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='without valid evidence'):
        snm.verify(map_yaml, require_authorized=True)


def test_initial_pose_is_not_required_to_verify_an_older_bundle(tmp_path):
    """老 bundle 没有 initial_pose 也要能 verify（否则会当成图坏了）。"""
    map_yaml = bundle(tmp_path)
    assert snm.verify(map_yaml)['initial_pose'] is None


# ---------------------------------------------------------------------------
# §P4-5：初始位姿要**真的喂进 AMCL**，不是只在配置模板里躺着
# ---------------------------------------------------------------------------

LAUNCH = ROOT / 'ros2_ws/src/robot_scene_nav_bringup/launch/localization_2d.launch.py'
ENTRY = ROOT / 'scripts/go2w/start_2d_localization.sh'


def test_entry_point_only_passes_arguments_the_launch_declares():
    """入口脚本传的参数名必须都在 launch 里声明过。

    名字打错在 ROS 2 里不会响亮地报错，而是那一路静默拿默认值 —— 对初始位姿来说
    就等于悄悄退回 (0,0,0)。这条把两边的名字钉在一起。
    """
    declared = set(re.findall(r"DeclareLaunchArgument\(\s*'([^']+)'", LAUNCH.read_text()))
    passed = set(re.findall(r'\b([a-z_]+):=', ENTRY.read_text()))
    assert passed, '一个 launch 参数都没解析到：正则或脚本结构变了'
    assert not passed - declared, '入口传了 launch 没声明的参数：%s' % sorted(passed - declared)
    assert {'mode', 'map', 'map_authorized', 'use_initial_pose',
            'initial_pose_x', 'initial_pose_y', 'initial_pose_yaw'} <= passed


def test_amcl_is_actually_fed_the_initial_pose():
    """amcl 节点那一段必须把这四个参数传下去（配置模板 + 无人喂 = 死模板）。"""
    text = LAUNCH.read_text()
    for key in ("'set_initial_pose'", "'initial_pose.x'", "'initial_pose.y'",
                "'initial_pose.yaw'"):
        assert key in text, key


def test_initial_pose_cli_prints_shell_assignments(tmp_path):
    map_yaml = bundle(tmp_path, initial_pose={'x': 1.25, 'y': -3.5, 'yaw': 0.75, 'frame': 'map'})
    result = subprocess.run([sys.executable, str(ROOT / 'scripts/go2w/save_navigation_map.py'),
                             '--initial-pose', str(map_yaml)],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert dict(line.split('=') for line in result.stdout.split()) == {
        'initial_pose_x': '1.25', 'initial_pose_y': '-3.5', 'initial_pose_yaw': '0.75'}


def test_initial_pose_cli_refuses_a_bundle_without_one(tmp_path):
    """没有初始位姿就退出 2 —— 定位模式宁可起不来，也不用 (0,0,0) 顶上。"""
    for absent in (None, {}, {'x': 0.0, 'y': None, 'yaw': 0.0}):
        root = tmp_path / repr(absent)
        result = subprocess.run(
            [sys.executable, str(ROOT / 'scripts/go2w/save_navigation_map.py'),
             '--initial-pose', str(bundle(root, initial_pose=absent))],
            capture_output=True, text=True)
        assert result.returncode == 2, result.stdout
        assert 'no recorded initial pose' in result.stderr


@pytest.mark.parametrize('values', [[], ['nan', '0', '0'], ['0', 'inf', '0'], ['bad', '0', '0']])
def test_localization_entry_refuses_missing_or_nonfinite_current_pose_before_ros(values):
    command = ['bash', str(ENTRY), '--mode', 'localization', '--map', '/unused/map.yaml']
    if values: command += ['--initial-pose', *values]
    result = subprocess.run(command, capture_output=True, text=True, timeout=5)
    assert result.returncode == 2
    assert 'initial' in result.stderr.lower()


@pytest.mark.parametrize('x,expected', [('1', 0), ('2', 2)])
def test_existing_amcl_pose_is_reused_only_for_the_same_current_estimate(tmp_path, x, expected):
    import shutil
    root = tmp_path / 'project'
    scripts = root / 'scripts/go2w';scripts.mkdir(parents=True)
    entry = scripts / ENTRY.name;shutil.copyfile(ENTRY, entry)
    (scripts / 'setup_environment.sh').write_text(':\n')
    # This test isolates argv identity matching. Epoch validation has its own
    # suite and must not turn the copied mini-project into a missing-file test.
    (scripts / 'localization_epoch.py').write_text('raise SystemExit(0)\n')
    runtime = root / 'outputs/autonomous_search/runtime';runtime.mkdir(parents=True)
    # An inert sleep carries a test argv; no ROS participant or physical arm.
    argv = ['ros2', 'launch', 'robot_scene_nav_bringup', 'localization_2d.launch.py', 'mode:=localization',
            'map:=/test/map.yaml', 'use_initial_pose:=true', 'initial_pose_x:=1.0', 'initial_pose_y:=0.0', 'initial_pose_yaw:=0.0']
    child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)', *argv])
    try:
        (runtime / 'localization_2d.pid').write_text(str(child.pid))
        result = subprocess.run(['bash', str(entry), '--mode', 'localization', '--map', '/test/map.yaml',
                                 '--initial-pose', x, '0', '0'], capture_output=True, text=True, timeout=5)
        assert result.returncode == expected, result.stdout+result.stderr
        if expected: assert 'different initial pose' in result.stderr
        else: assert 'Reusing' in result.stdout
    finally:
        child.terminate();child.wait(timeout=5)


@pytest.mark.parametrize('active_map', ['/test/map.yaml.other', '/test/map.yaml other'])
def test_reuse_does_not_confuse_a_map_path_prefix_with_the_requested_map(tmp_path, active_map):
    import shutil
    root = tmp_path / 'project';scripts = root / 'scripts/go2w';scripts.mkdir(parents=True)
    entry = scripts / ENTRY.name;shutil.copyfile(ENTRY, entry)
    (scripts / 'setup_environment.sh').write_text(':\n')
    (scripts / 'localization_epoch.py').write_text('raise SystemExit(0)\n')
    runtime = root / 'outputs/autonomous_search/runtime';runtime.mkdir(parents=True)
    argv = ['ros2', 'launch', 'robot_scene_nav_bringup', 'localization_2d.launch.py', 'mode:=localization',
            'map:='+active_map, 'initial_pose_x:=1.0', 'initial_pose_y:=0.0', 'initial_pose_yaw:=0.0']
    child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)', *argv])
    try:
        (runtime / 'localization_2d.pid').write_text(str(child.pid))
        result = subprocess.run(['bash', str(entry), '--mode', 'localization', '--map', '/test/map.yaml',
                                 '--initial-pose', '1', '0', '0'], capture_output=True, text=True, timeout=5)
        assert result.returncode == 2
        assert 'different saved map' in result.stderr
    finally:
        child.terminate();child.wait(timeout=5)
