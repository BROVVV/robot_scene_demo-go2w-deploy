"""Review bundles must be bounded, non-root for capture and restore the original."""
import configparser
import hashlib
import importlib.util
import json
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('camera_trial_guard',ROOT/'scripts/go2w/prepare_camera_trial_guard.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def parse(text):
    c=configparser.ConfigParser(interpolation=None);c.read_string(text);return c


def test_deadline_root_restore_and_nonroot_capture_contract():
    files,meta=module.bundle('20261109_06','test01','a'*64)
    review=parse(files[meta['review_unit']]);restore=parse(files[meta['restore_unit']]);timer=parse(files[meta['restore_timer']])
    assert review['Service']['User']=='unitree'
    assert review['Service']['RuntimeMaxSec']=='90s'
    assert review['Service']['TimeoutStopSec']=='8s'
    assert review['Service']['KillMode']=='control-group'
    assert '--width 1280 --height 720 --fps 30' in review['Service']['ExecStart']
    assert '--encoding-mode on-demand' in review['Service']['ExecStart']
    assert review['Service']['ExecStopPost']=='+/usr/bin/systemctl --no-block start '+meta['restore_unit']
    assert restore['Service']['Type']=='oneshot'
    assert restore['Service']['ExecStartPre']=='-/usr/bin/systemctl stop '+meta['review_unit']
    assert restore['Service']['ExecStart']=='/usr/bin/systemctl start realsense-stream.service'
    assert 'User' not in restore['Service']  # Root restores after the capture user exits.
    assert timer['Timer']['OnActiveSec']=='120s'
    assert timer['Timer']['Unit']==meta['restore_unit']
    assert all('Install' not in parse(text) for name,text in files.items() if name.endswith(('.service','.timer')))
    assert meta['motion_commands_included'] is False
    for name,sha in meta['files'].items():assert hashlib.sha256(files[name].encode()).hexdigest()==sha
    assert hashlib.sha256(files['activation-checklist.md'].encode()).hexdigest()==meta['activation_checklist_sha256']


@pytest.mark.parametrize('release,tag',[
    ('../other','test'),('20261109_06\nUser=root','test'),
    ('20261109_06','../bad'),('20261109_06','x\nExecStart=/bin/sh'),
])
def test_unit_names_and_paths_cannot_inject_commands(release,tag):
    with pytest.raises(ValueError):module.bundle(release,tag,'a'*64)


def test_prepare_requires_matching_frozen_camera_source(tmp_path):
    root=tmp_path/'20261109_06';(root/'scripts/go2w').mkdir(parents=True)
    camera=root/module.CAMERA_REL;camera.write_text('test source')
    manifest={'release':root.name,'files':{module.CAMERA_REL:'0'*64}}
    (root/'release_manifest.json').write_text(json.dumps(manifest));output=tmp_path/'out'
    with pytest.raises(ValueError):module.prepare(root,'test',output)
    assert not output.exists()
    manifest['files'][module.CAMERA_REL]=hashlib.sha256(camera.read_bytes()).hexdigest()
    (root/'release_manifest.json').write_text(json.dumps(manifest));meta=module.prepare(root,'test',output)
    assert json.loads((output/'bundle.json').read_text())==meta
    with pytest.raises(FileExistsError):module.prepare(root,'test',output)
