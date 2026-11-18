"""Filesystem-only switching, rollback and uncertain-commit fault injection."""
from pathlib import Path
from types import SimpleNamespace
import errno
import fcntl
import hashlib
import json
import os
import threading

import pytest

from app.deployment import release_link as m


@pytest.fixture
def fs(tmp_path):
    root=tmp_path/'releases';root.mkdir();journal=tmp_path/'journal';journal.mkdir(mode=0o700)
    pins={}
    for name in ['old','new']:
        folder=root/name;folder.mkdir();raw=json.dumps({'release':name,'files':{},'symlinks':{}}).encode()
        (folder/'release_manifest.json').write_bytes(raw);pins[name]=hashlib.sha256(raw).hexdigest()
    return SimpleNamespace(root=root,link=tmp_path/'current',journal=journal,pins=pins)


def choose(fs,name,expected=None):
    return m.select_release(fs.root,fs.link,
        expected=m.inspect_link(fs.root,fs.link) if expected is None else expected,
        release=name,manifest_sha256=fs.pins[name] if name is not None else None,journal_dir=fs.journal)


def test_switch_rollback_and_restore_original_absence(fs):
    empty=m.inspect_link(fs.root,fs.link);a=choose(fs,'old',empty)
    assert fs.link.resolve()==fs.root/'old'
    b=choose(fs,'new');undo=m.rollback_link(fs.root,fs.link,b,journal_dir=fs.journal)
    assert fs.link.resolve()==fs.root/'old'
    # The old receipt is stale after an intervening switch and rollback.
    with pytest.raises(m.LinkRefused,match='snapshot changed'):m.rollback_link(fs.root,fs.link,a,journal_dir=fs.journal)
    removed=choose(fs,None)
    assert removed.after==empty and not os.path.lexists(fs.link)
    for receipt in [a,b,undo,removed]:
        v=json.loads(Path(receipt.journal).read_text())
        assert v['stage']=='committed' and v['services_changed'] is False
        assert v['manifest_identity_only'] is True


def test_rollback_preserves_original_absolute_link_spelling(fs):
    fs.link.symlink_to(fs.root/'old');original=os.readlink(fs.link)
    receipt=choose(fs,'new');m.rollback_link(fs.root,fs.link,receipt,journal_dir=fs.journal)
    assert os.readlink(fs.link)==original


def test_regular_current_file_and_outside_link_are_never_overwritten(fs):
    fs.link.write_text('user data')
    with pytest.raises(m.LinkRefused,match='non-symlink'):choose(fs,'new')
    assert fs.link.read_text()=='user data';fs.link.unlink();fs.link.symlink_to(fs.journal)
    with pytest.raises(m.LinkRefused,match='directly'):m.inspect_link(fs.root,fs.link)
    assert fs.link.is_symlink()


@pytest.mark.parametrize('name',['../new','/tmp/new','new/child'])
def test_path_escape_is_refused(fs,name):
    with pytest.raises(m.LinkRefused):m.select_release(fs.root,fs.link,
        expected=m.inspect_link(fs.root,fs.link),release=name,
        manifest_sha256='0'*64,journal_dir=fs.journal)
    assert not os.path.lexists(fs.link)


def test_symlink_release_and_changed_manifest_pin_are_refused(fs):
    (fs.root/'alias').symlink_to(fs.root/'new')
    with pytest.raises(m.LinkRefused):m.select_release(fs.root,fs.link,
        expected=m.inspect_link(fs.root,fs.link),release='alias',
        manifest_sha256=fs.pins['new'],journal_dir=fs.journal)
    (fs.root/'new/release_manifest.json').write_text('{"release":"new","changed":true}')
    with pytest.raises(m.LinkRefused,match='pin mismatch'):choose(fs,'new')
    assert not os.path.lexists(fs.link)


def test_same_spelling_recreated_link_is_a_stale_snapshot(fs):
    choose(fs,'old');before=m.inspect_link(fs.root,fs.link);raw=os.readlink(fs.link)
    fs.link.unlink();fs.link.symlink_to(raw)
    with pytest.raises(m.LinkRefused,match='snapshot changed'):choose(fs,'new',before)
    assert fs.link.resolve()==fs.root/'old'


def test_lock_holder_prevents_a_second_transaction(fs):
    fd=os.open(str(fs.link.parent/'.current.lock'),os.O_CREAT|os.O_RDWR,0o600)
    try:
        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        with pytest.raises(m.LinkRefused,match='holds the lock'):choose(fs,'old')
    finally:os.close(fd)
    assert not os.path.lexists(fs.link)


def test_first_activation_never_overwrites_a_late_foreign_file(fs,monkeypatch):
    original=m._commit
    def competing(temporary,link,*,was_absent):
        if link==fs.link:link.write_text('foreign file appeared')
        return original(temporary,link,was_absent=was_absent)
    monkeypatch.setattr(m,'_commit',competing)
    with pytest.raises(m.CommitUncertain):choose(fs,'old')
    assert fs.link.read_text()=='foreign file appeared'


def test_journal_failure_before_commit_keeps_old_link(fs,monkeypatch):
    choose(fs,'old');before=m.inspect_link(fs.root,fs.link)
    def fail(*args,**kwargs):raise OSError(errno.ENOSPC,'injected journal full')
    monkeypatch.setattr(m,'_atomic_json',fail)
    with pytest.raises(OSError,match='journal full'):choose(fs,'new')
    assert m.inspect_link(fs.root,fs.link)==before


def test_directory_sync_failure_does_not_claim_link_unchanged(fs,monkeypatch):
    choose(fs,'old');before=m.inspect_link(fs.root,fs.link);original=m._fsync_dir
    def fail(path):
        if path==fs.link.parent:raise OSError(errno.EIO,'injected directory sync')
        original(path)
    monkeypatch.setattr(m,'_fsync_dir',fail)
    with pytest.raises(m.CommitUncertain) as caught:choose(fs,'new')
    assert fs.link.resolve()==fs.root/'new'
    assert caught.value.observed.release=='new'
    assert json.loads(Path(caught.value.journal).read_text())['stage']=='prepared'
    with pytest.raises(m.LinkRefused,match='snapshot changed'):choose(fs,'old',before)


def test_readers_never_see_a_gap_while_existing_link_is_replaced(fs):
    choose(fs,'old');stop=threading.Event();errors=[];seen=[]
    def reader():
        while not stop.is_set():
            try:seen.append(os.readlink(fs.link))
            except OSError as exc:errors.append(exc)
    thread=threading.Thread(target=reader);thread.start()
    try:
        for i in range(20):choose(fs,'new' if i%2==0 else 'old')
    finally:stop.set();thread.join(timeout=2)
    assert not thread.is_alive() and seen and not errors
    assert set(seen)<= {'releases/old','releases/new'}
