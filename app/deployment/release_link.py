"""Atomic, journaled release-link changes for a future service coordinator.

This pins manifest identity, not source-tree or functional acceptance. Callers
must complete those checks separately and stop the owned application first.
The advisory lock coordinates users of this module; arbitrary writers that
ignore it are outside that coordination contract. No services are invoked.
"""
from contextlib import contextmanager
from dataclasses import asdict, dataclass
import ctypes
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import time
from typing import Optional
import uuid


class LinkRefused(RuntimeError):
    pass


class CommitUncertain(RuntimeError):
    """Inspect actual state before retrying; a rename may have committed."""
    def __init__(self, journal, observed, cause):
        self.journal, self.observed = str(journal), observed
        super().__init__('release-link commit requires inspection: '+str(cause))


@dataclass(frozen=True)
class LinkSnapshot:
    raw_target: Optional[str]
    identity: Optional[tuple]
    release: Optional[str]
    manifest_sha256: Optional[str]


@dataclass(frozen=True)
class LinkReceipt:
    before: LinkSnapshot
    after: LinkSnapshot
    journal: str


def _layout(releases_root, current_link):
    root=Path(releases_root).resolve(strict=True)
    link=Path(current_link).absolute()
    if (not root.is_dir() or link.parent.resolve()!=root.parent
            or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]*',link.name)):
        raise LinkRefused('current link must be beside the release directory')
    return root,root.parent/link.name


def _manifest_pin(root, name):
    if not isinstance(name,str) or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]*',name):
        raise LinkRefused('invalid release name')
    target=root/name
    if target.is_symlink() or not target.is_dir() or target.resolve()!=target:
        raise LinkRefused('release must be a direct directory, not a symlink')
    try:fd=os.open(str(target/'release_manifest.json'),os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
    except OSError as exc:raise LinkRefused('manifest is unavailable or indirect') from exc
    with os.fdopen(fd,'rb') as stream:
        if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
            raise LinkRefused('manifest is not a regular file')
        raw=stream.read()
    if json.loads(raw).get('release')!=name:
        raise LinkRefused('manifest release identity mismatch')
    return hashlib.sha256(raw).hexdigest()


def _snapshot(root, link):
    try:first=link.lstat()
    except FileNotFoundError:return LinkSnapshot(None,None,None,None)
    if not stat.S_ISLNK(first.st_mode):
        raise LinkRefused('refusing to replace a non-symlink current path')
    raw=os.readlink(link);logical=Path(os.path.abspath(link.parent/raw))
    try:target=(link.parent/raw).resolve(strict=True)
    except OSError as exc:raise LinkRefused('current target is unavailable') from exc
    second=link.lstat()
    identity=lambda s:(s.st_dev,s.st_ino,s.st_ctime_ns)
    if identity(first)!=identity(second):raise LinkRefused('current changed during inspection')
    if target.parent!=root or logical!=target or logical.is_symlink():
        raise LinkRefused('current must point directly inside release directory')
    return LinkSnapshot(raw,identity(second),target.name,_manifest_pin(root,target.name))


def inspect_link(releases_root,current_link):
    """Read-only snapshot, including inode/ctime to reject stale ABA intents."""
    return _snapshot(*_layout(releases_root,current_link))


def _fsync_dir(path):
    fd=os.open(str(path),os.O_RDONLY|os.O_DIRECTORY)
    try:os.fsync(fd)
    finally:os.close(fd)


@contextmanager
def _locked(link):
    fd=os.open(str(link.parent/('.'+link.name+'.lock')),
               os.O_CREAT|os.O_RDWR|os.O_CLOEXEC|os.O_NOFOLLOW,0o600)
    try:
        s=os.fstat(fd)
        if not stat.S_ISREG(s.st_mode) or s.st_nlink!=1 or s.st_uid!=os.geteuid():
            raise LinkRefused('unsafe release lock')
        try:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError as exc:raise LinkRefused('another release transaction holds the lock') from exc
        yield
    finally:os.close(fd)


def _atomic_json(path,payload,*,create=False):
    temporary=path.with_name('.'+path.name+'.'+uuid.uuid4().hex+'.tmp')
    created=None
    try:
        fd=os.open(str(temporary),os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600)
        own=os.fstat(fd);created=(own.st_dev,own.st_ino,own.st_ctime_ns)
        with os.fdopen(fd,'w') as stream:
            json.dump(payload,stream,indent=2);stream.write('\n');stream.flush();os.fsync(stream.fileno())
        if create:_commit(temporary,path,was_absent=True)
        else:os.replace(temporary,path)
        _fsync_dir(path.parent)
    finally:
        if created is not None:
            try:
                found=temporary.lstat()
                # Writing the owned temporary changes ctime; device/inode
                # still identify the file whose descriptor we created.
                if (found.st_dev,found.st_ino)==created[:2]:temporary.unlink()
            except FileNotFoundError:pass


def _commit(temporary,link,*,was_absent):
    if not was_absent:
        os.replace(temporary,link)
        return
    # A newly appeared file must not be overwritten during first activation.
    libc=ctypes.CDLL(None,use_errno=True);rename=libc.renameat2
    rename.argtypes=[ctypes.c_int,ctypes.c_char_p,ctypes.c_int,ctypes.c_char_p,ctypes.c_uint]
    rename.restype=ctypes.c_int
    if rename(-100,os.fsencode(temporary),-100,os.fsencode(link),1):
        value=ctypes.get_errno();raise OSError(value,os.strerror(value),str(link))


def _transition(releases_root,current_link,*,expected,release,manifest_sha256,journal_dir,raw_target=None):
    root,link=_layout(releases_root,current_link);directory=Path(journal_dir).resolve(strict=True)
    info=directory.stat()
    if not directory.is_dir() or info.st_uid!=os.geteuid() or info.st_mode&0o022:
        raise LinkRefused('journal directory must be owned and not writable by group/others')
    with _locked(link):
        before=_snapshot(root,link)
        if before!=expected:raise LinkRefused('current snapshot changed; inspect before retrying')
        if release is None:
            if raw_target is not None or manifest_sha256 is not None:raise LinkRefused('absent target cannot carry a pin')
            desired=None
        else:
            if (not isinstance(manifest_sha256,str) or not re.fullmatch(r'[0-9a-f]{64}',manifest_sha256)
                    or _manifest_pin(root,release)!=manifest_sha256):
                raise LinkRefused('candidate manifest pin mismatch')
            desired=raw_target if raw_target is not None else os.path.relpath(root/release,link.parent)
            if (link.parent/desired).resolve(strict=True)!=root/release:raise LinkRefused('target spelling mismatch')
        if before.raw_target==desired:raise LinkRefused('release already selected')
        token=uuid.uuid4().hex;journal=directory/(token+'.json')
        record=dict(transaction=token,wall_time=time.time(),stage='prepared',
                    before=asdict(before),desired_target=desired,candidate_manifest_sha256=manifest_sha256,
                    manifest_identity_only=True,services_changed=False)
        _atomic_json(journal,record,create=True)
        temporary=link.parent/('.'+link.name+'.next.'+token)
        temporary_identity=None
        try:
            if desired is not None:
                os.symlink(desired,temporary);own=temporary.lstat()
                temporary_identity=(own.st_dev,own.st_ino,own.st_ctime_ns)
            if _snapshot(root,link)!=before:raise LinkRefused('current changed before commit')
            try:
                if desired is None:link.unlink()
                else:_commit(temporary,link,was_absent=before.raw_target is None)
                _fsync_dir(link.parent);after=_snapshot(root,link)
                record.update(stage='committed',after=asdict(after));_atomic_json(journal,record)
            except BaseException as exc:
                try:observed=_snapshot(root,link)
                except Exception:observed=None
                raise CommitUncertain(journal,observed,exc) from exc
            return LinkReceipt(before,after,str(journal))
        finally:
            if temporary_identity is not None:
                try:
                    own=temporary.lstat()
                    if (stat.S_ISLNK(own.st_mode)
                            and (own.st_dev,own.st_ino,own.st_ctime_ns)==temporary_identity
                            and os.readlink(temporary)==desired):temporary.unlink()
                except FileNotFoundError:pass


def select_release(releases_root,current_link,*,expected,release,manifest_sha256,journal_dir):
    """Select a pinned release, or restore absence with release/pin both None."""
    return _transition(releases_root,current_link,expected=expected,release=release,
                       manifest_sha256=manifest_sha256,journal_dir=journal_dir)


def rollback_link(releases_root,current_link,receipt,*,journal_dir):
    """Undo only this exact committed link; never overwrite a later change."""
    return _transition(releases_root,current_link,expected=receipt.after,
        release=receipt.before.release,manifest_sha256=receipt.before.manifest_sha256,
        journal_dir=journal_dir,raw_target=receipt.before.raw_target)
