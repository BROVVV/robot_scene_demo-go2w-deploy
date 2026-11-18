"""Static checks must not execute candidate code or change installed packages."""
import hashlib
import importlib.util
from pathlib import Path
import struct
import sys
import zipfile
import pytest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('polars_candidate_check',ROOT/'scripts/go2w/check_polars_candidate.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def wheel(tmp_path,*,machine=183,tag='cp38-abi3-manylinux_2_24_aarch64',requires='>=3.8',version='0.20.31',library=True,unsafe=False):
    path=tmp_path/'polars-0.20.31-cp38-abi3-manylinux_2_24_aarch64.whl'
    header=bytearray(64);header[:6]=b'\x7fELF\x02\x01';struct.pack_into('<H',header,18,machine)
    with zipfile.ZipFile(path,'w') as z:
        z.writestr('polars-0.20.31.dist-info/METADATA',f'Name: polars\nVersion: {version}\nRequires-Python: {requires}\n')
        z.writestr('polars-0.20.31.dist-info/WHEEL',f'Wheel-Version: 1.0\nTag: {tag}\n')
        if library:z.writestr('polars/polars.abi3.so',header)
        if unsafe:z.writestr('../outside.txt','no extraction allowed')
    pin={'wheel':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
         'package':'polars','version':'0.20.31','python_version':'3.8.10',
         'wheel_tag':'cp38-abi3-manylinux_2_24_aarch64','elf_machine':183,
         'ultralytics_polars_requirement':'polars>=0.20.0'}
    return path,pin


def test_static_check_does_not_import_or_install_the_target(tmp_path):
    path,pin=wheel(tmp_path);before=sys.modules.get('polars')
    report=module.validate_wheel(path,pin)
    assert report['sha256_verified'] and not report['native_probe_executed']
    assert report['environment_modified'] is False
    assert sys.modules.get('polars') is before


@pytest.mark.parametrize('changes',[
    {'machine':62},{'tag':'cp39-abi3-manylinux_2_24_aarch64'},
    {'requires':'>=3.9'},{'version':'0.19.12'},{'library':False},{'unsafe':True},
])
def test_wrong_target_or_unsafe_archive_is_rejected(tmp_path,changes):
    path,pin=wheel(tmp_path,**changes)
    with pytest.raises(ValueError):module.validate_wheel(path,pin)
    assert not (tmp_path.parent/'outside.txt').exists()


def test_tampering_is_rejected_before_zip_metadata_is_read(tmp_path):
    path,pin=wheel(tmp_path);path.write_bytes(b'not the pinned wheel')
    with pytest.raises(ValueError,match='hash mismatch'):module.validate_wheel(path,pin)


def test_metadata_requirement_must_accept_the_candidate_version(tmp_path):
    path,pin=wheel(tmp_path);pin['ultralytics_polars_requirement']='polars>=1.0'
    with pytest.raises(ValueError,match='constraint'):module.validate_wheel(path,pin)


def test_native_execution_requires_the_real_target_host():
    if (module.platform.machine()=='aarch64' and sys.version_info[:2]==(3,8)
            and sys.implementation.name=='cpython'):
        pytest.skip('This test checks refusal on a non-target host')
    with pytest.raises(ValueError,match='no target code executed'):module.native_guard()


def test_native_worker_syntax_is_compatible_with_python38():
    import ast
    ast.parse(module.EXPORT_PROBE_CODE,feature_version=(3,8))
