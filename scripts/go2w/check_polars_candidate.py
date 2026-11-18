#!/usr/bin/env python3
"""Check a pinned candidate; optional native probe unpacks temporarily, never installs."""
from __future__ import annotations
import argparse
import email
import hashlib
import json
from pathlib import Path
import platform
import struct
import subprocess
import sys
import tempfile
import zipfile

from packaging.specifiers import SpecifierSet
from packaging.requirements import Requirement

ROOT=Path(__file__).resolve().parents[2]


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for data in iter(lambda:stream.read(1048576),b''):h.update(data)
    return h.hexdigest()


def validate_wheel(path,pin):
    path=Path(path)
    if path.name!=pin['wheel'] or digest(path)!=pin['sha256']:
        raise ValueError('candidate wheel filename/hash mismatch')
    with zipfile.ZipFile(path) as z:
        names=z.namelist()
        for name in names:
            p=Path(name)
            if p.is_absolute() or '..' in p.parts:raise ValueError('unsafe wheel member')
        metadata=[n for n in names if n.endswith('.dist-info/METADATA')]
        wheel_info=[n for n in names if n.endswith('.dist-info/WHEEL')]
        if len(metadata)!=1 or len(wheel_info)!=1:raise ValueError('ambiguous wheel metadata')
        m=email.message_from_bytes(z.read(metadata[0]));w=email.message_from_bytes(z.read(wheel_info[0]))
        if m['Name'].lower()!=pin['package'] or m['Version']!=pin['version']:
            raise ValueError('candidate identity mismatch')
        if not SpecifierSet(m['Requires-Python']).contains(pin['python_version']):
            raise ValueError('Python version unsupported')
        if pin['wheel_tag'] not in w.get_all('Tag',[]):raise ValueError('candidate wheel tag mismatch')
        libraries=[]
        for name in names:
            if name.endswith('.so'):
                with z.open(name) as stream:header=stream.read(64)
                if (len(header)<20 or header[:6]!=b'\x7fELF\x02\x01'
                        or struct.unpack_from('<H',header,18)[0]!=pin['elf_machine']):
                    raise ValueError('native library is not 64-bit little-endian aarch64')
                libraries.append(name)
        if not libraries:raise ValueError('candidate native library missing')
    requirement=Requirement(pin['ultralytics_polars_requirement'])
    if not requirement.specifier.contains(pin['version']):raise ValueError('Ultralytics constraint unsatisfied')
    return {'package':pin['package'],'version':pin['version'],'sha256_verified':True,
            'python_compatible_with':pin['python_version'],'wheel_tag':pin['wheel_tag'],
            'aarch64_libraries':libraries,'ultralytics_constraint_satisfied_statically':True,
            'native_probe_executed':False,'environment_modified':False}


def native_guard():
    if (platform.machine()!='aarch64' or sys.version_info[:2]!=(3,8)
            or sys.implementation.name!='cpython'):
        raise ValueError('Native probe requires aarch64 CPython 3.8; no target code executed')


EXPORT_PROBE_CODE=r'''
import ast,csv,io,json,sys,zipfile
from pathlib import Path
candidate=Path(sys.argv[1]).resolve();sys.path.insert(0,str(candidate))
import polars as pl
Path(pl.__file__).resolve().relative_to(candidate)
assert pl.__version__==sys.argv[3]
with zipfile.ZipFile(sys.argv[2]) as z:source=z.read('ultralytics/utils/__init__.py').decode()
classes=[n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name=='DataExportMixin']
assert len(classes)==1
namespace={};exec(compile(ast.Module(body=classes,type_ignores=[]),'pinned_ultralytics_DataExportMixin','exec'),namespace)
class Export(namespace['DataExportMixin']):
    def summary(self,normalize=False,decimals=5):
        return [{'name':'cabinet','confidence':.9,'box':{'x1':1.,'y1':2.}},
                {'name':'chair','confidence':.7,'box':{'x1':3.,'y1':4.}}]
obj=Export();df=obj.to_df();assert df.to_dicts()==obj.summary()
rows=list(csv.DictReader(io.StringIO(obj.to_csv())));assert len(rows)==2 and rows[0]['name']=='cabinet'
text=obj.to_json();json.loads(text);assert 'cabinet' in text and 'chair' in text
print(json.dumps({'polars_version':pl.__version__,'module':str(pl.__file__),
 'dataframe_rows':df.height,'pinned_ultralytics_dataframe_csv_json_passed':True,
 'scope':'standalone native Polars and exact pinned export methods; no Torch/CUDA/YOLOE inference'}))
'''


def native_probe(wheel,ultralytics_wheel,pin):
    native_guard()
    if (Path(ultralytics_wheel).name!=pin['ultralytics_wheel']
            or digest(ultralytics_wheel)!=pin['ultralytics_sha256']):
        raise ValueError('Ultralytics reference wheel filename/hash mismatch')
    with tempfile.TemporaryDirectory(prefix='go2w_polars_candidate_') as directory:
        # validate_wheel checked the pinned archive and every member path.
        with zipfile.ZipFile(wheel) as z:z.extractall(directory)

        import os
        env=dict(os.environ,PYTHONNOUSERSITE='1',POLARS_MAX_THREADS='1')
        p=subprocess.run([sys.executable,'-I','-c',EXPORT_PROBE_CODE,directory,str(ultralytics_wheel),pin['version']],
                         env=env,text=True,capture_output=True,timeout=30)
        if p.returncode:raise RuntimeError('Native probe failed: '+p.stderr[-4000:])
        return json.loads(p.stdout)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wheel',required=True,type=Path)
    parser.add_argument('--pin',type=Path,default=ROOT/'requirements/robot-polars-py38-candidate.json')
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--native-probe',action='store_true')
    parser.add_argument('--ultralytics-wheel',type=Path)
    args=parser.parse_args();report={'passed':False,'environment_modified':False}
    try:
        if args.native_probe:native_guard()
        pin=json.loads(args.pin.read_text());report.update(validate_wheel(args.wheel,pin))
        if args.native_probe:
            if not args.ultralytics_wheel:raise ValueError('--ultralytics-wheel required')
            report['native_result']=native_probe(args.wheel,args.ultralytics_wheel,pin)
            report['native_probe_executed']=True
        report['passed']=True
    except Exception as error:report['error']=type(error).__name__+': '+str(error)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
    return 0 if report['passed'] else 2


if __name__=='__main__':raise SystemExit(main())
