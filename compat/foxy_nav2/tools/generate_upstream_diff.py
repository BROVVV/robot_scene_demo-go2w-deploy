#!/usr/bin/env python3
"""Verify pinned originals and regenerate the review diff from a local Git repo."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import subprocess


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('upstream',type=Path)
    args=parser.parse_args()
    base=Path(__file__).resolve().parents[1]
    lock=json.loads((base/'SOURCE_LOCK.json').read_text())
    mapping=json.loads((base/'SOURCE_MAP.json').read_text())
    changes=[];verified=0
    for original,digest in sorted(lock['upstream_files_sha256'].items()):
        before=subprocess.check_output(['git','show',lock['commit']+':'+original],cwd=args.upstream)
        if hashlib.sha256(before).hexdigest()!=digest:
            raise RuntimeError('Pinned upstream hash mismatch: '+original)
        verified+=1
        target=mapping.get(original)
        if target is None:continue
        after=(base/'src'/target).read_bytes()
        if before==after:continue
        changes.extend(difflib.unified_diff(before.decode().splitlines(keepends=True),
            after.decode().splitlines(keepends=True),fromfile='upstream/'+original,
            tofile='compat/foxy_nav2/src/'+target))
    unknown=set(mapping)-set(lock['upstream_files_sha256'])
    if unknown:raise RuntimeError('Unpinned source mapping: '+str(sorted(unknown)))
    (base/'UPSTREAM_PATCHES.diff').write_text(''.join(changes))
    print(json.dumps({'verified_originals':verified,'mapped_originals':len(mapping),
                     'diff_lines':len(changes)}))


if __name__=='__main__':main()
