#!/usr/bin/env python3
"""Verify both pinned controller sources and write the separate Foxy review diff."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('upstream', type=Path)
    args = parser.parse_args()
    base = Path(__file__).resolve().parents[1]
    lock = json.loads((base/'CONTROLLER_SOURCE_LOCK.json').read_text())
    if set(lock['base_files_sha256']) != set(lock['base_to_adapted']):
        raise RuntimeError('Controller base inventory and adapted mapping differ')
    changes = []
    for original, wanted in sorted(lock['base_files_sha256'].items()):
        raw = subprocess.check_output(
            ['git', 'show', lock['foxy_commit']+':'+original], cwd=args.upstream)
        if hashlib.sha256(raw).hexdigest() != wanted:
            raise RuntimeError('Foxy source hash mismatch: '+original)
        target = lock['base_to_adapted'][original]
        after = (base/'src/go2w_nav2_controller_compat'/target).read_bytes()
        changes.extend(difflib.unified_diff(
            raw.decode().splitlines(keepends=True), after.decode().splitlines(keepends=True),
            fromfile='foxy-0.4.7/'+original,
            tofile='compat/foxy_nav2/src/go2w_nav2_controller_compat/'+target))
    for original, wanted in sorted(lock['reference_files_sha256'].items()):
        raw = subprocess.check_output(
            ['git', 'show', lock['humble_commit']+':'+original], cwd=args.upstream)
        if hashlib.sha256(raw).hexdigest() != wanted:
            raise RuntimeError('Humble reference hash mismatch: '+original)
    for original, wanted in sorted(lock.get('foxy_context_files_sha256', {}).items()):
        raw = subprocess.check_output(
            ['git', 'show', lock['foxy_commit']+':'+original], cwd=args.upstream)
        if hashlib.sha256(raw).hexdigest() != wanted:
            raise RuntimeError('Foxy API context hash mismatch: '+original)
    (base/'CONTROLLER_UPSTREAM_PATCHES.diff').write_text(''.join(changes))
    print(json.dumps({'verified_foxy_base_files': len(lock['base_files_sha256']),
                      'verified_humble_reference_files': len(lock['reference_files_sha256']),
                      'verified_foxy_api_context_files': len(lock.get('foxy_context_files_sha256', {})),
                      'diff_lines': len(changes)}))


if __name__ == '__main__':
    main()
