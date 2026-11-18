#!/usr/bin/env python3
"""Inspect a release pointer without modifying links or starting services."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from app.deployment.release_link import inspect_link


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--releases-root',type=Path,required=True)
    parser.add_argument('--current',type=Path,required=True)
    args=parser.parse_args()
    try:
        state=inspect_link(args.releases_root,args.current)
        print(json.dumps({'snapshot':asdict(state),'read_only':True,
            'source_and_functional_acceptance_verified':False}))
    except Exception as exc:
        print(json.dumps({'read_only':True,'error':str(exc)}));return 2
    return 0


if __name__=='__main__':raise SystemExit(main())
