#!/usr/bin/env python3
"""Prepare bounded camera trial units only. Never installs units or starts services."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import re

ORIGINAL='realsense-stream.service'
ORIGINAL_SCRIPT_SHA='e67d0bbbbb079e10f9c53b34de3820a80b1882e51052855d958bcc0dbefbf41f'
CAMERA_REL='scripts/go2w/realsense_stream.py'


def bundle(release,tag,camera_sha):
    if not re.fullmatch(r'[0-9]{8}_[0-9]{2}',release):raise ValueError('invalid frozen release name')
    if not re.fullmatch(r'[a-z0-9][a-z0-9-]{0,39}',tag):raise ValueError('invalid trial tag')
    if not re.fullmatch(r'[0-9a-f]{64}',camera_sha):raise ValueError('invalid source fingerprint')
    review='go2w-camera-review-'+tag+'.service'
    restore='go2w-camera-restore-'+tag+'.service'
    timer='go2w-camera-restore-'+tag+'.timer'
    root='/home/unitree/robotscene_releases/'+release
    files={review:f'''[Unit]
Description=Bounded read-only RGB-D camera review ({tag})

[Service]
Type=simple
User=unitree
WorkingDirectory=/home/unitree/robotscene_runtime
Environment=PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
ExecStart=/usr/bin/python3 {root}/{CAMERA_REL} --width 1280 --height 720 --fps 30 --port 8080 --encoding-mode on-demand --snap-dir /home/unitree/robotscene_runtime/camera_review_snapshots
RuntimeMaxSec=90s
TimeoutStopSec=8s
KillMode=control-group
NoNewPrivileges=true
ExecStopPost=+/usr/bin/systemctl --no-block start {restore}
''',restore:f'''[Unit]
Description=Restore the original RGB-D service after camera review ({tag})

[Service]
Type=oneshot
ExecStartPre=-/usr/bin/systemctl stop {review}
ExecStart=/usr/bin/systemctl start {ORIGINAL}
TimeoutStartSec=40s
''',timer:f'''[Unit]
Description=Independent camera restoration deadline ({tag})

[Timer]
OnActiveSec=120s
AccuracySec=1s
Unit={restore}
Persistent=false
'''}
    manifest={'schema_version':1,'status':'prepared_not_installed_or_activated',
       'camera_release':release,'camera_source_sha256':camera_sha,
       'original_service':ORIGINAL,'original_script_sha256':ORIGINAL_SCRIPT_SHA,
       'review_unit':review,'restore_unit':restore,'restore_timer':timer,
       'camera_runtime_max_s':90,'independent_restore_deadline_s':120,
       'motion_commands_included':False,'files':{n:hashlib.sha256(s.encode()).hexdigest() for n,s in files.items()}}
    files['activation-checklist.md']=f'''# Camera trial bundle: {tag}

Prepared only. Do not install/start during low battery, shutdown or unknown
supply. This bundle does not grant motion authority and never starts a lease,
SDK, controller or Action server. Only the camera process is tested.

Before activation on the robot:

1. Read fresh battery and boot identity; require SOC >=30 and a stationary
   robot, verify no motion server/lease owner, and verify the PC map backup.
2. Verify the installed original script SHA is {ORIGINAL_SCRIPT_SHA}, original
   service is enabled/active, and capture its unit/drop-in hashes. Verify the
   candidate source SHA is {camera_sha}; never modify either source file.
3. Confirm these three uniquely named units do not already exist, verify the
   files against bundle.json, and run native systemd-analyze verify. Do not
   replace another unit with the same name.
4. Install only the three service/timer files under /run/systemd/system using
   root ownership, then daemon-reload. Do not enable them or add drop-ins to
   the original service. /run units vanish at reboot; the original remains
   enabled and unchanged.
5. Start {timer} and verify it is actually active BEFORE stopping the original
   service. If timer activation fails, leave the original camera running.
6. Stop {ORIGINAL}, then start {review}. Wrap the test in try/finally; finally
   starts {restore} regardless of result. Do not start any motion/SDK process.
7. For 15-30 seconds, measure real publication timestamps/device frame numbers,
   CPU/RSS and full RGB-D/JPEG/PNG/metadata compatibility. Keep the complete
   test plus restoration within the 120 second guard. /health.fps is not a
   real publication rate. Compare with a fresh baseline under the same load.
8. Stop the review, start the restore unit, verify original MainPID executable,
   unchanged script/unit/drop-in hashes and enabled state, and fresh decoded
   RGB-D frames. Cancel the timer ONLY after restoration is verified. Keep
   the timer armed if restoration fails; do not claim recovery from process
   state alone or reset USB/reboot automatically.
9. After restoration, collect logs and verify temporary processes are gone.
   Stop/reset only these exact temporary units, remove only their /run files,
   and daemon-reload. Preserve any error evidence. Never mask/disable/change
   the original service.

The review has a 90-second lifetime and an 8-second stop bound. Its stop
hook requests restoration without waiting; the root restore service stops
it synchronously before starting the original service. The independent
120-second timer also restores if the SSH/operator process disappears before
launching the review. These are software mechanisms; actual native validation,
USB recovery and original stream recovery must still be observed.
'''
    manifest['activation_checklist_sha256']=hashlib.sha256(files['activation-checklist.md'].encode()).hexdigest()
    return files,manifest


def prepare(source_root,tag,output):
    source_root=Path(source_root);output=Path(output)
    source=source_root/CAMERA_REL
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    declared=json.loads((source_root/'release_manifest.json').read_text())
    if declared['release']!=source_root.name or declared['files'][CAMERA_REL]!=digest:
        raise ValueError('camera source does not match the frozen manifest')
    files,manifest=bundle(source_root.name,tag,digest)
    output.mkdir(parents=True,exist_ok=False)
    for name,content in files.items():(output/name).write_text(content)
    (output/'bundle.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-source-root',required=True,type=Path)
    parser.add_argument('--tag',required=True)
    parser.add_argument('--output-dir',required=True,type=Path)
    args=parser.parse_args();print(json.dumps(prepare(args.camera_source_root,args.tag,args.output_dir)))


if __name__=='__main__':main()
