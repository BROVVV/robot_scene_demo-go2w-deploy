#!/usr/bin/env bash
# Offline SDK environment. Does not open DDS channels or acquire a lease.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime="${GO2W_RUNTIME_ROOT:-/home/unitree/robotscene_runtime}"
[[ "$(uname -m)" == aarch64 && -f /opt/ros/foxy/setup.bash ]] || exit 2
[[ $# == 0 || ( $# == 1 && "$1" == --verify-only ) ]] || exit 2
venv="$runtime/venvs/control-py38"
if [[ $# == 0 ]]; then
  [[ ! -e "$venv" ]] || { echo 'SDK venv exists; use --verify-only.' >&2; exit 2; }
  PYTHONNOUSERSITE=1 /usr/bin/python3 -m venv --system-site-packages "$venv"
  PYTHONNOUSERSITE=1 "$venv/bin/python" -m pip install \
    --disable-pip-version-check --no-index --no-deps --require-hashes \
    --find-links "$runtime/wheelhouse/sdk-py38" -r "$root/requirements/robot-sdk-py38.lock"
fi
# This is the existing motion launcher's SDK-only provider. Setting this in
# the ROS parent would mix SDK CycloneDDS 0.10.2 with Foxy RMW 0.7.
export LD_LIBRARY_PATH="$HOME/cyclonedds_0.10.2/lib"
unset CYCLONEDDS_HOME PYTHONHOME
export PYTHONNOUSERSITE=1 PYTHONPATH="$root/unitree_go2w_control/vendor/unitree_sdk2_python"
"$venv/bin/python" - "$root" <<'PY'
from pathlib import Path
import hashlib, importlib.metadata, json, platform, sys
root = Path(sys.argv[1])
assert sys.version_info[:2] == (3, 8) and platform.machine() == 'aarch64'
for line in (root/'requirements/robot-sdk-py38.lock').read_text().splitlines():
    requirement = line.split()[0]
    name, version = requirement.split('==')
    assert importlib.metadata.version(name) == version, requirement
for artifact in json.loads((root/'requirements/robot-sdk-native.json').read_text())['artifacts']:
    p = Path(artifact['runtime_path']); raw = p.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == artifact['sha256'], str(p)
    assert raw[:4] == b'\x7fELF' and int.from_bytes(raw[18:20], 'little') == 183
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.utils.crc import CRC
crc = CRC()
vectors = [[], [0], [0xffffffff], [0x12345678], list(range(32)), [0,0xffffffff]*8,
           [(i*2654435761)&0xffffffff for i in range(64)]]
for data in vectors:
    assert crc._crc_ctypes(data) == crc._crc_py(data)
dds = sorted({line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines()
              if 'libddsc.so' in line})
assert dds == [str((Path.home()/'cyclonedds_0.10.2/lib/libddsc.so').resolve())], dds
assert 'rclpy' not in sys.modules
assert all('/home/unitree/robotscene/' not in path for path in sys.path)
print(json.dumps({'python':sys.executable, 'dds_libraries':dds, 'crc_vectors':len(vectors),
                  'rclpy_imported':False, 'dds_channel_opened':False, 'passed':True}))
PY
