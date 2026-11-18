#!/usr/bin/env bash
# Convenience wrapper: read-only software; movement stays on the Unitree remote.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${script_dir}/bootstrap_webui.sh" --mode mapping "$@"
