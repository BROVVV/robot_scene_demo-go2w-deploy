#!/usr/bin/env bash
# Convenience wrapper: plan-only autonomous search unless motion flags are
# explicitly appended by the operator.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${script_dir}/bootstrap_webui.sh" --mode search "$@"
