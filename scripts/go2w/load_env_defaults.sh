#!/usr/bin/env bash
# Source deployment defaults without replacing explicit exported overrides.
# Values never pass through eval or command substitution and are not logged.
go2w_load_env_defaults() {
  local go2w_defaults_path="$1"
  [[ -f "$go2w_defaults_path" ]] || return 0
  local -A go2w_defaults_saved=()
  local go2w_defaults_name go2w_defaults_autoexport=0
  while IFS= read -r go2w_defaults_name; do
    if [[ "$go2w_defaults_name" != go2w_defaults_* &&
          "$go2w_defaults_name" =~ ^[a-zA-Z_][a-zA-Z0-9_]*$ && -v "$go2w_defaults_name" ]]; then
      go2w_defaults_saved["$go2w_defaults_name"]="${!go2w_defaults_name}"
    fi
  done < <(compgen -e)
  [[ "$-" == *a* ]] && go2w_defaults_autoexport=1
  set -a
  # shellcheck disable=SC1090
  source "$go2w_defaults_path"
  for go2w_defaults_name in "${!go2w_defaults_saved[@]}"; do
    printf -v "$go2w_defaults_name" '%s' "${go2w_defaults_saved[$go2w_defaults_name]}"
    export "$go2w_defaults_name"
  done
  if [[ "$go2w_defaults_autoexport" == 0 ]]; then
    set +a
  fi
}
