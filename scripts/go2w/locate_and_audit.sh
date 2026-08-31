#!/usr/bin/env bash
set -euo pipefail

# Read-only project discovery plus a reproducible, credential-free baseline.
# This script never sources .env, starts ROS nodes, acquires a Sport lease, or
# publishes any topic. Generated artifacts live under ignored outputs/.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
OUTPUT_ROOT="${GO2W_AUDIT_OUTPUT_DIR:-$PROJECT_ROOT/outputs/go2w_baseline}"
SNAPSHOT_ROOT="$OUTPUT_ROOT/snapshots"
PROJECTS_TSV="$OUTPUT_ROOT/project_candidates.tsv"
SUMMARY_MD="$OUTPUT_ROOT/audit_summary.md"

mkdir -p "$SNAPSHOT_ROOT"

candidate_file="$(mktemp)"
trap 'rm -f -- "$candidate_file"' EXIT

search_roots=("$HOME" /home)
if [[ -d /root/gpufree-data ]]; then
  search_roots+=(/root/gpufree-data)
fi
find "${search_roots[@]}" -maxdepth 5 -type d -name robot_scene_demo \
  -print 2>/dev/null \
  | while IFS= read -r candidate; do realpath -- "$candidate"; done \
  | sort -u >"$candidate_file"

printf 'path\tremote\tbranch\thead\tmtime\tdirty\treadme_lines\n' >"$PROJECTS_TSV"
while IFS= read -r candidate; do
  [[ -n "$candidate" ]] || continue
  remote="$(git -C "$candidate" remote get-url origin 2>/dev/null || true)"
  branch="$(git -C "$candidate" branch --show-current 2>/dev/null || true)"
  head="$(git -C "$candidate" rev-parse HEAD 2>/dev/null || true)"
  dirty="$(git -C "$candidate" status --porcelain=v1 2>/dev/null | wc -l)"
  mtime="$(stat -c '%y' "$candidate" 2>/dev/null || true)"
  readme_lines="$(wc -l <"$candidate/README.md" 2>/dev/null || printf '0')"
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$candidate" "$remote" "$branch" "$head" "$mtime" "$dirty" \
    "$readme_lines" >>"$PROJECTS_TSV"
done <"$candidate_file"

if ! git -C "$PROJECT_ROOT" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  printf 'ERROR: %s is not a Git worktree\n' "$PROJECT_ROOT" >&2
  exit 2
fi

git -C "$PROJECT_ROOT" status --short >"$OUTPUT_ROOT/git_status_short.txt"
git -C "$PROJECT_ROOT" rev-parse HEAD >"$OUTPUT_ROOT/git_head.txt"
git -C "$PROJECT_ROOT" remote -v >"$OUTPUT_ROOT/git_remotes.txt"
git -C "$PROJECT_ROOT" diff --binary >"$PROJECT_ROOT/outputs/pre_go2w_integration.patch"

cp -- "$PROJECT_ROOT/requirements.txt" "$SNAPSHOT_ROOT/requirements.txt"
cp -- "$PROJECT_ROOT/.env.example" "$SNAPSHOT_ROOT/env.example"
cp -- "$PROJECT_ROOT/README.md" "$SNAPSHOT_ROOT/README.md"
if [[ -f "$PROJECT_ROOT/docs/NAV2_INTEGRATION.md" ]]; then
  cp -- "$PROJECT_ROOT/docs/NAV2_INTEGRATION.md" \
    "$SNAPSHOT_ROOT/NAV2_INTEGRATION.md"
fi
if [[ -f "$PROJECT_ROOT/docs/NAV2_WEBUI_TESTING.md" ]]; then
  cp -- "$PROJECT_ROOT/docs/NAV2_WEBUI_TESTING.md" \
    "$SNAPSHOT_ROOT/NAV2_WEBUI_TESTING.md"
fi
find "$PROJECT_ROOT/tests" -maxdepth 1 -type f -name 'test_*.py' \
  -printf '%f\n' | sort >"$SNAPSHOT_ROOT/tests.txt"
find "$PROJECT_ROOT/ros2_ws/src" -mindepth 1 -maxdepth 1 -type d \
  -printf '%f\n' | sort >"$SNAPSHOT_ROOT/ros2_packages.txt"
sha256sum "$SNAPSHOT_ROOT"/* >"$OUTPUT_ROOT/snapshot_sha256.txt"

head_value="$(tr -d '\n' <"$OUTPUT_ROOT/git_head.txt")"
tracked_dirty_count="$({ git -C "$PROJECT_ROOT" diff --name-only; git -C "$PROJECT_ROOT" diff --cached --name-only; } | sort -u | sed '/^$/d' | wc -l)"
untracked_count="$(git -C "$PROJECT_ROOT" ls-files --others --exclude-standard | wc -l)"
test_count="$(wc -l <"$SNAPSHOT_ROOT/tests.txt")"
package_count="$(wc -l <"$SNAPSHOT_ROOT/ros2_packages.txt")"
env_state="absent"
if [[ -f "$PROJECT_ROOT/.env" ]]; then
  env_state="present (contents intentionally not inspected or copied)"
fi

cat >"$SUMMARY_MD" <<EOF
# Go2-W integration discovery baseline

- Selected project: \`$PROJECT_ROOT\`
- Selection reason: current workspace copy; origin matches BROVVV/robot_scene_demo.
- Git HEAD: \`$head_value\`
- Tracked dirty paths recorded: $tracked_dirty_count
- Untracked, non-ignored paths recorded: $untracked_count
- Existing .env: $env_state
- Test files inventoried: $test_count
- ROS 2 packages inventoried: $package_count
- Pre-integration patch: \`outputs/pre_go2w_integration.patch\`
- Candidate details: \`outputs/go2w_baseline/project_candidates.tsv\`

This audit is read-only with respect to the robot. It does not source secrets,
start ROS nodes, acquire a lease, or publish control messages.
EOF

printf 'Baseline written to %s\n' "$OUTPUT_ROOT"
