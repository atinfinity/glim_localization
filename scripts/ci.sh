#!/bin/bash
# Full verification, shared by local runs and GitHub Actions (inside the container).
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# The dependency check always runs unless the caller opts out. Only scripts/ci_local.sh does, and
# only when it has a recorded pass for this exact image and check_deps inputs.
skip_deps=false
for arg in "$@"; do
  case "${arg}" in
    --skip-deps) skip_deps=true ;;
    *) echo "usage: ci.sh [--skip-deps]" >&2; exit 2 ;;
  esac
done

# Per-run compiler cache statistics when ccache is enabled (CI).
if [[ -n "${CCACHE_DIR:-}" ]]; then
  ccache --zero-stats >/dev/null
fi

if [[ "${skip_deps}" == true ]]; then
  echo "SKIPPED: dependency check (recorded pass for this image)"
else
  "${script_dir}/check_deps.sh"
fi
"${script_dir}/build_core.sh"
"${script_dir}/build_colcon.sh"
"${script_dir}/test_colcon.sh"
"${script_dir}/test_interfaces.sh"
"${script_dir}/test_python.sh"
"${script_dir}/test_replay_e2e.sh"

if [[ -n "${CCACHE_DIR:-}" ]]; then
  ccache --show-stats
fi
echo "PASS: all checks"
