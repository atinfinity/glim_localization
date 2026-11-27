#!/bin/bash
# Full verification, shared by local runs and GitHub Actions (inside the container).
# Usage: ci.sh [--skip-deps] [--part all|core-K/N|rest]
#   all       (default) every check, in one run
#   core-K/N  the core build with the K-th of N shards of its ctest (GLIM_LOC_CTEST_SHARD); core-1/N
#             also runs the dependency check
#   rest      everything else: the core build without its ctest (the colcon checks need its install
#             and the synthetic dataset tool), colcon build and tests, interfaces, Python, replay e2e
# core-1/N .. core-N/N plus rest run the same checks as all; GitHub Actions runs them as parallel jobs
# (.github/workflows/ci.yml, tested in scripts/tests/test_ci_parts.py).
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  echo "usage: ci.sh [--skip-deps] [--part all|core-K/N|rest]" >&2
  exit 2
}

# The dependency check always runs unless the caller opts out. Only scripts/ci_local.sh does, and
# only when it has a recorded pass for this exact image and check_deps inputs.
skip_deps=false
part=all
while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-deps) skip_deps=true ;;
    --part) [[ $# -ge 2 ]] || usage; part="$2"; shift ;;
    --part=*) part="${1#--part=}" ;;
    *) usage ;;
  esac
  shift
done

shard=""
case "${part}" in
  all | rest) ;;
  core-*)
    shard="${part#core-}"
    if [[ ! "${shard}" =~ ^([1-9][0-9]*)/([1-9][0-9]*)$ ]] || (( BASH_REMATCH[1] > BASH_REMATCH[2] )); then
      echo "ci.sh: bad part ${part}: core-K/N needs 1 <= K <= N" >&2
      usage
    fi
    # Only the first shard runs the dependency check, which balances it against rest.
    [[ "${BASH_REMATCH[1]}" == 1 ]] || skip_deps=part
    ;;
  *) echo "ci.sh: bad part ${part}" >&2; usage ;;
esac
[[ "${part}" == rest ]] && skip_deps=part
echo "ci: part ${part}"

# CUDA mode in the CUDA image (docker/Dockerfile.cuda sets GLIM_LOC_BUILD_WITH_CUDA=ON): the core is
# configured with BUILD_WITH_CUDA=ON and the GPU tests are built. They skip when no GPU is visible; when
# nvidia-smi lists one, a skip is a failure (GLIM_LOC_REQUIRE_GPU=1, unless the caller set it).
# nvidia-smi is injected only with the "utility" driver capability, which Dockerfile.cuda sets in
# NVIDIA_DRIVER_CAPABILITIES next to "compute"; without it a visible GPU would let the tests skip.
# The image must also have been built from this tree (compose does not rebuild an existing tag).
if [[ "${GLIM_LOC_BUILD_WITH_CUDA:-OFF}" == ON ]]; then
  "${script_dir}/check_cuda_image_inputs.sh"
  gpus="$(nvidia-smi -L 2>/dev/null || true)"
  if [[ -n "${gpus}" ]]; then
    echo "ci: CUDA mode, GPU visible:"
    echo "${gpus}"
    export GLIM_LOC_REQUIRE_GPU="${GLIM_LOC_REQUIRE_GPU:-1}"
  else
    echo "ci: CUDA mode, no GPU visible (the GPU tests skip)"
  fi
else
  echo "ci: CPU mode"
fi

# Per-run compiler cache statistics when ccache is enabled (CI).
if [[ -n "${CCACHE_DIR:-}" ]]; then
  ccache --zero-stats >/dev/null
fi

case "${skip_deps}" in
  true) echo "SKIPPED: dependency check (recorded pass for this image)" ;;
  part) echo "SKIPPED: dependency check (run by part core-1/N)" ;;
  *) "${script_dir}/check_deps.sh" ;;
esac
case "${part}" in
  all)
    "${script_dir}/build_core.sh"
    ;;
  core-*)
    GLIM_LOC_CTEST_SHARD="${shard}" "${script_dir}/build_core.sh"
    ;;
  rest)
    GLIM_LOC_CORE_TESTS=skip "${script_dir}/build_core.sh"
    ;;
esac
if [[ "${part}" == all || "${part}" == rest ]]; then
  "${script_dir}/build_colcon.sh"
  "${script_dir}/test_colcon.sh"
  "${script_dir}/test_interfaces.sh"
  "${script_dir}/test_python.sh"
  "${script_dir}/test_replay_e2e.sh"
fi

if [[ -n "${CCACHE_DIR:-}" ]]; then
  ccache --show-stats
fi
if [[ "${part}" == all ]]; then
  echo "PASS: all checks"
else
  echo "PASS: part ${part}"
fi
