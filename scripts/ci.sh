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
