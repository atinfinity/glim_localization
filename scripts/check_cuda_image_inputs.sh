#!/bin/bash
# Checks that the CUDA image was built from this tree: the repository files it baked in (copied under
# /etc/glim_localization/inputs/ by docker/Dockerfile.cuda) must equal the same files of the tree.
# `docker compose run` never rebuilds an existing tag, so after a change to any of them the image is
# stale until `docker compose -f docker/compose.yaml build dev-cuda` runs again.
# Usage (in the CUDA image; scripts/ci.sh runs it in CUDA mode): scripts/check_cuda_image_inputs.sh
#   GLIM_LOC_ALLOW_STALE_IMAGE=1  warn instead of failing.
#   GLIM_LOC_IMAGE_INPUTS_DIR     the image's copy (default /etc/glim_localization/inputs; for tests).
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
inputs_dir="${GLIM_LOC_IMAGE_INPUTS_DIR:-/etc/glim_localization/inputs}"

# Everything the CUDA image bakes in from the repository. Keep equal to the inputs COPY lines at the
# end of docker/Dockerfile.cuda (scripts/tests/test_dockerfile_cuda.py checks it).
cuda_image_inputs=(
  docker/Dockerfile.cuda
  docker/entrypoint.sh
  scripts/requirements.txt
  third_party/patches/glim
  third_party/patches/glim_cuda
  third_party/patches/glim_ros2
)

sha256_files() { # file paths on stdin -> "<sha256>  <path>" per file
  if command -v sha256sum >/dev/null 2>&1; then xargs sha256sum; else xargs shasum -a 256; fi
}

listing() { # "<sha256>  ./<path>" per file under the inputs, sorted by path
  local paths=() path
  for path in "${cuda_image_inputs[@]}"; do
    if [[ -e "$1/${path}" ]]; then
      paths+=("./${path}")
    fi
  done
  [[ ${#paths[@]} -gt 0 ]] || return 0
  (cd "$1" && find "${paths[@]}" -type f | LC_ALL=C sort | sha256_files)
}

expected="$(listing "${repo_root}")"
actual=""
if [[ -d "${inputs_dir}" ]]; then
  actual="$(listing "${inputs_dir}")"
fi

if [[ -n "${actual}" && "${actual}" == "${expected}" ]]; then
  echo "check_cuda_image_inputs: the image matches this tree"
  exit 0
fi

{
  echo "check_cuda_image_inputs: the CUDA image was not built from this tree (stale image)."
  if [[ -z "${actual}" ]]; then
    echo "  ${inputs_dir} is missing: the image predates this check."
  else
    diff <(printf '%s\n' "${expected}") <(printf '%s\n' "${actual}") | sed -n 's/^</  tree: /p; s/^>/  image:/p' || true
  fi
  echo "  Rebuild it: docker compose -f docker/compose.yaml build dev-cuda"
} >&2
if [[ "${GLIM_LOC_ALLOW_STALE_IMAGE:-0}" == 1 ]]; then
  echo "check_cuda_image_inputs: continuing (GLIM_LOC_ALLOW_STALE_IMAGE=1)" >&2
  exit 0
fi
exit 1
