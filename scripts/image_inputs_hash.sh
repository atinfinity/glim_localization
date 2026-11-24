#!/bin/bash
# Fingerprint of everything the dev image bakes in from the repository, read from a git tree.
# Usage: scripts/image_inputs_hash.sh [--manifest] [<tree-ish>]   (default tree-ish: HEAD)
#   default     prints the SHA-256 of the manifest; CI uses it as the prebuilt image tag on GHCR.
#   --manifest  prints "<sha256>  ./<path>" per input file, sorted by path, exactly as a listing of
#               /etc/glim_localization/inputs/ in the image reads (scripts/ci_local.sh compares them).
# The Dockerfile copies each input under /etc/glim_localization/inputs/ at the same relative path
# (tested in scripts/tests/test_ci_local.py), so ci_local.sh and CI cannot disagree on the inputs.
set -euo pipefail

manifest=false
if [[ "${1:-}" == --manifest ]]; then
  manifest=true
  shift
fi
if [[ $# -gt 1 || "${1:-}" == -* ]]; then
  echo "usage: image_inputs_hash.sh [--manifest] [<tree-ish>]" >&2
  exit 2
fi
tree="${1:-HEAD}"

cd "$(dirname "${BASH_SOURCE[0]}")/.."

# Everything the image bakes in from the repository.
image_inputs=(
  docker/Dockerfile
  docker/entrypoint.sh
  scripts/requirements.txt
  third_party/patches/glim
  third_party/patches/glim_ros2
)

sha256_of() { # stdin
  if command -v sha256sum >/dev/null 2>&1; then sha256sum | cut -d' ' -f1; else shasum -a 256 | cut -d' ' -f1; fi
}

list_inputs() { # "<sha256>  ./<path>" per file of the tree, sorted by path
  local path
  git ls-tree -r --name-only "${tree}" -- "${image_inputs[@]}" | LC_ALL=C sort | while IFS= read -r path; do
    printf '%s  ./%s\n' "$(git cat-file blob "${tree}:${path}" | sha256_of)" "${path}"
  done
}

if [[ "${manifest}" == true ]]; then
  list_inputs
  exit 0
fi

listing="$(list_inputs)"
[[ -n "${listing}" ]] || { echo "image_inputs_hash: ${tree} has no docker/Dockerfile" >&2; exit 1; }
printf '%s\n' "${listing}" | sha256_of
