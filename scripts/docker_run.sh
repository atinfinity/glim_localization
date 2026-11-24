#!/bin/bash
# Run a command in the headless development container.
# Usage: scripts/docker_run.sh [--cuda] [command...]
#   --cuda  the CUDA image (service dev-cuda: glim_localization:dev-cuda or GLIM_LOC_CUDA_IMAGE, its
#           own build volume, all GPUs). Needs amd64, an NVIDIA driver and the NVIDIA Container Toolkit.
set -euo pipefail

service=dev
image="${GLIM_LOC_IMAGE:-}"
image_var=GLIM_LOC_IMAGE
if [[ "${1:-}" == --cuda ]]; then
  shift
  service=dev-cuda
  image="${GLIM_LOC_CUDA_IMAGE:-}"
  image_var=GLIM_LOC_CUDA_IMAGE
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${repo_root}/out"

tty_args=()
if [[ ! -t 0 ]]; then
  tty_args=(-T)
fi

# With GLIM_LOC_IMAGE (or GLIM_LOC_CUDA_IMAGE for --cuda) set, as scripts/ci_local.sh does, the image
# must already exist: compose would build a missing tag from the current tree (`run` has no --no-build,
# and `--pull never` still builds).
if [[ -n "${image}" ]] && ! docker image inspect "${image}" >/dev/null 2>&1; then
  echo "docker_run: image ${image} (${image_var}) does not exist; refusing to build it" >&2
  exit 1
fi

# Opt-in compiler cache (CI): GLIM_LOC_CCACHE_DIR is a host directory, mounted as the container's
# CCACHE_DIR (files in it are owned by the container user, root). CCACHE_MAXSIZE is passed through
# when set. Unset, nothing changes.
ccache_args=()
if [[ -n "${GLIM_LOC_CCACHE_DIR:-}" ]]; then
  mkdir -p "${GLIM_LOC_CCACHE_DIR}"
  ccache_args=(-v "$(cd "${GLIM_LOC_CCACHE_DIR}" && pwd):/ws/ccache" -e CCACHE_DIR=/ws/ccache)
  if [[ -n "${CCACHE_MAXSIZE:-}" ]]; then
    ccache_args+=(-e "CCACHE_MAXSIZE=${CCACHE_MAXSIZE}")
  fi
fi

exec docker compose -f "${repo_root}/docker/compose.yaml" run --rm ${tty_args[@]+"${tty_args[@]}"} \
  ${ccache_args[@]+"${ccache_args[@]}"} "${service}" "$@"
