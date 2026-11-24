#!/bin/bash
# Host-side wrapper for the full local verification (scripts/ci.sh in the container).
# Usage: scripts/ci_local.sh [--force]
#
# A clean tree is verified from a git-synced snapshot of the commit tree, never from the worktree:
#   <git-common-dir>/ci_local/src, mounted in the container with its own build volume
#   (glim_localization_ci_<hash of the clone>_build). One snapshot, one volume, and one waiting
#   host lock per clone, however many worktrees there are. A file whose content changes between
#   trees is rewritten (new mtime), so make always rebuilds it; unchanged files keep their mtime.
# A pass is recorded under out/ci_stamps, keyed by the tree hash and the Docker image ID, and reused
#   only on a clean tree. Skip-worktree and assume-unchanged files count as dirty.
# A dirty tree runs the working tree as scripts/docker_run.sh does, reusing and recording nothing.
# A docs-only change against origin/main skips build and tests and records nothing.
# --force wipes the build volume and ignores every recorded pass.
# GitHub Actions runs scripts/ci.sh directly and always does a full run.
#
# Test hooks (environment): CI_LOCAL_DOCKER (docker command), CI_LOCAL_IMAGE, CI_LOCAL_BASE_REF,
# CI_LOCAL_POLL (lock poll interval in seconds).
#
# Known limits, all fail safe: a reused PID in the lock makes a run wait; a waiter holding a stale PID
# can briefly move a newer holder's lock (needs a killed holder and several waiters); the snapshot is
# compared with `git add -A` without -f, so a tracked file matching an ignore rule stops every run.
set -euo pipefail

force=false
if [[ $# -gt 1 ]]; then
  echo "usage: ci_local.sh [--force]" >&2
  exit 2
elif [[ $# -eq 1 ]]; then
  [[ "$1" == --force ]] || { echo "usage: ci_local.sh [--force]" >&2; exit 2; }
  force=true
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"
docker_cmd="${CI_LOCAL_DOCKER:-docker}"
image="${CI_LOCAL_IMAGE:-glim_localization:dev}"
base_ref="${CI_LOCAL_BASE_REF:-origin/main}"
poll="${CI_LOCAL_POLL:-1}"
stamp_dir="${repo_root}/out/ci_stamps"

git_dir="$(git rev-parse --absolute-git-dir)"
common_dir="$(git rev-parse --path-format=absolute --git-common-dir)"
state="${common_dir}/ci_local"
src="${state}/src"
lock="${state}/lock"
clone_hash="$(printf '%s' "${common_dir}" | git hash-object --stdin | cut -c1-8)"
project="glim_localization_ci_${clone_hash}"

die() { echo "ci_local: $*" >&2; exit 1; }

dirty_reason() {
  if [[ -n "$(git status --porcelain --untracked-files=all)" ]]; then
    echo "uncommitted or untracked changes"
  elif git ls-files -v | grep -q '^[a-zS] '; then
    echo "skip-worktree or assume-unchanged files"
  fi
}

image_id() {
  local id
  id="$("${docker_cmd}" image inspect --format '{{.Id}}' "${image}" 2>/dev/null)" || return 1
  [[ -n "${id}" ]] || return 1
  printf '%s' "${id}"
}

# A stamp is valid only if its whole content is the expected prefix followed by a commit SHA.
stamp_valid() { # file, expected prefix
  local file="$1" prefix="$2" content
  [[ -s "${file}" ]] || return 1
  content="$(cat "${file}" 2>/dev/null)" || return 1
  [[ "${content}" =~ ^"${prefix}"[0-9a-f]{40}$ ]]
}

write_stamp() { # file, content
  mkdir -p "${stamp_dir}"
  local tmp
  tmp="$(mktemp "${stamp_dir}/.tmp.XXXXXX")"
  printf '%s' "$2" > "${tmp}"
  mv -f "${tmp}" "$1"
}

# Hash of everything check_deps.sh reads from the repository, in the tree being verified.
deps_hash() {
  git rev-parse "${tree}:scripts/check_deps.sh" "${tree}:scripts/requirements.in" "${tree}:scripts/check_deps" 2>/dev/null \
    | git hash-object --stdin
}

# ---- Resolve everything once. Later changes of HEAD or the worktree do not matter for a clean run:
# the container sees only the snapshot of ${tree}.
head_commit="$(git rev-parse HEAD)"
tree="$(git rev-parse 'HEAD^{tree}')"
image_id_start="$(image_id)" || die "cannot inspect Docker image ${image}"
safe_image="${image_id_start//[^A-Za-z0-9]/_}"
reason="$(dirty_reason)"
clean=true
[[ -z "${reason}" ]] || clean=false

full_stamp="${stamp_dir}/pass_${tree}_${safe_image}"
full_prefix="v1 tree=${tree} image=${image_id_start} commit="
deps_stamp=""
deps_prefix=""
if [[ "${clean}" == true ]] && deps="$(deps_hash)"; then
  deps_stamp="${stamp_dir}/deps_${safe_image}_${deps}"
  deps_prefix="v1 image=${image_id_start} deps=${deps} commit="
fi

reuse_message() {
  local recorded
  recorded="$(cat "${full_stamp}")"
  echo "ci_local: REUSED a recorded full pass for tree ${tree} (recorded at commit ${recorded##*commit=}, HEAD ${head_commit}); nothing was run"
}

if [[ "${clean}" != true ]]; then
  echo "ci_local: working tree is dirty (${reason}); running the working tree and recording nothing"
elif [[ "${force}" != true ]]; then
  if stamp_valid "${full_stamp}" "${full_prefix}"; then
    reuse_message
    exit 0
  fi
  if git rev-parse --verify -q "${base_ref}" >/dev/null && merge_base="$(git merge-base "${base_ref}" HEAD)"; then
    if git diff --name-only --no-renames -z "${merge_base}" HEAD | scripts/is_docs_only.sh; then
      echo "ci_local: SKIPPED build and tests, docs-only change against ${base_ref} (not recorded as a full pass)"
      exit 0
    fi
  fi
fi

# ---- One lock per clone, held until the stamp is written. The snapshot and the build volume are
# shared, so other runs wait.
lock_held=false
release_lock() {
  if [[ "${lock_held}" == true ]]; then
    rm -rf "${lock}"
    lock_held=false
  fi
}
# The container runs from a private tag of the resolved image ID, so a moved glim_localization:dev
# cannot change what is verified. Only this tag is ever removed, never the shared one.
run_tag=""
remove_run_tag() {
  if [[ -n "${run_tag}" ]]; then
    "${docker_cmd}" rmi "${run_tag}" >/dev/null 2>&1 || true
    run_tag=""
  fi
}
cleanup() {
  release_lock
  remove_run_tag
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

acquire_lock() {
  mkdir -p "${state}"
  local ticks=0 pid noid=0 other
  local every
  every="$(awk -v p="${poll}" 'BEGIN { n = int(10 / p); print (n < 1 ? 1 : n) }')"
  until mkdir "${lock}" 2>/dev/null; do
    pid="$(cat "${lock}/pid" 2>/dev/null || true)"
    if [[ -z "${pid}" ]]; then
      # The holder may be between mkdir and writing its PID; only a long silence means it died.
      noid=$((noid + 1))
      if [[ "${noid}" -gt 20 ]]; then pid=0; fi
    else
      noid=0
    fi
    if [[ -n "${pid}" ]] && { [[ "${pid}" == 0 ]] || ! kill -0 "${pid}" 2>/dev/null; }; then
      echo "ci_local: taking over the lock of a dead run (pid ${pid})"
      if mv "${lock}" "${lock}.stale.$$" 2>/dev/null; then
        other="$(cat "${lock}.stale.$$/pid" 2>/dev/null || true)"
        if [[ "${pid}" != 0 && "${other}" != "${pid}" ]]; then
          # We moved a lock that another run took in the meantime; put it back.
          mv "${lock}.stale.$$" "${lock}" 2>/dev/null || true
        else
          rm -rf "${lock}.stale.$$"
        fi
      fi
      continue
    fi
    if [[ $((ticks % every)) -eq 0 ]]; then
      other="$(cat "${lock}/worktree" 2>/dev/null || echo unknown)"
      echo "ci_local: waiting for the run in ${other} (pid ${pid:-unknown})"
    fi
    sleep "${poll}"
    ticks=$((ticks + 1))
  done
  lock_held=true
  printf '%s\n' "$$" > "${lock}/pid"
  printf '%s\n' "${repo_root}" > "${lock}/worktree"
}
acquire_lock

# The run we waited for may have recorded this very tree.
if [[ "${clean}" == true && "${force}" != true ]] && stamp_valid "${full_stamp}" "${full_prefix}"; then
  reuse_message
  exit 0
fi

# A container left behind by a killed run could still be using the volume.
wait_for_idle_containers() {
  local waited=0 running
  while running="$("${docker_cmd}" ps -q --filter "label=com.docker.compose.project=${project}" 2>/dev/null)" && [[ -n "${running}" ]]; do
    [[ "${waited}" -lt 600 ]] || die "a container of an earlier run is still using the CI volume"
    [[ $((waited % 10)) -ne 0 ]] || echo "ci_local: waiting for a container of an earlier run to exit"
    sleep 1
    waited=$((waited + 1))
  done
}

# ---- Clean run: prepare and verify the snapshot.
snapshot_tree() { # prints the tree hash of the snapshot (tracked and untracked, not ignored)
  local verify_index="${state}/verify_index"
  rm -f "${verify_index}"
  (cd "${src}" && GIT_DIR="${git_dir}" GIT_WORK_TREE="${src}" GIT_INDEX_FILE="${verify_index}" git add -A . \
    && GIT_DIR="${git_dir}" GIT_WORK_TREE="${src}" GIT_INDEX_FILE="${verify_index}" git write-tree)
}

sync_snapshot() {
  mkdir -p "${src}"
  (cd "${src}" && export GIT_DIR="${git_dir}" GIT_WORK_TREE="${src}" GIT_INDEX_FILE="${state}/index" \
    && git read-tree --reset -u "${tree}" && git clean -fdxq)
}

prepare_snapshot() {
  sync_snapshot
  if [[ "$(snapshot_tree)" != "${tree}" ]]; then
    echo "ci_local: snapshot does not match tree ${tree}; recreating it"
    rm -rf "${src}" "${state}/index"
    sync_snapshot
    [[ "$(snapshot_tree)" == "${tree}" ]] || die "cannot create a snapshot of tree ${tree}"
  fi
}

# Everything the image bakes in from the repository, listed in scripts/image_inputs_hash.sh (shared
# with CI, which tags the prebuilt image with its hash).
check_image_inputs() {
  local cache="${state}/image_inputs/${safe_image}" expected actual
  expected="$("${repo_root}/scripts/image_inputs_hash.sh" --manifest "${tree}")"
  [[ -n "${expected}" ]] || die "tree has no docker/Dockerfile"
  if [[ -s "${cache}" ]]; then
    actual="$(cat "${cache}")"
  else
    actual="$("${docker_cmd}" run --rm --entrypoint sh "${image_id_start}" -c \
      'cd /etc/glim_localization/inputs && find . -type f | LC_ALL=C sort | xargs sha256sum' 2>/dev/null)" || actual=""
    if [[ -n "${actual}" ]]; then
      mkdir -p "${state}/image_inputs"
      printf '%s' "${actual}" > "${cache}"
    fi
  fi
  [[ "${actual}" == "${expected}" ]] \
    || die "image was not built from this tree's Dockerfile, patches, requirements or entrypoint; run docker compose build"
}

# The container runs as root and leaves root-owned files (out/, caches) in the snapshot.
chown_snapshot() {
  [[ -d "${src}" ]] || return 0
  "${docker_cmd}" run --rm -v "${src}:/s" --entrypoint chown "${image_id_start}" -R "$(id -u):$(id -g)" /s >/dev/null \
    || { echo "ci_local: cannot give the snapshot back to the host user (docker chown failed); nothing recorded" >&2; return 1; }
}

volume_name="${project}_build"
wipe_volume() {
  if "${docker_cmd}" volume inspect "${volume_name}" >/dev/null 2>&1; then
    echo "ci_local: wiping build volume ${volume_name} ($1)"
    "${docker_cmd}" volume rm "${volume_name}" >/dev/null || die "cannot remove volume ${volume_name}"
  fi
}

wipe_reason() { # prints a reason when the build volume cannot be trusted for this tree
  local built_tree built_image container_now host_now skew
  if [[ "${force}" == true ]]; then echo "--force"; return; fi
  if [[ ! -s "${state}/built" ]]; then echo "no record of the last build"; return; fi
  read -r built_tree built_image < "${state}/built" || true
  if [[ "${built_image:-}" != "${image_id_start}" ]]; then echo "image changed"; return; fi
  if ! git cat-file -e "${built_tree}^{tree}" 2>/dev/null; then echo "last built tree is gone"; return; fi
  if [[ -n "$(git diff --no-renames --diff-filter=D --name-only "${built_tree}" "${tree}")" ]]; then
    echo "files were deleted since the last build"
    return
  fi
  # A cached CMake option() or find_package result would survive a changed default, so any change of a
  # build definition starts from a fresh volume; these files change rarely.
  if [[ -n "$(git diff --no-renames --name-only "${built_tree}" "${tree}" -- '*CMakeLists.txt' '*.cmake' '*.cmake.in' '*package.xml' scripts/build_core.sh scripts/build_colcon.sh scripts/ci.sh)" ]]; then
    echo "a build definition (CMake, package.xml or build script) changed"
    return
  fi
  container_now="$("${docker_cmd}" run --rm --entrypoint date "${image_id_start}" +%s 2>/dev/null)" || { echo "cannot read the container clock"; return; }
  host_now="$(date +%s)"
  skew=$((container_now - host_now))
  if [[ "${skew#-}" -gt 2 ]]; then echo "container clock differs from the host by ${skew} s"; return; fi
}

args=()
rc=0
if [[ "${clean}" == true ]]; then
  if [[ "${force}" == true ]]; then
    echo "ci_local: --force, ignoring every recorded pass"
    # An interrupted forced run must not leave the old pass behind.
    rm -f "${full_stamp}"
    [[ -z "${deps_stamp}" ]] || rm -f "${deps_stamp}"
  fi
  check_image_inputs
  chown_snapshot || exit 1
  if [[ "${force}" != true && -n "${deps_stamp}" ]] && stamp_valid "${deps_stamp}" "${deps_prefix}"; then
    echo "ci_local: dependency check skipped, recorded pass for this image and check_deps inputs"
    args=(--skip-deps)
  fi
  wait_for_idle_containers
  prepare_snapshot
  reason="$(wipe_reason)"
  if [[ -n "${reason}" ]]; then wipe_volume "${reason}"; fi
  # Without this file the next run wipes the volume; it is written only after the container ends.
  rm -f "${state}/built"
  run_tag="glim_localization:ci-$$"
  "${docker_cmd}" tag "${image_id_start}" "${run_tag}" || { run_tag=""; die "cannot tag image ${image_id_start} for the run"; }
  COMPOSE_PROJECT_NAME="${project}" GLIM_LOC_IMAGE="${run_tag}" "${src}/scripts/docker_run.sh" scripts/ci.sh ${args[@]+"${args[@]}"} || rc=$?
  printf '%s %s\n' "${tree}" "${image_id_start}" > "${state}/built"
  # A failed chown must not hide ci.sh's own exit code; either way nothing is recorded.
  chown_snapshot || { [[ "${rc}" -ne 0 ]] || rc=1; }
  # Only now: the chown runs from the image ID, and removing the tag may delete the image.
  remove_run_tag
else
  scripts/docker_run.sh scripts/ci.sh || rc=$?
fi

if [[ "${rc}" -ne 0 ]]; then
  if [[ "${clean}" == true ]]; then
    rm -f "${full_stamp}"
    # The dependency check ran in this failed run, so its pass is no longer trusted either.
    [[ ${#args[@]} -gt 0 || -z "${deps_stamp}" ]] || rm -f "${deps_stamp}"
  fi
  echo "ci_local: FAILED (exit ${rc}); no pass recorded" >&2
  exit "${rc}"
fi

if [[ "${clean}" != true ]]; then
  echo "ci_local: passed, but the tree was dirty so nothing is recorded"
  exit 0
fi

# Record only if what was checked is still exactly the tree and the image installed at the start.
image_id_end="$(image_id)" || die "passed, but cannot inspect the image afterwards; nothing recorded"
if [[ "${image_id_end}" != "${image_id_start}" ]]; then
  echo "ci_local: passed, but the image changed during the run; nothing recorded" >&2
  exit 1
fi
if [[ "$(snapshot_tree)" != "${tree}" ]]; then
  echo "ci_local: passed, but the snapshot changed during the run; nothing recorded" >&2
  exit 1
fi
write_stamp "${full_stamp}" "${full_prefix}${head_commit}"
[[ -z "${deps_stamp}" ]] || write_stamp "${deps_stamp}" "${deps_prefix}${head_commit}"
echo "ci_local: PASS, recorded for tree ${tree} (commit ${head_commit})"
