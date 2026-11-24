#!/bin/bash
# Read changed paths from stdin, NUL-separated (git diff --name-only -z). Exit 0 only if there is
# at least one path and every path is documentation. Keep the patterns in sync with paths-ignore
# in ci.yml.
set -euo pipefail

count=0
while IFS= read -r -d '' path || [[ -n "${path}" ]]; do
  [[ -n "${path}" ]] || continue
  count=$((count + 1))
  case "${path}" in
    *.md | docs/* | LICENSE) ;;
    *) exit 1 ;;
  esac
done

[[ "${count}" -gt 0 ]]
