#!/bin/bash
# Read changed paths from stdin, NUL-separated (git diff --name-only -z). Exit 0 only if there is
# at least one path and every path is documentation: Markdown, docs/, LICENSE, or an input of the
# docs site, which .github/workflows/docs.yml builds and checks. Keep the patterns in sync with
# paths-ignore in ci.yml.
set -euo pipefail

count=0
while IFS= read -r -d '' path || [[ -n "${path}" ]]; do
  [[ -n "${path}" ]] || continue
  count=$((count + 1))
  case "${path}" in
    *.md | docs/* | LICENSE) ;;
    zensical.toml | requirements-docs.txt | scripts/docs_site_index.py | .github/workflows/docs.yml) ;;
    *) exit 1 ;;
  esac
done

[[ "${count}" -gt 0 ]]
