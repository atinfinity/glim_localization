#!/bin/bash
# Regenerate scripts/requirements.txt from scripts/requirements.in in a clean venv (inside the container):
#   scripts/docker_run.sh scripts/update_requirements.sh
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv=/tmp/requirements_venv

rm -rf "${venv}"
python3 -m venv "${venv}"
"${venv}/bin/pip" install --quiet -r "${repo_root}/scripts/requirements.in"
{
  echo "# Fully pinned by 'pip freeze' in the container built from requirements.in (Python 3.12, Ubuntu 24.04)."
  echo "# Regenerate with scripts/update_requirements.sh when requirements.in changes."
  "${venv}/bin/pip" freeze
} > "${repo_root}/scripts/requirements.txt"
echo "wrote scripts/requirements.txt"
