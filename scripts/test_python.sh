#!/bin/bash
# Run the Python tests of the tooling scripts (inside the container).
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${GLIM_LOC_PYTHON:?GLIM_LOC_PYTHON is not set (run inside the dev container)}"

export PYTHONDONTWRITEBYTECODE=1
# The ROS environment puts incompatible packages on PYTHONPATH.
cd "${repo_root}"
env -u PYTHONPATH "${GLIM_LOC_PYTHON}" -m pytest -q -p no:cacheprovider scripts/tests
echo "PASS: python tests"
