#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PYTHON:-python3}"
cd "$ROOT"
"$PY" -m pip install -r requirements-build.txt
"$PY" packaging/build_portable.py --clean --with-lidar
