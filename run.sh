#!/usr/bin/env bash
# Launch Crosses & Noughts (YDLidar X3).
#
# Prefers the known-good local Python, but also works with PYTHON=/path/to/python
# or the first python3/python on PATH. run.py locates a locally built SDK.
#
# Extra flags pass straight through:
#   ./run.sh                                  settings screen, then play
#   ./run.sh --skip-settings --lidar sim      straight into a simulated game
#   ./run.sh --lidar real                     real hardware on /dev/ttyUSB0
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_PY="/home/levzzz/miniconda3/bin/python"
if [ -n "${PYTHON:-}" ]; then
    PY="$PYTHON"
elif [ -x "$DEFAULT_PY" ]; then
    PY="$DEFAULT_PY"
else
    PY="$(command -v python3 || command -v python)"
fi

if [ ! -x "$PY" ]; then
    echo "Python interpreter not found: $PY" >&2
    exit 1
fi

cd "$REPO"
exec "$PY" run.py "$@"
