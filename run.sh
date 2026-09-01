#!/usr/bin/env bash
# Launch Crosses & Noughts (YDLidar X3).
#
# Handles the two things that are easy to forget:
#   * the miniconda python (system python 3.14 segfaults in the lidar SDK)
#   * PYTHONPATH for the SWIG module, needed by real-lidar mode
#
# Extra flags pass straight through:
#   ./run.sh                                  settings screen, then play
#   ./run.sh --skip-settings --lidar sim      straight into a simulated game
#   ./run.sh --lidar real                     real hardware on /dev/ttyUSB0
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="/home/levzzz/miniconda3/bin/python"

if [ ! -x "$PY" ]; then
    echo "miniconda python not found at $PY" >&2
    exit 1
fi

export PYTHONPATH="$REPO/YDLidar-SDK/build/python${PYTHONPATH:+:$PYTHONPATH}"

cd "$REPO"
exec "$PY" -m game.run "$@"
