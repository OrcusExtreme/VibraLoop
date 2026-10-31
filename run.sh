#!/usr/bin/env bash
# Start ProjectLab_System with the project virtual environment (.venv created by setup.sh).
cd "$(dirname "$0")" || exit 1
export PYTHONUTF8=1
VENV_PY=.venv/bin/python

if [ ! -x "$VENV_PY" ] || ! "$VENV_PY" -c 'import flask_socketio, sklearn, serial, watchdog' >/dev/null 2>&1; then
    echo "[ERROR] The virtual environment .venv is missing or was copied from another PC. Run: bash setup.sh" >&2
    exit 1
fi

echo "Starting ProjectLab_System. Open http://localhost:5000 in a web browser. Press Ctrl+C to stop."
exec "$VENV_PY" cnc_app.py
