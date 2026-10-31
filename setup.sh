#!/usr/bin/env bash
# ProjectLab_System environment setup for Linux/macOS.
# Creates .venv with 64-bit Python 3.12 - 3.14, installs requirements.txt and checks the result.
# Usage: bash setup.sh [--recreate] [--download-wheels]
cd "$(dirname "$0")" || exit 1
export PYTHONUTF8=1
export PIP_DISABLE_PIP_VERSION_CHECK=1

for candidate in python3.14 python3.13 python3.12 python3 python; do
    if command -v "$candidate" >/dev/null 2>&1 &&
        "$candidate" -c 'import sys; sys.exit(0 if (3, 12) <= sys.version_info[:2] <= (3, 14) and sys.maxsize > 2**32 else 1)' >/dev/null 2>&1; then
        echo "Using $(command -v "$candidate")"
        exec "$candidate" scripts/setup_env.py "$@"
    fi
done

echo "[ERROR] 64-bit Python 3.12 - 3.14 was not found. Install python3.12 or newer (with the venv module) and run again." >&2
exit 1
