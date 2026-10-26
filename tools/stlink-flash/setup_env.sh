#!/usr/bin/env bash
# Create the Python environment (stlinkenv) used for flashing over ST-Link/SWD.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_DIR="${HERE}/stlinkenv"

if [ ! -x "${ENV_DIR}/bin/python" ]; then
    python3 -m venv "${ENV_DIR}"
fi
"${ENV_DIR}/bin/pip" install --quiet --upgrade pip
"${ENV_DIR}/bin/pip" install --quiet -r "${HERE}/requirements.txt"

echo "stlinkenv ready: ${ENV_DIR}"
"${ENV_DIR}/bin/pyocd" --version
