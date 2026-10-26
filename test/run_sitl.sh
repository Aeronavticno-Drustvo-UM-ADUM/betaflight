#!/usr/bin/env bash
# Start Betaflight SITL standalone, to connect the configurator to it (no simulator needed).
#
#   test/run_sitl.sh [--build] [--reset] [--verbose]
#
#   --build    rebuild SITL before starting
#   --reset    delete the saved config (eeprom.bin) first
#   --verbose  do not filter the flash write log lines
#
# Configurator: port "Manual Selection", enter tcp://127.0.0.1:5761 (UART1), then Connect.
# Further UARTs listen on 5762, 5763, ... The config is kept in test/sitl-run/eeprom.bin.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/.." && pwd)"
ELF="${ROOT}/obj/main/betaflight_SITL.elf"
RUN_DIR="${HERE}/sitl-run"
PORT=5761
BUILD=0
RESET=0
VERBOSE=0

for arg in "$@"; do
    case "${arg}" in
        --build)   BUILD=1 ;;
        --reset)   RESET=1 ;;
        --verbose) VERBOSE=1 ;;
        -h|--help) sed -n '2,11p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "Unknown option: ${arg} (see --help)" >&2; exit 1 ;;
    esac
done

if [ "${BUILD}" -eq 1 ] || [ ! -x "${ELF}" ]; then
    echo "Building SITL..."
    make -C "${ROOT}" TARGET=SITL -j"$(nproc)"
fi

if ss -ltn 2>/dev/null | grep -q ":${PORT} "; then
    echo "Port ${PORT} is already in use: is another SITL running? (pkill -x betaflight_SITL)" >&2
    exit 1
fi

mkdir -p "${RUN_DIR}"
[ "${RESET}" -eq 1 ] && rm -f "${RUN_DIR}/eeprom.bin"

# Stop the whole pipeline (SITL + filter) when the script ends or is interrupted
cleanup() { trap - EXIT TERM INT; kill 0 2>/dev/null || true; }
trap cleanup EXIT TERM INT

echo "Betaflight SITL: connect the configurator to tcp://127.0.0.1:${PORT} (Manual Selection). Ctrl-C to stop."
cd "${RUN_DIR}"
if [ "${VERBOSE}" -eq 1 ]; then
    stdbuf -o0 -e0 "${ELF}"
else
    stdbuf -o0 -e0 "${ELF}" 2>&1 | grep --line-buffered -v '^\[FLASH_'
fi
