#!/usr/bin/env bash
# Start Betaflight SITL standalone, to connect the configurator to it (no simulator needed).
#
#   test/tools/run_sitl.sh [--servo] [--config] [--build] [--reset] [--verbose]
#
#   --servo    run the servo driver SITL (USE_SERVO_DRIVER, obj/main_servotest) instead of stock SITL
#   --config   with --servo: apply test/servo_config.txt through the CLI after start (SITL then reboots)
#   --build    rebuild SITL before starting
#   --reset    delete the saved config (eeprom.bin) first
#   --verbose  do not filter the flash write log lines
#
# Configurator: port "Manual Selection", enter tcp://127.0.0.1:5761 (UART1), then Connect.
# Further UARTs listen on 5762 (UART2, MAVLink with servo_config.txt), 5763 (UART3, GPS), ...
# The config is kept in test/sitl-run/eeprom.bin (stock) or test/sitl-run-servo/eeprom.bin (--servo).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/../.." && pwd)"
PORT=5761
SERVO=0
CONFIG=0
BUILD=0
RESET=0
VERBOSE=0

for arg in "$@"; do
    case "${arg}" in
        --servo)   SERVO=1 ;;
        --config)  CONFIG=1 ;;
        --build)   BUILD=1 ;;
        --reset)   RESET=1 ;;
        --verbose) VERBOSE=1 ;;
        -h|--help) sed -n '2,14p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "Unknown option: ${arg} (see --help)" >&2; exit 1 ;;
    esac
done

if [ "${SERVO}" -eq 1 ]; then
    OBJ_DIR="${ROOT}/obj/main_servotest"
    RUN_DIR="${HERE}/../sitl-run-servo"    # not sitl-run-servotest: the test harness deletes its eeprom.bin
    MAKE_EXTRA=(OBJECT_DIR="${OBJ_DIR}" EXTRA_FLAGS=-DUSE_SERVO_DRIVER)
else
    [ "${CONFIG}" -eq 1 ] && { echo "--config needs --servo" >&2; exit 1; }
    OBJ_DIR="${ROOT}/obj/main"
    RUN_DIR="${HERE}/../sitl-run"
    MAKE_EXTRA=()
fi
ELF="${OBJ_DIR}/betaflight_SITL.elf"

if [ "${BUILD}" -eq 1 ] || [ ! -x "${ELF}" ]; then
    echo "Building SITL..."
    # SITL uses the host gcc: dummy GCC_VERSION/GCC_REQUIRED_VERSION skip the arm-none-eabi-gcc check
    make -C "${ROOT}" TARGET=SITL GCC_VERSION=none GCC_REQUIRED_VERSION=none "${MAKE_EXTRA[@]}" -j"$(nproc)"
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

if [ "${CONFIG}" -eq 1 ]; then
    # Apply servo_config.txt over the CLI (same code as the test harness). In SITL, `save` exits the process;
    # the normal start below then loads the saved config.
    echo "Applying test/servo_config.txt..."
    (cd "${RUN_DIR}" && exec "${ELF}" >/dev/null 2>&1) &
    CFG_PID=$!
    for _ in $(seq 50); do ss -ltn 2>/dev/null | grep -q ":${PORT} " && break; sleep 0.1; done
    python3 - "${HERE}/../common" <<'PYEOF'
import sys
sys.path.insert(0, sys.argv[1])
import harness
sock = harness.cli_session()
if sock is None:
    sys.exit("No CLI banner on tcp 5761, config not applied; run again (SITL startup race, TODO K)")
problems = harness.apply_config(False, sock)
print("servo_config.txt applied" + "".join("\n  problem: " + p for p in problems), flush=True)
PYEOF
    for _ in $(seq 50); do kill -0 "${CFG_PID}" 2>/dev/null || break; sleep 0.1; done
    kill "${CFG_PID}" 2>/dev/null || true
    wait "${CFG_PID}" 2>/dev/null || true
    for _ in $(seq 50); do ss -ltn 2>/dev/null | grep -q ":${PORT} " || break; sleep 0.1; done
fi

echo "Betaflight SITL: connect the configurator to tcp://127.0.0.1:${PORT} (Manual Selection). Ctrl-C to stop."
cd "${RUN_DIR}"
if [ "${VERBOSE}" -eq 1 ]; then
    stdbuf -o0 -e0 "${ELF}"
else
    stdbuf -o0 -e0 "${ELF}" 2>&1 | grep --line-buffered -v '^\[FLASH_'
fi
