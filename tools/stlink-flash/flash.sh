#!/usr/bin/env bash
# Flash a built target over ST-Link/SWD.
#
#   flash.sh [TARGET] [--build] [--erase-chip] [--no-reset] [--dry-run]
#
# TARGET defaults to SATELLITEBOARD; the newest obj/*_TARGET_*.hex is flashed.
# The chip is detected from DBGMCU_IDCODE and the matching pyocd target is chosen
# (installing its CMSIS pack on first use). Override with PYOCD_TARGET=<name>.
# Everything is also written to logs/flash-<timestamp>.log.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/../.." && pwd)"
PYOCD="${HERE}/stlinkenv/bin/pyocd"
LOG_DIR="${HERE}/logs"
LOG_FILE="${LOG_DIR}/flash-$(date +%Y%m%d-%H%M%S).log"

TARGET=SATELLITEBOARD
BUILD=0
ERASE=auto
RESET=1
DRY_RUN=0

# --- logging -----------------------------------------------------------------
if [ -t 2 ]; then C_RED=$'\e[31m'; C_YEL=$'\e[33m'; C_GRN=$'\e[32m'; C_OFF=$'\e[0m'; else C_RED=; C_YEL=; C_GRN=; C_OFF=; fi
mkdir -p "${LOG_DIR}"
: > "${LOG_FILE}"

_log() { # level colour message...
    local level="$1" colour="$2"; shift 2
    printf '%s[%s] %-5s%s %s\n' "${colour}" "$(date +%H:%M:%S)" "${level}" "${C_OFF}" "$*" >&2
    printf '[%s] %-5s %s\n' "$(date +%H:%M:%S)" "${level}" "$*" >> "${LOG_FILE}"
}
info()  { _log INFO  "" "$@"; }
ok()    { _log OK    "${C_GRN}" "$@"; }
warn()  { _log WARN  "${C_YEL}" "$@"; }
error() { _log ERROR "${C_RED}" "$@"; }
hint()  { _log HINT  "${C_YEL}" "$@"; }
die()   { error "$1"; shift; for h in "$@"; do hint "${h}"; done; error "Log: ${LOG_FILE}"; exit 1; }

# Run a command, show its output live and keep it in the log and in $OUT.
run() {
    OUT="$("$@" 2>&1 | tee -a "${LOG_FILE}" >&2; exit "${PIPESTATUS[0]}")"
    return $?
}
# Same but quiet on screen (output goes to the log and $OUT only).
run_quiet() {
    OUT="$("$@" 2>&1)"; local rc=$?
    printf '%s\n' "${OUT}" >> "${LOG_FILE}"
    return ${rc}
}

# --- arguments ---------------------------------------------------------------
for arg in "$@"; do
    case "${arg}" in
        --build)       BUILD=1 ;;
        --erase-chip)  ERASE=chip ;;
        --no-reset)    RESET=0 ;;
        --dry-run)     DRY_RUN=1 ;;
        -h|--help)     sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        -*)            die "Unknown option: ${arg}" "Use --help for usage" ;;
        *)             TARGET="${arg}" ;;
    esac
done

info "Target ${TARGET}, log ${LOG_FILE}"

# --- environment -------------------------------------------------------------
[ -x "${PYOCD}" ] || die "stlinkenv is missing" "Run ${HERE}/setup_env.sh first"

# --- build -------------------------------------------------------------------
if [ "${BUILD}" -eq 1 ]; then
    info "Building ${TARGET}"
    run make -C "${ROOT}" TARGET="${TARGET}" -j"$(nproc)" || die "Build failed" "See the make output above"
fi

# --- firmware image ----------------------------------------------------------
HEX="$(ls -t "${ROOT}"/obj/*_"${TARGET}"_*.hex 2>/dev/null | head -n1 || true)"
[ -n "${HEX}" ] || die "No hex file for ${TARGET} in ${ROOT}/obj" "Build it: make TARGET=${TARGET}  (or pass --build)"
[ -s "${HEX}" ] || die "Hex file is empty: ${HEX}" "Rebuild the target"
info "Image: $(basename "${HEX}") ($(stat -c %s "${HEX}") bytes, built $(date -r "${HEX}" '+%F %T'))"
if [ -n "$(find "${ROOT}/src/main" -newer "${HEX}" \( -name '*.c' -o -name '*.h' \) -print -quit 2>/dev/null)" ]; then
    warn "Sources are newer than the hex: it may be out of date (use --build)"
fi

# --- probe -------------------------------------------------------------------
if ! run_quiet "${PYOCD}" list; then
    die "pyocd could not list probes" "Output: ${OUT}"
fi
if ! grep -qiE 'st-?link' <<<"${OUT}"; then
    if lsusb 2>/dev/null | grep -qi '0483:'; then
        die "An ST device is on USB but pyocd cannot open it (permissions)" \
            "Run ${HERE}/fix_permissions.sh once, then unplug and replug the ST-Link"
    fi
    die "No ST-Link found" "Check the USB cable and that the ST-Link is plugged in" "lsusb should list ID 0483:xxxx"
fi
ok "Probe: $(grep -iE 'st-?link' <<<"${OUT}" | head -n1 | sed 's/  */ /g')"

# --- chip detection ----------------------------------------------------------
detect_target() {
    # Generic Cortex-M connection, read DBGMCU_IDCODE (F1 family at 0xE0042000)
    if ! run_quiet "${PYOCD}" commander -t cortex_m -c "read32 0xE0042000"; then
        return 1
    fi
    DEV_ID_RAW="$(grep -oiE '^e0042000: +[0-9a-f]{8}' <<<"${OUT}" | awk '{print $2}')"
    [ -n "${DEV_ID_RAW}" ] || return 1
    DEV_ID=$(( 0x${DEV_ID_RAW} & 0xFFF ))
    case "${DEV_ID}" in
        $((0x412))) CHIP_DESC="STM32F103 low density (16-32 KB)";    DETECTED=stm32f103c6 ;;
        $((0x410))) CHIP_DESC="STM32F103 medium density (64-128 KB)"; DETECTED=stm32f103cb ;;
        $((0x414))) CHIP_DESC="STM32F103 high density (256-512 KB)";  DETECTED=stm32f103rc ;;
        $((0x430))) CHIP_DESC="STM32F103 XL density (768 KB-1 MB)";   DETECTED=stm32f103rc ;;
        *)          CHIP_DESC="unknown (DEV_ID 0x$(printf '%03x' "${DEV_ID}"))"; DETECTED= ;;
    esac
    return 0
}

if [ -n "${PYOCD_TARGET:-}" ]; then
    CHIP="${PYOCD_TARGET}"
    info "Using PYOCD_TARGET=${CHIP} (no chip detection)"
else
    detect_target || die "Cannot read the chip over SWD" \
        "Is the SWD cable connected (SWDIO, SWCLK, GND, 3V3)?" \
        "Is the board powered? Try holding NRST while running the script" \
        "Last output: $(tail -n1 <<<"${OUT}")"
    info "Chip: ${CHIP_DESC}, IDCODE 0x${DEV_ID_RAW}"
    [ -n "${DETECTED}" ] || die "Unsupported chip" "Set PYOCD_TARGET=<pyocd target name> (pyocd list --targets)"
    CHIP="${DETECTED}"
fi

# Targets that are not built into pyocd need their CMSIS pack
if ! "${PYOCD}" list --targets 2>/dev/null | grep -qE "^ +${CHIP} "; then
    info "pyocd target ${CHIP} needs its CMSIS pack, installing"
    run "${PYOCD}" pack install "${CHIP}" || die "Pack install failed for ${CHIP}" \
        "This needs network access once. Check the connection and retry"
    "${PYOCD}" list --targets 2>/dev/null | grep -qE "^ +${CHIP} " || die "Target ${CHIP} still unknown after pack install"
fi
info "pyocd target: ${CHIP}"

# --- flash -------------------------------------------------------------------
FLASH_ARGS=(flash --target "${CHIP}" --erase "${ERASE}")
[ "${RESET}" -eq 1 ] || FLASH_ARGS+=(--no-reset)

if [ "${DRY_RUN}" -eq 1 ]; then
    info "Dry run: ${PYOCD} ${FLASH_ARGS[*]} ${HEX}"
    exit 0
fi

info "Flashing (erase: ${ERASE})"
if run "${PYOCD}" "${FLASH_ARGS[@]}" "${HEX}"; then
    ok "Flashed ${TARGET}"
    [ "${RESET}" -eq 1 ] && info "Target reset after programming"
    info "First flash of a new layout: in the CLI run 'defaults' then 'save'"
    exit 0
fi

# --- failure analysis --------------------------------------------------------
error "Flashing failed"
if grep -qiE 'page failure|result code' <<<"${OUT}"; then
    hint "Flash programming failed: wrong flash algorithm / page size for the chip, or write protection"
    hint "Chip is ${CHIP_DESC:-unknown}, tried pyocd target ${CHIP}. Override with PYOCD_TARGET=<name>"
    hint "Try --erase-chip; if it still fails, check the option bytes (read/write protection)"
elif grep -qiE 'no available debug probes|no connected' <<<"${OUT}"; then
    hint "The probe disappeared during flashing: check the USB cable, replug the ST-Link"
elif grep -qiE 'read protect|RDP|protected' <<<"${OUT}"; then
    hint "The chip is read-protected: remove protection (this erases the flash) before flashing"
elif grep -qiE 'transfer fault|dp error|timeout|swd|no ack' <<<"${OUT}"; then
    hint "SWD communication problem: check wiring and board power, try holding NRST"
else
    hint "See the pyocd output above and ${LOG_FILE}"
fi
error "Log: ${LOG_FILE}"
exit 1
