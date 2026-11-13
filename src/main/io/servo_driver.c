/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>   // before platform.h: the MAVLink headers include it and sprintf is poisoned afterwards
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SERVO_DRIVER

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/motor.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/servos.h"

#include "io/adum_mavlink.h"
#include "io/gps.h"
#include "io/serial.h"
#include "io/servo_driver.h"

#ifdef USE_CLI
#include "cli/cli.h"
#endif

#include "pg/motor.h"
#include "pg/pg_ids.h"

#define SERVO_DRIVER_ONE_SECOND_US      1000000
#define SERVO_DRIVER_MAVLINK_SYSID      1       // not checked on receive, placeholders until the ids are agreed
#define SERVO_DRIVER_MAVLINK_COMPID     1
#define SERVO_DRIVER_PULSE_MIN          500     // the only limits applied to servos
#define SERVO_DRIVER_PULSE_MAX          2500
#define SERVO_DRIVER_PULSE_INVALID      0       // 0 and UINT16_MAX in ADUM_AUX_CHANNELS mean "no value"
#define SERVO_DRIVER_HEADING_MIN_SPEED  300     // mm/s, below this heading_motion is not flagged valid
#define SERVO_DRIVER_DEFAULT_BAUD       BAUD_115200

#define SERVO_DRIVER_LINK_READY_FRAMES  5       // P1: consecutive valid AUX frames
#define SERVO_DRIVER_LINK_CRC_QUIET_US  1000000 // P1: no CRC error for this long
#define SERVO_DRIVER_CONFIG_QUIET_US    2000000 // P4: no MSP write / motor test for this long
#define SERVO_DRIVER_REJECT_TEXT_US     500000  // ARM REJECTED text at most 2 Hz
#define SERVO_DRIVER_LOW_RATE_HZ        20      // below this AUX frame rate: warning
#define SERVO_DRIVER_LOW_RATE_TEXT_US   10000000
#define SERVO_DRIVER_PARSER_RESET_US    1000000 // no valid frame for this long: reset the parser, every 1 s
#define SERVO_DRIVER_UART_REINIT_US     2000000 // no valid frame for this long: re-init the UART, every 2 s
#define SERVO_DRIVER_SILENCE_CAP_US     10000000 // cap of the silence measured while no session; must stay > max link timeout (5 s) and > UART re-init time

#define MAVLINK_CHANNEL                 MAVLINK_COMM_0

PG_REGISTER_WITH_RESET_TEMPLATE(servoDriverConfig_t, servoDriverConfig, PG_SERVO_DRIVER_CONFIG, 0);

PG_RESET_TEMPLATE(servoDriverConfig_t, servoDriverConfig,
    .linkTimeoutMs = 500,
    .bootGraceMs = 2000,
);

typedef enum {
    SD_DISARMED = 0,
    SD_ARMED,
    SD_FAILSAFE,
} servoDriverState_e;

typedef enum {
    REJECT_LINK_NOT_READY = 0,
    REJECT_BOOT_GRACE,
    REJECT_MOTOR_CONFIG,
    REJECT_CONFIG_ACTIVE,
    REJECT_NONE,
} rejectReason_e;

static const char *const rejectText[] = {
    [REJECT_LINK_NOT_READY] = "ARM REJECTED: LINK_NOT_READY",
    [REJECT_BOOT_GRACE]     = "ARM REJECTED: BOOT_GRACE",
    [REJECT_MOTOR_CONFIG]   = "ARM REJECTED: MOTOR_CONFIG",
    [REJECT_CONFIG_ACTIVE]  = "ARM REJECTED: CONFIG_ACTIVE",
};

static serialPort_t *mavlinkPort;
static mavlink_message_t rxMessage;
static mavlink_status_t rxStatus;

static uint16_t motorPulse[SERVO_DRIVER_MOTOR_COUNT];
static servoDriverState_e state;
static bool sessionActive;      // link session: valid ADUM_AUX_CHANNELS seen and none missing for longer than the link timeout
static bool prevFrameArmed;     // arming_status of the previous AUX frame of the session
static uint8_t validFrames;     // consecutive valid AUX frames of the session (saturates), cleared by a parse/CRC error
static bool sessionAtLastTick;  // the current session was already active at the last 1 Hz tick
static timeUs_t lastAuxUs;      // last valid ADUM_AUX_CHANNELS, the only frame that refreshes the link timer
static timeUs_t lastCrcErrorUs;
static bool haveCrcError;
static bool bootGraceDone;
static timeUs_t lastConfigActivityUs;
static bool haveConfigActivity;
static bool saveBusy;

static timeUs_t lastTickUs;     // 1 Hz: HEARTBEAT, SYS_STATUS, window statistics
static timeUs_t lastRejectTextUs;
static bool haveRejectText;
static timeUs_t lastLowRateTextUs;
static bool haveLowRateText;
static timeUs_t lastParserResetUs;
#ifndef SIMULATOR_BUILD
static timeUs_t lastUartResetUs;
#endif

// Counters of SYS_STATUS (wrap at 65535)
static uint16_t errorsComm;     // CRC / parse errors since boot
static uint16_t txDrops;        // frames dropped because the TX buffer was full
static uint16_t clamped;        // values clamped or replaced by "no value" handling
// Window of the last second
static uint16_t winAuxFrames;
static uint16_t winMessages;
static uint16_t winLost;
static bool haveSeq;
static uint8_t lastSeq;

#ifdef USE_SERVO_DRIVER_DIAG
// Diagnostic counters, read over SWD (test/tools/read_diag.py, one "mdw" of the whole struct). Wrap freely.
// Word layout = field order below: identity 0-2, frames 3-6, arm edges and rejects 7-11, mirrors 12-13, transitions 14-19, timing 20-23.
// bootCount: F1 has no no-init RAM section (startup clears .bss), so it is always 1 here.
typedef enum {
    DIAG_CAUSE_NONE = 0,
    DIAG_CAUSE_FIRST_FRAME_ARMED,   // first frame of a session says ARMED
    DIAG_CAUSE_EDGE_PASS,           // observed arm edge passed the checks
    DIAG_CAUSE_DISARM_FRAME,        // frame says DISARMED while armed
    DIAG_CAUSE_LINK_TIMEOUT,        // link timeout while armed -> FAILSAFE
    DIAG_CAUSE_FAILSAFE_RECOVER,    // frame after FAILSAFE
} servoDriverDiagCause_e;

typedef struct {
    uint32_t magic;                 // 0x5D1A6001, set at init
    uint32_t resetCsr;              // RCC->CSR at boot (reset cause)
    uint32_t bootCount;             // always 1 (see above)
    uint32_t auxFrames;             // valid ADUM_AUX_CHANNELS frames
    uint32_t auxArmedFrames;
    uint32_t auxDisarmedFrames;
    uint32_t newSessions;
    uint32_t armEdges;              // observed arm edges that went through the checks
    uint32_t edgeRejects;
    uint32_t lastRejectReason;      // rejectReason_e
    uint32_t rejectTextsSent;
    uint32_t rejectTextsDropped;    // servoDriverSend failed for the reject text
    uint32_t txDrops;               // mirror
    uint32_t parseErrors;           // mirror of errorsComm
    uint32_t stateChanges;
    uint32_t lastFromState;         // servoDriverState_e
    uint32_t lastToState;
    uint32_t lastCause;             // servoDriverDiagCause_e
    uint32_t lastAuxFrameNo;        // auxFrames at the last transition
    uint32_t lastTransitionMs;
    uint32_t maxRxBytesPerCycle;    // bytes read in one servoDriverUpdate call
    uint32_t maxCycleGapUs;         // largest gap between two servoDriverUpdate calls
    uint32_t firstArmedFrameNo;     // auxFrames of the first ARMED frame after a DISARMED one
    uint32_t armedAtFrameNo;        // auxFrames when the state became ARMED
} servoDriverDiag_t;

#define SERVO_DRIVER_DIAG_MAGIC 0x5D1A6001

volatile servoDriverDiag_t servoDriverDiag __attribute__((used));   // not static: findable by name in the ELF

static timeUs_t diagLastUpdateUs;
static bool diagHaveLastUpdate;
static uint32_t diagRxBytesThisCycle;

// Sets the magic, the reset cause and the boot count
static void diagInit(void)
{
    servoDriverDiag.magic = SERVO_DRIVER_DIAG_MAGIC;
#ifdef STM32F10X
    servoDriverDiag.resetCsr = cachedRccCsrValue;
#endif
    servoDriverDiag.bootCount = 1;
}

// Counts one valid AUX frame and detects the first ARMED frame after a DISARMED one
static void diagNoteAuxFrame(bool armedFrame, bool newSession, bool previousFrameArmed)
{
    servoDriverDiag.auxFrames++;
    if (armedFrame) {
        servoDriverDiag.auxArmedFrames++;
    } else {
        servoDriverDiag.auxDisarmedFrames++;
    }
    if (newSession) {
        servoDriverDiag.newSessions++;
    }
    if (armedFrame && !previousFrameArmed) {
        servoDriverDiag.firstArmedFrameNo = servoDriverDiag.auxFrames;
    }
}

// Counts one arm edge that went through the checks, and its result
static void diagNoteArmEdge(rejectReason_e reason)
{
    servoDriverDiag.armEdges++;
    if (reason != REJECT_NONE) {
        servoDriverDiag.edgeRejects++;
        servoDriverDiag.lastRejectReason = reason;
    }
}

// Counts a reject text that was sent or dropped
static void diagNoteRejectText(bool sent)
{
    if (sent) {
        servoDriverDiag.rejectTextsSent++;
    } else {
        servoDriverDiag.rejectTextsDropped++;
    }
}

// Records the last state transition and its cause
static void diagNoteTransition(servoDriverState_e from, servoDriverState_e to, servoDriverDiagCause_e cause)
{
    servoDriverDiag.stateChanges++;
    servoDriverDiag.lastFromState = from;
    servoDriverDiag.lastToState = to;
    servoDriverDiag.lastCause = cause;
    servoDriverDiag.lastAuxFrameNo = servoDriverDiag.auxFrames;
    servoDriverDiag.lastTransitionMs = millis();
    if (to == SD_ARMED) {
        servoDriverDiag.armedAtFrameNo = servoDriverDiag.auxFrames;
    }
}

// Measures the gap between two update calls
static void diagNoteCycleStart(timeUs_t currentTimeUs)
{
    diagRxBytesThisCycle = 0;
    if (diagHaveLastUpdate) {
        const uint32_t gapUs = (uint32_t)cmpTimeUs(currentTimeUs, diagLastUpdateUs);
        if (gapUs > servoDriverDiag.maxCycleGapUs) {
            servoDriverDiag.maxCycleGapUs = gapUs;
        }
    }
    diagLastUpdateUs = currentTimeUs;
    diagHaveLastUpdate = true;
}

// Counts one received byte
static void diagNoteRxByte(void)
{
    diagRxBytesThisCycle++;
}

// Closes the cycle: max bytes read, and the mirrors of the SYS_STATUS counters
static void diagNoteCycleEnd(uint16_t txDropCount, uint16_t parseErrorCount)
{
    if (diagRxBytesThisCycle > servoDriverDiag.maxRxBytesPerCycle) {
        servoDriverDiag.maxRxBytesPerCycle = diagRxBytesThisCycle;
    }
    servoDriverDiag.txDrops = txDropCount;
    servoDriverDiag.parseErrors = parseErrorCount;
}
#else
typedef int servoDriverDiagCause_e;     // cause is ignored without the diagnostics
enum {
    DIAG_CAUSE_NONE,
    DIAG_CAUSE_FIRST_FRAME_ARMED,
    DIAG_CAUSE_EDGE_PASS,
    DIAG_CAUSE_DISARM_FRAME,
    DIAG_CAUSE_LINK_TIMEOUT,
    DIAG_CAUSE_FAILSAFE_RECOVER,
};
#define diagInit()
#define diagNoteAuxFrame(armedFrame, newSession, previousFrameArmed)
#define diagNoteArmEdge(reason)
#define diagNoteRejectText(sent)
#define diagNoteTransition(from, to, cause)
#define diagNoteCycleStart(currentTimeUs)
#define diagNoteRxByte()
#define diagNoteCycleEnd(txDropCount, parseErrorCount)
#endif

static bool pulseValid(uint16_t pulseUs)
{
    return pulseUs != SERVO_DRIVER_PULSE_INVALID && pulseUs != UINT16_MAX;
}

// Motors are held at the minimum command (zero throttle) unless armed. Called every task cycle.
static void servoDriverWriteMotors(void)
{
    float values[SERVO_DRIVER_MOTOR_COUNT];
    for (int i = 0; i < SERVO_DRIVER_MOTOR_COUNT; i++) {
        values[i] = (state == SD_ARMED) ? motorPulse[i] : motorConfig()->mincommand;
    }
    motorWriteAll(values);
}

// Servos always follow the pulse that arrived, whatever the state. No limits apart from the clamp.
static void servoDriverWriteServo(int index, uint16_t pulseUs)
{
    const uint16_t limited = constrain(pulseUs, SERVO_DRIVER_PULSE_MIN, SERVO_DRIVER_PULSE_MAX);
    if (limited != pulseUs) {
        clamped++;
    }
    pwmWriteServo(index, limited);
}

// Sends a frame if the port has room for all of it (never blocks, never sends half a frame). Returns false if dropped.
static bool servoDriverSend(const mavlink_message_t *msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t length = mavlink_msg_to_send_buffer(buffer, msg);
    if (!mavlinkPort || serialTxBytesFree(mavlinkPort) < length) {
        txDrops++;
        return false;
    }
    serialWriteBuf(mavlinkPort, buffer, length);
    return true;
}

static bool servoDriverSendStatusText(uint8_t severity, const char *text)
{
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(SERVO_DRIVER_MAVLINK_SYSID, SERVO_DRIVER_MAVLINK_COMPID, &msg, severity, text, 0, 0);
    return servoDriverSend(&msg);
}

// Changes the state; the cause is only recorded by the diagnostics
static void servoDriverSetState(servoDriverState_e newState, servoDriverDiagCause_e cause)
{
    if (newState == state) {
        return;
    }
    diagNoteTransition(state, newState, cause);
    state = newState;
    for (int i = 0; i < SERVO_DRIVER_MOTOR_COUNT; i++) {
        motorPulse[i] = motorConfig()->mincommand;
    }
    // ARMED flag stays set in FAILSAFE (armed intent): the upstream "not while armed" checks keep working
    if (state != SD_DISARMED) {
        ENABLE_ARMING_FLAG(ARMED);
    } else {
        DISABLE_ARMING_FLAG(ARMED);
    }
}

static void servoDriverSendHeartbeat(void)
{
    uint8_t baseMode = 0;
    uint8_t systemStatus = MAV_STATE_STANDBY;
    if (state == SD_ARMED) {
        baseMode = MAV_MODE_FLAG_SAFETY_ARMED;
        systemStatus = MAV_STATE_ACTIVE;
    } else if (state == SD_FAILSAFE) {
        systemStatus = MAV_STATE_CRITICAL;
    }
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(SERVO_DRIVER_MAVLINK_SYSID, SERVO_DRIVER_MAVLINK_COMPID, &msg,
        MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, baseMode, 0, systemStatus);
    servoDriverSend(&msg);
}

static void servoDriverSendSysStatus(void)
{
    // lost AUX frames of the last second from sequence gaps, in 0.01 %
    const uint32_t total = (uint32_t)winMessages + winLost;
    const uint16_t dropRate = total ? (uint16_t)((uint32_t)winLost * 10000 / total) : 0;
    mavlink_message_t msg;
    mavlink_msg_sys_status_pack(SERVO_DRIVER_MAVLINK_SYSID, SERVO_DRIVER_MAVLINK_COMPID, &msg,
        0, 0, 0, 0, 0, 0, 0, dropRate, errorsComm, txDrops, clamped, 0, 0, 0, 0, 0);
    servoDriverSend(&msg);
}

// Days since 1970-01-01 of a proleptic Gregorian date (Howard Hinnant's algorithm)
static int32_t daysFromCivil(int32_t year, uint8_t month, uint8_t day)
{
    year -= month <= 2;
    const int32_t era = (year >= 0 ? year : year - 399) / 400;
    const int32_t yearOfEra = year - era * 400;
    const int32_t dayOfYear = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
    const int32_t dayOfEra = yearOfEra * 365 + yearOfEra / 4 - yearOfEra / 100 + dayOfYear;
    return era * 146097 + dayOfEra - 719468;
}

static void servoDriverSendGpsPvt(const gpsPvt_t *pvt)
{
    const bool fixOk = pvt->flags & 0x01;
    const bool diffCorrections = pvt->flags & 0x02;
    const unsigned carrierSolution = (pvt->flags >> 6) & 0x03;       // 1 float, 2 fixed

    uint8_t fixType = GPS_FIX_TYPE_NO_FIX;                            // also time-only and dead reckoning only
    if (fixOk && pvt->fixType == 2) {
        fixType = GPS_FIX_TYPE_2D_FIX;
    } else if (fixOk && (pvt->fixType == 3 || pvt->fixType == 4)) {
        if (carrierSolution == 2) {
            fixType = GPS_FIX_TYPE_RTK_FIXED;
        } else if (carrierSolution == 1) {
            fixType = GPS_FIX_TYPE_RTK_FLOAT;
        } else if (diffCorrections) {
            fixType = GPS_FIX_TYPE_DGPS;
        } else {
            fixType = GPS_FIX_TYPE_3D_FIX;
        }
    }

    uint8_t flags = 0;
    uint64_t timeUsec = 0;
    if ((pvt->valid & 0x03) == 0x03 && pvt->year >= 1970) {          // validDate and validTime
        const int64_t seconds = (int64_t)daysFromCivil(pvt->year, pvt->month, pvt->day) * 86400
            + pvt->hour * 3600 + pvt->min * 60 + pvt->sec;
        timeUsec = (uint64_t)(seconds * 1000000 + pvt->nano / 1000);
        flags |= ADUM_GPS_PVT_FLAGS_TIME_VALID;
    }
    if (diffCorrections || carrierSolution) {
        flags |= ADUM_GPS_PVT_FLAGS_DIFF_CORRECTIONS;
    }
    if (fixType >= GPS_FIX_TYPE_3D_FIX && pvt->gSpeed >= SERVO_DRIVER_HEADING_MIN_SPEED) {
        flags |= ADUM_GPS_PVT_FLAGS_HEADING_VALID;
    }

    mavlink_message_t msg;
    mavlink_msg_adum_gps_pvt_pack(SERVO_DRIVER_MAVLINK_SYSID, SERVO_DRIVER_MAVLINK_COMPID, &msg,
        timeUsec, pvt->lat, pvt->lon, pvt->hMSL, pvt->height, pvt->velN, pvt->velE, pvt->velD, pvt->headMot,
        pvt->hAcc, pvt->vAcc, pvt->sAcc, pvt->pDOP ? pvt->pDOP : UINT16_MAX, fixType, pvt->numSV, flags);
    servoDriverSend(&msg);
}

// Own-health checks P1-P5 of the arm edge (docs/arming_safety.md section 4)
static rejectReason_e servoDriverCheckArmEdge(void)
{
    if (validFrames < SERVO_DRIVER_LINK_READY_FRAMES
        || haveCrcError) {
        return REJECT_LINK_NOT_READY;
    }
    if (!bootGraceDone) {
        return REJECT_BOOT_GRACE;
    }
    if (motorConfig()->dev.motorPwmProtocol == PWM_TYPE_DISABLED || !motorIsEnabled()) {
        return REJECT_MOTOR_CONFIG;
    }
    if (
#ifdef USE_CLI
        cliMode ||
#endif
        saveBusy
        || haveConfigActivity) {
        return REJECT_CONFIG_ACTIVE;
    }
    return REJECT_NONE;
}

static void servoDriverHandleAux(const mavlink_message_t *msg, timeUs_t currentTimeUs)
{
    mavlink_adum_aux_channels_t aux;
    mavlink_msg_adum_aux_channels_decode(msg, &aux);
    const bool armedFrame = (aux.arming_status == ADUM_ARMING_STATUS_ARMED);

    const bool newSession = !sessionActive;
    if (newSession) {
        sessionActive = true;
        sessionAtLastTick = false;
        validFrames = 0;
    }
    diagNoteAuxFrame(armedFrame, newSession, prevFrameArmed);
    if (validFrames < UINT8_MAX) {
        validFrames++;
    }
    lastAuxUs = currentTimeUs;
    lastParserResetUs = currentTimeUs;  // recovery resets are measured from the last valid frame
#ifndef SIMULATOR_BUILD
    lastUartResetUs = currentTimeUs;
#endif
    winAuxFrames++;

    switch (state) {
    case SD_DISARMED:
        if (armedFrame) {
            if (newSession) {
                // First frame of a link session says ARMED (reboot or link recovered while the master was armed): the master is primary, no checks
                servoDriverSetState(SD_ARMED, DIAG_CAUSE_FIRST_FRAME_ARMED);
            } else if (!prevFrameArmed) {
                // Observed arm edge: own-health checks. On reject the board stays disarmed until the next edge.
                const rejectReason_e reason = servoDriverCheckArmEdge();
                diagNoteArmEdge(reason);
                if (reason == REJECT_NONE) {
                    servoDriverSetState(SD_ARMED, DIAG_CAUSE_EDGE_PASS);
                } else if (!haveRejectText) {
                    haveRejectText = true;
                    lastRejectTextUs = currentTimeUs;
                    const bool textSent = servoDriverSendStatusText(MAV_SEVERITY_WARNING, rejectText[reason]);
                    diagNoteRejectText(textSent);
                }
            }
        }
        break;
    case SD_ARMED:
        if (!armedFrame) {
            servoDriverSetState(SD_DISARMED, DIAG_CAUSE_DISARM_FRAME);
        }
        break;
    case SD_FAILSAFE:
        // In-flight recovery: no checks, the master decides
        servoDriverSetState(armedFrame ? SD_ARMED : SD_DISARMED, DIAG_CAUSE_FAILSAFE_RECOVER);
        servoDriverSendStatusText(MAV_SEVERITY_CRITICAL, "FAILSAFE: link restored");
        break;
    }
    prevFrameArmed = armedFrame;

    // Servos 1-5. Servos 6-8 of the message have no output on this board.
    for (int i = 0; i < SERVO_DRIVER_SERVO_COUNT; i++) {
        if (pulseValid(aux.servo[i])) {
            servoDriverWriteServo(i, aux.servo[i]);
        } else {
            clamped++;
        }
    }

    // The motor values arrive in the same frame as the arming status, so they are never older than the arm state
    if (state == SD_ARMED) {
        for (int i = 0; i < SERVO_DRIVER_MOTOR_COUNT; i++) {
            if (pulseValid(aux.motor[i])) {
                motorPulse[i] = constrain(aux.motor[i], motorConfig()->mincommand, motorConfig()->maxthrottle);
                if (motorPulse[i] != aux.motor[i]) {
                    clamped++;
                }
            } else {
                motorPulse[i] = motorConfig()->mincommand;
                clamped++;
            }
        }
    }
}

// Sequence gaps count as lost frames (all messages of the master share the sequence counter)
static void servoDriverTrackSequence(uint8_t seq)
{
    if (haveSeq) {
        winLost += (uint8_t)(seq - lastSeq - 1);
    }
    haveSeq = true;
    lastSeq = seq;
    winMessages++;
}

// Clear the "have" flag of each timestamp window once it has expired, so an old timestamp is never compared (int32 wraps at 35.8 min)
static void servoDriverExpireWindows(timeUs_t currentTimeUs)
{
    if (haveCrcError && cmpTimeUs(currentTimeUs, lastCrcErrorUs) >= SERVO_DRIVER_LINK_CRC_QUIET_US) {
        haveCrcError = false;
    }
    if (haveConfigActivity && cmpTimeUs(currentTimeUs, lastConfigActivityUs) >= SERVO_DRIVER_CONFIG_QUIET_US) {
        haveConfigActivity = false;
    }
    if (haveRejectText && cmpTimeUs(currentTimeUs, lastRejectTextUs) >= SERVO_DRIVER_REJECT_TEXT_US) {
        haveRejectText = false;
    }
    if (haveLowRateText && cmpTimeUs(currentTimeUs, lastLowRateTextUs) >= SERVO_DRIVER_LOW_RATE_TEXT_US) {
        haveLowRateText = false;
    }
}

// No valid ADUM_AUX_CHANNELS for the link timeout ends the link session; while armed that is FAILSAFE (it never times out)
static void servoDriverCheckLinkTimeout(timeUs_t currentTimeUs)
{
    if (sessionActive
        && cmpTimeUs(currentTimeUs, lastAuxUs) > (int32_t)servoDriverConfig()->linkTimeoutMs * 1000) {
        sessionActive = false;
        haveSeq = false;
        if (state == SD_ARMED) {
            servoDriverSetState(SD_FAILSAFE, DIAG_CAUSE_LINK_TIMEOUT);
            servoDriverSendStatusText(MAV_SEVERITY_CRITICAL, "FAILSAFE: link lost");
        }
    }
}

static void servoDriverOpenPort(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_MAVLINK);
    if (portConfig) {
        baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
        if (baudRateIndex == BAUD_AUTO) {
            baudRateIndex = SERVO_DRIVER_DEFAULT_BAUD;
        }
        mavlinkPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_MAVLINK, NULL, NULL,
            baudRates[baudRateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
    }
}

// Link recovery: reset the parser every 1 s and re-init the UART every 2 s while no valid frame arrives
static void servoDriverRecoverLink(timeUs_t currentTimeUs)
{
    if (!sessionActive && cmpTimeUs(currentTimeUs, lastAuxUs) > SERVO_DRIVER_SILENCE_CAP_US) {
        // keep the silence far below 2^31 us (35.8 min): the cap exceeds every recovery interval
        lastAuxUs = currentTimeUs - SERVO_DRIVER_SILENCE_CAP_US;
    }
    const int32_t silentUs = cmpTimeUs(currentTimeUs, lastAuxUs);
    if (silentUs >= SERVO_DRIVER_PARSER_RESET_US && cmpTimeUs(currentTimeUs, lastParserResetUs) >= SERVO_DRIVER_PARSER_RESET_US) {
        lastParserResetUs = currentTimeUs;
        mavlink_reset_channel_status(MAVLINK_CHANNEL);
    }
#ifndef SIMULATOR_BUILD
    // ASSUMPTION: on SITL the serial port is a TCP socket, re-opening it is not meaningful: the parser reset is enough there
    if (silentUs >= SERVO_DRIVER_UART_REINIT_US && cmpTimeUs(currentTimeUs, lastUartResetUs) >= SERVO_DRIVER_UART_REINIT_US) {
        lastUartResetUs = currentTimeUs;
        if (mavlinkPort) {
            closeSerialPort(mavlinkPort);
            mavlinkPort = NULL;
        }
        servoDriverOpenPort();
    }
#endif
}

#ifdef STM32F10X
// Independent watchdog, 250 ms nominal (LSI 40 kHz +-50 %: real timeout 170-500 ms): prescaler /32 -> 1250 Hz, reload 312
static void servoDriverWatchdogStart(void)
{
    IWDG->KR = 0x5555;      // unlock PR / RLR
    IWDG->PR = 3;
    IWDG->RLR = 312;
    IWDG->KR = 0xAAAA;
    IWDG->KR = 0xCCCC;      // start, cannot be stopped
}

void servoDriverWatchdogFeed(void)
{
    IWDG->KR = 0xAAAA;
}
#else
void servoDriverWatchdogFeed(void)
{
}
#endif

void servoDriverInit(void)
{
    diagInit();
    servoDriverOpenPort();

    // Until the first frame arrives: servos in the middle, motors at zero throttle
    for (int i = 0; i < SERVO_DRIVER_SERVO_COUNT; i++) {
        servoDriverWriteServo(i, servoConfig()->dev.servoCenterPulse);
    }
    motorEnable();
    servoDriverWriteMotors();
#ifdef STM32F10X
    servoDriverWatchdogStart();
#endif
}

void servoDriverUpdate(timeUs_t currentTimeUs)
{
    servoDriverWatchdogFeed();
    diagNoteCycleStart(currentTimeUs);

    if (!bootGraceDone && currentTimeUs / 1000 >= servoDriverConfig()->bootGraceMs) {
        bootGraceDone = true;
    }

    // Expire the timestamp windows first: the use sites below test the flags only
    servoDriverExpireWindows(currentTimeUs);

    // Failsafe check first, so that the HEARTBEAT of this cycle shows the state
    servoDriverCheckLinkTimeout(currentTimeUs);

    if (mavlinkPort) {
        mavlink_status_t *channelStatus = mavlink_get_channel_status(MAVLINK_CHANNEL);
        while (serialRxBytesWaiting(mavlinkPort)) {
            const uint8_t c = serialRead(mavlinkPort);
            diagNoteRxByte();
            const uint8_t result = mavlink_parse_char(MAVLINK_CHANNEL, c, &rxMessage, &rxStatus);
            // parse errors of this call: counted inside the frame parser (copied to rxStatus) and bad CRC (added by mavlink_parse_char)
            const uint16_t errors = rxStatus.packet_rx_drop_count + channelStatus->parse_error;
            channelStatus->parse_error = 0;
            if (errors) {
                errorsComm += errors;
                haveCrcError = true;
                lastCrcErrorUs = currentTimeUs;
                validFrames = 0;
            }
            if (result == MAVLINK_FRAMING_OK) {
                servoDriverTrackSequence(rxMessage.seq);
                if (rxMessage.msgid == MAVLINK_MSG_ID_ADUM_AUX_CHANNELS) {
                    servoDriverHandleAux(&rxMessage, currentTimeUs);
                }
                // incoming HEARTBEAT and other messages are otherwise ignored
            }
        }
        // a frame that arrives after a longer gap is the first of a new session: handled above, the timeout check ran before
    }

    diagNoteCycleEnd(txDrops, errorsComm);

    servoDriverRecoverLink(currentTimeUs);

    // Motor output on every cycle in every state: the ESCs always see a continuous pulse train
    servoDriverWriteMotors();

    if (mavlinkPort) {
        const int32_t sinceTickUs = cmpTimeUs(currentTimeUs, lastTickUs);
        // negative: lastTickUs is older than 2^31 us (it is never in the future)
        if (sinceTickUs >= SERVO_DRIVER_ONE_SECOND_US || sinceTickUs < 0) {
            // low frame rate: only for a session that was already active during the whole window
            if (sessionActive && sessionAtLastTick && winAuxFrames < SERVO_DRIVER_LOW_RATE_HZ
                && !haveLowRateText) {
                haveLowRateText = true;
                lastLowRateTextUs = currentTimeUs;
                servoDriverSendStatusText(MAV_SEVERITY_WARNING, "LINK: low frame rate");
            }
            lastTickUs = currentTimeUs;
            sessionAtLastTick = sessionActive;
            servoDriverSendHeartbeat();
            servoDriverSendSysStatus();
            winAuxFrames = 0;
            winMessages = 0;
            winLost = 0;
        }

        gpsPvt_t pvt;
        if (gpsGetNewPvt(&pvt)) {
            servoDriverSendGpsPvt(&pvt);
        }
    }
}

bool servoDriverIsArmed(void)
{
    return state == SD_ARMED;
}

bool servoDriverConfigLocked(void)
{
    return state != SD_DISARMED;
}

void servoDriverNoteConfigActivity(void)
{
    lastConfigActivityUs = micros();
    haveConfigActivity = true;
}

void servoDriverSetSaveBusy(bool busy)
{
    saveBusy = busy;
}

#endif // USE_SERVO_DRIVER
