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
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/servos.h"

#include "io/adum_mavlink.h"
#include "io/gps.h"
#include "io/serial.h"
#include "io/servo_driver.h"

#include "pg/motor.h"

#define SERVO_DRIVER_COMMS_TIMEOUT_US   500000  // armed and no valid MAVLink frame for this long: disarm
#define SERVO_DRIVER_HEARTBEAT_US       1000000
#define SERVO_DRIVER_MAVLINK_SYSID      1       // not checked on receive, placeholders until the ids are agreed
#define SERVO_DRIVER_MAVLINK_COMPID     1
#define SERVO_DRIVER_PULSE_MIN          500     // the only limits applied to servos
#define SERVO_DRIVER_PULSE_MAX          2500
#define SERVO_DRIVER_PULSE_INVALID      0       // 0 and UINT16_MAX in ADUM_AUX_CHANNELS mean "no value"
#define SERVO_DRIVER_HEADING_MIN_SPEED  300     // mm/s, below this heading_motion is not flagged valid
#define SERVO_DRIVER_DEFAULT_BAUD       BAUD_115200

#define MAVLINK_CHANNEL                 MAVLINK_COMM_0

static serialPort_t *mavlinkPort;
static mavlink_message_t rxMessage;
static mavlink_status_t rxStatus;

static uint16_t motorPulse[SERVO_DRIVER_MOTOR_COUNT];
static bool armed;              // arming_status of the last ADUM_AUX_CHANNELS, cleared on loss of communication
static timeUs_t lastCommsUs;    // last valid MAVLink frame of any kind
static timeUs_t lastHeartbeatUs;

static bool pulseValid(uint16_t pulseUs)
{
    return pulseUs != SERVO_DRIVER_PULSE_INVALID && pulseUs != UINT16_MAX;
}

// Motors are held at the minimum command (zero throttle) unless armed
static void servoDriverWriteMotors(void)
{
    float values[SERVO_DRIVER_MOTOR_COUNT];
    for (int i = 0; i < SERVO_DRIVER_MOTOR_COUNT; i++) {
        values[i] = armed ? motorPulse[i] : motorConfig()->mincommand;
    }
    motorWriteAll(values);
}

// Servos always follow the pulse that arrived, whatever the arm state. No limits apart from the clamp.
static void servoDriverWriteServo(int index, uint16_t pulseUs)
{
    pwmWriteServo(index, constrain(pulseUs, SERVO_DRIVER_PULSE_MIN, SERVO_DRIVER_PULSE_MAX));
}

static void servoDriverSetArmed(bool newArmed)
{
    if (newArmed == armed) {
        return;
    }
    armed = newArmed;
    for (int i = 0; i < SERVO_DRIVER_MOTOR_COUNT; i++) {
        motorPulse[i] = motorConfig()->mincommand;
    }
    if (armed) {
        ENABLE_ARMING_FLAG(ARMED);
    } else {
        DISABLE_ARMING_FLAG(ARMED);
    }
}

// Sends a frame if the port has room for all of it (never blocks, never sends half a frame)
static void servoDriverSend(const mavlink_message_t *msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t length = mavlink_msg_to_send_buffer(buffer, msg);
    if (serialTxBytesFree(mavlinkPort) < length) {
        return;
    }
    serialWriteBuf(mavlinkPort, buffer, length);
}

static void servoDriverSendHeartbeat(void)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(SERVO_DRIVER_MAVLINK_SYSID, SERVO_DRIVER_MAVLINK_COMPID, &msg,
        MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0, 0,
        armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
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

static void servoDriverHandleMessage(const mavlink_message_t *msg)
{
    if (msg->msgid != MAVLINK_MSG_ID_ADUM_AUX_CHANNELS) {
        return;     // HEARTBEAT only counts as communication
    }

    mavlink_adum_aux_channels_t aux;
    mavlink_msg_adum_aux_channels_decode(msg, &aux);

    servoDriverSetArmed(aux.arming_status == ADUM_ARMING_STATUS_ARMED);

    // Servos 1-5. Servos 6-8 of the message have no output on this board.
    for (int i = 0; i < SERVO_DRIVER_SERVO_COUNT; i++) {
        if (pulseValid(aux.servo[i])) {
            servoDriverWriteServo(i, aux.servo[i]);
        }
    }

    // The motor values arrive in the same frame as the arming status, so they are never older than the arm state
    if (armed) {
        for (int i = 0; i < SERVO_DRIVER_MOTOR_COUNT; i++) {
            motorPulse[i] = pulseValid(aux.motor[i]) ?
                constrain(aux.motor[i], motorConfig()->mincommand, motorConfig()->maxthrottle) : motorConfig()->mincommand;
        }
    }
    servoDriverWriteMotors();
}

void servoDriverInit(void)
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

    // Until the first frame arrives: servos in the middle, motors at zero throttle
    for (int i = 0; i < SERVO_DRIVER_SERVO_COUNT; i++) {
        servoDriverWriteServo(i, servoConfig()->dev.servoCenterPulse);
    }
    motorEnable();
    servoDriverWriteMotors();
}

void servoDriverUpdate(timeUs_t currentTimeUs)
{
    if (mavlinkPort) {
        while (serialRxBytesWaiting(mavlinkPort)) {
            const uint8_t c = serialRead(mavlinkPort);
            if (mavlink_parse_char(MAVLINK_CHANNEL, c, &rxMessage, &rxStatus)) {
                lastCommsUs = currentTimeUs;
                servoDriverHandleMessage(&rxMessage);
            }
        }

        if (cmpTimeUs(currentTimeUs, lastHeartbeatUs) >= SERVO_DRIVER_HEARTBEAT_US) {
            lastHeartbeatUs = currentTimeUs;
            servoDriverSendHeartbeat();
        }

        gpsPvt_t pvt;
        if (gpsGetNewPvt(&pvt)) {
            servoDriverSendGpsPvt(&pvt);
        }
    }

    // Lost communication: disarm, which pulls the motors to zero. The servos stay where the last frame put them.
    if (armed && cmpTimeUs(currentTimeUs, lastCommsUs) > SERVO_DRIVER_COMMS_TIMEOUT_US) {
        servoDriverSetArmed(false);
        servoDriverWriteMotors();
    }
}

bool servoDriverIsArmed(void)
{
    return armed;
}

#endif // USE_SERVO_DRIVER
