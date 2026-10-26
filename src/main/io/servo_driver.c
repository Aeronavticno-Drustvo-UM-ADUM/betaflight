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

#include "io/serial.h"
#include "io/servo_driver.h"

#include "pg/motor.h"

// mavlink library uses unnames unions that's causes GCC to complain if -Wpedantic is used
// until this is resolved in mavlink library - ignore -Wpedantic for mavlink code
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

#define SERVO_DRIVER_RC_TIMEOUT_US      500000  // disarm when no RC_CHANNELS_OVERRIDE for this long
#define SERVO_DRIVER_HEARTBEAT_US       1000000
#define SERVO_DRIVER_MAVLINK_SYSID      1
#define SERVO_DRIVER_MAVLINK_COMPID     MAV_COMP_ID_SERVO1
#define SERVO_DRIVER_PULSE_MIN          500
#define SERVO_DRIVER_PULSE_MAX          2500
#define SERVO_DRIVER_CHANNEL_COUNT      8       // RC_CHANNELS_OVERRIDE channels 1..8: servos first, then motors

#define MAVLINK_CHANNEL                 MAVLINK_COMM_0
#define MAVLINK_UINT16_IGNORE           UINT16_MAX // RC_CHANNELS_OVERRIDE: channel is not overridden

static serialPort_t *mavlinkPort;
static mavlink_message_t rxMessage;
static mavlink_status_t rxStatus;

static uint16_t servoPulse[SERVO_DRIVER_SERVO_COUNT];
static uint16_t motorPulse[SERVO_DRIVER_MOTOR_COUNT];
static bool armRequested;       // arm state requested over MAVLink
static bool armed;              // armRequested and a recent RC_CHANNELS_OVERRIDE
static timeUs_t lastRcUs;
static bool rcReceived;         // at least one RC_CHANNELS_OVERRIDE since the last disarm
static timeUs_t lastHeartbeatUs;

static void servoDriverWriteMotors(void)
{
    float values[SERVO_DRIVER_MOTOR_COUNT];
    for (int i = 0; i < SERVO_DRIVER_MOTOR_COUNT; i++) {
        values[i] = motorPulse[i];
    }
    motorWriteAll(values);
}

// Safe state: servos in the middle, motors at the minimum command (ESC disarmed)
static void servoDriverSafeOutputs(void)
{
    const uint16_t centre = servoConfig()->dev.servoCenterPulse;
    for (int i = 0; i < SERVO_DRIVER_SERVO_COUNT; i++) {
        servoPulse[i] = centre;
        pwmWriteServo(i, centre);
    }
    for (int i = 0; i < SERVO_DRIVER_MOTOR_COUNT; i++) {
        motorPulse[i] = motorConfig()->mincommand;
    }
    servoDriverWriteMotors();
}

static void servoDriverSetArmed(bool newArmed)
{
    if (newArmed == armed) {
        return;
    }
    armed = newArmed;
    if (armed) {
        ENABLE_ARMING_FLAG(ARMED);
    } else {
        DISABLE_ARMING_FLAG(ARMED);
        servoDriverSafeOutputs();
    }
}

static void servoDriverSendHeartbeat(void)
{
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    const uint8_t baseMode = armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0;
    mavlink_msg_heartbeat_pack(SERVO_DRIVER_MAVLINK_SYSID, SERVO_DRIVER_MAVLINK_COMPID, &msg,
        MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, baseMode, 0, armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
    const uint16_t length = mavlink_msg_to_send_buffer(buffer, &msg);
    for (uint16_t i = 0; i < length; i++) {
        serialWrite(mavlinkPort, buffer[i]);
    }
}

static void servoDriverHandleMessage(const mavlink_message_t *msg, timeUs_t currentTimeUs)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
        // arm state of the ground station / companion computer
        armRequested = (mavlink_msg_heartbeat_get_base_mode(msg) & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        break;

    case MAVLINK_MSG_ID_COMMAND_LONG: {
        mavlink_command_long_t command;
        mavlink_msg_command_long_decode(msg, &command);
        if (command.command == MAV_CMD_COMPONENT_ARM_DISARM
            && (command.target_system == 0 || command.target_system == SERVO_DRIVER_MAVLINK_SYSID)) {
            armRequested = command.param1 >= 0.5f;
        }
        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
        mavlink_rc_channels_override_t rc;
        mavlink_msg_rc_channels_override_decode(msg, &rc);
        if (rc.target_system != 0 && rc.target_system != SERVO_DRIVER_MAVLINK_SYSID) {
            break;
        }
        const uint16_t channels[8] = {
            rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw,
            rc.chan5_raw, rc.chan6_raw, rc.chan7_raw, rc.chan8_raw
        };
        for (int i = 0; i < SERVO_DRIVER_CHANNEL_COUNT; i++) {
            if (channels[i] == MAVLINK_UINT16_IGNORE || channels[i] == 0) {
                continue;
            }
            if (i < SERVO_DRIVER_SERVO_COUNT) {
                servoPulse[i] = constrain(channels[i], SERVO_DRIVER_PULSE_MIN, SERVO_DRIVER_PULSE_MAX);
            } else {
                motorPulse[i - SERVO_DRIVER_SERVO_COUNT] = constrain(channels[i], motorConfig()->mincommand, motorConfig()->maxthrottle);
            }
        }
        lastRcUs = currentTimeUs;
        rcReceived = true;
        break;
    }

    default:
        break;
    }
}

void servoDriverInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_MAVLINK);
    if (portConfig) {
        baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
        if (baudRateIndex == BAUD_AUTO) {
            baudRateIndex = BAUD_57600;
        }
        mavlinkPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_MAVLINK, NULL, NULL,
            baudRates[baudRateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
    }

    motorEnable();
    servoDriverSafeOutputs();
}

void servoDriverUpdate(timeUs_t currentTimeUs)
{
    if (mavlinkPort) {
        while (serialRxBytesWaiting(mavlinkPort)) {
            const uint8_t c = serialRead(mavlinkPort);
            if (mavlink_parse_char(MAVLINK_CHANNEL, c, &rxMessage, &rxStatus)) {
                servoDriverHandleMessage(&rxMessage, currentTimeUs);
            }
        }

        if (cmpTimeUs(currentTimeUs, lastHeartbeatUs) >= SERVO_DRIVER_HEARTBEAT_US) {
            lastHeartbeatUs = currentTimeUs;
            servoDriverSendHeartbeat();
        }
    }

    // Link loss: disarm and make the ground side re-arm explicitly once the link is back
    if (rcReceived && cmpTimeUs(currentTimeUs, lastRcUs) > SERVO_DRIVER_RC_TIMEOUT_US) {
        rcReceived = false;
        armRequested = false;
    }

    servoDriverSetArmed(armRequested && rcReceived);

    if (armed) {
        for (int i = 0; i < SERVO_DRIVER_SERVO_COUNT; i++) {
            pwmWriteServo(i, servoPulse[i]);
        }
        servoDriverWriteMotors();
    }
}

bool servoDriverIsArmed(void)
{
    return armed;
}

#endif // USE_SERVO_DRIVER
