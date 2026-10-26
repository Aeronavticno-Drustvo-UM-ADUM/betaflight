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

#pragma once

#include "common/time.h"

// Servo driver mode: the flight controller core (gyro, PID, RX, mixer) is not run.
// Servo and motor outputs are driven directly from MAVLink RC_CHANNELS_OVERRIDE
// (channels 1-5 are servos, channels 6-8 are motors, pulse width in us), and the
// arm state arrives in MAVLink HEARTBEAT (or MAV_CMD_COMPONENT_ARM_DISARM).
// The outputs are disarmed, and the servos are centred, when no RC_CHANNELS_OVERRIDE is received for
// SERVO_DRIVER_RC_TIMEOUT_US: servos centre, motors go to the mincommand pulse.

#define SERVO_DRIVER_SERVO_COUNT   5
#define SERVO_DRIVER_MOTOR_COUNT   3

void servoDriverInit(void);
void servoDriverUpdate(timeUs_t currentTimeUs);
bool servoDriverIsArmed(void);
