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
// The master flight controller sends MAVLink 2.0 ADUM_AUX_CHANNELS (arming status, servo[8], motor[3] in us):
// - servos 1-5 always move to the pulse that arrives, armed or not (only a 500-2500 us clamp applies)
// - motors 1-3 stay at the minimum command unless the last frame said armed; the values of an armed frame apply at once
// - no valid MAVLink frame for SERVO_DRIVER_COMMS_TIMEOUT_US while armed: disarm, motors go to the
//   minimum command and the servos keep the position of the last frame
// The board answers with a HEARTBEAT and with ADUM_GPS_PVT for every GPS solution.

#define SERVO_DRIVER_SERVO_COUNT   5
#define SERVO_DRIVER_MOTOR_COUNT   3

void servoDriverInit(void);
void servoDriverUpdate(timeUs_t currentTimeUs);
bool servoDriverIsArmed(void);
